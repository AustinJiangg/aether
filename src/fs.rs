//! Stage 8: a tiny in-memory file system.
//!
//! There is no disk driver, so files live entirely in the heap and vanish on
//! reboot. The structure is a tree of [`Node`]s: a `File` holds bytes, a `Dir`
//! holds a name-to-`Node` map. The root is a directory. Paths look like
//! `/docs/hello.txt`; operations resolve a path by splitting on `/` and walking
//! the tree from the root.
//!
//! This is deliberately simple — no permissions, no timestamps, no real on-disk
//! layout (inodes, blocks, a superblock). It is the "file abstraction" in its most
//! basic form: named, hierarchical, byte-content files you can create, read, write,
//! list, and remove. A single global [`Mutex`]-guarded [`RamFs`] holds the whole
//! tree; only the shell touches it.
//!
//! ## Stage 14a: the VFS seam
//!
//! Stage 14 adds a real *on-disk* filesystem (FAT) alongside this in-memory one. So
//! that the shell (and, later, system calls) need not care which kind of filesystem
//! backs a path, the operations are factored into a [`FileSystem`] trait — the
//! "virtual filesystem" (VFS) layer that real kernels put between user code and the
//! concrete filesystem drivers. [`RamFs`] is the first implementor; the FAT driver
//! (Stage 14b) will be the second, slotting in behind the same trait.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::vec::Vec;

use spin::Mutex;

/// One entry in the tree: either a file (its bytes) or a directory (its children).
enum Node {
    File(Vec<u8>),
    Dir(BTreeMap<String, Node>),
}

/// What can go wrong with a filesystem operation. A fieldless enum, so `Copy`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FsError {
    /// A path component (or the target) does not exist.
    NotFound,
    /// Tried to create something at a name that is already taken.
    Exists,
    /// Expected a file but found a directory.
    IsDir,
    /// Expected a directory but found a file.
    NotDir,
    /// The filesystem does not support this operation: a write to a read-only on-disk
    /// volume, or descending into a subdirectory a minimal driver cannot yet walk.
    Unsupported,
    /// An underlying block-device or on-disk-format error (an ATA read failed, a FAT
    /// cluster chain is corrupt, ...). The VFS surfaces it without further interpretation.
    Io,
}

impl FsError {
    /// A short human-readable message, used by the shell to report errors.
    pub fn as_str(self) -> &'static str {
        match self {
            FsError::NotFound => "no such file or directory",
            FsError::Exists => "already exists",
            FsError::IsDir => "is a directory",
            FsError::NotDir => "not a directory",
            FsError::Unsupported => "operation not supported",
            FsError::Io => "input/output error",
        }
    }
}

/// Split a path into its non-empty components, so `/`, `/a`, `/a/`, and `//a`
/// all behave sensibly (e.g. `/docs//hello` -> ["docs", "hello"]).
///
/// `pub(crate)` so the FAT driver shares the exact same path semantics as `RamFs`.
pub(crate) fn components(path: &str) -> impl Iterator<Item = &str> {
    path.split('/').filter(|c| !c.is_empty())
}

/// The virtual-filesystem interface: the set of operations every filesystem must
/// provide, regardless of where its bytes actually live (the heap, a disk, …).
///
/// This is the seam that lets [`RamFs`] and the coming FAT driver coexist: callers
/// (the shell, tests) work against `&dyn FileSystem`, and the concrete type behind
/// it can be swapped without touching them. Paths are always `/`-separated and
/// resolved from the filesystem's own root.
pub trait FileSystem {
    /// Create an empty directory at `path`.
    fn mkdir(&mut self, path: &str) -> Result<(), FsError>;
    /// Create or overwrite the file at `path` with `data`.
    fn write(&mut self, path: &str, data: &[u8]) -> Result<(), FsError>;
    /// Read the bytes of the file at `path`.
    fn read(&self, path: &str) -> Result<Vec<u8>, FsError>;
    /// List the directory at `path` as `(name, is_dir)` pairs.
    fn list(&self, path: &str) -> Result<Vec<(String, bool)>, FsError>;
    /// Remove the file or directory at `path`.
    fn remove(&mut self, path: &str) -> Result<(), FsError>;
    /// Whether `path` names an existing directory.
    fn is_dir(&self, path: &str) -> bool;
}

/// The whole in-memory file system: just its root directory.
pub struct RamFs {
    root: Node,
}

impl RamFs {
    /// An empty filesystem (only the root directory). `const` so it can
    /// initialize the global `FS` static at compile time.
    pub const fn new() -> RamFs {
        RamFs {
            root: Node::Dir(BTreeMap::new()),
        }
    }

    /// Look up the node at `path`, or `None` if any component is missing or a
    /// non-final component is a file (you cannot descend into a file).
    fn lookup(&self, path: &str) -> Option<&Node> {
        let mut node = &self.root;
        for comp in components(path) {
            node = match node {
                Node::Dir(entries) => entries.get(comp)?,
                Node::File(_) => return None,
            };
        }
        Some(node)
    }

    /// Look up the *parent directory* of `path` (mutably) plus the final
    /// component's name — the shape every create/remove needs. Returns `None` if
    /// the parent chain is missing, a parent is a file, or `path` is the root.
    fn lookup_parent_mut(&mut self, path: &str) -> Option<(&mut BTreeMap<String, Node>, String)> {
        let comps: Vec<&str> = components(path).collect();
        let (last, parents) = comps.split_last()?; // None if path is "/" (no comps)

        let mut node = &mut self.root;
        for comp in parents {
            node = match node {
                Node::Dir(entries) => entries.get_mut(*comp)?,
                Node::File(_) => return None,
            };
        }
        match node {
            Node::Dir(entries) => Some((entries, String::from(*last))),
            Node::File(_) => None,
        }
    }
}

impl FileSystem for RamFs {
    /// Create an empty directory at `path`. The parent must exist; the name must
    /// be free.
    fn mkdir(&mut self, path: &str) -> Result<(), FsError> {
        let (parent, name) = self.lookup_parent_mut(path).ok_or(FsError::NotFound)?;
        if parent.contains_key(&name) {
            return Err(FsError::Exists);
        }
        parent.insert(name, Node::Dir(BTreeMap::new()));
        Ok(())
    }

    /// Create or overwrite a file at `path` with `data`. The parent must exist,
    /// and the name must not already be a directory.
    fn write(&mut self, path: &str, data: &[u8]) -> Result<(), FsError> {
        let (parent, name) = self.lookup_parent_mut(path).ok_or(FsError::NotFound)?;
        if let Some(Node::Dir(_)) = parent.get(&name) {
            return Err(FsError::IsDir);
        }
        parent.insert(name, Node::File(data.to_vec()));
        Ok(())
    }

    /// Read a file's bytes (a copy). Fails if the path is missing or a directory.
    fn read(&self, path: &str) -> Result<Vec<u8>, FsError> {
        match self.lookup(path) {
            Some(Node::File(data)) => Ok(data.clone()),
            Some(Node::Dir(_)) => Err(FsError::IsDir),
            None => Err(FsError::NotFound),
        }
    }

    /// List a directory: each child as `(name, is_dir)`. Fails if the path is
    /// missing or a file.
    fn list(&self, path: &str) -> Result<Vec<(String, bool)>, FsError> {
        match self.lookup(path) {
            Some(Node::Dir(entries)) => Ok(entries
                .iter()
                .map(|(name, node)| (name.clone(), matches!(node, Node::Dir(_))))
                .collect()),
            Some(Node::File(_)) => Err(FsError::NotDir),
            None => Err(FsError::NotFound),
        }
    }

    /// Remove a file or directory (a directory takes its whole subtree with it,
    /// since dropping the `Node` drops its children). The root cannot be removed.
    fn remove(&mut self, path: &str) -> Result<(), FsError> {
        let (parent, name) = self.lookup_parent_mut(path).ok_or(FsError::NotFound)?;
        match parent.remove(&name) {
            Some(_) => Ok(()),
            None => Err(FsError::NotFound),
        }
    }

    /// Whether `path` names an existing directory (used by the shell's `cd`).
    fn is_dir(&self, path: &str) -> bool {
        matches!(self.lookup(path), Some(Node::Dir(_)))
    }
}

/// The kernel's root in-memory file system. Guarded by a spinlock; only the shell
/// (task context) and boot touch it, so a plain `Mutex` is enough.
static FS: Mutex<RamFs> = Mutex::new(RamFs::new());

/// A single optional mounted filesystem (Stage 14b-3): a real on-disk volume (the FAT driver)
/// layered onto the root tree at [`MOUNT_POINT`]. `None` until [`mount`] installs one. This is
/// a deliberately minimal "mount table" — one entry — but it is the real VFS idea: a path under
/// the mount point is served by a *different* filesystem, transparently to the caller.
static MOUNT: Mutex<Option<Box<dyn FileSystem + Send>>> = Mutex::new(None);

/// Where the mounted volume appears in the path namespace: `/mnt/HELLO.TXT` reads the disk's
/// `HELLO.TXT`, while everything outside `/mnt` stays in the in-memory tree.
const MOUNT_POINT: &str = "/mnt";

/// Install `filesystem` at [`MOUNT_POINT`] so paths under it are served by that filesystem.
/// Also creates the mount-point directory in the root tree (ignoring "already exists") so it
/// lists and `cd`s like any other directory. Replaces any previously mounted volume.
pub fn mount(filesystem: Box<dyn FileSystem + Send>) {
    let _ = FS.lock().mkdir(MOUNT_POINT); // ensure the mount point exists; ignore if it does
    *MOUNT.lock() = Some(filesystem);
}

/// If `path` lies at or under [`MOUNT_POINT`], return the path *within* the mounted volume (the
/// prefix stripped; the mount point itself becomes `/`). `None` means "use the root FS". The
/// boundary check (the next char is `/`, or the string ends) stops `/mntfoo` matching `/mnt`.
fn mount_subpath(path: &str) -> Option<String> {
    let rest = path.strip_prefix(MOUNT_POINT)?;
    if rest.is_empty() {
        Some(String::from("/")) // exactly the mount point -> the volume's root
    } else if rest.starts_with('/') {
        Some(String::from(rest)) // under the mount point
    } else {
        None // a different name that merely shares the prefix (e.g. "/mnt2")
    }
}

/// Route one filesystem operation to whichever filesystem backs `path`: the mounted volume if
/// `path` is under [`MOUNT_POINT`] and something is mounted there (with the prefix stripped),
/// otherwise the root in-memory tree. `op` receives `&mut dyn FileSystem`, which also serves
/// the read-only (`&self`) operations.
fn dispatch<R>(path: &str, op: impl FnOnce(&mut dyn FileSystem, &str) -> R) -> R {
    if let Some(sub) = mount_subpath(path) {
        let mut mount = MOUNT.lock();
        if let Some(fs) = mount.as_mut() {
            return op(fs.as_mut(), &sub);
        }
        // Nothing mounted there: fall through and let the root tree handle the literal path.
    }
    op(&mut *FS.lock(), path)
}

// Thin wrappers — the API the shell calls. Each routes through `dispatch`, so the shell need
// not know whether a path lands in the in-memory tree or on the FAT disk.

/// Create an empty directory at `path`.
pub fn mkdir(path: &str) -> Result<(), FsError> {
    dispatch(path, |fs, p| fs.mkdir(p))
}

/// Create or overwrite the file at `path` with `data`.
pub fn write(path: &str, data: &[u8]) -> Result<(), FsError> {
    dispatch(path, |fs, p| fs.write(p, data))
}

/// Read the bytes of the file at `path`.
pub fn read(path: &str) -> Result<Vec<u8>, FsError> {
    dispatch(path, |fs, p| fs.read(p))
}

/// List the directory at `path` as `(name, is_dir)` pairs.
pub fn list(path: &str) -> Result<Vec<(String, bool)>, FsError> {
    dispatch(path, |fs, p| fs.list(p))
}

/// Remove the file or directory at `path`.
pub fn remove(path: &str) -> Result<(), FsError> {
    dispatch(path, |fs, p| fs.remove(p))
}

/// Whether `path` is an existing directory.
pub fn is_dir(path: &str) -> bool {
    dispatch(path, |fs, p| fs.is_dir(p))
}
