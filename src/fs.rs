//! Stage 8: a tiny in-memory file system.
//!
//! There is no disk driver, so files live entirely in the heap and vanish on
//! reboot. The structure is a tree of [`Node`]s: a `File` holds bytes, a `Dir`
//! holds a name-to-`Node` map. The root is a directory. Paths look like
//! `/docs/hello.txt`; operations resolve a path by splitting on `/` and walking
//! the tree from the root.
//!
//! This is deliberately simple — no permissions, no timestamps, no real on-disk
//! layout (inodes, blocks, a superblock), and no VFS abstraction layer. It is the
//! "file abstraction" in its most basic form: named, hierarchical, byte-content
//! files you can create, read, write, list, and remove. A single global
//! [`Mutex`]-guarded [`RamFs`] holds the whole tree; only the shell touches it.

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
}

impl FsError {
    /// A short human-readable message, used by the shell to report errors.
    pub fn as_str(self) -> &'static str {
        match self {
            FsError::NotFound => "no such file or directory",
            FsError::Exists => "already exists",
            FsError::IsDir => "is a directory",
            FsError::NotDir => "not a directory",
        }
    }
}

/// Split a path into its non-empty components, so `/`, `/a`, `/a/`, and `//a`
/// all behave sensibly (e.g. `/docs//hello` -> ["docs", "hello"]).
fn components(path: &str) -> impl Iterator<Item = &str> {
    path.split('/').filter(|c| !c.is_empty())
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

/// The kernel's single in-memory file system. Guarded by a spinlock; only the
/// shell (task context) touches it, so a plain `Mutex` is enough.
static FS: Mutex<RamFs> = Mutex::new(RamFs::new());

// Thin locking wrappers — the API the shell calls. Each takes the lock, runs one
// operation, and releases it (results are owned, so nothing borrows the tree
// across the lock).

/// Create an empty directory at `path`.
pub fn mkdir(path: &str) -> Result<(), FsError> {
    FS.lock().mkdir(path)
}

/// Create or overwrite the file at `path` with `data`.
pub fn write(path: &str, data: &[u8]) -> Result<(), FsError> {
    FS.lock().write(path, data)
}

/// Read the bytes of the file at `path`.
pub fn read(path: &str) -> Result<Vec<u8>, FsError> {
    FS.lock().read(path)
}

/// List the directory at `path` as `(name, is_dir)` pairs.
pub fn list(path: &str) -> Result<Vec<(String, bool)>, FsError> {
    FS.lock().list(path)
}

/// Remove the file or directory at `path`.
pub fn remove(path: &str) -> Result<(), FsError> {
    FS.lock().remove(path)
}

/// Whether `path` is an existing directory.
pub fn is_dir(path: &str) -> bool {
    FS.lock().is_dir(path)
}
