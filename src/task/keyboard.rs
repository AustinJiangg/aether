//! The async keyboard task.
//!
//! Stage 3 decoded and echoed keystrokes *inside* the keyboard interrupt
//! handler. That is the wrong place for real work: an interrupt handler should
//! return fast, and it must not take a lock that the code it interrupted might
//! already hold (a guaranteed deadlock). Stage 5 splits the job in two:
//!
//! - The interrupt handler (in `interrupts.rs`) does the bare minimum — read the
//!   raw scancode byte and call [`add_scancode`], which only pushes onto a
//!   lock-free queue and wakes this task. No locks, no allocation, no decoding.
//! - This task ([`print_keypresses`]) waits on a [`ScancodeStream`], decodes the
//!   bytes into characters with `pc-keyboard`, and echoes them — all in ordinary
//!   task context, where blocking and locking are fine.
//!
//! The two are connected by a global [`ArrayQueue`] (the bytes) and an
//! [`AtomicWaker`] (the "there is new input" signal). This producer/consumer
//! split is the canonical motivation for async in a kernel.

use core::pin::Pin;
use core::task::{Context, Poll};

use conquer_once::spin::OnceCell;
use crossbeam_queue::ArrayQueue;
use futures_util::stream::{Stream, StreamExt};
use futures_util::task::AtomicWaker;
use pc_keyboard::{layouts::Us104Key, DecodedKey, HandleControl, PS2Keyboard, ScancodeSet1};

use crate::{print, serial_print, serial_println};

/// How many unconsumed scancodes we buffer between interrupt and task. 100 is
/// far more than a human can outrun the executor by; if it ever fills, we drop
/// input rather than grow (no allocation in the interrupt path).
const SCANCODE_QUEUE_CAPACITY: usize = 100;

/// The bridge between the interrupt handler (producer) and the task (consumer).
///
/// A `OnceCell` because we cannot build the `ArrayQueue` in a `const` initializer
/// (it allocates, so it needs the heap, which only exists after boot). It is
/// initialized exactly once, when the `ScancodeStream` is created.
static SCANCODE_QUEUE: OnceCell<ArrayQueue<u8>> = OnceCell::uninit();

/// The "new input is available" signal. The task stores its waker here before it
/// sleeps; the interrupt handler calls `wake()` after enqueuing a scancode.
/// `AtomicWaker` is lock-free, so the handler can touch it without risking a
/// deadlock.
static WAKER: AtomicWaker = AtomicWaker::new();

/// Push a raw scancode onto the queue and wake the keyboard task.
///
/// Called from the keyboard interrupt handler, so it must be interrupt-safe:
/// `ArrayQueue::push` and `AtomicWaker::wake` are both lock-free, and the success
/// path never prints (printing takes the serial lock and could deadlock). The
/// warnings below only fire on the cold paths (queue full, or a keypress in the
/// brief window before the queue is initialized), where dropping a byte is fine.
pub(crate) fn add_scancode(scancode: u8) {
    if let Ok(queue) = SCANCODE_QUEUE.try_get() {
        if queue.push(scancode).is_err() {
            serial_println!("WARNING: scancode queue full; dropping keyboard input");
        } else {
            WAKER.wake();
        }
    } else {
        serial_println!("WARNING: scancode queue uninitialized; dropping keyboard input");
    }
}

/// An async stream of raw scancodes drained from [`SCANCODE_QUEUE`].
///
/// The private field stops anyone outside this module from constructing one with
/// `ScancodeStream {}`; they must go through [`new`](Self::new), which guarantees
/// the queue is initialized first.
pub struct ScancodeStream {
    _private: (),
}

impl ScancodeStream {
    /// Create the stream, initializing the global queue on the way. Must be
    /// called exactly once — a second call means two consumers would race for the
    /// same bytes, so we panic instead.
    pub fn new() -> Self {
        SCANCODE_QUEUE
            .try_init_once(|| ArrayQueue::new(SCANCODE_QUEUE_CAPACITY))
            .expect("ScancodeStream::new must be called only once");
        ScancodeStream { _private: () }
    }
}

impl Stream for ScancodeStream {
    type Item = u8;

    /// Yield the next scancode, or register to be polled again when one arrives.
    ///
    /// The double check around registering the waker closes a race: without it, a
    /// scancode could be pushed (and `wake()` called) in the gap between the first
    /// `pop` returning `None` and us storing the waker — that wake would be lost
    /// and the task could sleep forever with input waiting.
    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<u8>> {
        let queue = SCANCODE_QUEUE
            .try_get()
            .expect("scancode queue not initialized");

        // Fast path: a byte is already waiting, so don't bother with the waker.
        if let Some(scancode) = queue.pop() {
            return Poll::Ready(Some(scancode));
        }

        // Slow path: advertise our waker, then re-check the queue.
        WAKER.register(cx.waker());
        match queue.pop() {
            Some(scancode) => {
                // A byte slipped in; we're returning `Ready`, so drop the waker.
                WAKER.take();
                Poll::Ready(Some(scancode))
            }
            None => Poll::Pending,
        }
    }
}

/// The keyboard task: decode scancodes into characters and echo them.
///
/// `scancodes.next().await` suspends this task whenever the queue is empty; the
/// executor is free to run something else (or sleep the CPU) until the interrupt
/// handler wakes us. Decoding lives here, off the interrupt path, where taking
/// the screen/serial locks is safe.
pub async fn print_keypresses() {
    let mut scancodes = ScancodeStream::new();
    let mut keyboard = PS2Keyboard::new(ScancodeSet1::new(), Us104Key, HandleControl::Ignore);

    while let Some(scancode) = scancodes.next().await {
        // A key event may span several scancode bytes, so `add_byte` returns
        // `Ok(None)` until it has assembled a complete event.
        if let Ok(Some(event)) = keyboard.add_byte(scancode) {
            if let Some(key) = keyboard.process_keyevent(event) {
                match key {
                    DecodedKey::Unicode(character) => {
                        print!("{}", character);
                        serial_print!("{}", character);
                    }
                    DecodedKey::RawKey(raw) => {
                        print!("{:?}", raw);
                        serial_print!("{:?}", raw);
                    }
                }
            }
        }
    }
}
