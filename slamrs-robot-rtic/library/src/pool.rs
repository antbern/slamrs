use core::{
    cell::UnsafeCell,
    ops::{Deref, DerefMut},
    sync::atomic::Ordering,
};

/// A pool of `M` fixed-sized buffers that can be acquired and sent across "threads" / through channels.
#[derive(Debug)]
pub struct BufferPool<const N: usize, const M: usize> {
    buffers: [UnsafeCell<[u8; N]>; M],
    borrows: [portable_atomic::AtomicU8; M],
}

impl<const N: usize, const M: usize> BufferPool<N, M> {
    pub fn new() -> Self {
        Self {
            buffers: [const { UnsafeCell::new([0u8; N]) }; M],
            borrows: [const { portable_atomic::AtomicU8::new(0) }; M],
        }
    }

    pub fn acquire(&self) -> Option<OwnedBuffer<'_, N>> {
        for (buffer, borrowed) in self.buffers.iter().zip(self.borrows.iter()) {
            // try to acquire the buffer, if it's not already borrowed
            if borrowed
                .compare_exchange(0, 1, Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
            {
                // success, we can now safely access the buffer
                #[allow(unsafe_code)]
                return Some(OwnedBuffer { buffer, borrowed });
            }
        }
        None
    }

    // fn acquire_shared(&self, idx: usize)
}

/// A buffer that is owned and can be accessed mutably.
pub struct OwnedBuffer<'a, const N: usize> {
    buffer: &'a UnsafeCell<[u8; N]>,
    borrowed: &'a portable_atomic::AtomicU8,
}

impl<'a, const N: usize> OwnedBuffer<'a, N> {
    #[allow(unsafe_code)]
    pub fn shared(self) -> SharedBuffer<'a, N> {
        // increase the borrowed count (since destructor will decrease it)
        self.borrowed.add(1, Ordering::Relaxed);
        SharedBuffer {
            buffer: unsafe { &*self.buffer.get() }, // SAFETY: we consume ourselves so no mutable references can exist
            borrowed: self.borrowed,
        }
    }
}
impl<'a, const N: usize> Drop for OwnedBuffer<'a, N> {
    fn drop(&mut self) {
        // release the buffer
        self.borrowed.sub(1, Ordering::Relaxed);
    }
}

impl<'a, const N: usize> Deref for OwnedBuffer<'a, N> {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        self.as_ref()
    }
}

impl<'a, const N: usize> DerefMut for OwnedBuffer<'a, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut()
    }
}

#[allow(unsafe_code)]
impl<'a, const N: usize> AsRef<[u8]> for OwnedBuffer<'a, N> {
    fn as_ref(&self) -> &[u8] {
        // SAFETY: we are the sole owner of the buffer, so it's safe to access it
        unsafe { &*self.buffer.get() }
    }
}
#[allow(unsafe_code)]
impl<'a, const N: usize> AsMut<[u8]> for OwnedBuffer<'a, N> {
    fn as_mut(&mut self) -> &mut [u8] {
        // SAFETY: we are the sole owner of the buffer, so it's safe to access it
        unsafe { &mut *self.buffer.get() }
    }
}

/// A reference-counted shared buffer that can be cloned and accessed immutably.
pub struct SharedBuffer<'a, const N: usize> {
    buffer: &'a [u8; N],
    borrowed: &'a portable_atomic::AtomicU8,
}

// on drop, release the buffer by subtracting 1 from the borrowed count
impl<const N: usize> Drop for SharedBuffer<'_, N> {
    fn drop(&mut self) {
        // release the buffer
        self.borrowed.sub(1, Ordering::Relaxed);
    }
}

impl<'a, const N: usize> Clone for SharedBuffer<'a, N> {
    fn clone(&self) -> Self {
        // increase the borrowed count
        self.borrowed.add(1, Ordering::Relaxed);
        SharedBuffer {
            buffer: self.buffer,
            borrowed: self.borrowed,
        }
    }
}

impl<'a, const N: usize> AsRef<[u8]> for SharedBuffer<'a, N> {
    fn as_ref(&self) -> &[u8] {
        self.buffer
    }
}
impl<'a, const N: usize> Deref for SharedBuffer<'a, N> {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        self.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_buffer_pool() {
        let pool = BufferPool::<32, 4>::new();
        let mut buffer = pool.acquire().unwrap();
        buffer[0] = 1;
        let buffer = buffer.shared();
        assert_eq!(buffer[0], 1);
        assert_eq!(&buffer[1..], &[0u8; 31]);
    }

    #[test]
    fn test_buffer_empty() {
        let pool = BufferPool::<32, 4>::new();
        let _buffer1 = pool.acquire().unwrap();
        let _buffer2 = pool.acquire().unwrap();
        let _buffer3 = pool.acquire().unwrap();
        let _buffer4 = pool.acquire().unwrap();
        assert!(pool.acquire().is_none());
        drop(_buffer1);
        let _buffer5 = pool.acquire().unwrap();
        assert!(pool.acquire().is_none());
    }

    #[test]
    fn test_multiple_shared() {
        let pool = BufferPool::<32, 1>::new();
        let buffer = pool.acquire().unwrap();
        assert!(pool.acquire().is_none());

        let shared1 = buffer.shared();
        let shared2 = shared1.clone();
        assert!(pool.acquire().is_none());

        drop(shared1);
        assert!(pool.acquire().is_none());
        drop(shared2);
        assert!(pool.acquire().is_some());
    }

    #[test]
    fn test_shared_buffer_is_send() {
        fn needs_send<T: Send>(_: T) {}

        let pool = BufferPool::<32, 1>::new();
        let buffer = pool.acquire().unwrap();
        let shared = buffer.shared();
        needs_send(shared);
    }
}
