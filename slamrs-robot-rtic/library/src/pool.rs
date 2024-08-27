#![allow(unsafe_code)]
use core::ptr::NonNull;

pub struct Pool<const N: usize, const K: usize> {
    store: &'static [[u8; N]; K],

    /// Stores all the pointers to the chunks of memory in a stack-like fashion
    ptr_stack: [Option<NonNull<[u8; N]>>; K],

    /// Points to one above the next valid element (eg the first invalid element)
    ptr_stack_pointer: usize,
}

impl<const N: usize, const K: usize> Pool<N, K> {
    pub fn new(store: &'static [[u8; N]; K]) -> Self {
        let mut ptr_stack = [None; K];

        for i in 0..K {
            let ptr = &store[i] as *const _ as *mut [u8; N];
            // SAFETY: The pointer is valid and the memory is initialized
            ptr_stack[i] = Some(unsafe { NonNull::new_unchecked(ptr) });
        }

        Self {
            store,
            ptr_stack,
            ptr_stack_pointer: K,
        }
    }

    /// Obtain an array from the pool, returning `None` if the pool is empty
    pub fn obtain(&mut self) -> Option<ByteArray<N>> {
        if self.ptr_stack_pointer == 0 {
            return None;
        }

        self.ptr_stack_pointer -= 1;
        let ptr = self.ptr_stack[self.ptr_stack_pointer].take();
        ptr.map(|ptr| ByteArray { ptr })
    }

    /// Release the obtained array into the pool, returning the array if the pool is full (should
    /// not happen)
    pub fn release(&mut self, array: ByteArray<N>) -> Option<ByteArray<N>> {
        if self.ptr_stack_pointer == K {
            return Some(array);
        }
        self.ptr_stack[self.ptr_stack_pointer] = Some(array.ptr);
        self.ptr_stack_pointer += 1;
        None
    }
}

pub struct ByteArray<const N: usize> {
    ptr: NonNull<[u8; N]>,
}
impl<const N: usize> core::ops::Deref for ByteArray<N> {
    type Target = [u8; N];

    fn deref(&self) -> &Self::Target {
        // SAFETY: The pointer is valid and the memory is initialized
        unsafe { self.ptr.as_ref() }
    }
}

impl<const N: usize> core::ops::DerefMut for ByteArray<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: The pointer is valid and the memory is initialized
        unsafe { self.ptr.as_mut() }
    }
}
