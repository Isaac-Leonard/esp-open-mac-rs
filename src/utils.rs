use std::ops::{BitAnd, BitOr, Deref, Index};

use vcell::VolatileCell;

#[repr(C)]
pub struct Register<T> {
    reg: *const VolatileCell<T>,
}

impl<T: Copy> Register<T> {
    #[inline]
    pub unsafe fn read(&self) -> T {
        (&*self.reg).get()
    }

    #[inline]
    pub unsafe fn set(&self, val: T) {
        (&*self.reg).set(val)
    }

    // Unsafety: Need to be sure that the location at self+index is also a MMIO register
    #[inline]
    pub unsafe fn offset(&self, index: isize) -> Self {
        Self {
            reg: self.reg.offset(index),
        }
    }

    #[inline]
    pub fn addr(&self) -> *const T {
        self.reg.cast()
    }

    #[inline]
    pub const fn at(addr: usize) -> Self {
        Self {
            reg: unsafe { addr as *const VolatileCell<T> },
        }
    }
}

impl<T: BitOr<Output = T> + Copy> Register<T> {
    pub unsafe fn xor_with(&self, val: T) {
        self.set(self.read() | val)
    }
}

impl<T: BitAnd<Output = T> + Copy> Register<T> {
    pub unsafe fn and_with(&self, val: T) {
        self.set(self.read() & val)
    }
}
