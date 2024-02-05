use std::ops::{BitAnd, BitOr, Deref, Index};

use vcell::VolatileCell;

#[repr(C)]
pub struct Register<T> {
    reg: VolatileCell<T>,
}

impl<T: Copy> Register<T> {
    pub fn read(&self) -> T {
        self.reg.get()
    }
    pub fn set(&self, val: T) {
        self.reg.set(val)
    }

    // Unsafety: Need to be sure that the location at self+index is also a MMIO register
    pub unsafe fn offset(&self, index: isize) -> &Self {
        (self as *const Self).offset(index).as_ref().unwrap()
    }

    pub fn addr(&self) -> *const T {
        self.reg.as_ptr()
    }

    pub const fn at(addr: usize) -> &'static Self {
        unsafe { (addr as *const Register<T>).as_ref().unwrap() }
    }
}

impl<T: BitOr<Output = T> + Copy> Register<T> {
    pub fn xor_with(&self, val: T) {
        self.set(self.read() | val)
    }
}

impl<T: BitAnd<Output = T> + Copy> Register<T> {
    pub fn and_with(&self, val: T) {
        self.set(self.read() & val)
    }
}
