/* automatically generated by rust-bindgen 0.69.2 */

#[repr(C)]
#[derive(Copy, Clone, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct __BindgenBitfieldUnit<Storage> {
    storage: Storage,
}
impl<Storage> __BindgenBitfieldUnit<Storage> {
    #[inline]
    pub const fn new(storage: Storage) -> Self {
        Self { storage }
    }
}
impl<Storage> __BindgenBitfieldUnit<Storage>
where
    Storage: AsRef<[u8]> + AsMut<[u8]>,
{
    #[inline]
    pub fn get_bit(&self, index: usize) -> bool {
        debug_assert!(index / 8 < self.storage.as_ref().len());
        let byte_index = index / 8;
        let byte = self.storage.as_ref()[byte_index];
        let bit_index = if cfg!(target_endian = "big") {
            7 - (index % 8)
        } else {
            index % 8
        };
        let mask = 1 << bit_index;
        byte & mask == mask
    }
    #[inline]
    pub fn set_bit(&mut self, index: usize, val: bool) {
        debug_assert!(index / 8 < self.storage.as_ref().len());
        let byte_index = index / 8;
        let byte = &mut self.storage.as_mut()[byte_index];
        let bit_index = if cfg!(target_endian = "big") {
            7 - (index % 8)
        } else {
            index % 8
        };
        let mask = 1 << bit_index;
        if val {
            *byte |= mask;
        } else {
            *byte &= !mask;
        }
    }
    #[inline]
    pub fn get(&self, bit_offset: usize, bit_width: u8) -> u64 {
        debug_assert!(bit_width <= 64);
        debug_assert!(bit_offset / 8 < self.storage.as_ref().len());
        debug_assert!((bit_offset + (bit_width as usize)) / 8 <= self.storage.as_ref().len());
        let mut val = 0;
        for i in 0..(bit_width as usize) {
            if self.get_bit(i + bit_offset) {
                let index = if cfg!(target_endian = "big") {
                    bit_width as usize - 1 - i
                } else {
                    i
                };
                val |= 1 << index;
            }
        }
        val
    }
    #[inline]
    pub fn set(&mut self, bit_offset: usize, bit_width: u8, val: u64) {
        debug_assert!(bit_width <= 64);
        debug_assert!(bit_offset / 8 < self.storage.as_ref().len());
        debug_assert!((bit_offset + (bit_width as usize)) / 8 <= self.storage.as_ref().len());
        for i in 0..(bit_width as usize) {
            let mask = 1 << i;
            let val_bit_is_set = val & mask == mask;
            let index = if cfg!(target_endian = "big") {
                bit_width as usize - 1 - i
            } else {
                i
            };
            self.set_bit(index + bit_offset, val_bit_is_set);
        }
    }
}
pub const IEEE80211_TYPE_MGT: u32 = 0;
pub const IEEE80211_TYPE_CTL: u32 = 1;
pub const IEEE80211_TYPE_DATA: u32 = 2;
pub const IEEE80211_TYPE_MGT_SUBTYPE_BEACON: u32 = 8;
pub const IEEE80211_TYPE_MGT_SUBTYPE_ACTION: u32 = 13;
pub const IEEE80211_TYPE_MGT_SUBTYPE_PROBE_REQ: u32 = 4;
pub const IEEE80211_TYPE_MGT_SUBTYPE_PROBE_RESP: u32 = 5;
pub const IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION: u32 = 11;
pub const IEEE80211_TYPE_MGT_SUBTYPE_DEAUTHENTICATION: u32 = 12;
pub const IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ: u32 = 0;
pub const IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP: u32 = 1;
pub const IEEE80211_TYPE_MGT_SUBTYPE_DISASSOCIATION: u32 = 10;
pub const IEEE80211_TYPE_CTL_SUBTYPE_ACK: u32 = 13;
pub const IEEE80211_TYPE_DATA_SUBTYPE_DATA: u32 = 0;
pub type int_least64_t = i64;
pub type uint_least64_t = u64;
pub type int_fast64_t = i64;
pub type uint_fast64_t = u64;
pub type int_least32_t = i32;
pub type uint_least32_t = u32;
pub type int_fast32_t = i32;
pub type uint_fast32_t = u32;
pub type int_least16_t = i16;
pub type uint_least16_t = u16;
pub type int_fast16_t = i16;
pub type uint_fast16_t = u16;
pub type int_least8_t = i8;
pub type uint_least8_t = u8;
pub type int_fast8_t = i8;
pub type uint_fast8_t = u8;
pub type intmax_t = ::std::os::raw::c_longlong;
pub type uintmax_t = ::std::os::raw::c_ulonglong;
pub type macaddr_t = [u8; 6usize];
#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
pub struct mac80211_frame {
    pub frame_control: mac80211_frame_mac80211_frame_control,
    pub duration_id: u16,
    pub receiver_address: macaddr_t,
    pub transmitter_address: macaddr_t,
    pub address_3: macaddr_t,
    pub sequence_control: mac80211_frame_mac80211_sequence_control,
    pub data_and_fcs: [u8; 2316usize],
}
#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
pub struct mac80211_frame_mac80211_frame_control {
    pub _bitfield_align_1: [u8; 0],
    pub _bitfield_1: __BindgenBitfieldUnit<[u8; 2usize]>,
}
#[test]
fn bindgen_test_layout_mac80211_frame_mac80211_frame_control() {
    assert_eq!(
        ::std::mem::size_of::<mac80211_frame_mac80211_frame_control>(),
        2usize,
        concat!(
            "Size of: ",
            stringify!(mac80211_frame_mac80211_frame_control)
        )
    );
    assert_eq!(
        ::std::mem::align_of::<mac80211_frame_mac80211_frame_control>(),
        1usize,
        concat!(
            "Alignment of ",
            stringify!(mac80211_frame_mac80211_frame_control)
        )
    );
}
impl mac80211_frame_mac80211_frame_control {
    #[inline]
    pub fn protocol_version(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(0usize, 2u8) as u32) }
    }
    #[inline]
    pub fn set_protocol_version(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(0usize, 2u8, val as u64)
        }
    }
    #[inline]
    pub fn type_(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(2usize, 2u8) as u32) }
    }
    #[inline]
    pub fn set_type(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(2usize, 2u8, val as u64)
        }
    }
    #[inline]
    pub fn sub_type(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(4usize, 4u8) as u32) }
    }
    #[inline]
    pub fn set_sub_type(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(4usize, 4u8, val as u64)
        }
    }
    #[inline]
    pub fn to_ds(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(8usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_to_ds(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(8usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn from_ds(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(9usize, 1u8) as u32) }
    }
    #[inline]
    pub fn set_from_ds(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(9usize, 1u8, val as u64)
        }
    }
    #[inline]
    pub fn _flags(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(10usize, 6u8) as u32) }
    }
    #[inline]
    pub fn set__flags(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(10usize, 6u8, val as u64)
        }
    }
    #[inline]
    pub fn new_bitfield_1(
        protocol_version: ::std::os::raw::c_uint,
        type_: ::std::os::raw::c_uint,
        sub_type: ::std::os::raw::c_uint,
        to_ds: ::std::os::raw::c_uint,
        from_ds: ::std::os::raw::c_uint,
        _flags: ::std::os::raw::c_uint,
    ) -> __BindgenBitfieldUnit<[u8; 2usize]> {
        let mut __bindgen_bitfield_unit: __BindgenBitfieldUnit<[u8; 2usize]> = Default::default();
        __bindgen_bitfield_unit.set(0usize, 2u8, {
            let protocol_version: u32 = unsafe { ::std::mem::transmute(protocol_version) };
            protocol_version as u64
        });
        __bindgen_bitfield_unit.set(2usize, 2u8, {
            let type_: u32 = unsafe { ::std::mem::transmute(type_) };
            type_ as u64
        });
        __bindgen_bitfield_unit.set(4usize, 4u8, {
            let sub_type: u32 = unsafe { ::std::mem::transmute(sub_type) };
            sub_type as u64
        });
        __bindgen_bitfield_unit.set(8usize, 1u8, {
            let to_ds: u32 = unsafe { ::std::mem::transmute(to_ds) };
            to_ds as u64
        });
        __bindgen_bitfield_unit.set(9usize, 1u8, {
            let from_ds: u32 = unsafe { ::std::mem::transmute(from_ds) };
            from_ds as u64
        });
        __bindgen_bitfield_unit.set(10usize, 6u8, {
            let _flags: u32 = unsafe { ::std::mem::transmute(_flags) };
            _flags as u64
        });
        __bindgen_bitfield_unit
    }
}
#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
pub struct mac80211_frame_mac80211_sequence_control {
    pub _bitfield_align_1: [u8; 0],
    pub _bitfield_1: __BindgenBitfieldUnit<[u8; 2usize]>,
}
#[test]
fn bindgen_test_layout_mac80211_frame_mac80211_sequence_control() {
    assert_eq!(
        ::std::mem::size_of::<mac80211_frame_mac80211_sequence_control>(),
        2usize,
        concat!(
            "Size of: ",
            stringify!(mac80211_frame_mac80211_sequence_control)
        )
    );
    assert_eq!(
        ::std::mem::align_of::<mac80211_frame_mac80211_sequence_control>(),
        1usize,
        concat!(
            "Alignment of ",
            stringify!(mac80211_frame_mac80211_sequence_control)
        )
    );
}
impl mac80211_frame_mac80211_sequence_control {
    #[inline]
    pub fn fragment_number(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(0usize, 4u8) as u32) }
    }
    #[inline]
    pub fn set_fragment_number(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(0usize, 4u8, val as u64)
        }
    }
    #[inline]
    pub fn sequence_number(&self) -> ::std::os::raw::c_uint {
        unsafe { ::std::mem::transmute(self._bitfield_1.get(4usize, 12u8) as u32) }
    }
    #[inline]
    pub fn set_sequence_number(&mut self, val: ::std::os::raw::c_uint) {
        unsafe {
            let val: u32 = ::std::mem::transmute(val);
            self._bitfield_1.set(4usize, 12u8, val as u64)
        }
    }
    #[inline]
    pub fn new_bitfield_1(
        fragment_number: ::std::os::raw::c_uint,
        sequence_number: ::std::os::raw::c_uint,
    ) -> __BindgenBitfieldUnit<[u8; 2usize]> {
        let mut __bindgen_bitfield_unit: __BindgenBitfieldUnit<[u8; 2usize]> = Default::default();
        __bindgen_bitfield_unit.set(0usize, 4u8, {
            let fragment_number: u32 = unsafe { ::std::mem::transmute(fragment_number) };
            fragment_number as u64
        });
        __bindgen_bitfield_unit.set(4usize, 12u8, {
            let sequence_number: u32 = unsafe { ::std::mem::transmute(sequence_number) };
            sequence_number as u64
        });
        __bindgen_bitfield_unit
    }
}
#[test]
fn bindgen_test_layout_mac80211_frame() {
    const UNINIT: ::std::mem::MaybeUninit<mac80211_frame> = ::std::mem::MaybeUninit::uninit();
    let ptr = UNINIT.as_ptr();
    assert_eq!(
        ::std::mem::size_of::<mac80211_frame>(),
        2340usize,
        concat!("Size of: ", stringify!(mac80211_frame))
    );
    assert_eq!(
        ::std::mem::align_of::<mac80211_frame>(),
        1usize,
        concat!("Alignment of ", stringify!(mac80211_frame))
    );
    assert_eq!(
        unsafe { ::std::ptr::addr_of!((*ptr).frame_control) as usize - ptr as usize },
        0usize,
        concat!(
            "Offset of field: ",
            stringify!(mac80211_frame),
            "::",
            stringify!(frame_control)
        )
    );
    assert_eq!(
        unsafe { ::std::ptr::addr_of!((*ptr).duration_id) as usize - ptr as usize },
        2usize,
        concat!(
            "Offset of field: ",
            stringify!(mac80211_frame),
            "::",
            stringify!(duration_id)
        )
    );
    assert_eq!(
        unsafe { ::std::ptr::addr_of!((*ptr).receiver_address) as usize - ptr as usize },
        4usize,
        concat!(
            "Offset of field: ",
            stringify!(mac80211_frame),
            "::",
            stringify!(receiver_address)
        )
    );
    assert_eq!(
        unsafe { ::std::ptr::addr_of!((*ptr).transmitter_address) as usize - ptr as usize },
        10usize,
        concat!(
            "Offset of field: ",
            stringify!(mac80211_frame),
            "::",
            stringify!(transmitter_address)
        )
    );
    assert_eq!(
        unsafe { ::std::ptr::addr_of!((*ptr).address_3) as usize - ptr as usize },
        16usize,
        concat!(
            "Offset of field: ",
            stringify!(mac80211_frame),
            "::",
            stringify!(address_3)
        )
    );
    assert_eq!(
        unsafe { ::std::ptr::addr_of!((*ptr).sequence_control) as usize - ptr as usize },
        22usize,
        concat!(
            "Offset of field: ",
            stringify!(mac80211_frame),
            "::",
            stringify!(sequence_control)
        )
    );
    assert_eq!(
        unsafe { ::std::ptr::addr_of!((*ptr).data_and_fcs) as usize - ptr as usize },
        24usize,
        concat!(
            "Offset of field: ",
            stringify!(mac80211_frame),
            "::",
            stringify!(data_and_fcs)
        )
    );
}
