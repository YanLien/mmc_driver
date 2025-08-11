pub mod mci_hal;

use core::ptr::NonNull;

pub struct SdController {
    base_addr: NonNull<mci_hal::SdRegisters>,
}

impl SdController {
    pub fn new(base_addr: NonNull<u8>) -> Self {
        SdController {
            base_addr: base_addr.cast::<mci_hal::SdRegisters>(),
        }
    }

    pub fn registers(&self) -> &mci_hal::SdRegisters {
        unsafe { self.base_addr.as_ref() }
    }

    pub fn registers_mut(&mut self) -> &mut mci_hal::SdRegisters {
        unsafe { self.base_addr.as_mut() }
    }
}
