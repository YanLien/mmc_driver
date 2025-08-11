pub mod sdhci_hal;

use core::ptr::NonNull;

pub struct EmmcController {
    base_addr: NonNull<sdhci_hal::EmmcRegisters>,
}

impl EmmcController {
    pub fn new(base_addr: NonNull<u8>) -> Self {
        EmmcController {
            base_addr: base_addr.cast::<sdhci_hal::EmmcRegisters>(),
        }
    }

    pub fn registers(&self) -> &sdhci_hal::EmmcRegisters {
        unsafe { self.base_addr.as_ref() }
    }

    pub fn registers_mut(&mut self) -> &mut sdhci_hal::EmmcRegisters {
        unsafe { self.base_addr.as_mut() }
    }
}
