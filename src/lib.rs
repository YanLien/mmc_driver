#![no_std]
#![no_main]
#![recursion_limit = "256"]
#![feature(alloc_error_handler)]

extern crate alloc;

pub mod clk;
pub mod mmc_common;
pub mod mmc_core;
pub mod mmc_host;
pub mod reg;

use core::{fmt::Write, ptr::NonNull, time::Duration};
use log::error;

pub trait Kernel {
    fn sleep(duration: Duration);
}

pub(crate) fn mci_sleep(duration: Duration) {
    unsafe extern "Rust" {
        fn _mci_sleep(duration: Duration);
    }

    unsafe {
        _mci_sleep(duration);
    }
}

#[macro_export]
macro_rules! set_impl {
    ($t: ty) => {
        #[unsafe(no_mangle)]
        unsafe fn _mci_sleep(duration: Duration) {
            <$t as $crate::Kernel>::sleep(duration)
        }
    };
}

/// Dumps the contents of a register at the specified address.
pub fn dump_register(addr: NonNull<u8>, size: usize, words_per_line: usize) {
    let ptr = addr.as_ptr() as *const u32;
    let count = size / 4;
    let base = addr.as_ptr() as usize;

    error!(
        "=== Register dump: 0x{:x} + 0x{:x} ({} words/line) ===",
        base, size, words_per_line
    );

    for i in (0..count).step_by(words_per_line) {
        let end = (i + words_per_line).min(count);
        let addr = base + i * 4;
        let word_count = end - i;

        let mut output = heapless::String::<256>::new();
        let _ = write!(output, "{:08x}:", addr);

        for j in 0..word_count {
            let _ = write!(output, " {:08x}", unsafe { *ptr.add(i + j) });
        }

        error!("{}", output.as_str());
    }

    error!("=== End of register dump ===");
}
