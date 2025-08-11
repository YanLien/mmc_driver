#![no_std]
#![no_main]
#![feature(used_with_arg)]

extern crate alloc;

#[bare_test::tests]
mod tests {
    use core::{panic, ptr::NonNull, time::Duration};

    use alloc::boxed::Box;
    use bare_test::{
        GetIrqConfig,
        fdt_parser::Node,
        globals::{PlatformInfoKind, global_val},
        irq::{IrqHandleResult, IrqParam},
        mem::iomap,
        time::spin_delay,
    };
    use log::*;
    use mmc_driver::{
        Kernel,
        clk::{Clk, ClkError, init_global_clk},
        dump_register,
        mmc_host::sdhci_dwcmshc::{EmmcController, sdhci_hal::EmmcRegisters},
        set_impl,
    };
    use rk3568_clk::{CRU, cru_clksel_con28_bits::*};
    
    use tock_registers::interfaces::Readable;

    /// frequency constants
    const KHZ: u32 = 1_000;
    const MHZ: u32 = 1_000 * KHZ;

    struct SKernel;

    impl Kernel for SKernel {
        fn sleep(duration: Duration) {
            spin_delay(duration);
        }
    }

    set_impl!(SKernel);

    #[test]
    fn test_mmc_driver() {
        info!("Testing MMC driver...");
        let PlatformInfoKind::DeviceTree(fdt) = &global_val().platform_info;
        let fdt_parser = fdt.get();

        if let Some(phytium_info) = fdt_parser.find_compatible(&["phytium,mci"]).next() {
            let device_reg = phytium_info.reg().unwrap().next().unwrap();
            debug!(
                "Found Phytium device at {:#x}, Size: {:#x}",
                device_reg.address,
                device_reg.size.unwrap()
            );

            let mmc_base = iomap(
                (device_reg.address as usize).into(),
                device_reg.size.unwrap(),
            );
            debug!(
                "Mapped device base address: {:#x}",
                mmc_base.as_ptr() as usize
            );

            test_device(&phytium_info, mmc_base, None);
        } else if let Some(rk3568_info) = fdt_parser
            .find_compatible(&["rockchip,rk3568-dwcmshc"])
            .next()
        {
            let device_reg = rk3568_info.reg().unwrap().next().unwrap();
            debug!(
                "Found RK3568 device at {:#x}, Size: {:#x}",
                device_reg.address,
                device_reg.size.unwrap()
            );

            let mmc_base = iomap(
                (device_reg.address as usize).into(),
                device_reg.size.unwrap(),
            );

            // Try to find clock controller
            let clk_base = fdt_parser
                .find_compatible(&["rockchip,rk3568-cru"])
                .next()
                .map(|clock_info| {
                    let clk_reg = clock_info.reg().unwrap().next().unwrap();
                    debug!(
                        "Found clock controller at {:#x}, Size: {:#x}",
                        clk_reg.address,
                        clk_reg.size.unwrap()
                    );
                    iomap((clk_reg.address as usize).into(), clk_reg.size.unwrap())
                });
            debug!(
                "Mapped device base address: {:#x}",
                mmc_base.as_ptr() as usize
            );

            test_device(&rk3568_info, mmc_base, clk_base);
        } else {
            panic!("Unsupported device or no compatible device found");
        };

        info!("Test MMC driver Passed");
    }

    fn test_device(device_info: &Node<'_>, mmc_base: NonNull<u8>, clk_base: Option<NonNull<u8>>) {
        debug!("MMC base: {:#x}", mmc_base.as_ptr() as usize);

        if cfg!(feature = "irq") {
            debug!("Testing interrupts for device: {}", device_info.name());
            interrupt_register(device_info);
        }

        if let Some(clk_base) = clk_base {
            debug!("Clock base: {:#x}", clk_base.as_ptr() as usize);
            let _ = init_clk(clk_base.as_ptr() as usize);
        }

        let emmc_regs = EmmcController::new(mmc_base);

        info!(
            "eMMC SDMA Address: {:#x}",
            emmc_regs.registers().sdmasa.get()
        );

        dump_register(mmc_base, 0x10, 8);
    }

    fn interrupt_register(device_info: &Node<'_>) {
        let irq_info = device_info.irq_info().unwrap();

        IrqParam {
            intc: irq_info.irq_parent,
            cfg: irq_info.cfgs[0].clone(),
        }
        .register_builder(|_irq_num| {
            mmc_interrupt_handler();
            IrqHandleResult::Handled
        })
        .register();

        debug!(
            "registered irq {:?} for {:?}, irq_parent: {:?}, trigger: {:?}",
            irq_info.cfgs[0].irq,
            device_info.name(),
            irq_info.irq_parent,
            irq_info.cfgs[0].trigger
        );
    }

    // Not implemented here, temporary location.
    fn mmc_interrupt_handler() {
        todo!("Handle MMC interrupt");
    }

    pub fn init_clk(core_clk_index: usize) -> Result<(), ClkError> {
        let mmc_clk = ClkDriver::new(core_clk_index as u64);
        let static_clk: &'static dyn Clk = Box::leak(Box::new(mmc_clk));
        init_global_clk(static_clk);
        Ok(())
    }

    pub struct ClkDriver(CRU);

    impl ClkDriver {
        pub fn new(cru_address: u64) -> Self {
            ClkDriver(CRU::new(cru_address as *mut _))
        }
    }

    impl Clk for ClkDriver {
        fn init(&self) -> Result<(), ClkError> {
            Ok(())
        }

        fn mmc_get_clk(&self) -> Result<u64, ClkError> {
            let con = self.0.cru_clksel_get_cclk_emmc();
            let rate = con >> CRU_CLKSEL_CCLK_EMMC_POS;
            Ok(rate as u64)
        }

        fn mmc_set_clk(&self, rate: u64) -> Result<u64, ClkError> {
            info!("Setting eMMC clock to {} Hz", rate);
            let src_clk = match rate as u32 {
                r if r == 24 * MHZ => CRU_CLKSEL_CCLK_EMMC_XIN_SOC0_MUX,
                r if r == 52 * MHZ || r == 50 * MHZ => CRU_CLKSEL_CCLK_EMMC_CPL_DIV_50M,
                r if r == 100 * MHZ => CRU_CLKSEL_CCLK_EMMC_CPL_DIV_100M,
                r if r == 150 * MHZ => CRU_CLKSEL_CCLK_EMMC_GPL_DIV_150M,
                r if r == 200 * MHZ => CRU_CLKSEL_CCLK_EMMC_GPL_DIV_200M,
                r if r == 400 * KHZ || r == 375 * KHZ => CRU_CLKSEL_CCLK_EMMC_SOC0_375K,
                _ => panic!("Unsupported eMMC clock rate: {} Hz", rate),
            };
            self.0.cru_clksel_set_cclk_emmc(src_clk);
            Ok(rate)
        }
    }
}
