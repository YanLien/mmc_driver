#![no_std]
#![no_main]
#![feature(used_with_arg)]

#[bare_test::tests]
mod tests {
    use core::panic;

    use bare_test::{
        GetIrqConfig,
        globals::{PlatformInfoKind, global_val},
        irq::{IrqHandleResult, IrqParam},
        mem::iomap,
    };
    use log::*;

    #[test]
    fn test_mmc_driver() {
        info!("Testing MMC driver...");
        let PlatformInfoKind::DeviceTree(fdt) = &global_val().platform_info;
        let fdt_parser = fdt.get();

        let device_info = if fdt_parser.find_compatible(&["phytium,mci"]).next().is_some() {
            fdt_parser.find_compatible(&["phytium,mci"]).next().unwrap()
        } else if fdt_parser.find_compatible(&["rockchip,rk3568-dwcmshc"]).next().is_some() {
            fdt_parser.find_compatible(&["rockchip,rk3568-dwcmshc"]).next().unwrap()
        } else {
            panic!("Unsupported device or no compatible device found");
        };

        let device_reg = device_info.reg().unwrap().next().unwrap();

        info!(
            "device reg: {:#x}, device reg size: {:#x}",
            device_reg.address,
            device_reg.size.unwrap()
        );

        let reg_base = iomap(
            (device_reg.address as usize).into(),
            device_reg.size.unwrap(),
        );

        info!(
            "Mapped device base address: {:#x}",
            reg_base.as_ptr() as usize
        );

        // if cfg!(feature = "irq") {
        //     let irq_info = device_info.irq_info().unwrap();

        //     IrqParam {
        //         intc: irq_info.irq_parent,
        //         cfg: irq_info.cfgs[0].clone(),
        //     }
        //     .register_builder(|_irq_num| {
        //         fsdif_interrupt_handler();
        //         IrqHandleResult::Handled
        //     })
        //     .register();

        //     info!(
        //         "registered irq {:?} for {:?}, irq_parent: {:?}, trigger: {:?}",
        //         irq_info.cfgs[0].irq,
        //         device_info.name(),
        //         irq_info.irq_parent,
        //         irq_info.cfgs[0].trigger
        //     );
        // }

        info!("Test MMC driver Passed");
    }
}
