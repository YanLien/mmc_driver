# mmc_driver

This guide describes how to quickly test SD/MMC functionality on Firefly ROC-RK3568-PC / Phytium-PI development boards.

## Quick Start

### Prerequisites

+ Development Boards:
    + [Firefly ROC-RK3568-PC](https://wiki.t-firefly.com/zh_CN/ROC-RK3568-PC/preface.html)
    + [Phytium PI](https://gitee.com/phytium_embedded/phytium-embedded-docs)

+ Tool Dependencies: Install [ostool](https://github.com/ZR233/ostool) (for serial flashing and debugging)

### Install ostool:
``` bash
cargo install ostool
```

### Configure Serial Connection

Create configuration file:

``` bash
touch .bare-test.toml
``` 

Edit the `.bare-test.toml` file according to your development board type:

**For Firefly ROC-RK3568-PC**:

``` toml
# rk3568-firefly configuration
serial = "/dev/ttyUSB0"
baud_rate = 1500000
dtb_file = "./firmware/rk3568-firefly.dtb"
```

**For Phytium-PI**:
``` toml
# phytium-pi configuration
serial = "/dev/ttyUSB0"
baud_rate = 115200
dtb_file = "./fireware/phytium-pi.dtb"

[net]
interface = "wlp0s20f3"
```

Please adjust the serial device path and `.dtb` file path according to your actual setup.

### Testing Method

Compile and flash the test:

``` bash
cargo test --release --test test -- --show-output --uboot
```

The system will automatically use the above configuration to test via serial connection. Ensure the device is connected and permissions allow access to `/dev/ttyUSB0`.