use tock_registers::{
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite, WriteOnly},
};

register_structs! {
    pub EmmcRegisters {
        // SDMA System Address Register
        (0x0000 => pub sdmasa: ReadWrite<u32, SDMASA::Register>),
        // Block Size Register
        (0x0004 => pub blocksize: ReadWrite<u16, BLOCKSIZE::Register>),
        // 16-bit Block Count Register
        (0x0006 => blockcount: ReadWrite<u16, BLOCKCOUNT::Register>),
        // Argument Register
        (0x0008 => argument: ReadWrite<u32, ARGUMENT::Register>),
        // Transfer Mode Register
        (0x000C => xfer_mode: ReadWrite<u16, XFER_MODE::Register>),
        // Command Register
        (0x000E => cmd: ReadWrite<u16, CMD::Register>),
        // Response Register 0/1
        (0x0010 => resp01: ReadOnly<u32, RESP01::Register>),
        // Response Register 2/3
        (0x0014 => resp23: ReadOnly<u32, RESP23::Register>),
        // Response Register 4/5
        (0x0018 => resp45: ReadOnly<u32, RESP45::Register>),
        // Response Register 6/7
        (0x001C => resp67: ReadOnly<u32, RESP67::Register>),
        // Buffer Data Port Register
        (0x0020 => buf_data: ReadWrite<u32, BUF_DATA::Register>),
        // Present State Register
        (0x0024 => pstate: ReadOnly<u32, PSTATE::Register>),
        // Host Control 1 Register
        (0x0028 => host_ctrl1: ReadWrite<u8, HOST_CTRL1::Register>),
        // Power Control Register
        (0x0029 => pwr_ctrl: ReadWrite<u8, PWR_CTRL::Register>),
        // Block Gap Control Register
        (0x002A => bgap_ctrl: ReadWrite<u8, BGAP_CTRL::Register>),
        // _reserved area
        (0x002B => _reserved0),
        // Clock Control Register
        (0x002C => clk_ctrl: ReadWrite<u16, CLK_CTRL::Register>),
        // Timeout Control Register
        (0x002E => tout_ctrl: ReadWrite<u8, TOUT_CTRL::Register>),
        // Software Reset Register
        (0x002F => sw_rst: ReadWrite<u8, SW_RST::Register>),
        // Normal Interrupt Status Register
        (0x0030 => normal_int_stat: ReadWrite<u16, NORMAL_INT_STAT::Register>), /// W1C
        // Error Interrupt Status Register
        (0x0032 => error_int_stat: ReadWrite<u16, ERROR_INT_STAT::Register>), /// W1C
        // Normal Interrupt Status Enable Register
        (0x0034 => normal_int_stat_en: ReadWrite<u16, NORMAL_INT_STAT_EN::Register>),
        // Error Interrupt Status Enable Register
        (0x0036 => error_int_stat_en: ReadWrite<u16, ERROR_INT_STAT_EN::Register>),
        // Normal Interrupt Signal Enable Register
        (0x0038 => normal_int_signal_en: ReadWrite<u16, NORMAL_INT_SIGNAL_EN::Register>),
        // Error Interrupt Signal Enable Register
        (0x003A => error_int_signal_en: ReadWrite<u16, ERROR_INT_SIGNAL_EN::Register>),
        // Auto CMD Error Status Register
        (0x003C => auto_cmd_stat: ReadOnly<u16, AUTO_CMD_STAT::Register>),
        // Host Control 2 Register
        (0x003E => host_ctrl2: ReadWrite<u16, HOST_CTRL2::Register>),
        // Capabilities Register 1
        (0x0040 => capabilities1: ReadOnly<u32, CAPABILITIES1::Register>),
        // Capabilities Register 2
        (0x0044 => capabilities2: ReadOnly<u32, CAPABILITIES2::Register>),
        // _reserved area
        (0x0048 => _reserved1),
        // Force Event Register for Auto CMD Error Status Register
        (0x0050 => force_auto_cmd_stat: WriteOnly<u16, FORCE_AUTO_CMD_STAT::Register>),
        // Force Event Register for Error Interrupt Status Register
        (0x0052 => force_err_int_stat: WriteOnly<u16, FORCE_ERR_INT_STAT::Register>),
        // ADMA Error Status Register
        (0x0054 => adma_err_stat: ReadOnly<u8, ADMA_ERR_STAT::Register>),
        // _reserved area
        (0x0055 => _reserved2),
        // ADMA System Address Register
        (0x0058 => adma_sa: ReadWrite<u32, ADMA_SA::Register>),
        // _reserved area
        (0x005C => _reserved3),
        // Preset Value for Initialization
        (0x0060 => preset_init: ReadOnly<u16, PRESET_INIT::Register>),
        // Preset Value for Default Speed
        (0x0062 => preset_ds: ReadOnly<u16, PRESET_DS::Register>),
        // Preset Value for High Speed
        (0x0064 => preset_hs: ReadOnly<u16, PRESET_HS::Register>),
        // Preset Value for SDR12
        (0x0066 => preset_sdr12: ReadOnly<u16, PRESET_SDR12::Register>),
        // Preset Value for SDR25
        (0x0068 => preset_sdr25: ReadOnly<u16, PRESET_SDR25::Register>),
        // Preset Value for SDR50
        (0x006A => preset_sdr50: ReadOnly<u16, PRESET_SDR50::Register>),
        // Preset Value for SDR104
        (0x006C => preset_sdr104: ReadOnly<u16, PRESET_SDR104::Register>),
        // Preset Value for DDR50
        (0x006E => preset_ddr50: ReadOnly<u16, PRESET_DDR50::Register>),
        // _reserved area
        (0x0070 => _reserved4),
        // ADMA3 Integrated Descriptor Address Register
        (0x0078 => adma_id: ReadWrite<u32, ADMA_ID::Register>),
        // _reserved area
        (0x007C => _reserved5),
        // Slot Interrupt Status Register
        (0x00FC => slot_intr_status: ReadOnly<u16, SLOT_INTR_STATUS::Register>),
        // Host Controller Version
        (0x00FE => host_cntrl_vers: ReadOnly<u16, HOST_CNTRL_VERS::Register>),
        // _reserved area
        (0x0100 => _reserved6),
        // Command Queuing Version Register
        (0x0180 => cqver: ReadOnly<u32, CQVER::Register>),
        // Command Queuing Capabilities Register
        (0x0184 => cqcap: ReadOnly<u32, CQCAP::Register>),
        // Command Queuing Configuration Register
        (0x0188 => cqcfg: ReadWrite<u32, CQCFG::Register>),
        // Command Queuing Control Register
        (0x018C => cqctrl: ReadWrite<u32, CQCTRL::Register>),
        // Command Queuing Interrupt Status Register        record
        (0x0190 => cqis: ReadWrite<u32, CQIS::Register>),
        // Command Queuing Interrupt Status Enable Register
        (0x0194 => cqise: ReadWrite<u32, CQISE::Register>),
        // Command Queuing Interrupt Signal Enable Register
        (0x0198 => cqisge: ReadWrite<u32, CQISGE::Register>),
        // Command Queuing Interrupt Coalescing Register
        (0x019C => cqic: ReadWrite<u32, CQIC::Register>),
        // Command Queuing Task Descriptor List Base Address Register
        (0x01A0 => cqtdlba: ReadWrite<u32, CQTDLBA::Register>),
        // _reserved area
        (0x01A4 => _reserved7),
        // Command Queuing DoorBell Register
        (0x01A8 => cqtdbr: ReadWrite<u32, CQTDBR::Register>),
        // Command Queuing TaskClear Notification Register
        (0x01AC => cqtdbn: ReadWrite<u32, CQTDBN::Register>),
        // Command Queuing Device Queue Status Register
        (0x01B0 => cqdqs: ReadOnly<u32, CQDQS::Register>),
        // Command Queuing Device Pending Tasks Register
        (0x01B4 => cqdpt: ReadOnly<u32, CQDPT::Register>),
        // Command Queuing Task Clear Register
        (0x01B8 => cqtclr: ReadWrite<u32, CQTCLR::Register>),
        // _reserved area
        (0x01BC => _reserved8),
        // Command Queuing Send Status Configuration 1 Register
        (0x01C0 => cqssc1: ReadWrite<u32, CQSSC1::Register>),
        // Command Queuing Send Status Configuration 2 Register
        (0x01C4 => cqssc2: ReadWrite<u32>),
        // Command Queuing Command Response For Direct Command Register
        (0x01C8 => cqcrdct: ReadOnly<u32>),
        // _reserved area
        (0x01CC => _reserved9),
        // Command Queuing Command Response Mode Error Mask Register
        (0x01D0 => cqrmem: ReadWrite<u32, CQRMEM::Register>),
        // Command Queuing Task Error Information Register
        (0x01D4 => cqterri: ReadOnly<u32, CQTERRI::Register>),
        // Command Queuing Command Response Index Register
        (0x01D8 => cqcri: ReadOnly<u32, CQCRI::Register>),
        // Command Queuing Command Response Argument Register
        (0x01DC => cqcra: ReadOnly<u32, CQCRA::Register>),
        // _reserved area
        (0x01E0 => _reserved10),
        // Host Version ID Register
        (0x0500 => ver_id: ReadOnly<u32, VER_ID::Register>),
        // Host Version Type Register
        (0x0504 => ver_type: ReadOnly<u32, VER_TYPE::Register>),
        // Host Control 3 Register
        (0x0508 => host_ctrl3: ReadWrite<u8, HOST_CTRL3::Register>),
        // _reserved area
        (0x0509 => _reserved11),
        // EMMC Control Register
        (0x052C => emmc_ctrl: ReadWrite<u16, EMMC_CTRL::Register>),
        // Boot Control Register
        (0x052E => boot_ctrl: ReadWrite<u16, BOOT_CTRL::Register>),
        // _reserved area
        (0x0530 => _reserved12),
        // Boot Control Register
        (0x0540 => at_ctrl: ReadWrite<u32, AT_CTRL::Register>),
        // Boot Control Register
        (0x0544 => at_stat: ReadWrite<u32, AT_STAT::Register>),
        // _reserved area
        (0x0548 => _reserved13),
        // DLL Global Control Register
        (0x0800 => dll_ctrl: ReadWrite<u32, DLL_CTRL::Register>),
        // DLL Control Register For RXCLK
        (0x0804 => dll_rxclk: ReadWrite<u32, DLL_RXCLK::Register>),
        // DLL Control Register For TXCLK
        (0x0808 => dll_txclk: ReadWrite<u32, DLL_TXCLK::Register>),
        // DLL Control Register For STRBIN
        (0x080C => dll_strbin: ReadOnly<u32, DLL_STRBIN::Register>),
        // _reserved area
        (0x0810 => _reserved14),
        // DLL Status Register 0
        (0x0840 => dll_status0: ReadOnly<u32, DLL_STATUS0::Register>),
        // DLL Status Register 1
        (0x0844 => dll_status1: ReadOnly<u32, DLL_STATUS1::Register>),
        // _reserved area
        (0x0848 => _reserved15),
        (0x10000 => @END),
    }
}

register_bitfields![
    u8,

    pub HOST_CTRL1 [
        // Data Transfer Width.
        // For SD/eMMC mode, this bit selects the data transfer width of the Host Controller.
        // The Host Driver sets it to match the data width of the SD/eMMC card.
        DAT_XFER_WIDTH OFFSET(1) NUMBITS(1) [
            // 1'b0: 1-bit mode
            BusOneBit = 0,
            // 1'b1: 4-bit mode
            BusFourBit = 1
        ],
        // High Speed Enable.
        // In SD/eMMC mode, this bit is used to determine the selection of preset value for High Speed mode.
        // Before setting this bit, the Host Driver checks the High Speed Support in the Capabilities register.
        HIGH_SPEED_EN OFFSET(2) NUMBITS(1) [
            // 1'b0: Normal Speed mode
            NormalSpeed = 0,
            // 1'b1: High Speed mode
            HighSpeed = 1
        ],
        // DMA Select.
        // This field is used to select the DMA type.
        // When Host Version 4 Enable is 1 in Host Control 2 register:
        // 2'h0: SDMA is selected
        // 2'h1: Reserved
        // 2'h2: ADMA2 is selected
        // 2'h3: ADMA2 or ADMA3 is selected
        // When Host Version 4 Enable is 0 in Host Control 2 register:
        // 2'h0: SDMA is selected
        // 2'h1: Reserved
        // 2'h2: 32-bit Address ADMA2 is selected
        // 2'h3: Reserved
        DMA_SEL OFFSET(3) NUMBITS(2) [
            // 2'h0: SDMA is selected
            Sdma = 0,
            // 2'h1: Reserved
            Reserved1 = 1,
            // 2'h2: ADMA2 is selected (32-bit Address ADMA2 when Host Version 4 Enable = 0)
            Adma2 = 2,
            // 2'h3: ADMA2 or ADMA3 is selected (Reserved when Host Version 4 Enable = 0)
            Adma2OrAdma3 = 3
        ],
        // Extended Data Transfer Width.
        // This bit controls 8-bit bus width mode of embedded device.
        EXT_DAT_XFER OFFSET(5) NUMBITS(1) [
            // 1'b0: Bus Width is selected by the Data Transfer Width
            UseDatXferWidth = 0,
            // 1'b1: 8-bit Bus Width
            EightBit = 1
        ],
        // Card Detect Test Level.
        // This bit is enabled while the Card Detect Signal Selection is set to 1 and it indicates
        // whether a card inserted or not.
        CARD_DETECT_TEST_LVL OFFSET(6) NUMBITS(1) [
            // 1'b0: No Card
            NoCard = 0,
            // 1'b1: Card Inserted
            CardInserted = 1
        ],
        // Card Detect Signal Selection.
        // This bit selects a source for card detection. When the source for the card detection is switched,
        // the interrupt must be disabled during the switching period.
        CARD_DETECT_SIG_SEL OFFSET(7) NUMBITS(1) [
            // 1'b0: SDCD# (card_detect_n signal) is selected (for normal use)
            HardwareSignal = 0,
            // 1'b1: Card Detect Test Level is selected (for test purpose)
            TestLevel = 1
        ],
    ],

    pub PWR_CTRL [
        // If this bit is cleared, the Host Controller stops the SD Clock by clearing the SD_CLK_IN bit in the CLK_CTRL_R register.
        SD_BUS_PWR OFFSET(0) NUMBITS(1) [
            // 1'b0: Power off
            PowerOff = 0,
            // 1'b1: Power on
            PowerOn = 1
        ],
    ],

    pub BGAP_CTRL [
        // Stop At Block Gap Request.
        // This bit is used to stop executing read and write transactions at the next block gap for
        // non-DMA, SDMA, and ADMA transfers.
        STOP_BG_REQ OFFSET(0) NUMBITS(1) [
            // 1'b0: Transfer
            ContinueTransfer = 0,
            // 1'b1: Stop
            StopAtBlockGap = 1
        ],
        // Continue Request.
        // This bit is used to restart the transaction, which was stopped using the Stop At Block Gap Request.
        // The Host Controller automatically clears this bit when the transaction restarts. If stop at block gap
        // request is set to 1, any write to this bit is ignored.
        CONTINUE_REQ OFFSET(1) NUMBITS(1) [
            // 1'b0: No Affect
            NoAction = 0,
            // 1'b1: Restart
            RestartTransfer = 1
        ],
        // Read Wait Control.
        // This bit is used to enable the read wait protocol to stop read data using DAT[2] line if the card
        // supports read wait. Otherwise, the Host Controller has to stop the card clock to hold the read data.
        RD_WAIT_CTRL OFFSET(2) NUMBITS(1) [
            // 1'b0: Disable Read Wait Control
            Disabled = 0,
            // 1'b1: Enable Read Wait Control
            Enabled = 1
        ],
        // Interrupt At Block Gap.
        // This bit is valid only in the 4-bit mode of an SDIO card and is used to select a sample point in the
        // interrupt cycle. Setting to 1 enables interrupt detection at the block gap for a multiple block transfer.
        INT_AT_BGAP OFFSET(3) NUMBITS(1) [
            // 1'b0: Disable interrupt detection at block gap
            Disabled = 0,
            // 1'b1: Enable interrupt detection at block gap (SDIO 4-bit mode only)
            Enabled = 1
        ],
    ],

    pub TOUT_CTRL [
        // Data Timeout Counter Value.
        // This value determines the interval by which DAT line time-outs are detected.
        // Refer to the Data Time-out Error in the Error Interrupt Status register for information on factors that dictate time-out generation. Time-out clock frequency will be generated by dividing the sd clock TMCLK by this value. When setting this register, prevent inadvertent time-out events by clearing the Data Timeout Error Status Enable (in the Error Interrupt Status Enable register).
        TOUT_CNT OFFSET(0) NUMBITS(4) []
    ],

    pub SW_RST [
        // Software Reset For All.
        // This reset affects the entire Host Controller except for the card
        // detection circuit. During its initialization, the Host Driver sets this
        // bit to 1 to reset the Host Controller. All registers are reset except
        // the capabilities register. If this bit is set to 1, the Host Driver
        // must issue reset command and reinitialize the card.
        SW_RST_ALL OFFSET(0) NUMBITS(1) [
            // 1'b0: Work
            Work = 0,
            // 1'b1: Reset
            Reset = 1
        ],
        // Software Reset For CMD line.
        // This bit resets only a part of the command circuit to be able to
        // issue a command. This reset is effective only for a command
        // issuing circuit (including response error statuses related to
        // Command Inhibit (CMD) control) and does not affect the data
        // transfer circuit. Host Controller can continue data transfer even
        // after this reset is executed while handling subcommand-response
        // errors.
        // The following registers and bits are cleared by this bit:
        // a. Present State register - Command Inhibit (CMD) bit
        // b. Normal Interrupt Status register - Command Complete bit
        // c. Error Interrupt Status - Response error statuses related to
        // Command Inhibit (CMD) bit
        SW_RST_CMD OFFSET(1) NUMBITS(1) [
            // 1'b0: Work
            Work = 0,
            // 1'b1: Reset
            Reset = 1
        ],
        // Software Reset For DAT line.
        // This bit is used in SD/eMMC mode and it resets only a part of the
        // data circuit and the DMA circuit is also reset.
        // The following registers and bits are cleared by this bit:
        // a. Buffer Data Port register:
        // Buffer is cleared and initialized.
        // b. Present state register:
        // Buffer Read Enable
        // Buffer Write Enable
        // Read Transfer Active
        // Write Transfer Active
        // DAT Line Active
        // Command Inhibit (DAT)
        // c. Block Gap Control register:
        // Continue Request
        // Stop At Block Gap Request
        // d. Normal Interrupt status register:
        // Buffer Read Ready
        // Buffer Write Ready
        // DMA Interrupt
        // Block Gap Event
        // Transfer Complete
        SW_RST_DAT OFFSET(2) NUMBITS(1) [
            // 1'b0: Work
            Work = 0,
            // 1'b1: Reset
            Reset = 1
        ]
    ],

    pub ADMA_ERR_STAT [
        // ADMA Error States.
        // These bits indicate the state of ADMA when an error occurs
        // during ADMA data transfer.
        ADMA_ERR_STATES OFFSET(0) NUMBITS(2) [
            //  Stop DMA - SYS_ADR register points to a location next to the error descriptor
            STOP_DMA = 0,
            // Fetch Descriptor - SYS_ADR register points to the error descriptor
            FETCH_DESCRIPTOR = 1,
            // Never set this state (reserved)
            RESERVED = 2,
            // Transfer Data - SYS_ADR register points to a location next to the error descriptor
            TRANSFER_DATA = 3,
        ],
        // ADMA Length Mismatch Error States.
        // This error occurs in the following instances:
        // a. While the Block Count Enable is being set, the total data length
        // specified by the Descriptor table is different from that specified
        // by the Block Count and Block Length;
        // b. When the total data length cannot be divided by the block length
        ADMA_LEN_ERR OFFSET(2) NUMBITS(1) [
            // No Error
            NO_ERROR = 0,
            // Error occurred
            ERROR = 1,
        ],
    ],

    pub HOST_CTRL3 [
        // Command conflict check.
        // Host Controller monitors the CMD line whenever a command is
        // issued and checks whether the value driven on sd_cmd_out
        // matches the value on sd_cmd_in at next subsequent edge of
        // cclk_tx to determine command conflict error. This bit is cleared
        // only if the feed back delay (including IO Pad delay) is more than
        // (t_card_clk_period - t_setup), where t_setup is the setup time of
        // a flop in Host Controller. The I/O pad delay is consistent across
        // CMD and DATA lines, and it is within the value:
        // (2*t_card_clk_period - t_setup)
        CMD_CONFLICT_CHECK OFFSET(0) NUMBITS(1) [
            // Disable command conflict check
            DISABLE = 0,
            // Check for command conflict after 1 card clock cycle
            ENABLE = 1,
        ],
        // Internal clock gating disable control.
        // This bit must be used to disable IP's internal clock gating when
        // required. when disabled clocks are not gated. Clocks to the core
        // (except hclk) must be stopped when programming this bit.
        SW_CG_DIS OFFSET(4) NUMBITS(1) [
            // Internal clock gating is disabled, clocks are not gated internally
            CLOCK_GATING_DISABLED = 0,
            // Internal clock gates are active and clock gating is controlled internally
            CLOCK_GATING_ENABLED = 1,
        ],
    ],
];

register_bitfields![
    u16,

    pub BLOCKSIZE [
        // Transfer Block Size.
        // These bits specify the block size of data transfers. In case of memory, it is set to 512 bytes.
        // It can be accessed only if no transaction is executing. Read operations during transfers may
        // return an invalid value, and write operations are ignored.
        XFER_BLOCK_SIZE OFFSET(0) NUMBITS(11) [],
        // SDMA Buffer Boundary.
        // These bits specify the size of contiguous buffer in system memory. The SDMA transfer waits at
        // every boundary specified by these fields and the Host Controller generates the DMA interrupt to
        // request the Host Driver to update the SDMA System Address register.
        SDMA_BUF_BDARY OFFSET(12) NUMBITS(3) [
            // 4K bytes SDMA Buffer Boundary
            Boundary4K = 0,
            // 8K bytes SDMA Buffer Boundary
            Boundary8K = 1,
            // 16K bytes SDMA Buffer Boundary
            Boundary16K = 2,
            // 32K bytes SDMA Buffer Boundary
            Boundary32K = 3,
            // 64K bytes SDMA Buffer Boundary
            Boundary64K = 4,
            // 128K bytes SDMA Buffer Boundary
            Boundary128K = 5,
            // 256K bytes SDMA Buffer Boundary
            Boundary256K = 6,
            // 512K bytes SDMA Buffer Boundary
            Boundary512K = 7
        ]
    ],

    pub BLOCKCOUNT [
        // 16-bit Block Count.
        // If the Host Version 4 Enable bit is set 0 or the 16-bit Block Count register is set to non-zero,
        // the 16-bit Block Count register is selected.
        // If the Host Version 4 Enable bit is set 1 and the 16-bit Block Count register is set to zero,
        // the 32-bit Block Count register is selected.
        // Note: For Host Version 4 Enable = 0, this register must be set to 0 before programming the 32-bit
        // block count register when Auto CMD23 is enabled for non-DMA and ADMA modes.
        BLOCK_CNT OFFSET(0) NUMBITS(16) [],
    ],

    pub XFER_MODE [
        // DMA Enable.
        // This bit enables the DMA functionality. If this bit is set to 1, a DMA operation begins when the
        // Host Driver writes to the Command register. You can select one of the DMA modes by using DMA Select
        // in the Host Control 1 register.
        DMA_ENABLE OFFSET(0) NUMBITS(1) [
            Disabled = 0,
            Enabled = 1
        ],
        // Block Count Enable.
        // This bit is used to enable the Block Count register, which is relevant for multiple block transfers.
        // If this bit is set to 0, the Block Count register is disabled, which is useful in executing an
        // infinite transfer. The Host Driver must set this bit to 0 when ADMA is used.
        BLOCK_COUNT_ENABLE OFFSET(1) NUMBITS(1) [
            Disabled = 0,
            Enabled = 1,
        ],
        // Auto Command Enable.
        // This field determines use of Auto Command functions.
        AUTO_CMD_ENABLE OFFSET(2) NUMBITS(2) [
            // Auto Command Disabled
            Disabled = 0,
            // Auto CMD12 Enable
            AutoCmd12 = 1,
            // Auto CMD23 Enable
            AutoCmd23 = 2,
            // Auto CMD Auto Select
            AutoSelect = 3
        ],
        // Data Transfer Direction Select.
        // This bit defines the direction of DAT line data transfers. This bit is set to 1 by the Host Driver
        // to transfer data from the SD/eMMC card to the Host Controller and it is set to 0 for all other commands.
        DATA_XFER_DIR OFFSET(4) NUMBITS(1) [
            // Write (Host to Card)
            Write = 0,
            // Read (Card to Host)
            Read = 1
        ],
        // Multi/Single Block Select.
        // This bit is set when issuing multiple-block transfer commands using the DAT line. If this bit is set to 0, it is not necessary to set the Block Count register.
        MULTI_BLK_EN OFFSET(5) NUMBITS(1) [
            // Single Block
            SingleBlock = 0,
            // Multiple Block
            MultipleBlock = 1
        ],
        // Response Type R1/R5.
        // This bit selects either R1 or R5 as a response type when the Response Error Check is selected.
        RESP_TYPE OFFSET(6) NUMBITS(1) [
            // R1
            R1 = 0,
            // R5
            R5 = 1
        ],
        // Response Error Check Enable.
        // The Host Controller supports response check function to avoid overhead of response error check by
        // Host driver. Response types of only R1 and R5 can be checked by the Controller. If the Host Controller
        // checks the response error, set this bit to 1 and set Response Interrupt Disable to 1. If an error is
        // detected, the Response Error interrupt is generated in the Error Interrupt Status register.
        // Note:
        // a. Response error check must not be enabled for any response type other than R1 and R5.
        // b. Response check must not be enabled for the tuning command.
        RESP_ERR_CHK_ENABLE OFFSET(7) NUMBITS(1) [
            // Response Error Check is disabled
            Disabled = 0,
            // Response Error Check is enabled
            Enabled = 1
        ],
        // Response Interrupt Disable.
        // The Host Controller supports response check function to avoid overhead of response error check by the
        // Host driver. Response types of only R1 and R5 can be checked by the Controller. If Host Driver checks
        // the response error, set this bit to 0 and wait for Command Complete Interrupt and then check the response
        // register. If the Host Controller checks the response error, set this bit to 1 and set the Response Error
        // Check Enable bit to 1. The Command Complete Interrupt is disabled by this bit regardless of the Command
        // Complete Signal Enable.
        // Note: During tuning (when the Execute Tuning bit in the Host Control2 register is set), the Command
        // Complete Interrupt is not generated irrespective of the Response Interrupt Disable setting.
        RESP_INT_DISABLE OFFSET(8) NUMBITS(1) [
            // Response Interrupt is enabled
            Enabled = 0,
            // Response Interrupt is disabled
            Disabled = 1
        ],
    ],

    pub CMD [
        // Response Type Select.
        // This bit indicates the type of response expected from the card.
        RESP_TYPE_SELECT OFFSET(0) NUMBITS(2) [
            // 2'h0: No Response
            NoResponse = 0,
            // 2'h1: Response length 136
            Response136 = 1,
            // 2'h2: Response length 48
            Response48 = 2,
            // 2'h3: Response length 48, check Busy after response.
            Response48CheckBusy = 3
        ],
        // Sub Command Flag.
        // This bit distinguishes between a main command and a sub command.
        SUB_CMD_FLAG OFFSET(2) NUMBITS(1) [
            // 1'b0: Main Command
            MainCommand = 0,
            // 1'b1: Sub Command
            SubCommand = 1
        ],
        // Command CRC Check Enable.
        // This bit enables the Host Controller to check the CRC field in the response. If an error is detected,
        // it is reported as a Command CRC error.
        // Note: a. CRC Check enable must be set to 0 for the command with no response, R3 response, and R4 response.
        //       b. For the tuning command, this bit must always be set to 1 to enable the CRC check.
        CMD_CRC_CHK_ENABLE OFFSET(3) NUMBITS(1) [
            // 1'b0: Disable
            Disabled = 0,
            // 1'b1: Enable
            Enabled = 1
        ],
        // Command Index Check Enable.
        // This bit enables the Host Controller to check the index field in the response to verify if it has the
        // same value as the command index. If the value is not the same, it is reported as a Command Index error.
        // Note: a. Index Check enable must be set to 0 for the command with no response, R2 response, R3 response
        //          and R4 response.
        //       b. For the tuning command, this bit must always be set to enable the index check.
        CMD_IDX_CHK_ENABLE OFFSET(4) NUMBITS(1) [
            // 1'b0: Disable
            Disabled = 0,
            // 1'b1: Enable
            Enabled = 1
        ],
        // Data Present Select.
        // This bit is set to 1 to indicate that data is present and that the data is transferred using the DAT line.
        // This bit is set to 0 in the following instances:
        // a. Command using the CMD line
        // b. Command with no data transfer but using busy signal on the DAT[0] line
        // c. Resume Command
        DATA_PRESENT_SEL OFFSET(5) NUMBITS(1) [
            // 1'b0: No Data Present
            NoData = 0,
            // 1'b1: Data Present
            DataPresent = 1
        ],
        // Command Type.
        // These bits indicate the command type.
        // Note: While issuing Abort CMD using CMD12/CMD52 or reset CMD using CMD0/CMD52,
        //       CMD_TYPE field shall be set to 0x3.
        CMD_TYPE OFFSET(6) NUMBITS(2) [
            // 2'h0: Normal
            Normal = 0,
            // 2'h1: Suspend
            Suspend = 1,
            // 2'h2: Resume
            Resume = 2,
            // 2'h3: Abort
            Abort = 3
        ],
        // Command Index.
        // These bits are set to the command number that is specified in
        // bits 45-40 of the Command Format.
        CMD_INDEX OFFSET(8) NUMBITS(6) []
    ],

    pub CLK_CTRL [
        // Internal Clock Enable.
        // This bit is set to 0 when the Host Driver is not using the Host Controller.
        // The Host Controller must stop its internal clock to enter a very low power state. However, registers can still be read and written to. The value is reflected on the intclk_en signal.
        // Note: If this bit is not used to control the internal clock (base clock and master clock), it is recommended to set this bit to 1 .
        INTERNAL_CLK_EN OFFSET(0) NUMBITS(1) [
            // Stop internal clock
            STOP = 0b0,
            // Start internal clock oscillation
            OSCILLATE = 0b1,
        ],
        // Internal Clock Stable.
        // This bit enables the Host Driver to check the clock stability twice
        // after the Internal Clock Enable bit is set and after the PLL Enable
        // bit is set. This bit reflects the synchronized value of the
        // intclk_stable signal after the Internal Clock Enable bit is set to 1
        // and also reflects the synchronized value of the card_clk_stable
        // signal after the PLL Enable bit is set to 1.
        INTERNAL_CLK_STABLE OFFSET(1) NUMBITS(1) [
            // Clock not ready/stable
            NOT_READY = 0b0,
            // Clock ready and stable
            READY = 0b1,
        ],
        // SD/eMMC Clock Enable.
        // This bit stops the SDCLK or RCLK when set to 0. The
        // SDCLK/RCLK Frequency Select bit can be changed when this bit is
        // set to 0.
        SD_CLK_EN OFFSET(2) NUMBITS(1) [
            // Disable providing SDCLK/RCLK
            DISABLE = 0b0,
            // Enable providing SDCLK/RCLK
            ENABLE = 0b1,
        ],
        // Clock Generator Select.
        // This bit is used to select the clock generator mode in
        // SDCLK/RCLK Frequency Select. If Preset Value Enable = 0, this
        // bit is set by the Host Driver. If Preset Value Enable = 1, this bit is
        // automatically set to a value specified in one of the
        // Preset Value registers. The value is reflected on the
        // card_clk_gen_sel signal.
        CLK_GEN_SELECT OFFSET(5) NUMBITS(1) [
            // Divided Clock Mode
            DIVIDED_CLOCK = 0b0,
            // Programmable Clock Mode
            PROGRAMMABLE_CLOCK = 0b1,
        ],
        // These bits specify the upper 2 bits of 10-bit SDCLK/RCLK Frequency Select control.
        UPPER_FREQ_SEL OFFSET(6) NUMBITS(2) [],
        // SDCLK/RCLK Frequency Select.
        // These bits are used to select the frequency of the SDCLK signal.
        // These bits depend on setting of Preset Value Enable in the Host
        // Control 2 register. If Preset Value Enable = 0, these bits are set
        // by the Host Driver. If Preset Value Enable = 1, these bits are
        // automatically set to a value specified in one of the Preset Value
        // register. The value is reflected on the lower 8-bit of the card_clk_freq_sel signal.
        FREQ_SEL OFFSET(8) NUMBITS(8) [],
    ],

    pub NORMAL_INT_STAT [
        // Command Complete.
        // In an SD/eMMC Mode, this bit is set when the end bit of a
        // response except for Auto CMD12 and Auto CMD23.
        // This interrupt is not generated when the Response Interrupt
        // Disable in Transfer Mode Register is set to 1.
        CMD_COMPLETE OFFSET(0) NUMBITS(1) [
            // No Command Complete
            NO_COMPLETE = 0b0,
            // Command Complete
            COMPLETE = 0b1,
        ],
        // Transfer Complete.
        // This bit is set when a read/write transfer and a command with
        // status busy is completed.
        XFER_COMPLETE OFFSET(1) NUMBITS(1) [
            // Not complete
            NOT_COMPLETE = 0b0,
            // Command execution is completed
            COMPLETE = 0b1,
        ],
        // Block Gap Event.
        // This bit is set when both read/write transaction is stopped at
        // block gap due to a Stop at Block Gap Request.
        BGAP_EVENT OFFSET(2) NUMBITS(1) [
            // No Block Gap Event
            NO_EVENT = 0b0,
            // Transaction stopped at block gap
            STOPPED = 0b1,
        ],
        // DMA Interrupt.
        // This bit is set if the Host Controller detects the SDMA Buffer
        // Boundary during transfer. In case of ADMA, by setting the Int
        // field in the descriptor table, the Host controller generates this
        // interrupt. This interrupt is not generated after a Transfer
        // Complete.
        DMA_INTERRUPT OFFSET(3) NUMBITS(1) [
            // No DMA Interrupt
            NO_INTERRUPT = 0b0,
            // DMA Interrupt is generated
            INTERRUPT = 0b1,
        ],
        // Buffer Write Ready.
        // This bit is set if the Buffer Write Enable changes from 0 to 1.
        BUF_WR_READY OFFSET(4) NUMBITS(1) [
            // Not ready to write buffer
            NOT_READY = 0b0,
            // Ready to write buffer
            READY = 0b1,
        ],
        // Buffer Read Ready.
        // This bit is set if the Buffer Read Enable changes from 0 to 1.
        BUF_RD_READY OFFSET(5) NUMBITS(1) [
            // Not ready to read buffer
            NOT_READY = 0b0,
            // Ready to read buffer
            READY = 0b1,
        ],
        // Card Insertion.
        // This bit is set if the Card Inserted in the Present State register
        // changes from 0 to 1.
        CARD_INSERTION OFFSET(6) NUMBITS(1) [
            // Card state stable or Debouncing
            STABLE = 0b0,
            // Card Inserted
            INSERTED = 0b1,
        ],
        // Card Removal.
        // This bit is set if the Card Inserted in the Present State register
        // changes from 1 to 0.
        CARD_REMOVAL OFFSET(7) NUMBITS(1) [
            // Card state stable or Debouncing
            STABLE = 0b0,
            // Card Removed
            REMOVED = 0b1,
        ],
        // Card Interrupt.
        // This bit reflects the synchronized value of DAT[1] Interrupt Input
        // for SD Mode.
        CARD_INTERRUPT OFFSET(8) NUMBITS(1) [
            // No Card Interrupt
            NO_INTERRUPT = 0b0,
            // Generate Card Interrupt
            INTERRUPT = 0b1,
        ],
        // Re-tuning Event.
        // This bit is set if the Re-Tuning Request changes from 0 to 1. Re-Tuning request is not supported.
        RE_TUNE_EVENT OFFSET(12) NUMBITS(1) [],
        // FX Event.
        // This status is set when R[14] of response register is set to 1 and
        // Response Type R1/R5 is set to 0 in Transfer Mode register. This
        // interrupt is used with response check function.
        FX_EVENT OFFSET(13) NUMBITS(1) [
            // No Event
            NO_EVENT = 0b0,
            // FX Event is detected
            DETECTED = 0b1,
        ],
        // Command Queuing Event.
        // This status is set if Command Queuing/Crypto related event has
        // occurred in eMMC/SD mode. Read CQHCI's CQIS/CRNQIS register
        // for more details.
        CQE_EVENT OFFSET(14) NUMBITS(1) [
            // No Event
            NO_EVENT = 0b0,
            // Command Queuing Event is detected
            DETECTED = 0b1,
        ],
        // Error Interrupt.
        // If any of the bits in the Error Interrupt Status register are set,
        // then this bit is set.
        ERROR_INT_STAT OFFSET(15) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
    ],

    pub ERROR_INT_STAT [
        // Command Timeout Error.
        // In SD/eMMC Mode,this bit is set only if no response is returned
        // within 64 SD clock cycles from the end bit of the command. If the
        // Host Controller detects a CMD line conflict, along with Command
        // CRC Error bit, this bit is set to 1, without waiting for 64 SD/eMMC
        // card clock cycles.
        CMD_TOUT_ERR OFFSET(0) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Time out
            TIMEOUT = 0b1,
        ],
        // Command CRC Error.
        // Command CRC Error is generated in SD/eMMC mode for following
        // two cases.
        // a. If a response is returned and the Command Timeout Error is
        // set to 0 (indicating no timeout), this bit is set to 1 when detecting
        // a CRC error in the command response.
        // b. The Host Controller detects a CMD line conflict by monitoring
        // the CMD line when a command is issued. If the Host Controller
        // drives the CMD line to 1 level, but detects 0 level on the CMD line
        // at the next SD clock edge, then the Host Controller aborts the
        // command (stop driving CMD line) and set this bit to 1. The
        // Command Timeout Error is also set to 1 to distinguish a CMD line conflict.
        CMD_CRC_ERR OFFSET(1) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // CRC Error generated
            CRC_ERROR = 0b1,
        ],
        // Command End Bit Error.
        // This bit is set when detecting that the end bit of a command
        // response is 0 in SD/eMMC mode.
        CMD_END_BIT_ERR OFFSET(2) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // End Bit Error generated
            END_BIT_ERROR = 0b1,
        ],
        // Command Index Error.
        // This bit is set if a Command Index error occurs in the command
        // respons in SD/eMMC mode.
        CMD_IDX_ERR OFFSET(3) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
        // Data Timeout Error.
        // This bit is set in SD/eMMC mode when detecting one of the
        // following timeout conditions:
        // a. Busy timeout for R1b, R5b type
        // b. Busy timeout after Write CRC status
        // c. Write CRC Status timeout
        // d. Read Data timeout
        DATA_TOUT_ERR OFFSET(4) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Time out
            TIMEOUT = 0b1,
        ],
        // Data CRC Error.
        // This error occurs in SD/eMMC mode when detecting CRC error
        // when transferring read data which uses the DAT line, when
        // detecting the Write CRC status having a value of other than 010
        // or when write CRC status timeout.
        DATA_CRC_ERR OFFSET(5) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
        // Data End Bit Error.
        // This error occurs in SD/eMMC mode either when detecting 0 at
        // the end bit position of read data that uses the DAT line or at the
        // end bit position of the CRC status.
        DATA_END_BIT_ERR OFFSET(6) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
        // Auto CMD Error.
        // This error status is used by Auto CMD12 and Auto CMD23 in
        // SD/eMMC mode. This bit is set when detecting that any of the
        // bits D00 to D05 in Auto CMD Error Status register has changed
        // from 0 to 1. D07 is effective in case of Auto CMD12. Auto CMD
        // Error Status register is valid while this bit is set to 1 and may be
        // cleared by clearing of this bit.
        AUTO_CMD_ERR OFFSET(8) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
        // ADMA Error.
        // This bit is set when the Host Controller detects error during
        // ADMA-based data transfer. The error could be due to following
        // reasons:
        // a. Error response received from System bus (Master I/F)
        // b, ADMA3,ADMA2 Descriptors invalid
        // c. CQE Task or Transfer descriptors invalid
        // When the error occurs, the state of the ADMA is saved in the
        // ADMA Error Status register.
        // In eMMC CQE mode:
        // The Host Controller generates this Interrupt when it detects an
        // invalid descriptor data (Valid=0) at the ST_FDS state. ADMA Error
        // State in the ADMA Error Status indicates that an error has
        // occurred in ST_FDS state. The Host Driver may find that Valid bit
        // is not set at the error descriptor.
        ADMA_ERR OFFSET(9) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
        // Tuning Error.
        // This bit is set when an unrecoverable error is detected in a tuning
        // circuit except during the tuning procedure (occurrence of an error
        // during tuning procedure is indicated by Sampling Clock Select in
        // the Host Control 2 register). By detecting
        // Tuning Error, Host Driver needs to abort a command executing
        // and perform tuning. To reset tuning circuit, Sampling Clock Select
        // is set to 0 before executing tuning procedure. The Tuning Error is
        // higher priority than the other error interrupts generated during
        // data transfer. By detecting Tuning Error, the Host Driver must
        // discard data transferred by a current read/write command and
        // retry data transfer after the Host Controller retrieved from the
        // tuning circuit error.
        TUNING_ERR OFFSET(10) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
        // Response Error,
        // Host Controller Version 4.00 supports response error check
        // function to avoid overhead of response error check by Host Driver
        // during DMA execution. If Response Error Check Enable is set to 1
        // in the Transfer Mode register, Host
        // Controller Checks R1 or R5 response. If an error is detected in a
        // response, this bit is set to 1.
        RESP_ERR OFFSET(11) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
        // Boot Acknowledgement Error.
        // This bit is set when there is a timeout for boot acknowledgement
        // or when detecting boot ack status having a value other than 010.
        // This is applicable only when boot acknowledgement is expected in eMMC mode.
        BOOT_ACK_ERR OFFSET(12) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
    ],

    pub NORMAL_INT_STAT_EN [
        // Command Complete Status Enable.
        CMD_COMPLETE_STAT_EN OFFSET(0) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Transfer Complete Status Enable.
        XFER_COMPLETE_STAT_EN OFFSET(1) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Block Gap Event Status Enable.
        BGAP_EVENT_STAT_EN OFFSET(2) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // DMA Interrupt Status Enable.
        DMA_INTERRUPT_STAT_EN OFFSET(3) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Buffer Write Ready Status Enable.
        BUF_WR_READY_STAT_EN OFFSET(4) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Buffer Read Ready Status Enable.
        BUF_RD_READY_STAT_EN OFFSET(5) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Card Insertion Status Enable.
        CARD_INSERTION_STAT_EN OFFSET(6) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Card Removal Status Enable.
        CARD_REMOVAL_STAT_EN OFFSET(7) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Card Interrupt Status Enable.
        CARD_INTERRUPT_STAT_EN OFFSET(8) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // CQE Event Status Enable.
        CQE_EVENT_STAT_EN OFFSET(14) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
    ],

    pub ERROR_INT_STAT_EN [
        // Command Timeout Error Status Enable.
        CMD_TOUT_ERR_STAT_EN OFFSET(0) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Command CRC Error Status Enable.
        CMD_CRC_ERR_STAT_EN OFFSET(1) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Command End Bit Error Status Enable.
        CMD_END_BIT_ERR_STAT_EN OFFSET(2) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Command Index Error Status Enable.
        CMD_IDX_ERR_STAT_EN OFFSET(3) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Data Timeout Error Status Enable.
        DATA_TOUT_ERR_STAT_EN OFFSET(4) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Data CRC Error Status Enable.
        DATA_CRC_ERR_STAT_EN OFFSET(5) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Data End Bit Error Status Enable.
        DATA_END_BIT_ERR_STAT_EN OFFSET(6) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Auto CMD Error Status Enable.
        AUTO_CMD_ERR_STAT_EN OFFSET(8) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // ADMA Error Status Enable.
        ADMA_ERR_STAT_EN OFFSET(9) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Tuning Error Status Enable.
        TUNING_ERR_STAT_EN OFFSET(10) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Response Error Status Enable.
        RESP_ERR_STAT_EN OFFSET(11) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Boot Acknowledgment Error Status Enable.
        BOOT_ACK_ERR_STAT_EN OFFSET(12) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
    ],

    pub NORMAL_INT_SIGNAL_EN [
        // Command Complete Signal Enable.
        CMD_COMPLETE_SIGNAL_EN OFFSET(0) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Transfer Complete Signal Enable.
        XFER_COMPLETE_SIGNAL_EN OFFSET(1) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Block Gap Event Signal Enable.
        BGAP_EVENT_SIGNAL_EN OFFSET(2) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // DMA Interrupt Signal Enable.
        DMA_INTERRUPT_SIGNAL_EN OFFSET(3) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Buffer Write Ready Signal Enable.
        BUF_WR_READY_SIGNAL_EN OFFSET(4) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Buffer Read Ready Signal Enable.
        BUF_RD_READY_SIGNAL_EN OFFSET(5) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Card Insertion Signal Enable.
        CARD_INSERTION_SIGNAL_EN OFFSET(6) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Card Removal Signal Enable.
        CARD_REMOVAL_SIGNAL_EN OFFSET(7) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Card Interrupt Signal Enable.
        CARD_INTERRUPT_SIGNAL_EN OFFSET(8) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // CQE Event Signal Enable.
        CQE_EVENT_SIGNAL_EN OFFSET(14) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
    ],

    pub ERROR_INT_SIGNAL_EN [
        // Command Timeout Error Signal Enable.
        CMD_TOUT_ERR_SIGNAL_EN OFFSET(0) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Command CRC Error Signal Enable.
        CMD_CRC_ERR_SIGNAL_EN OFFSET(1) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Command End Bit Error Signal Enable.
        CMD_END_BIT_ERR_SIGNAL_EN OFFSET(2) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Command Index Error Signal Enable.
        CMD_IDX_ERR_SIGNAL_EN OFFSET(3) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Data Timeout Error Signal Enable.
        DATA_TOUT_ERR_SIGNAL_EN OFFSET(4) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Data CRC Error Signal Enable.
        DATA_CRC_ERR_SIGNAL_EN OFFSET(5) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Data End Bit Error Signal Enable.
        DATA_END_BIT_ERR_SIGNAL_EN OFFSET(6) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Auto CMD Error Signal Enable.
        AUTO_CMD_ERR_SIGNAL_EN OFFSET(8) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // ADMA Error Signal Enable.
        ADMA_ERR_SIGNAL_EN OFFSET(9) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Tuning Error Signal Enable.
        TUNING_ERR_SIGNAL_EN OFFSET(10) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Response Error Signal Enable.
        RESP_ERR_SIGNAL_EN OFFSET(11) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Boot Acknowledgment Error Signal Enable.
        BOOT_ACK_ERR_SIGNAL_EN OFFSET(12) NUMBITS(1) [
            // Masked
            MASKED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
    ],

    pub AUTO_CMD_STAT [
        // Auto CMD12 Not Executed.
        // If multiple memory block data transfer is not started due to a
        // command error, this bit is not set because it is not necessary to
        // issue an Auto CMD12. Setting this bit to 1 means that the Host
        // Controller cannot issue Auto CMD12 to stop multiple memory
        // block data transfer, due to some error. If this bit is set to 1, error
        // status bits (D04-D01) is meaningless. This bit is set to 0 when
        // Auto CMD Error is generated by Auto CMD23.
        AUTO_CMD12_NOT_EXEC OFFSET(0) NUMBITS(1) [
            // Executed
            EXECUTED = 0b0,
            // Not Executed
            NOT_EXECUTED = 0b1,
        ],
        // Auto CMD Timeout Error.
        // This bit is set if no response is returned with 64 SDCLK cycles
        // from the end bit of the command.
        // If this bit is set to 1, error status bits (D04-D01) are meaningless.
        AUTO_CMD_TOUT_ERR OFFSET(1) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Time out
            TIMEOUT = 0b1,
        ],
        // Auto CMD CRC Error.
        // This bit is set when detecting a CRC error in the command response.
        AUTO_CMD_CRC_ERR OFFSET(2) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // CRC Error Generated
            CRC_ERROR = 0b1,
        ],
        // Auto CMD End Bit Error.
        // This bit is set when detecting that the end bit of command response is 0.
        AUTO_CMD_EBIT_ERR OFFSET(3) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // End Bit Error Generated
            END_BIT_ERROR = 0b1,
        ],
        // Auto CMD Index Error.
        // This bit is set if the command index error occurs in response to a command.
        AUTO_CMD_IDX_ERR OFFSET(4) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
        // Auto CMD Response Error.
        // This bit is set when Response Error Check Enable in the Transfer
        // Mode register is set to 1 and an error is detected in R1 response
        // of either Auto CMD12 or CMD13. This status is ignored if any bit
        // between D00 to D04 is set to 1.
        AUTO_CMD_RESP_ERR OFFSET(5) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Error
            ERROR = 0b1,
        ],
        // Command Not Issued By Auto CMD12 Error.
        // If this bit is set to 1, CMD_wo_DAT is not executed due to an
        // Auto CMD12 Error (D04-D01) in this register. This bit is set to 0
        // when Auto CMD Error is generated by Auto CMD23.
        CMD_NOT_ISSUED_AUTO_CMD12 OFFSET(7) NUMBITS(1) [
            // No Error
            NO_ERROR = 0b0,
            // Not Issued
            NOT_ISSUED = 0b1,
        ],
    ],

    pub HOST_CTRL2 [
        // UHS Mode/eMMC Speed Mode Select.
        // These bits are used to select UHS mode in the SD mode of
        // operation. In eMMC mode, these bits are used to select eMMC Speed mode.
        UHS_MODE_SEL OFFSET(0) NUMBITS(3) [
            // SDR12/Legacy
            SDR12_LEGACY = 0b000,
            // SDR25/High Speed SDR
            SDR25_HS_SDR = 0b001,
            // SDR50
            SDR50 = 0b010,
            // SDR104/HS200
            SDR104_HS200 = 0b011,
            // DDR50/High Speed DDR
            DDR50_HS_DDR = 0b100,
            // HS400
            HS400 = 0b111,
            // Others: Reserved
        ],
        // 1.8V Signaling Enable.
        SIGNALING_EN OFFSET(3) NUMBITS(1) [
            // 3.3V Signalling
            SIGNALING_3V3 = 0b0,
            // 1.8V Signalling
            SIGNALING_1V8 = 0b1,
        ],
        // Execute Tuning.
        // This bit is set to 1 to start the tuning procedure in UHSI/eMMC
        // speed modes and this bit is automatically cleared when tuning
        // procedure is completed.
        EXEC_TUNING OFFSET(6) NUMBITS(1) [
            // Not Tuned or Tuning completed
            NOT_TUNED = 0b0,
            // Execute Tuning
            EXECUTE = 0b1,
        ],
        // Sampling Clock Select.
        // This bit is used by the Host Controller to select the sampling clock
        // in SD/eMMC mode to receive CMD and DAT. This bit is set by the
        // tuning procedure and is valid after the completion of tuning
        // (when Execute Tuning is cleared). Setting this bit to 1 means that
        // tuning is completed successfully and setting this bit to 0 means
        // that tuning has failed.
        SAMPLE_CLK_SEL OFFSET(7) NUMBITS(1) [
            // Fixed clock is used to sample data
            FIXED_CLOCK = 0b0,
            // Tuned clock is used to sample data
            TUNED_CLOCK = 0b1,
        ],
        // This bit should be set to 0 for SD/eMMC Interface.
        UHS2_IF_ENABLE OFFSET(8) NUMBITS(1) [],
        // ADMA2 Length Mode.
        // This bit selects ADMA2 Length mode to be either 16-bit or 26-bit.
        ADMA2_LEN_MODE OFFSET(10) NUMBITS(1) [
            // 16-bit Data Length Mode
            LENGTH_16BIT = 0b0,
            // 26-bit Data Length Mode
            LENGTH_26BIT = 0b1,
        ],
        // CMD23 Enable.
        // If the card supports CMD23, this bit is set to 1. This bit is used to
        // select Auto CMD23 or Auto CMD12 for ADMA3 datatransfer.
        CMD23_ENABLE OFFSET(11) NUMBITS(1) [
            // Auto CMD23 is disabled
            DISABLED = 0b0,
            // Auto CMD23 is enabled
            ENABLED = 0b1,
        ],
        // Host Version 4 Enable.
        // This bit selects either Version 3.00 compatible mode or Version 4
        // mode.
        HOST_VER4_ENABLE OFFSET(12) NUMBITS(1) [
            // Version 3.00 compatible mode
            VERSION_3 = 0b0,
            // Version 4 mode
            VERSION_4 = 0b1,
        ],
        // 64-bit Addressing.
        // This bit is effective when Host Version 4 Enable is set to 1.
        ADDRESSING OFFSET(13) NUMBITS(1) [
            // 32 bits addressing
            ADDRESSING_32BIT = 0b0,
            // 64 bits addressing
            ADDRESSING_64BIT = 0b1,
        ],
        // Asynchronous Interrupt Enable.
        // This bit can be set if a card supports asynchronous interrupts and
        // Asynchronous Interrupt Support is set to 1 in the Capabilities
        // register.
        ASYNC_INT_ENABLE OFFSET(14) NUMBITS(1) [
            // Disabled
            DISABLED = 0b0,
            // Enabled
            ENABLED = 0b1,
        ],
        // Preset Value Enable.
        // This bit enables automatic selection of SDCLK frequency and
        // Driver strength Preset Value registers. When Preset Value Enable
        // is set, SDCLK frequency generation (Frequency Select and Clock
        // Generator Select) and the driver strength selection are performed
        // by the controller.
        PRESET_VAL_ENABLE OFFSET(15) NUMBITS(1) [
            // SDCLK and Driver Strength are controlled by Host Driver
            HOST_CONTROLLED = 0b0,
            // Automatic Selection by Preset Value are Enabled
            AUTO_PRESET = 0b1,
        ],
    ],

    pub FORCE_AUTO_CMD_STAT [
        // Force Event for Auto CMD12 Not Executed.
        FORCE_AUTO_CMD12_NOT_EXEC OFFSET(0) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Auto CMD12 Not Executed Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Auto CMD Timeout Error.
        FORCE_AUTO_CMD12_TOUT_ERR OFFSET(1) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Auto CMD Timeout Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Auto CMD CRC Error.
        FORCE_AUTO_CMD_CRC_ERR OFFSET(2) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Auto CMD CRC Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Auto CMD End Bit Error.
        FORCE_AUTO_CMD_EBIT_ERR OFFSET(3) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Auto CMD End Bit Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Auto CMD Index Error.
        FORCE_AUTO_CMD_IDX_ERR OFFSET(4) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Auto CMD Index Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Auto CMD Response Error.
        FORCE_AUTO_CMD_RESP_ERR OFFSET(5) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Auto CMD Response Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Command Not Issued By Auto CMD12 Error.
        FORCE_CMD_NOT_ISSUED_AUTO_CMD12 OFFSET(7) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Command Not Issued By Auto CMD12 Error Status is set
            FORCE_SET = 0b1,
        ],
    ],

    pub FORCE_ERR_INT_STAT [
        // Force Event for Command Timeout Error.
        FORCE_CMD_TOUT_ERR OFFSET(0) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Command Timeout Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Command CRC Error.
        FORCE_CMD_CRC_ERR OFFSET(1) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Command CRC Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Command End Bit Error.
        FORCE_CMD_END_BIT_ERR OFFSET(2) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Command End Bit Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Command Index Error.
        FORCE_CMD_IDX_ERR OFFSET(3) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Command Index Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Data Timeout Error.
        FORCE_DATA_TOUT_ERR OFFSET(4) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Data Timeout Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Data CRC Error.
        FORCE_DATA_CRC_ERR OFFSET(5) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Data CRC Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Data End Bit Error.
        FORCE_DATA_END_BIT_ERR OFFSET(6) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Data End Bit Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Auto CMD Error.
        FORCE_AUTO_CMD_ERR OFFSET(8) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Auto CMD Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for ADMA Error.
        FORCE_ADMA_ERR OFFSET(9) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // ADMA Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Tuning Error (UHS-I Mode only)
        FORCE_TUNING_ERR OFFSET(10) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Tuning Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Response Error.
        FORCE_RESP_ERR OFFSET(11) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Response Error Status is set
            FORCE_SET = 0b1,
        ],
        // Force Event for Boot Ack error.
        FORCE_BOOT_ACK_ERR OFFSET(12) NUMBITS(1) [
            // Not Affected
            NOT_AFFECTED = 0b0,
            // Boot Ack Error Status is set
            FORCE_SET = 0b1,
        ],
    ],

    pub PRESET_INIT [
        // SDCLK/RCLK Frequency Select Value.
        // 10-bit preset value to be set in SDCLK/RCLK Frequency Select
        // field of the Clock Control register described by a Host System.
        FREQ_SEL_VAL OFFSET(0) NUMBITS(10) [],
        // Clock Generator Select Value.
        // This bit is effective when the Host Controller supports a
        // programmable clock generator.
        CLK_GEN_SEL_VAL OFFSET(10) NUMBITS(1) [
            // Host Controller Ver2.0 Compatible Clock Generator
            VER2_COMPATIBLE = 0,
            // Programmable Clock Generator
            PROGRAMMABLE = 1,
        ],
    ],

    pub PRESET_DS [
        // SDCLK/RCLK Frequency Select Value.
        // 10-bit preset value to be set in SDCLK/RCLK Frequency Select
        // field of the Clock Control register described by a Host System.
        FREQ_SEL_VAL OFFSET(0) NUMBITS(10) [],
        // Clock Generator Select Value.
        // This bit is effective when the Host Controller supports a
        // programmable clock generator.
        CLK_GEN_SEL_VAL OFFSET(10) NUMBITS(1) [
            // Host Controller Ver2.0 Compatible Clock Generator
            VER2_COMPATIBLE = 0,
            // Programmable Clock Generator
            PROGRAMMABLE = 1,
        ],
    ],

    pub PRESET_HS [
        // SDCLK/RCLK Frequency Select Value.
        // 10-bit preset value to be set in SDCLK/RCLK Frequency Select
        // field of the Clock Control register described by a Host System.
        FREQ_SEL_VAL OFFSET(0) NUMBITS(10) [],
        // Clock Generator Select Value.
        // This bit is effective when the Host Controller supports a
        // programmable clock generator.
        CLK_GEN_SEL_VAL OFFSET(10) NUMBITS(1) [
            // Host Controller Ver2.0 Compatible Clock Generator
            VER2_COMPATIBLE = 0,
            // Programmable Clock Generator
            PROGRAMMABLE = 1,
        ],
    ],

    pub PRESET_SDR12 [
        // SDCLK/RCLK Frequency Select Value.
        // 10-bit preset value to be set in SDCLK/RCLK Frequency Select
        // field of the Clock Control register described by a Host System.
        FREQ_SEL_VAL OFFSET(12) NUMBITS(1) [],
        // Clock Generator Select Value.
        // This bit is effective when the Host Controller supports a
        // programmable clock generator.
        CLK_GEN_SEL_VAL OFFSET(12) NUMBITS(1) [
            // Host Controller Ver2.0 Compatible Clock Generator
            VER2_COMPATIBLE = 0,
            // Programmable Clock Generator
            PROGRAMMABLE = 1,
        ],
    ],

    pub PRESET_SDR25 [
        // SDCLK/RCLK Frequency Select Value.
        // 10-bit preset value to be set in SDCLK/RCLK Frequency Select
        // field of the Clock Control register described by a Host System.
        FREQ_SEL_VAL OFFSET(12) NUMBITS(1) [],
        // Clock Generator Select Value.
        // This bit is effective when the Host Controller supports a
        // programmable clock generator.
        CLK_GEN_SEL_VAL OFFSET(12) NUMBITS(1) [
            // Host Controller Ver2.0 Compatible Clock Generator
            VER2_COMPATIBLE = 0,
            // Programmable Clock Generator
            PROGRAMMABLE = 1,
        ],
    ],

    pub PRESET_SDR50 [
        // SDCLK/RCLK Frequency Select Value.
        // 10-bit preset value to be set in SDCLK/RCLK Frequency Select
        // field of the Clock Control register described by a Host System.
        FREQ_SEL_VAL OFFSET(12) NUMBITS(1) [],
        // Clock Generator Select Value.
        // This bit is effective when the Host Controller supports a
        // programmable clock generator.
        CLK_GEN_SEL_VAL OFFSET(12) NUMBITS(1) [
            // Host Controller Ver2.0 Compatible Clock Generator
            VER2_COMPATIBLE = 0,
            // Programmable Clock Generator
            PROGRAMMABLE = 1,
        ],
    ],

    pub PRESET_SDR104 [
        // SDCLK/RCLK Frequency Select Value.
        // 10-bit preset value to be set in SDCLK/RCLK Frequency Select
        // field of the Clock Control register described by a Host System.
        FREQ_SEL_VAL OFFSET(12) NUMBITS(1) [],
        // Clock Generator Select Value.
        // This bit is effective when the Host Controller supports a
        // programmable clock generator.
        CLK_GEN_SEL_VAL OFFSET(12) NUMBITS(1) [
            // Host Controller Ver2.0 Compatible Clock Generator
            VER2_COMPATIBLE = 0,
            // Programmable Clock Generator
            PROGRAMMABLE = 1,
        ],
    ],

    pub PRESET_DDR50 [
        // SDCLK/RCLK Frequency Select Value.
        // 10-bit preset value to be set in SDCLK/RCLK Frequency Select
        // field of the Clock Control register described by a Host System.
        FREQ_SEL_VAL OFFSET(12) NUMBITS(1) [],
        // Clock Generator Select Value.
        // This bit is effective when the Host Controller supports a
        // programmable clock generator.
        CLK_GEN_SEL_VAL OFFSET(12) NUMBITS(1) [
            // Host Controller Ver2.0 Compatible Clock Generator
            VER2_COMPATIBLE = 0,
            // Programmable Clock Generator
            PROGRAMMABLE = 1,
        ],
    ],

    pub SLOT_INTR_STATUS [
        // Interrupt signal for each Slot.
        // Host Controller support single card slot. This register shall always return 0.
        INTR_SLOT OFFSET(0) NUMBITS(8) [],
    ],

    pub HOST_CNTRL_VERS [
        // Specification Version Number.
        // Values:
        // 8'h0: SD Host Controller Specification Version 1.00
        // 8'h1: SD Host Controller Specification Version 2.00
        // 8'h2: SD Host Controller Specification Version 3.00
        // 8'h3: SD Host Controller Specification Version 4.00
        // 8'h4: SD Host Controller Specification Version 4.10
        // 8'h5: SD Host Controller Specification Version 4.20
        SPEC_VERSION_NUM OFFSET(0) NUMBITS(8) [],
        // Vendor Version Number.
        VENDOR_VERSION_NUM OFFSET(8) NUMBITS(8) []
    ],

    pub EMMC_CTRL [
        // eMMC Card present.
        // This bit indicates the type of card connected. An application
        // program this bit based on the card connected to Host Controller.
        CARD_IS_EMMC OFFSET(0) NUMBITS(1) [
            // Card connected to Host Controller is a non-eMMC card
            NON_EMMC_CARD = 0b0,
            // Card connected to Host Controller is an eMMC card
            EMMC_CARD = 0b1,
        ],
        // Disable Data CRC Check.
        // This bit controls masking of CRC16 error for Card Write in eMMC
        // mode. This is useful in bus testing (CMD19) for an eMMC device.
        // In bus testing, an eMMC card does not send CRC status for a
        // block, which may generate CRC error. This
        // CRC error can be masked using this bit during bus testing.
        DISABLE_DATA_CRC_CHK OFFSET(1) NUMBITS(1) [
            // DATA CRC check is enabled
            CRC_CHECK_ENABLED = 0b0,
            // DATA CRC check is disabled
            CRC_CHECK_DISABLED = 0b1,
        ],
        // EMMC Device Reset signal control.
        // This register field controls the sd_rst_n output of Host Controller.
        EMMC_RST_N OFFSET(2) NUMBITS(1) [
            // Reset to eMMC device asserted
            RESET_ASSERTED = 0b0,
            // Reset to eMMC device is deasserted
            RESET_DEASSERTED = 0b1,
        ],
        // Output Enable control for EMMC Device Reset signal PAD control.
        EMMC_RST_N_OE OFFSET(3) NUMBITS(1) [
            // sd_rst_n_oe is 0
            OUTPUT_DISABLED = 0b0,
            // sd_rst_n_oe is 1
            OUTPUT_ENABLED = 0b1,
        ],
        // Enhanced Strobe Enable.
        // This bit instructs Host to sample the CMD line using data strobe
        // for HS400 mode.
        ENH_STROBE_ENABLE OFFSET(8) NUMBITS(1) [
            // CMD line is sampled using cclk_rx for HS400 mode
            CLOCK_SAMPLING = 0b0,
            // CMD line is sampled using data strobe for HS400 mode
            STROBE_SAMPLING = 0b1,
        ],
        // Scheduler algorithm selected for execution.
        // This bit selects the Algorithm used for selecting one of the many
        // ready tasks for execution.
        CQE_ALGO_SEL OFFSET(9) NUMBITS(1) [
            // Priority based reordering with FCFS to resolve equal priority tasks
            PRIORITY_BASED = 0b0,
            // First come First serve, in the order of DBR rings
            FIRST_COME_FIRST_SERVE = 0b1,
        ],
        // Enable or Disable CQE's PREFETCH feature.
        // This field allows Software to disable CQE's data prefetch feature when set to 1.
        CQE_PREFETCH_DISABLE OFFSET(10) NUMBITS(1) [
            // CQE can Prefetch data for successive WRITE transfers and pipeline successive READ transfers
            PREFETCH_ENABLED = 0b0,
            // Prefetch for WRITE and Pipeline for READ are disabled
            PREFETCH_DISABLED = 0b1,
        ],
    ],

    pub BOOT_CTRL [
        // Mandatory Boot Enable.
        // This bit is used to initiate the mandatory boot operation. The
        // application sets this bit along with VALIDATE_BOOT bit.
        // Writing 0 is ignored. The Host Controller clears this bit after the
        // boot transfer is completed or terminated.
        MAN_BOOT_EN OFFSET(0) NUMBITS(1) [
            // Mandatory boot disable
            DISABLED = 0b0,
            // Mandatory boot enable
            ENABLED = 0b1,
        ],
        // Validate Mandatory Boot Enable bit.
        // This bit is used to validate the MAN_BOOT_EN bit.
        VALIDATE_BOOT OFFSET(7) NUMBITS(1) [
            // Ignore Mandatory boot Enable bit
            IGNORE = 0b0,
            // Validate Mandatory boot enable bit
            VALIDATE = 0b1,
        ],
        // Boot Acknowledge Enable.
        // When this bit set, Host checks for boot acknowledge start pattern
        // of 0-1-0 during boot operation. This bit is applicable for both
        // mandatory and alternate boot mode.
        BOOT_ACK_ENABLE OFFSET(8) NUMBITS(1) [
            // Boot Ack disable
            DISABLED = 0b0,
            // Boot Ack enable
            ENABLED = 0b1,
        ],
        // Boot Ack Timeout Counter Value.
        // This value determines the interval by which boot ack timeout (50
        // ms) is detected when boot ack is expected during boot operation.
        // Values:
        // 4'h0: TMCLK x 2^13
        // 4'h1: TMCLK x 2^14
        // ......
        // 4'hE: TMCLK x 2^27
        // 4'hF: Reserved
        BOOT_TOUT_CNT OFFSET(12) NUMBITS(4) [],
    ],
];

register_bitfields![
    u32,

    pub SDMASA [
        // 32-bit Block Count (SDMA System Address).
        // 1. SDMA System Address (Host Version 4 Enable = 0): This register contains the system memory address for an SDMA transfer in the 32-bit addressing mode. When the Host Controller stops an SDMA transfer, this register points to the system address of the next contiguous data position. It can be accessed only if no transaction is executing. Reading this register during data transfers may return an invalid value.
        // 2. 32-bit Block Count (Host Version 4 Enable = 1): From the Host Controller Version 4.10 specification, this register is redefined as 32-bit Block Count. The Host Controller decrements the block count of this register for every block transfer and the data transfer stops when the count reaches zero. This register must be accessed when no transaction is executing. Reading this register during data transfers may return invalid value.
        // Note:
        // a. For Host Version 4 Enable = 0, the Host driver does not program the system address in this register while operating in ADMA mode. The system address must be programmed in the ADMA System Address register.
        // b. For Host Version 4 Enable = 0, the Host driver programs a non-zero 32-bit block count value in this register when Auto CMD23 is enabled for non-DMA and ADMA modes. Auto CMD23 cannot be used with SDMA.
        // c. This register must be programmed with a non-zero value for data transfer if the 32-bit Block count register is used instead of the 16-bit Block count register.
        BLOCKCNT_SDMASA OFFSET(0) NUMBITS(32) []
    ],

    pub ARGUMENT [
        // Command Argument.
        // These bits specify the SD/eMMC command argument that is specified in bits 39-8 of the Command format.
        ARGUMENT OFFSET(0) NUMBITS(32) []
    ],

    pub RESP01 [
        // Command Response.
        // These bits reflect 39-8 bits of SD/eMMC Response Field.
        // Note: For Auto CMD, the 32-bit response (bits 39-8 of the Response Field) is updated in the RESP67 register.
        RESP01 OFFSET(0) NUMBITS(32) [],
    ],

    pub RESP23 [
        // Command Response.
        // These bits reflect 71-40 bits of the SD/eMMC Response Field.
        RESP23 OFFSET(0) NUMBITS(32) [],
    ],

    pub RESP45 [
        // Command Response.
        // These bits reflect 103-72 bits of the Response Field.
        RESP45 OFFSET(0) NUMBITS(32) [],
    ],

    pub RESP67 [
        // Command Response
        // These bits reflect bits 135-104 of SD/EMMC Response Field.
        // For Auto CMD, this register also reflects the 32-bit response (bits 39-8 of the Response Field).
        RESP67 OFFSET(0) NUMBITS(32) [],
    ],

    pub BUF_DATA [
        // Buffer Data.
        // These bits enable access to the Host Controller packet buffer.
        BUF_DATA OFFSET(0) NUMBITS(32) [],
    ],

    pub PSTATE [
        // Command Inhibit (CMD).
        // If this bit is set to 0, it indicates that the CMD line is not in use
        // and the Host controller can issue an SD/eMMC command using
        // the CMD line. This bit is
        // set when the command register is written. This bit is cleared
        // when the command response is received. This bit is not cleared
        // by the response of auto CMD12/23 but cleared by the response of
        // read/write command.
        CMD_INHIBIT OFFSET(0) NUMBITS(1) [
            // Host Controller is ready to issue a command
            READY = 0b0,
            // Host Controller is not ready to issue a command
            NOT_READY = 0b1,
        ],
        // Command Inhibit (DAT).
        // This bit is applicable for SD/eMMC mode and is generated if either
        // DAT line active or Read transfer active is set to 1. If this bit is set
        // to 0, it indicates that the Host Controller can issue subsequent
        // SD/eMMC commands. For the UHS-II
        // mode, this bit is irrelevant and always returns 0.
        CMD_INHIBIT_DAT OFFSET(1) NUMBITS(1) [
            // Can issue command which used DAT line
            CAN_ISSUE = 0b0,
            // Cannot issue command which used DAT line
            CANNOT_ISSUE = 0b1,
        ],
        // DAT Line Active (SD/eMMC Mode only).
        // This bit indicates whether one of the DAT lines on the SD/eMMC
        // bus is in use.
        // In the case of read transactions, this bit indicates whether a read
        // transfer is executing on the SD/eMMC bus.
        // In the case of write transactions, this bit indicates whether a write
        // transfer is executing on the SD/eMMC bus.
        // For a command with busy, this status indicates whether the
        // command executing busy is executing on an SD/eMMC bus.
        DAT_LINE_ACTIVE OFFSET(2) NUMBITS(1) [
            // DAT Line Inactive
            INACTIVE = 0b0,
            // DAT Line Active
            ACTIVE = 0b1,
        ],
        // Re-Tuning Request.
        // Host Controller does not generate retuning request. The software
        // must maintain the Retuning timer.
        RE_TUNE_REQ OFFSET(3) NUMBITS(1) [],
        // DAT[7:4] Line Signal Level.
        // This bit is used to check the DAT line level to recover from errors
        // and for debugging. These bits reflect the value of the sd_dat_in
        // (upper nibble) signal.
        DAT_7_4 OFFSET(4) NUMBITS(4) [],
        // Write Transfer Active.
        // This status indicates whether a write transfer is active for
        // SD/eMMC mode.
        WR_XFER_ACTIVE OFFSET(8) NUMBITS(1) [
            // No valid data
            NO_DATA = 0b0,
            // Transferring data
            TRANSFERRING = 0b1,
        ],
        // Read Transfer Active.
        // This bit indicates whether a read transfer is active for SD/eMMC mode.
        RD_XFER_ACTIVE OFFSET(9) NUMBITS(1) [
            // No valid data
            NO_DATA = 0b0,
            // Transferring data
            TRANSFERRING = 0b1,
        ],
        // Buffer Write Enable.
        // This bit is used for non-DMA transfers. This bit is set if space is
        // available for writing data.
        BUF_WR_ENABLE OFFSET(10) NUMBITS(1) [
            // Write disable
            DISABLED = 0b0,
            // Write enable
            ENABLED = 0b1,
        ],
        // Buffer Read Enable.
        // This bit is used for non-DMA transfers. This bit is set if valid data
        // exists in the Host buffer.
        BUF_RD_ENABLE OFFSET(11) NUMBITS(1) [
            // Read disable
            DISABLED = 0b0,
            // Read enable
            ENABLED = 0b1,
        ],
        // Card Inserted.
        // This bit indicates whether a card has been inserted. The Host
        // Controller debounces this signal so that Host Driver need not wait
        // for it to stabilize.
        CARD_INSERTED OFFSET(16) NUMBITS(1) [
            // Reset, Debouncing, or No card
            NO_CARD = 0b0,
            // Card Inserted
            INSERTED = 0b1,
        ],
        // Card Stable.
        // This bit indicates the stability of the Card Detect Pin Level. A card
        // is not detected if this bit is set to 1 and the value of the
        // CARD_INSERTED bit is 0.
        CARD_STABLE OFFSET(17) NUMBITS(1) [
            // Reset or Debouncing
            UNSTABLE = 0b0,
            // No Card or Inserted
            STABLE = 0b1,
        ],
        // Card Detect Pin Level.
        // This bit reflects the inverse synchronized value of the
        // card_detect_n signal.
        CARD_DETECT_PIN_LEVEL OFFSET(18) NUMBITS(1) [
            // No card present
            NO_CARD = 0b0,
            // Card Present
            CARD_PRESENT = 0b1,
        ],
        // Write Protect Switch Pin Level.
        // This bit is supported only for memory and combo cards. This bit
        // reflects the synchronized value of the card_write_prot signal.
        WR_PROTECT_SW_LVL OFFSET(19) NUMBITS(1) [
            // Write protected
            PROTECTED = 0b0,
            // Write enabled
            ENABLED = 0b1,
        ],
        // DAT[3:0] Line Signal Level.
        // This bit is used to check the DAT line level to recover from errors
        // and for debugging. These bits reflect the value of the sd_dat_in
        // (lower nibble) signal.
        DAT_3_0 OFFSET(20) NUMBITS(4) [],
        // Command-Line Signal Level.
        // This bit is used to check the CMD line level to recover from errors
        // and for debugging. These bits reflect the value of the sd_cmd_in signal.
        CMD_LINE_LVL OFFSET(24) NUMBITS(1) [],
        // Command Not Issued by Error.
        // This bit is set if a command cannot be issued after setting the
        // command register due to an error except the Auto CMD12 error.
        CMD_ISSUE_ERR OFFSET(27) NUMBITS(1) [
            // No error for issuing a command
            NO_ERROR = 0b0,
            // Command cannot be issued
            ERROR = 0b1,
        ],
        // Sub Command Status.
        // This bit is used to distinguish between a main command and a
        // sub command status.
        SUB_CMD_STAT OFFSET(28) NUMBITS(1) [
            // Main Command Status
            MAIN_COMMAND = 0b0,
            // Sub Command Status
            SUB_COMMAND = 0b1,
        ],
    ],

    pub CAPABILITIES1 [
        // Timeout Clock Frequency.
        // This bit shows the base clock frequency used to detect Data
        // Timeout Error. The Timeout Clock unit defines the unit of timeout
        // clock frequency. It can be KHz or MHz.
        TOUT_CLK_FREQ OFFSET(0) NUMBITS(6) [],
        // Timeout Clock Unit.
        // This bit shows the unit of base clock frequency used to detect
        // Data Timeout Error.
        TOUT_CLK_UNIT OFFSET(7) NUMBITS(1) [
            // KHz
            KHZ = 0b0,
            // MHz
            MHZ = 0b1,
        ],
        // Base Clock Frequency for SD clock.
        // These bits indicate the base (maximum) clock frequency for the
        // SD Clock. The definition of these bits depend on the Host
        // Controller Version.
        BASE_CLK_FREQ OFFSET(8) NUMBITS(8) [],
        // Maximum Block Length.
        // This bit indicates the maximum block size that the Host driver
        // can read and write to the buffer in the Host Controller. The buffer
        // transfers this block size without wait cycles. The transfer block
        // length is always 512 bytes for the SD Memory irrespective of this bit.
        MAX_BLK_LEN OFFSET(16) NUMBITS(2) [
            // 512 Byte
            BYTES_512 = 0b00,
            // 1024 Byte
            BYTES_1024 = 0b01,
            // 2048 Byte
            BYTES_2048 = 0b10,
            // Reserved
            RESERVED = 0b11,
        ],
        // 8-bit Support for Embedded Device.
        // This bit indicates whether the Host Controller is capable of using
        // an 8-bit bus width mode.
        Embedded_8_BIT OFFSET(18) NUMBITS(1) [
            // 8-bit Bus Width not Supported
            NOT_SUPPORTED = 0b0,
            // 8-bit Bus Width Supported
            SUPPORTED = 0b1,
        ],
        // ADMA2 Support.
        // This bit indicates whether the Host Controller is capable of using ADMA2.
        ADMA2_SUPPORT OFFSET(19) NUMBITS(1) [
            // ADMA2 not Supported
            NOT_SUPPORTED = 0b0,
            // ADMA2 Supported
            SUPPORTED = 0b1,
        ],
        // High Speed Support.
        // This bit indicates whether the Host Controller and the Host
        // System supports High Speed mode and they can supply the SD
        // Clock frequency from 25 MHz to 50 MHz.
        HIGH_SPEED_SUPPORT OFFSET(21) NUMBITS(1) [
            // High Speed not Supported
            NOT_SUPPORTED = 0b0,
            // High Speed Supported
            SUPPORTED = 0b1,
        ],
        // SDMA Support.
        // This bit indicates whether the Host Controller is capable of using
        // SDMA to transfer data between the system memory and the Host
        // Controller directly.
        SDMA_SUPPORT OFFSET(22) NUMBITS(1) [
            // SDMA not Supported
            NOT_SUPPORTED = 0b0,
            // SDMA Supported
            SUPPORTED = 0b1,
        ],
        // Suspense/Resume Support.
        // This bit indicates whether the Host Controller supports
        // Suspend/Resume functionality. If this bit is 0, the Host Driver
        // does not issue either Suspend or Resume commands because the
        // Suspend and Resume mechanism is not supported.
        SUS_RES_SUPPORT OFFSET(23) NUMBITS(1) [
            // Not Supported
            NOT_SUPPORTED = 0b0,
            // Supported
            SUPPORTED = 0b1,
        ],
        // 64-bit System Address Support for V4.
        // This bit sets the Host Controller to support 64-bit System
        // Addressing of V4 mode.
        SYS_ADDR_64_V4 OFFSET(27) NUMBITS(1) [
            // 64-bit System Address for V4 is Not Supported
            NOT_SUPPORTED = 0b0,
            // 64-bit System Address for V4 is Supported
            SUPPORTED = 0b1,
        ],
        // 64-bit System Address Support for V3.
        // This bit sets the Host controller to support 64-bit System
        // Addressing of V3 mode.
        SYS_ADDR_64_V3 OFFSET(28) NUMBITS(1) [
            // 64-bit System Address for V3 is Not Supported
            NOT_SUPPORTED = 0b0,
            // 64-bit System Address for V3 is Supported
            SUPPORTED = 0b1,
        ],
        // Asynchronous Interrupt Support (SD Mode only).
        ASYNC_INT_SUPPORT OFFSET(29) NUMBITS(1) [
            // Asynchronous Interrupt Not Supported
            NOT_SUPPORTED = 0b0,
            // Asynchronous Interrupt Supported
            SUPPORTED = 0b1,
        ],
        // Slot Type.
        // These bits indicate usage of a slot by a specific Host System.
        SLOT_TYPE OFFSET(30) NUMBITS(2) [
            // Removable Card Slot
            REMOVABLE = 0b00,
            // Embedded Slot for One Device
            EMBEDDED = 0b01,
            // Shared Bus Slot (SD mode)
            SHARED_BUS = 0b10,
            // Reserved
            RESERVED = 0b11,
        ],
    ],

    pub CAPABILITIES2 [
        // SDR50 Support.
        SDR50_SUPPORT OFFSET(0) NUMBITS(1) [
            // SDR50 is Not Supported
            NOT_SUPPORTED = 0b0,
            // SDR50 is Supported
            SUPPORTED = 0b1,
        ],
        // SDR104 Support.
        SDR104_SUPPORT OFFSET(1) NUMBITS(1) [
            // SDR104 is Not Supported
            NOT_SUPPORTED = 0b0,
            // SDR104 is Supported
            SUPPORTED = 0b1,
        ],
        // DDR50 Support.
        DDR50_SUPPORT OFFSET(2) NUMBITS(1) [
            // DDR50 is Not Supported
            NOT_SUPPORTED = 0b0,
            // DDR50 is Supported
            SUPPORTED = 0b1,
        ],
        // UHS-II Support.
        UHS2_SUPPORT OFFSET(3) NUMBITS(1) [
            // UHS2 is Not Supported
            NOT_SUPPORTED = 0b0,
            // UHS2 is Supported
            SUPPORTED = 0b1,
        ],
        // This bit indicates support of Driver Type A for 1.8 Signaling.
        DRV_TYPEA OFFSET(4) NUMBITS(1) [
            // Driver Type A is Not Supported
            NOT_SUPPORTED = 0b0,
            // Driver Type A is Supported
            SUPPORTED = 0b1,
        ],
        // This bit indicates support of Driver Type C for 1.8 Signaling.
        DRV_TYPEC OFFSET(5) NUMBITS(1) [
            // Driver Type C is Not Supported
            NOT_SUPPORTED = 0b0,
            // Driver Type C is Supported
            SUPPORTED = 0b1,
        ],
        // This bit indicates support of Driver Type D for 1.8 Signaling.
        DRV_TYPED OFFSET(6) NUMBITS(1) [
            // Driver Type D is Not Supported
            NOT_SUPPORTED = 0b0,
            // Driver Type D is Supported
            SUPPORTED = 0b1,
        ],
        // Timer Count for Re-Tuning.
        RETUNE_CNT OFFSET(8) NUMBITS(4) [
            // Re-Tuning Timer disabled
            DISABLED = 0b0000,
            // 1 seconds
            SEC_1 = 0b0001,
            // 2 seconds
            SEC_2 = 0b0010,
            // 4 seconds
            SEC_4 = 0b0011,
            // 8 seconds
            SEC_8 = 0b0100,
            // 16 seconds
            SEC_16 = 0b0101,
            // 32 seconds
            SEC_32 = 0b0110,
            // 64 seconds
            SEC_64 = 0b0111,
            // 128 seconds
            SEC_128 = 0b1000,
            // 256 seconds
            SEC_256 = 0b1001,
            // 512 seconds
            SEC_512 = 0b1010,
            // 1024 seconds
            SEC_1024 = 0b1011,
            // Get information from other source
            OTHER_SOURCE = 0b1111,
        ],
        // Use Tuning for SDR50.
        USE_TUNING_SDR50 OFFSET(13) NUMBITS(1) [
            // SDR50 does not require tuning
            NOT_REQUIRED = 0b0,
            // SDR50 requires tuning
            REQUIRED = 0b1,
        ],
        // Re-Tuning Modes.
        // These bits select the re-tuning method and limit the maximum
        // data length.
        RE_TUNING_MODES OFFSET(14) NUMBITS(2) [
            // Timer
            TIMER = 0b00,
            // Timer and Re-Tuning Request (Not supported)
            TIMER_AND_REQUEST = 0b01,
            // Auto Re-Tuning (for transfer)
            AUTO_RETUNING = 0b10,
            // Reserved
            RESERVED = 0b11,
        ],
        // Clock Multiplier.
        // These bits indicate the clock multiplier of the programmable clock
        // generator. Setting these bits to 0 means that the Host Controller
        // does not support a programmable clock generator.
        CLK_MUL OFFSET(16) NUMBITS(8) [],
        // ADMA3 Support.
        // This bit indicates whether the Host Controller is capable of using ADMA3.
        ADMA3_SUPPORT OFFSET(25) NUMBITS(1) [
            // ADMA3 not Supported
            NOT_SUPPORTED = 0b0,
            // ADMA3 Supported
            SUPPORTED = 0b1,
        ],
    ],

    pub ADMA_SA [
        // ADMA System Address.
        // These bits indicate the 32 bits of the ADMA system address.
        // a. SDMA: If Host Version 4 Enable is set to 1, this register stores the system address of the data location
        // b. ADMA2: This register stores the byte address of the executing command of the descriptor table
        // c. ADMA3: This register is set by ADMA3. ADMA2 increments the address of this register that points to the next line, every time a Descriptor line is fetched.
        ADMA_SA OFFSET(0) NUMBITS(32) [],
    ],

    pub ADMA_ID [
        // ADMA Integrated Descriptor Address.
        // These bits indicate the 32-bit of the ADMA Integrated Descriptor address. The start address of Integrated Descriptor is set to these register bits. The ADMA3 fetches one Descriptor Address and increments these bits to indicate the next Descriptor address.
        ADMA_ID OFFSET(0) NUMBITS(32) [],
    ],

    pub CQVER [
        // eMMC Version Suffix (2nd digit right of decimal point), in BCD format
        EMMC_VER_SUFFIX OFFSET(0) NUMBITS(4) [],
        // eMMC Minor Version Number(digit right of decimal point), in BCD format
        EMMC_VER_MINOR OFFSET(4) NUMBITS(4) [],
        // eMMC Major Version Number (digit left of decimal point), in BCD format
        EMMC_VER_MAJOR OFFSET(8) NUMBITS(4) [],
    ],

    pub CQCAP [
        // Internal Timer Clock Frequency Value.
        // TCFMUL and ITCFVAL indicate the frequency of the clock used for
        // interrupt coalescing timer and for determining the polling period
        // when using periodic SEND_QUEUE_ STATUS (CMD13) polling.
        // The clock frequency is calculated as ITCFVAL* ITCFMUL.
        // For example, to encode 19.2 MHz, ITCFVAL shall be C0h (= 192
        // decimal) and ITCFMUL shall be 2h (0.1 MHz)
        // 192 * 0.1 MHz=19.2 MHz
        ITCFVAL OFFSET(0) NUMBITS(10) [],
        // Internal Timer Clock Frequency Multiplier.
        // ITCFMUL and ITCFVAL indicate the frequency of the clock used for
        // interrupt coalescing timer and for determining the SQS polling
        // period. See ITCFVAL definition for details.
        ITCFMUL OFFSET(12) NUMBITS(4) [
            // 0.001 MHz
            MHZ_0_001 = 0b0000,
            // 0.01 MHz
            MHZ_0_01 = 0b0001,
            // 0.1 MHz
            MHZ_0_1 = 0b0010,
            // 1 MHz
            MHZ_1 = 0b0011,
            // 10 MHz
            MHZ_10 = 0b0100,
            // Others: Reserved
        ],
        // Crypto Support.
        // This bit indicates whether the Host Controller supports
        // cryptographic operations.
        CRYPTO_SUPPORT OFFSET(28) NUMBITS(1) [
            // Crypto not Supported
            NOT_SUPPORTED = 0b0,
            // Crypto Supported
            SUPPORTED = 0b1,
        ],
    ],

    pub CQCFG [
        // Command Queueing Enable.
        // When CQE is disable, the software controls the eMMC bus using
        // the registers between the addresses 0x000 to 0x1FF.
        // Before the software writes to this bit, the software verifies that
        // the eMMC host controller is in idle state and there are no ongoing
        // commands or data transfers. When software wants to exit
        // command queuing mode, it clears all previous tasks (if any)
        // before setting this bit to 0.
        CQ_EN OFFSET(0) NUMBITS(1) [
            // Disable command queuing
            DISABLED = 0b0,
            // Enable command queuing
            ENABLED = 0b1,
        ],
        // Crypto General Enable.
        // Enable/Disable bit for Crypto Engine. If cryptographic operations
        // are not supported, this status bit is reserved.
        CR_GENERAL_EN OFFSET(1) NUMBITS(1) [
            // Disable cryptographic operations for all transactions
            DISABLED = 0b0,
            // Enable cryptographic operations for transactions where TD.CE=1 or CRNQP.CE=1
            ENABLED = 0b1,
        ],
        // This bit indicates whether the task descriptor size is 128 bits or
        // 64 bits as detailed in Data Structures section. This bit can only be
        // configured when Command Queueing Enable bit is 0 (command
        // queueing is disabled)
        TASK_DESC_SIZE OFFSET(8) NUMBITS(1) [
            // Task descriptor size is 64 bits
            SIZE_64BIT = 0b0,
            // Task descriptor size is 128 bits
            SIZE_128BIT = 0b1,
        ],
        // Direct Command (DCMD) Enable.
        // This bit indicates to the hardware whether the Task Descriptor in
        // slot #31 of the TDL is a Data Transfer Task Descriptor, or a Direct
        // Command Task Descriptor.
        // CQE uses this bit when a task is issued in slot #31, to determine
        // how to decode the Task Descriptor.
        DCMD_EN OFFSET(12) NUMBITS(1) [
            // Task descriptor in slot #31 is a Data Transfer Task Descriptor
            DATA_TRANSFER = 0b0,
            // Task descriptor in slot #31 is a DCMD Task Descriptor
            DIRECT_COMMAND = 0b1,
        ],
    ],

    pub CQCTRL [
        // Halt request and resume.
        HALT OFFSET(0) NUMBITS(1) [
            // 1'b0: RESUME_CQE. Software writes 0 to this bit to exit from the halt state and resume CQE activity
            Resume = 0,
            // 1'b1: HALT_CQE. Software writes 1 to this bit when it wants to acquire software control over the
            // eMMC bus and to disable CQE from issuing command on the bus. For example, issuing a Discard Task
            // command (CMDQ_TASK_MGMT). When the software writes 1, CQE completes the ongoing task (if any in
            // progress). After the task is completed and the CQE is in idle state, CQE does not issue new commands
            // and indicates to the software by setting this bit to 1. The software can poll on this bit until it
            // is set to 1 and only then send commands on the eMMC bus.
            Halt = 1
        ],
        // Clear all tasks.
        // This bit can only be written when the controller is halted. This bit does not clear tasks in the device. The software has to use the CMDQ_TASK_MGMT command to clear device's queue.
        CLR_ALL_TASKS OFFSET(8) NUMBITS(1) [
            // 1'b0: Programming 0 has no effect
            NoAction = 0,
            // 1'b1: Clears all the tasks in the controller
            ClearControllerTasks = 1
        ],
    ],

    pub CQIS [
        // Halt Complete Interrupt.
        // This status bit is asserted (if CQISTE.HAC=1) when halt bit in
        // CQCTL register transitions from 0 to 1 indicating that host
        // controller has completed its current ongoing task and has entered halt state.
        HAC OFFSET(0) NUMBITS(1) [
            // 1'b0: HAC Interrupt is not set
            NotSet = 0,
            // 1'b1: HAC Interrupt is set
            Set = 1
        ],
        // Task Complete Interrupt.
        // This status bit is asserted(if CQISTE.TCC=1) when at least one of the following two conditions are met:
        // a. A task is completed and the INT bit is set in its Task Descriptor
        // b. Interrupt caused by Interrupt Coalescing logic
        TCC OFFSET(1) NUMBITS(1) [
            // 1'b0: TCC Interrupt is not set
            NotSet = 0,
            // 1'b1: TCC Interrupt is set
            Set = 1
        ],
        // Response Error Detected Interrupt.
        // This status bit is asserted (if CQISTE.RED=1) when a response is
        // received with an error bit set in the device status field.
        // Software uses CQRMEM register to configure which device status
        // bit fields may trigger an interrupt, and which are masked.
        RED OFFSET(2) NUMBITS(1) [
            // 1'b0: RED Interrupt is not set
            NotSet = 0,
            // 1'b1: RED Interrupt is set
            Set = 1
        ],
        // Task Cleared Interrupt.
        // This status bit is asserted (if CQISTE.TCL=1) when a task clear
        // operation is completed by CQE. The completed task clear
        // operation is either an individual task clear (CQTCLR) or clearing of all tasks (CQCTL).
        TCL OFFSET(3) NUMBITS(1) [
            // 1'b0: TCL Interrupt is not set
            NotSet = 0,
            // 1'b1: TCL Interrupt is set
            Set = 1
        ],
    ],

    pub CQISE [
        // Halt Complete Interrupt status enable.
        HAC_STE OFFSET(0) NUMBITS(1) [
            // 1'b0: CQIS.HAC is disabled
            Disabled = 0,
            // 1'b1: CQIS.HAC is set when its interrupt condition is active
            Enabled = 1
        ],

        // Task Complete Interrupt status enable.
        TCC_STE OFFSET(1) NUMBITS(1) [
            // 1'b0: CQIS.TCC is disabled
            Disabled = 0,
            // 1'b1: CQIS.TCC is set when its interrupt condition is active
            Enabled = 1
        ],

        // Response Error Detected Interrupt status enable.
        RED_STE OFFSET(2) NUMBITS(1) [
            // 1'b0: CQIS.RED is disabled
            Disabled = 0,
            // 1'b1: CQIS.RED is set when its interrupt condition is active
            Enabled = 1
        ],

        // Task Cleared Interrupt status enable.
        TCL_STE OFFSET(3) NUMBITS(1) [
            // 1'b0: CQIS.TCL is disabled
            Disabled = 0,
            // 1'b1: CQIS.TCL is set when its interrupt condition is active
            Enabled = 1
        ],
    ],

    pub CQISGE [
        // Halt Complete Interrupt signal enable.
        HAC_SGE OFFSET(0) NUMBITS(1) [
            // 1'b0: CQIS.HAC interrupt signal generation is disabled
            Disabled = 0,
            // 1'b1: CQIS.HAC interrupt signal generation is active
            Enabled = 1
        ],

        // Task Complete Interrupt signal enable.
        TCC_SGE OFFSET(1) NUMBITS(1) [
            // 1'b0: CQIS.TCC interrupt signal generation is disabled
            Disabled = 0,
            // 1'b1: CQIS.TCC interrupt signal generation is active
            Enabled = 1
        ],

        // Response Error Detected Interrupt signal enable.
        RED_SGE OFFSET(2) NUMBITS(1) [
            // 1'b0: CQIS.RED interrupt signal generation is disabled
            Disabled = 0,
            // 1'b1: CQIS.RED interrupt signal generation is active
            Enabled = 1
        ],

        // Task Cleared Interrupt signal enable.
        TCL_SGE OFFSET(3) NUMBITS(1) [
            // 1'b0: CQIS.TCL interrupt signal generation is disabled
            Disabled = 0,
            // 1'b1: CQIS.TCL interrupt signal generation is active
            Enabled = 1
        ],
    ],

    pub CQIC [
        // Interrupt Coalescing Timeout Value.
        // Software uses this field to configure the maximum time allowed between the completion of a task
        // on the bus and the generation of an interrupt.
        // Timer Operation: The timer is reset by software during the interrupt service routine. It starts
        // running when a data transfer task with INT=0 is completed, after the timer was reset. When the
        // timer reaches the value configured in ICTOVAL field it generates an interrupt and stops.
        // The timer's unit is equal to 1024 clock periods of the clock whose frequency is specified in
        // the Internal Timer Clock Frequency field CQCAP register.
        // 7'h0: Timer is disabled. Timeout-based interrupt is not generated
        // 7'h1: Timeout on 01x1024 cycles of timer clock frequency
        // 7'h2: Timeout on 02x1024 cycles of timer clock frequency
        // ........
        // 7'h7f: Timeout on 127x1024 cycles of timer clock frequency
        // In order to write to this field, the TOUT_VAL_WEN bit must be set at the same write operation.
        TOUT_VAL OFFSET(0) NUMBITS(7) [],

        // When software writes 1 to this bit, the value TOUT_VAL is updated with the contents written on the same cycle.
        TOUT_VAL_WEN OFFSET(7) NUMBITS(1) [
            // 1'b0: clears TOUT_VAL_WEN
            Disabled = 0,
            // 1'b1: Sets TOUT_VAL_WEN
            Enabled = 1
        ],

        // Interrupt Coalescing Counter Threshold filed.
        // Software uses this field to configure the number of task completions (only tasks with INT=0 in
        // the Task Descriptor), which are required in order to generate an interrupt.
        // Counter Operation: As data transfer tasks with INT=0 complete, they are counted by CQE. The counter
        // is reset by software during the interrupt service routine. The counter stops counting when it reaches
        // the value configured in INTC_TH, and generates interrupt.
        // 5'h0: Interrupt coalescing feature disabled
        // 5'h1: Interrupt coalescing interrupt generated after 1 task when INT=0 completes
        // 5'h2: Interrupt coalescing interrupt generated after 2 tasks when INT=0 completes
        // ........
        // 5'h1f: Interrupt coalescing interrupt generated after 31 tasks when INT=0 completes
        // To write to this field, the INTC_TH_WEN bit must be set during the same write operation.
        INTC_TH OFFSET(8) NUMBITS(5) [],

        // Interrupt Coalescing Counter Threshold Write Enable.
        // When software writes 1 to this bit, the value INTC_TH is updated with the contents written on the same cycle.
        INTC_TH_WEN OFFSET(15) NUMBITS(1) [
            // 1'b0: Clears INTC_TH_WEN
            Disabled = 0,
            // 1'b1: Sets INTC_TH_WEN
            Enabled = 1
        ],

        // Counter and Timer Reset.
        // When host driver writes 1, the interrupt coalescing timer and counter are reset.
        INTC_RST OFFSET(16) NUMBITS(1) [
            // 1'b0: No Effect
            NoEffect = 0,
            // 1'b1: Interrupt coalescing timer and counter are reset
            ResetCounterAndTimer = 1
        ],

        // Interrupt Coalescing Status Bit.
        // This bit indicates to the software whether any tasks (with INT=0) have completed and counted
        // towards interrupt coalescing (that is, this is set if and only if INTC counter > 0).
        INTC_STAT OFFSET(20) NUMBITS(1) [
            // 1'b0: INT0 Task completions have not occurred since last counter reset (INTC counter == 0)
            NoTasksCompleted = 0,
            // 1'b1: At least one INT0 task completion has been counted (INTC counter > 0)
            TasksCompleted = 1
        ],

        // Interrupt Coalescing Enable Bit.
        INTC_EN OFFSET(31) NUMBITS(1) [
            // 1'b0: Interrupt coalescing mechanism is disabled
            Disabled = 0,
            // 1'b1: Interrupt coalescing mechanism is active. Interrupts are counted and timed, and coalesced interrupts are generated
            Enabled = 1
        ],
    ],

    pub CQTDLBA [
        // Task Descriptor List Base Address.
        // This register stores the LSB bits (bits 31:0) of the byte address of the head of the Task Descriptor List in system memory. The size of the task descriptor list is 32 * (Task Descriptor size + Transfer Descriptor size) as configured by Host driver. This address shall be set on Byte1 KByte boundary. The lower 10 bits of this register shall be set to 0 by software and shall be ignored by CQE.
        TDLBA OFFSET(0) NUMBITS(32) []
    ],

    pub CQTDBR [
        // Command Queueing Task Doorbell.
        // Software shall configure TDLBA and TDLBAU, and enable CQE in
        // CQCFG before using this register.
        // Writing 1 to bit n of this register triggers CQE to start processing
        // the task encoded in slot n of the TDL.
        // CQE always processes tasks in-order according to the order
        // submitted to the list by CQTDBR write transactions.
        // CQE processes Data Transfer tasks by reading the Task Descriptor
        // and sending QUEUED_TASK_PARAMS (CMD44) and
        // QUEUED_TASK_ADDRESS (CMD45) commands to the device.
        // CQE processes DCMD tasks (in slot #31, when enabled) by
        // reading the Task Descriptor, and generating the command
        // encoded by its index and argument.
        // The corresponding bit is cleared to 0 by CQE in one of the following events:
        // a. When a task execution is completed (with success or error)
        // b. The task is cleared using CQTCLR register
        // c. All tasks are cleared using CQCTL register
        // d. CQE is disabled using CQCFG register
        // Software may initiate multiple tasks at the same time (batch
        // submission) by writing 1 to multiple bits of this register in the same transaction.
        // In the case of batch submission:
        // CQE shall process the tasks in order of the task index, starting
        // with the lowest index. If one or more tasks in the batch are
        // marked with QBR, the ordering of execution will be based on said processing order.
        DBR OFFSET(0) NUMBITS(32) [],
    ],

    pub CQTDBN [
        // Task Complete Notification.
        // Each of the 32 bits are bit mapped to the 32 tasks.
        // Bit-N(1): Task-N has completed execution (with success or errors)
        // Bit-N(0): Task-N has not completed, could be pending or not submitted.
        // On task completion, software may read this register to know which tasks have finished. After reading this register, software may clear the relevant bit fields by writing 1 to the corresponding bits.
        TCN OFFSET(0) NUMBITS(32) [],
    ],

    pub CQDQS [
        // Device Queue Status.
        // Each of the 32 bits are bit mapped to the 32 tasks.
        // Bit-N(1): Device has marked task N as ready for execution
        // Bit-N(0): Task-N is not ready for execution. This task could be pending in device or not submitted.
        // Host controller updates this register with response of the Device Queue Status command.
        DQS OFFSET(0) NUMBITS(32) [],
    ],

    pub CQDPT [
        // Device Pending Tasks.
        // Each of the 32 bits are bit mapped to the 32 tasks.
        // Bit-N(1): Task-N has been successfully queued into the device and is awaiting execution
        // Bit-N(0): Task-N is not yet queued.
        // Bit n of this register is set if and only if QUEUED_TASK_PARAMS (CMD44) and QUEUED_TASK_ADDRESS (CMD45) were sent for this specific task and if this task hasnt been executed yet. CQE shall set this bit after receiving a successful response for CMD45. CQE shall clear this bit after the task has completed execution.
        // Software reads this register in the task-discard procedure to determine if the task is queued in the device.
        DPT OFFSET(0) NUMBITS(32) [],
    ],

    pub CQTCLR [
        // Command Queueing Task Clear.
        // Writing 1 to bit n of this register orders CQE to clear a task which software has previously issued.
        // This bit can only be written when CQE is in Halt state as indicated in CQCFG register Halt bit. When software writes 1 to a bit in this register, CQE updates the value to 1, and starts clearing the data
        // structures related to the task. CQE clears the bit fields (sets a value of 0) in CQTCLR and in CQTDBR once the clear operation is complete. Software must poll on the CQTCLR until it is cleared to verify that a clear operation was done.
        TCLR OFFSET(0) NUMBITS(32) [],
    ],

    pub CQSSC1 [
        // Send Status Command Idle Timer.
        // This field configures the polling period to be used when using
        // periodic SEND_QUEUE_STATUS (CMD13) polling. Periodic polling
        // is used when tasks are pending in the device, but no data
        // transfer is in progress. When a SEND_QUEUE_STATUS response
        // indicates that no task is ready for execution, CQE counts the
        // configured time until it issues the next SEND_QUEUE_STATUS.
        // Timer units are clock periods of the clock whose frequency is
        // specified in the Internal Timer Clock Frequency field CQCAP
        // register. The minimum value is 0001h (1 clock period) and the
        // maximum value is FFFFh (65535 clock periods).
        // For example, a CQCAP field value of 0 indicates a 19.2 MHz clock
        // frequency (period = 52.08 ns). If the setting in CQSSC1.CIT is
        // 1000h, the calculated polling period is 4096*52.08 ns= 213.33 ns.
        // Should be programmed only when CQCFG.CQ_EN is '0'.
        SQSCMD_IDLE_TMR OFFSET(0) NUMBITS(16) [],
        // Send Status Command Block Counter.
        // This field indicates when SQS CMD is sent while data transfer is inprogress.
        // A value of 'n' indicates that CQE sends status command on the
        // CMD line, during the transfer of data block BLOCK_CNTn, on the
        // data lines, where BLOCK_CNT is the number of blocks in the  current transaction.
        // 4'h0: SEND_QUEUE_STATUS (CMD13) command is not sent during the transaction. Instead, it is sent only when the data lines are idle.
        // 4'h1: SEND_QUEUE_STATUS command is to be sent during the last block of the transaction.
        // 4'h2: SEND_QUEUE_STATUS command when last 2 blocks are pending.
        // 4'h3: SEND_QUEUE_STATUS command when last 3 blocks are pending.
        // ........
        // 4'hf: SEND_QUEUE_STATUS command when last 15 blocks are pending.
        // Should be programmed only when CQCFG.CQ_EN is '0'.
        SQSCMD_BLK_CNT OFFSET(16) NUMBITS(4) [],
    ],

    pub CQSSC2 [
        // Send Queue RCA.
        // This field provides CQE with the contents of the 16-bit RCA field in SEND_QUEUE_STATUS (CMD13) command argument.
        // CQE copies this field to bits 31:16 of the argument when transmitting SEND_ QUEUE_STATUS (CMD13) command.
        SQSCMD_RCA OFFSET(0) NUMBITS(16) []
    ],

    pub CQCRDCT [
        // Direct Command Last Response.
        // This register contains the response of the command generated by the last direct command (DCMD) task that was sent.
        // Contents of this register are valid only after bit 31 of CQTDBR register is cleared by the controller.
        DCMD_RESP OFFSET(0) NUMBITS(32) [],
    ],

    pub CQRMEM [
        // Response Mode Error Mask.
        // The bits of this field are bit mapped to the device response.
        // This bit is used as an interrupt mask on the device status filed that is received in R1/R1b responses.
        // 1'b0: When a R1/R1b response is received, bit i in the device status is ignored.
        // The reset value of this register is set to trigger an interrupt on all "Error" type bits in the device status.
        // 1'b1: When a R1/R1b response is received, with a bit i in the  device status set, a RED interrupt is generated.
        // Note: Responses to CMD13 (SQS) encode the QSR so that they are ignored by this logic.
        RESP_ERR_MASK OFFSET(0) NUMBITS(32) [],
    ],

    pub CQTERRI [
        // Response Mode Error Command Index.
        // This field captures the index of the command that was executed on the command line when the error occurred.
        RESP_ERR_CMD_INDX OFFSET(0) NUMBITS(6) [],
        // Response Mode Error Task ID.
        // This field captures the ID of the task which was executed on the command line when the error occurred.
        RESP_ERR_TASKID OFFSET(8) NUMBITS(5) [],
        // Response Mode Error Fields Valid.
        // This bit is updated when an error is detected while a command transaction was in progress.
        RESP_ERR_FIELDS_VALID OFFSET(15) NUMBITS(1) [
            // 1'b0: Ignore contents of RESP_ERR_TASKID and RESP_ERR_CMD_INDX
            Invalid = 0,
            // 1'b1: Response-related error is detected. Check contents of RESP_ERR_TASKID and RESP_ERR_CMD_INDX fields
            Valid = 1
        ],
        // Data Transfer Error Command Index.
        // This field captures the index of the command that was executed and whose data transfer has errors.
        TRANS_ERR_CMD_INDX OFFSET(16) NUMBITS(6) [],
        // Data Transfer Error Task ID.
        // This field captures the ID of the task that was executed and whose data transfer has errors.
        TRANS_ERR_TASKID OFFSET(24) NUMBITS(5) [],
        // Data Transfer Error Field Valid.
        // This bit is updated when an error is detected while a data transfer transaction was in progress.
        TRANS_ERR_FIELDS_VALID OFFSET(31) NUMBITS(1) [
            // 1'b0: Ignore contents of TRANS_ERR_TASKID and TRANS_ERR_CMD_INDX
            Invalid = 0,
            // 1'b1: Data transfer related error detected. Check contents of TRANS_ERR_TASKID and TRANS_ERR_CMD_INDX fields
            Valid = 1
        ],
    ],

    pub CQCRI [
        // Last Command Response Index
        // This field stores the index of the last received command response.
        // CQE shall update the value every time a command response is received.
        CMD_RESP_INDX OFFSET(0) NUMBITS(6) [],
    ],

    pub CQCRA [
        // Last Command Response Argument.
        // This field stores the argument of the last received command.
        // CQE shall update the value every time a command response is received.
        CMD_RESP_ARG OFFSET(0) NUMBITS(32) [],
    ],

    pub VER_ID [
        // Current version number.
        VER_ID OFFSET(0) NUMBITS(32) [],
    ],

    pub VER_TYPE [
        // Version type.
        VER_TYPE OFFSET(0) NUMBITS(32) [],
    ],

    pub AT_CTRL [
        // Sampling window Threshold enable.
        // Selects the tuning mode. Field should be programmed only when SAMPLE_CLK_SEL is '0'.
        SWIN_TH_EN OFFSET(2) NUMBITS(1) [
            // LARGEST_WIN_MODE. Tuning engine sweeps all taps and settles at the largest window.
            LargestWindow = 0,
            // THRESHOLD_MODE. Tuning engine selects the first complete sampling window that meets the threshold set by SWIN_TH_VAL field.
            ThresholdMode = 1
        ],
        // Framing errors are not generated when executing tuning.
        // This debug bit allows users to report these errors.
        RPT_TUNE_ERR OFFSET(3) NUMBITS(1) [
            // Default mode where as per host no errors are reported
            NoReport = 0,
            // Debug mode for reporting framing errors
            ReportErrors = 1
        ],
        // This fields enables software-managed tuning flow.
        SW_TUNE_EN OFFSET(4) NUMBITS(1) [
            // Software-managed tuning disabled
            Disabled = 0,
            // Software-managed tuning enabled
            Enabled = 1
        ],
        // When enabled, clock gate control output (clk2card_on) is pulled low before changing
        // phase select codes on tuning_cclk_sel and autotuning_cclk_sel. This effectively stops
        // the Device/Card clock, cclk_rx. Changing phase code when clocks are stopped ensures
        // glitch free phase switching.
        TUNE_CLK_STOP_EN OFFSET(16) NUMBITS(1) [
            // Clocks not stopped
            ClocksRunning = 0,
            // Clocks stopped during phase code change
            ClocksStopped = 1
        ],
        // Maximum Latency specification between cclk_tx and cclk_rx.
        PRE_CHANGE_DLY OFFSET(17) NUMBITS(2) [
            // Less than 1-cycle latency
            LessThan1Cycle = 0,
            // Less than 2-cycle latency
            LessThan2Cycles = 1,
            // Less than 3-cycle latency
            LessThan3Cycles = 2,
            // Less than 4-cycle latency
            LessThan4Cycles = 3
        ],
        // Time taken for phase switching and stable clock output.
        POST_CHANGE_DLY OFFSET(19) NUMBITS(2) [
            // Less than 1-cycle latency
            LessThan1Cycle = 0,
            // Less than 2-cycle latency
            LessThan2Cycles = 1,
            // Less than 3-cycle latency
            LessThan3Cycles = 2,
            // Less than 4-cycle latency
            LessThan4Cycles = 3
        ],
    ],

    pub AT_STAT [
        // Centered Phase code. Reading this field returns the current value on tuning_cclk_sel output. Setting AT_CTRL.SW_TUNE_EN enables software to write to this field and its contents are reflected on tuning_cclk_sel.
        CENTER_PH_CODE OFFSET(0) NUMBITS(8) [],
        // Right Edge Phase code. Reading this field returns the phase code value used by Auto-tuning engine to sample data on Right edge of sampling window.
        R_EDGE_PH_CODE OFFSET(8) NUMBITS(8) [],  // RO
        // Left Edge Phase code. Reading this field returns the phase code value used by Auto-tuning engine to sample data on Left edge of sampling window.
        L_EDGE_PH_CODE OFFSET(16) NUMBITS(8) [], // RO
    ],

    pub DLL_CTRL [
        // DLL working indication.
        DLL_START OFFSET(0) NUMBITS(1) [
            NotWork = 0,
            Work = 1
        ],
        // DLL soft reset indication.
        DLL_SRST OFFSET(1) NUMBITS(1) [
            Normal = 0,
            Reset = 1,
        ],
        // DLL increment value.
        DLL_INCRMENT OFFSET(8) NUMBITS(8) [],
        // DLL start point for phase detect.
        DLL_START_POINT OFFSET(16) NUMBITS(8) [],
        // DLL bypass mode select.
        DLL_BYPASS_MODE OFFSET(24) NUMBITS(1) [
            Normal = 0,
            Bypass = 1,
        ],
    ],

    pub DLL_RXCLK [
        // Tap number for RX clock.
        // Every clock is divided into 32 taps equably, and the max value is 31. Use tapnum to select which tap to be used for RX clock.
        RX_TAP_NUM OFFSET(0) NUMBITS(5) [],
        // Tap value for RX clock.
        // It denotes delay element number for RX clock.
        RX_TAP_VALUE OFFSET(8) NUMBITS(8) [],
        // Total delay number of selected tap for RX clock.
        RX_DELAY_NUM OFFSET(16) NUMBITS(8) [],
        // Tapnum selection for RX clock.
        // Tapnum selection for RX clock.
        RX_TAP_NUM_SEL OFFSET(24) NUMBITS(1) [
            // Tapnum comes from tuning result
            FromTuning = 0,
            // Tapnum comes from software (RX_TAP_NUM)
            FromRegister = 1
        ],
        // Tap value selection for RX clock.
        RX_TAP_VALUE_SEL OFFSET(25) NUMBITS(1) [
            // 1'b0: Tapvalue equals to (DLL_LOCK_VALUE*2)%256
            Calculated = 0,
            // 1'b1: Tapvalue comes from software (TX_TAP_VALUE)
            FromRegister = 1
        ],
        // Delay number selection for RX clock.
        RX_DELAY_NUM_SEL OFFSET(26) NUMBITS(1) [
            // Delaynum comes from hardware calculation
            AutoCalculated = 0,
            // Delaynum comes from software (RX_DELAY_NUM)
            FromRegister = 1
        ],
        // RX clock output selection.
        RX_CLK_OUT_SEL OFFSET(27) NUMBITS(1) [
            // RX clock output is from RX clock source or RX clock source with inversion, determined by RX_CLK_SRC_SEL
            ClockSource = 0,
            // RX clock output is delayed by delayline
            DelayLine = 1
        ],
        // RX clock change window. When high, RX clock is gated.
        RX_CLK_CHANGE_WINDOW OFFSET(28) NUMBITS(1) [
            Normal = 0,
            Gated = 1,
        ],
        // RX clock source selection.
        // This bit should be set to 1 for normal operation.
        RX_CLK_SRC_SEL OFFSET(29) NUMBITS(1) [
            // RX clock source is inverted.
            Inverted = 0,
            // RX clock source is no-inverted.
            Normal = 1,
        ],
    ],

    pub DLL_TXCLK [
        // Tap number for TX clock.
        // Every clock is divided into 32 taps equably, and the max value is 31. Use tapnum to select which tap to be used for TX clock.
        TX_TAP_NUM OFFSET(0) NUMBITS(5) [],
        // Tap value for TX clock.
        // It denotes delay element number for TX clock.
        TX_TAP_VALUE OFFSET(8) NUMBITS(8) [],
        // Total delay number of selected tap for TX clock.
        TX_DELAY_NUM OFFSET(16) NUMBITS(8) [],
        // Tapnum selection for TX clock.
        TX_TAP_NUM_SEL OFFSET(24) NUMBITS(1) [
            // Tapnum is 8
            Fixed8 = 0,
            // Tapnum comes from software (TX_TAP_NUM)
            FromRegister = 1
        ],
        // Tap value selection for TX clock.
        TX_TAP_VALUE_SEL OFFSET(25) NUMBITS(1) [
            // Tapvalue equals to (DLL_LOCK_VALUE * 2) % 256
            Calculated = 0,
            // Tapvalue comes from software (TX_TAP_VALUE)
            FromRegister = 1
        ],
        // Delay number selection for TX clock.
        TX_DELAY_NUM_SEL OFFSET(26) NUMBITS(1) [
            // Delaynum comes from hardware calculation
            AutoCalculated = 0,
            // Delaynum comes from software (TX_DELAY_NUM)
            FromRegister = 1
        ],
        // TX clock output selection.
        TX_CLK_OUT_SEL OFFSET(27) NUMBITS(1) [
            // TX clock output is from TX clock source with inversion
            DirectInverted = 0,
            // TX clock output is delayed by delayline
            DelayLine = 1
        ]
    ],

    pub DLL_STRBIN [
        // Tap number for STRBIN.
        // Every clock is divided into 32 taps equably, and the max value is 31. Use tapnum to select which tap to be used for STRBIN.
        STRBIN_TAP_NUM OFFSET(0) NUMBITS(5) [],
        // Tap value for STRBIN.
        // It denotes delay element number for STRBIN.
        STRBIN_TAP_VALUE OFFSET(8) NUMBITS(8) [],
        // Total delay number of selected tap for STRBIN.
        STRBIN_DELAY_NUM OFFSET(16) NUMBITS(8) [],
        // Tapnum selection for STRBIN.
        STRBIN_TAP_NUM_SEL OFFSET(24) NUMBITS(1) [
            // Tapnum is 8
            Fixed8 = 0,
            // Tapnum comes from software (STRBIN_TAP_NUM)
            FromRegister = 1
        ],
        STRBIN_TAP_VALUE_SEL OFFSET(25) NUMBITS(1) [
            // Tapvalue equals to (DLL_LOCK_VALUE * 2) % 256
            Calculated = 0,
            // Tapvalue comes from software (STRBIN_TAP_VALUE)
            FromRegister = 1
        ],
        STRBIN_DELAY_NUM_SEL OFFSET(26) NUMBITS(1) [
            // Delaynum comes from hardware calculation
            AutoCalculated = 0,
            // Delaynum comes from software (STRBIN_DELAY_NUM)
            FromRegister = 1
        ],
        // STRBIN delay enable.
        STRBIN_DELAY_ENA OFFSET(27) NUMBITS(1) [
            Disabled = 0,
            Enabled = 1
        ]
    ],

    pub DLL_STATUS0 [
        // DLL lock value for half sdclk cycle.
        // It denotes the delay element number needed for DLL locked. It is valid when dll_lock is high.
        DLL_LOCK_VALUE OFFSET(0) NUMBITS(8) [],
        // DLL lock indication.
        DLL_LOCK OFFSET(8) NUMBITS(1) [
            Not_Locked = 0,
            Locked = 1
        ],
        // DLL phase detection is timeout or not.
        DLL_LOCK_TIMEOUT OFFSET(9) NUMBITS(1) [
            Not_Timed_Out = 0,
            Timed_Out = 1
        ]
    ],

    pub DLL_STATUS1 [
        // Delay element number used for TX clock.
        DLL_TXCLK_DELAY_VALUE OFFSET(0) NUMBITS(8) [],
        // Delay element number used for RX clock.
        DLL_RXCLK_DELAY_VALUE OFFSET(8) NUMBITS(8) [],
        // Delay element number used for STRBIN.
        DLL_STRBIN_DELAY_VALUE OFFSET(16) NUMBITS(8) []
    ]
];
