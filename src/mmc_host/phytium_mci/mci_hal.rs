use tock_registers::{
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite},
};

register_structs! {
    pub SdRegisters {
        (0x0000 => cntrl: ReadWrite<u32, CNTRL::Register>),
        (0x0004 => pwren: ReadWrite<u32, PWREN::Register>),
        (0x0008 => clkdiv: ReadWrite<u32, CLKDIV::Register>),
        (0x000C => _reserved0),
        (0x0010 => clkena: ReadWrite<u32, CLKENA::Register>),
        (0x0014 => tmout: ReadWrite<u32, TMOUT::Register>),
        (0x0018 => ctype: ReadWrite<u32, CTYPE::Register>),
        (0x001C => blksiz: ReadWrite<u32, BLKSIZ::Register>),
        (0x0020 => bytcnt: ReadWrite<u32, BYTCNT::Register>),
        (0x0024 => int_mask: ReadWrite<u32, INT_MASK::Register>),
        (0x0028 => cmdarg: ReadWrite<u32, CMDARG::Register>),
        (0x002C => cmd: ReadWrite<u32, CMD::Register>),
        (0x0030 => resp0: ReadOnly<u32, RESP0::Register>),
        (0x0034 => resp1: ReadOnly<u32, RESP1::Register>),
        (0x0038 => resp2: ReadOnly<u32, RESP2::Register>),
        (0x003C => resp3: ReadOnly<u32, RESP3::Register>),
        (0x0040 => masked_ints: ReadOnly<u32, MASKED_INTS::Register>),
        (0x0044 => raw_ints: ReadWrite<u32, RAW_INTS::Register>),
        (0x0048 => status: ReadOnly<u32, STATUS::Register>),
        (0x004C => fifoth: ReadWrite<u32, FIFOTH::Register>),
        (0x0050 => card_detect: ReadOnly<u32, CARD_DETECT::Register>),
        (0x0054 => card_write_prt: ReadOnly<u32, CARD_WRITE_PRT::Register>),
        (0x0058 => cksts: ReadOnly<u32, CKSTS::Register>),
        (0x005C => tran_card_cnt_mx: ReadOnly<u32, TRAN_CRD_CNT_MX::Register>),
        (0x0060 => tran_fifo_cnt_mx: ReadOnly<u32, TRAN_FIFO::Register>),
        (0x0064 => debnce: ReadWrite<u32, DEBNCE::Register>),
        (0x0068 => uid: ReadWrite<u32, UID::Register>),
        (0x006C => vid: ReadOnly<u32, VID::Register>),
        (0x0070 => _reserved1),
        (0x0074 => uhs_reg: ReadWrite<u32, UHS_REG::Register>),
        (0x0078 => card_reset: ReadWrite<u32, CARD_RESET::Register>),
        (0x007C => _reserved2),
        (0x0080 => bus_mode_reg: ReadWrite<u32, BUS_MODE_REG::Register>),
        (0x0084 => _reserved3),
        (0x0088 => desc_list_star_reg_l: ReadWrite<u32, DESC_LIST_STAR_REG_L::Register>),
        (0x008C => desc_list_star_reg_u: ReadWrite<u32, DESC_LIST_STAR_REG_U::Register>),
        (0x0090 => status_reg: ReadWrite<u32, STATUS_REG::Register>),
        (0x0094 => intr_en_reg: ReadWrite<u32, INTR_EN_REG::Register>),
        (0x0098 => cur_desc_addr_reg_l: ReadWrite<u32, CUR_DESC_ADDR_REG_L::Register>),
        (0x009C => cur_desc_addr_reg_u: ReadWrite<u32, CUR_DESC_ADDR_REG_U::Register>),
        (0x00A0 => cur_buf_addr_reg_l: ReadWrite<u32, CUR_BUF_ADDR_REG_L::Register>),
        (0x00A4 => cur_buf_addr_reg_u: ReadWrite<u32, CUR_BUF_ADDR_REG_U::Register>),
        (0x00A8 => _reserved4),
        (0x0100 => cardthctl: ReadWrite<u32, CARDTHCTL::Register>),
        (0x0104 => _reserved5),
        (0x0108 => clksrc: ReadWrite<u32, CLKSRC::Register>),
        (0x010C => _reserved6),
        (0x0110 => enable_shift: ReadWrite<u32, ENABLE_SHIFT::Register>),
        (0x0114 => _reserved7),
        (0x0200 => data: ReadWrite<u32, DATA::Register>),
        (0x0204 => _reserved8),
        (0x1000 => @END),
    }
}

register_bitfields![
    u32,

    pub CNTRL [
        // 复位控制器，除 DMA，FIFO。
        CONTROLLER_RESET OFFSET(0) NUMBITS(1) [],
        // 复位 FIFO。1 有效。
        FIFO_RESET OFFSET(1) NUMBITS(1) [],
        // 复位内部 DMA。1 有效。
        DMA_RESET OFFSET(2) NUMBITS(1) [],
        // 保留
        RES OFFSET(3) NUMBITS(1) [],
        // 全局中断使能配置，和 int_mask_n 寄存器相与使能对应中断。1：使能。
        INT_ENABLE OFFSET(4) NUMBITS(1) [],
        // 保留外部 DMA 模式使能。
        RES_DMA_ENABLE OFFSET(5) NUMBITS(1) [],
        // SDIO 读等待。1 有效。
        READ_WAIT OFFSET(6) NUMBITS(1) [],
        // MMC 中断自动响应配置。1 有效。
        SEND_IRQ_RESPONSE OFFSET(7) NUMBITS(1) [],
        // 读暂停异常清除 data FSM。1 有效。
        ABORT_READ_DATA OFFSET(8) NUMBITS(1) [],
        // 发送 CCSD (不支持)。
        SEND_CCSD OFFSET(9) NUMBITS(1) [],
        // 对应 CCD，自动 STOP(不支持)。
        SEND_AUTO_STOP_CCSD OFFSET(10) NUMBITS(1) [],
        // 0：小端；1：大端。
        ENDIAN OFFSET(0) NUMBITS(11) [],
        // A 电压选择。1 有效。
        CARD_VOLTAGE_A OFFSET(16) NUMBITS(4) [],
        // B 电压选择。1 有效。
        CARD_VOLTAGE_B OFFSET(20) NUMBITS(4) [],
        // 外部开漏输出。1 有效。
        ENABLE_OD_PULLUP OFFSET(24) NUMBITS(1) [],
        // 用内部 DMA。1 有效。
        USE_INTERNAL_DMAC OFFSET(25) NUMBITS(1) [],
    ],

    pub PWREN [
        // 卡供电开关。0：关 1：开
        POWER_ENABLE OFFSET(0) NUMBITS(32) [
            PowerOn = 1,
            PowerOff = 0,
        ],
    ],

    pub CLKDIV [
        // 时钟分频参数设置，分频参数=2*CLK_DIVIDER
        CLK_DIVIDER OFFSET(0) NUMBITS(8) [],
        // 输出相位区间设置
        CLK_DRV OFFSET(8) NUMBITS(8) [],
        // 采样相位区间设置
        CLK_SMPL OFFSET(16) NUMBITS(8) [],
    ],

    pub CLKENA [
        // card 时钟使能控制 0：Clock disabled 1：Clock enabled
        CCLK_ENABLE OFFSET(0) NUMBITS(16) [
            ClockEnabled = 1,
            ClockDisabled = 0
        ],
        // 功耗模式控制 0：非低功耗 1：低功耗
        CCLK_LOW_POWER OFFSET(16) NUMBITS(16) [
            LowPower = 1,
            NormalPower = 0
        ],
    ],

    pub TMOUT [
        // 响应超时（以卡时钟为单位）
        RESPONSE_TIMEOUT OFFSET(0) NUMBITS(8) [],
        // 读卡超时（以卡时钟为单位）
        DATA_TIMEOUT OFFSET(8) NUMBITS(24) [],
    ],

    pub CTYPE [
        // 0：1-bit mode；1：4-bit mode
        CARD0_WIDTH2 OFFSET(0) NUMBITS(16) [
            OneBitMode = 0,
            FourBitMode = 1
        ],
        // Non 8-bit mode 1-8-bit mode
        CARD0_WIDTH1 OFFSET(16) NUMBITS(16) [],
    ],

    pub BLKSIZ [
        // Block Size.
        BLOCK_SIZE OFFSET(0) NUMBITS(32) [],
    ],

    pub BYTCNT [
        // 传输字节数
        BYTE_COUNT OFFSET(0) NUMBITS(32) [],
    ],

    pub INT_MASK [
        // 卡检测中断 0：屏蔽；1：使能
        CD_INT_MASK OFFSET(0) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 响应错误中断
        RE_INT_MASK OFFSET(1) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 命令传输完成中断
        CMD_INT_MASK OFFSET(2) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // Data transfer over (DTO) interrupt enable
        DTO_INT_MASK OFFSET(3) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 发送 FIFO 请求中断
        TXDR_INT_MASK OFFSET(4) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 接收 FIFO 请求中断
        RXDR_INT_MASK OFFSET(5) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 响应 CRC 错误中断
        RCRC_INT_MASK OFFSET(6) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 数据 CRC 校验错误中断
        DCRC_INT_MASK OFFSET(7) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 响应超时中断
        RTO_INT_MASK OFFSET(8) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 数据读超时中断
        DRTO_INT_MASK OFFSET(9) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 数据 starv/电源切换中断
        HTO_INT_MASK OFFSET(10) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // FIFO 上下溢中断
        FRUN_INT_MASK OFFSET(11) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 硬件锁存中断
        HLE_INT_MASK OFFSET(12) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 起始位错误/busy 撤销中断
        SBE_BCI_INT_MASK OFFSET(13) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // Auto command 完成中断
        ACD_INT_MASK OFFSET(14) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // 读写结束位错误/写未收到 CRC 中断
        EBE_INT_MASK OFFSET(15) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
        // SDIO interrupt 中断
        SDIO_INT_MASK_CARD0 OFFSET(16) NUMBITS(1) [
            Masked = 0,
            Enabled = 1,
        ],
    ],

    pub CMDARG [
        // 命令参数
        CMD_ARG OFFSET(0) NUMBITS(32) [],
    ],

    pub CMD [
        // 命令索引
        CMD_INDEX OFFSET(0) NUMBITS(6) [],
        // 0：无响应；1：有响应
        RESPONSE_EXPECT OFFSET(6) NUMBITS(1) [
            NoResponse = 0,
            Response = 1,
        ],
        // 0：短响应；1：长响应
        RESPONSE_LENGTH OFFSET(7) NUMBITS(1) [
            ShortResp = 0,
            LongResp = 1,
        ],
        // 0：不检查 CRC；1：检查 CRC
        CHECK_RESPONSE_CRC OFFSET(8) NUMBITS(1) [
            NoCheckCRC = 0,
            CheckCRC = 1,
        ],
        // 0：DTA 无数据；1：DAT 有数据
        DATA_EXPECTED OFFSET(9) NUMBITS(1) [
            NoData = 0,
            Data = 1,
        ],
        // 0：读卡；1：写卡
        READ_WRITE OFFSET(10) NUMBITS(1) [
            Read = 0,
            Write = 1,
        ],
        // 1：自动发送 stop
        SEND_AUTO_STOP OFFSET(12) NUMBITS(1) [],
        // 0：立即发送命令
        WAIT_PRVDATA_COMPLETE OFFSET(13) NUMBITS(1) [],
        // 1：stop/abort 操作
        STOP_ABORT_CMD OFFSET(14) NUMBITS(1) [],
        // 在发送命令之前，等待 80 个 cycle 初始时钟序列完成
        SEND_INITIALIZATION OFFSET(15) NUMBITS(1) [],
        // 保留
        CARD_NUMBER OFFSET(16) NUMBITS(5) [],
        // 1：不发送命令，只更新时钟 REG
        UPDATE_CLOCK_REGISTERS_ONLY OFFSET(21) NUMBITS(1) [],
        // 0：无电压切换；1：使能电压切换
        VOLT_SWITCH OFFSET(28) NUMBITS(1) [],
        // 与命令的发出相关，此位必须为 1。
        // 0：旁路 HOLD Register
        // 1：使能 HOLD Register
        USE_HOLD_REG OFFSET(29) NUMBITS(1) [],
        // 启动命令
        START_CMD OFFSET(30) NUMBITS(2) [],
    ],

    pub RESP0 [
        // 响应寄存器 0 bit[31:0]
        RESPONSE0 OFFSET(0) NUMBITS(32) []
    ],

    pub RESP1 [
        // 响应寄存器 1 bit[63:32]
        RESPONSE1 OFFSET(0) NUMBITS(32) [],
    ],

    pub RESP2 [
        // 响应寄存器 2 bit[95:64]
        RESPONSE2 OFFSET(0) NUMBITS(32) [],
    ],

    pub RESP3 [
        // 响应寄存器 3 bit[127:96]
        RESPONSE3 OFFSET(0) NUMBITS(32) [],
    ],

    pub MASKED_INTS [
        // 卡检测(CD)
        CARD_DETECT_INTERRUPT OFFSET(0) NUMBITS(1) [],
        // 响应错误(RE)
        RESPONSE_ERROR_INTERRUPT OFFSET(1) NUMBITS(1) [],
        // 命令完成(CD)
        COMMAND_DONE_INTERRUPT OFFSET(2) NUMBITS(1) [],
        // 数据传输完成(DTO)
        DATA_TRANSFER_OVER_INTERRUPT OFFSET(3) NUMBITS(1) [],
        // TX FIFO 数据请求(TXDR)
        TRANSMIT_RECEIVE_FIFO_DATA_INTERRUPT OFFSET(4) NUMBITS(1) [],
        // RX FIFO 数据请求(RXDR)
        RECEIVE_FIFO_DATA_REQUEST_INTERRUPT OFFSET(5) NUMBITS(1) [],
        // 响应 CRC 错误(RCRC)
        RESPONSE_CRC_ERROR_INTERRUPT OFFSET(6) NUMBITS(1) [],
        // 数据 CRC 错误(DCRC)
        DATA_CRC_ERROR_INTERRUPT OFFSET(7) NUMBITS(1) [],
        // 响应超时(RTO)
        RESPONSE_TIMEOUT_INTERRUPT OFFSET(8) NUMBITS(1) [],
        // 数据读超时(DRTO)
        DATA_READ_TIMEOUT_INTERRUPT OFFSET(9) NUMBITS(1) [],
        // 数据饥饿超时(HTO)/ Volt_switch
        HOST_TIMEOUT_INTERRUPT OFFSET(10) NUMBITS(1) [],
        // FIFO 上/下溢错误(FRUN)
        FIFO_UNDER_OVER_RUN_INTERRUPT OFFSET(11) NUMBITS(1) [],
        // 硬件锁存写错误(HLE)
        HARDWARE_LOCKED_WRITE_INTERRUPT OFFSET(12) NUMBITS(1) [],
        // 起始位错误(SBE)/Busy 完成(BCI)
        BUSY_COMPLETE_INTERRUPT_INTERRUPT OFFSET(13) NUMBITS(1) [],
        // 自动命令完成(ACD)
        AUTO_COMMAND_DONE_INTERRUPT OFFSET(14) NUMBITS(1) [],
        // 读写 End-bit 错误/写未收到 CRC
        END_BIT_ERROR_INTERRUPT OFFSET(15) NUMBITS(1) [],
        // SDIO card 中断
        SDIO_INTERRUPT_CARD0 OFFSET(16) NUMBITS(1) [],
    ],

    pub RAW_INTS [
        // 卡检测(CD)
        CARD_DETECT_STATUS OFFSET(0) NUMBITS(1) [],
        // 响应错误(RE)
        RESPONSE_ERROR_STATUS OFFSET(1) NUMBITS(1) [],
        // 命令完成(CD)
        COMMAND_DONE_STATUS OFFSET(2) NUMBITS(1) [],
        // 数据传输完成 (DTO)
        DATA_TRANSFER_OVER_STATUS OFFSET(3) NUMBITS(1) [],
        // TX FIFO 数据请求(TXDR) NON-DMA 使用
        TRANSMIT_RECEIVE_FIFO_DATA_STATUS OFFSET(4) NUMBITS(1) [],
        // RX FIFO 数据请求(RXDR) NON-DMA 使用
        RECEIVE_FIFO_DATA_REQUEST_INTERRUPT OFFSET(5) NUMBITS(1) [],
        // 响应 CRC 错误(RCRC)
        RESPONSE_CRC_ERROR_STATUS OFFSET(6) NUMBITS(1) [],
        // 数据 CRC 错误(DCRC)
        DATA_CRC_ERROR_STATUS OFFSET(7) NUMBITS(1) [],
        // 响应超时 (RTO)
        RESPONSE_TIMEOUT_STATUS OFFSET(8) NUMBITS(1) [],
        // 数据读超时(DRTO)
        DATA_READ_TIMEOUT_STATUS OFFSET(9) NUMBITS(1) [],
        // 数据饥饿超时(HTO)/Volt_switch
        HOST_TIMEOUT_STATUS OFFSET(10) NUMBITS(1) [],
        // FIFO 上/下溢错误(FRUN)
        FIFO_UNDER_OVER_RUN_STATUS OFFSET(11) NUMBITS(1) [],
        // 硬件锁存写错误(HLE)
        HARDWARE_LOCKED_WRITE_STATUS OFFSET(12) NUMBITS(1) [],
        // 起始位错误(SBE)/Busy 完成(BCI)
        BUSY_COMPLETE_STATUS OFFSET(13) NUMBITS(1) [],
        // 自动命令完成 (ACD)
        AUTO_COMMAND_DONE_STATUS OFFSET(14) NUMBITS(1) [],
        // 读写 End-bit 错误/写未收到 CRC
        END_BIT_ERROR_STATUS OFFSET(15) NUMBITS(1) [],
        // SDIO card 中断
        SDIO_INTERRUPT_CARD0 OFFSET(16) NUMBITS(1) [],
    ],

    pub STATUS [
        // 达到 FIFO_RX 标记
        FIFO_RX_WATERMARK OFFSET(0) NUMBITS(1) [],
        // 达到 FIFO_TX 标记
        FIFO_TX_WATERMARK OFFSET(1) NUMBITS(1) [],
        // FIFO empty
        FIFO_EMPTY OFFSET(2) NUMBITS(1) [],
        // FIFO full
        FIFO_FULL OFFSET(3) NUMBITS(1) [],
        // cmd FSM
        COMMAND_FSM_STATES OFFSET(4) NUMBITS(4) [],
        // DATA[3] 卡在位检测 0：不在位；1：在位
        DATA_3_STATUS OFFSET(8) NUMBITS(1) [
            NotPresent = 0,
            Present = 1,
        ],
        // 卡 busy 0：not busy；1：busy
        DATA_BUSY OFFSET(9) NUMBITS(1) [
            NotBusy = 0,
            Busy = 1,
        ],
        // DATA TX|RX FSM busy 0：not busy；1：busy
        DATA_STATE_MC_BUSY OFFSET(10) NUMBITS(1) [
            NotBusy = 0,
            Busy = 1,
        ],
        // 响应索引
        RESPONSE_INDEX OFFSET(11) NUMBITS(6) [],
        // FIFO 填充计数器
        FIFO_COUNT OFFSET(17) NUMBITS(13) [],
        // DMA 确认
        DMA_ACK OFFSET(30) NUMBITS(1) [],
        // DMA 请求
        DMA_REQ OFFSET(31) NUMBITS(1) [],
    ],

    pub FIFOTH [
        // FIFO threshold
        TX_WMark OFFSET(0) NUMBITS(12) [],
        // FIFO threshold
        RX_WMark OFFSET(16) NUMBITS(12) [],
        // SRC/DEST_MSIZE
        DMA_Multiple_Transaction_Size OFFSET(28) NUMBITS(3) [
            // 000：1
            // 001：4
            // 010：8
            // 011：16
            // 100：32
            // 101：64
            // 110：128
            // 111：256
        ],
    ],

    pub CARD_DETECT [
        // 1：卡不在位；0：卡在位
        CARD0_DETECT_N OFFSET(0) NUMBITS(32) [
            NotPresent = 1,
            Present = 0,
        ],
    ],

    pub CARD_WRITE_PRT [
        // 1：写保护；0：无写保护
        WRITE_PROTECT_0 OFFSET(0) NUMBITS(32) [
            WriteProtected = 1,
            NotWriteProtected = 0,
        ],
    ],

    pub CKSTS [
        // Device 接口模块时钟 ready
        CCLK_RDY OFFSET(0) NUMBITS(32) [],
    ],

    pub TRAN_CRD_CNT_MX [
        // Device 接口模块到 Device 传输的字节数
        TRANS_CARD_BYTE_COUNT OFFSET(0) NUMBITS(32) [],
    ],

    pub TRAN_FIFO [
        // MEM&FIFO 之间传输的字节数
        TRANS_FIFO_BYTE_COUNT OFFSET(0) NUMBITS(32) []
    ],

    pub DEBNCE [
        // 去抖时钟数，参考值 5-25 ms
        DEBOUNCE_COUNT OFFSET(0) NUMBITS(32) [],
    ],

    pub UID [
        // 用户 ID
        UID OFFSET(0) NUMBITS(32) [],
    ],

    pub VID [
        // 控制器版本
        VID OFFSET(0) NUMBITS(32) [],
    ],

    pub UHS_REG [
        // 外部调压器接口电压 0：3.3V Vdd；1：1.8V Vdd
        VOLT_REG_0 OFFSET(0) NUMBITS(16) []
    ],

    pub CARD_RESET [
        // 1：运行；0：复位
        CARD0_RESET OFFSET(0) NUMBITS(32) [],
    ],

    pub BUS_MODE_REG [
        // 软复位 IDMA 复位内部 REG，自动清 0
        SWR OFFSET(0) NUMBITS(1) [],
        // 固定 Burst  0：SINGLE & INCR 1：自动选择 SINGLE，INCR4，INCR8，或者INCR16
        FB OFFSET(1) NUMBITS(1) [],
        // IDMAC 使能
        DE OFFSET(7) NUMBITS(1) [],
        // burst LEN
        PBL OFFSET(8) NUMBITS(3) [
            // 000：1 transfers
            // 001：4 transfers
            // 010：8 transfers
            // 011：16 transfers
            // 100：32 transfers
            // 101：64 transfers
            // 110：128 transfers
            // 111：256 transfers
        ],
    ],

    pub DESC_LIST_STAR_REG_L [
        // desc_list_star_reg[31:0]
        DBADDRL OFFSET(0) NUMBITS(32) [],
    ],

    pub DESC_LIST_STAR_REG_U [
        // desc_list_star_reg[63:32]
        DBADDRU OFFSET(0) NUMBITS(32) [],
    ],

    pub STATUS_REG [
        // 发送完成中断，针对描述符
        TI OFFSET(0) NUMBITS(1) [],
        // 接收完成中断，针对描述符
        RI OFFSET(1) NUMBITS(1) [],
        // 总线错误中断 1：清除 bit[12:10]中断
        FBE OFFSET(2) NUMBITS(1) [],
        // 描述符不可读中断
        DU OFFSET(3) NUMBITS(2) [],
        // 卡错误汇总
        // EBE：结束位错误
        // RTO：响应超时/Boot Ack 超时
        // RCRC：响应 CRC 错误
        // SBE：起始位错误
        // DRTO：数据读超时/BDS 超时
        // DCRC：数据 CRC 错误
        // RE：响应错误
        CES OFFSET(5) NUMBITS(1) [],
        // 正常中断汇总
        NIS OFFSET(6) NUMBITS(3) [],
        // 异常中断汇总
        AIS OFFSET(9) NUMBITS(1) [],
        // 异常标识
        // 3'b001：发送异常
        // 3'b010：接收异常
        // bit[2]置 1，不产生该中断
        EB OFFSET(10) NUMBITS(3) [],
        // DMAC FSM 状态
        FSM OFFSET(13) NUMBITS(19) [],
    ],

    pub INTR_EN_REG [
        // 发送中断使能
        TIE OFFSET(0) NUMBITS(1) [],
        // 接收中断使能
        RIE OFFSET(1) NUMBITS(1) [],
        // 总线错误中断使能
        FBEE OFFSET(2) NUMBITS(1) [],
        // 描述符不可读中断使能
        DUE OFFSET(4) NUMBITS(1) [],
        // 卡错误中断使能
        CESE OFFSET(5) NUMBITS(1) [],
        // 正常中断使能
        NIE OFFSET(8) NUMBITS(1) [],
        // 异常中断使能
        AISE OFFSET(9) NUMBITS(1) [],
    ],

    pub CUR_DESC_ADDR_REG_L [
        // cur_desc_addr_reg[31:0]
        DSCADDRL OFFSET(0) NUMBITS(32) [],
    ],

    pub CUR_DESC_ADDR_REG_U [
        // cur_desc_addr_reg[63:32]
        DSCADDRU OFFSET(0) NUMBITS(32) [],
    ],

    pub CUR_BUF_ADDR_REG_L [
        // cur_buf_addr_reg[31:0]
        BUFADDRL OFFSET(0) NUMBITS(32) [],
    ],

    pub CUR_BUF_ADDR_REG_U [
        // cur_buf_addr_reg[63:32]
        BUFADDRU OFFSET(0) NUMBITS(32) [],
    ],

    pub CARDTHCTL [
        // 卡读 Threshold 使能
        CARDRDTHREN OFFSET(0) NUMBITS(1) [],
        // Busy 清中断
        BUSY_CLR_INT_EN OFFSET(1) NUMBITS(1) [],
        // 写卡 Threshold 使能
        CARDWRTHREN OFFSET(2) NUMBITS(1) [],
        // 读卡阈值大小，只有在接收 FIFO 中有满足
        // 该值定义的空间可用时，才进行传输。建
        // 议该配置值大于或等于读取传输的 BLOCK
        // 大小，以确保在数据块之间卡时钟不会停
        // 止。
        // N 取值范围为 23~28。
        // N=28：FIFO_DEPTH 为 256
        // N=27：FIFO_DEPTH 为 128
        // N=26：FIFO_DEPTH 为 64
        // N=25：FIFO_DEPTH 为 32
        // N=24：FIFO_DEPTH 为 16
        // N=23：FIFO_DEPTH 为 8
        // CARDRDTHRESHOLD OFFSET(16) NUMBITS(32) [],
    ],

    pub CLKSRC [
        // 外部时钟－控制器内部设备接口模块时钟源使能。
        EXT_CLK_ENABLE OFFSET(1) NUMBITS(1) [],
        // 分频参数，CIU f= CLK_DIV_CTRL +1，MIN=1。
        CLK_DIV_CTRL OFFSET(8) NUMBITS(6) [],
        // 采样相位参数，相对于控制器端时钟相位点。
        CLK_SMPL_PHASE_CTRL OFFSET(16) NUMBITS(6) [],
        // 输出相位参数，相对于控制器端时钟相位点。
        CLK_DRV_PHASE_CTRL OFFSET(24) NUMBITS(6) [],
    ],

    pub ENABLE_SHIFT [
        // 00：默认相位 ；01：移位使能到下一个上升沿；10：移位使能到下一个下降沿
        ENABLE_SHIFT OFFSET(0) NUMBITS(2) [],
    ],

    pub DATA [
        // 数据 FIFO 寄存器
        DATA OFFSET(0) NUMBITS(32) [],
    ],
];
