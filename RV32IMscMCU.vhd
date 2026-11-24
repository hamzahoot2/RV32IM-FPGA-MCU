--============================================================================
-- RV32IM Single-Cycle MCU - Stage 1
-- CPU + external DTCM + GPIO + Basic Timer register bank
--============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.cond_compilation_package.ALL;
library altera_mf;
use altera_mf.altera_mf_components.all;

entity RV32IMscMCU is
    generic(
        MODELSIM : integer := G_MODELSIM
    );
    port(
        CLOCK_50 : in  std_logic;
        KEY      : in  std_logic_vector(3 downto 0);
        SW       : in  std_logic_vector(9 downto 0);
        LEDR     : out std_logic_vector(9 downto 0);
        HEX0     : out std_logic_vector(6 downto 0);
        HEX1     : out std_logic_vector(6 downto 0);
        HEX2     : out std_logic_vector(6 downto 0);
        HEX3     : out std_logic_vector(6 downto 0);
        HEX4     : out std_logic_vector(6 downto 0);
        HEX5     : out std_logic_vector(6 downto 0);
        PWMOUT   : out std_logic;
        CAPIN1   : in  std_logic;
        CAPIN2   : in  std_logic
    );
end entity;

architecture structural of RV32IMscMCU is
    constant DATA_BUS_WIDTH  : integer := 32;
    constant DTCM_ADDR_WIDTH : integer := G_ADDRWIDTH;

    signal rst_w          : std_logic;
    signal mclk_w         : std_logic;
    signal data_addr_w    : std_logic_vector(31 downto 0);
    signal data_wr_w      : std_logic_vector(31 downto 0);
    signal data_rd_w      : std_logic_vector(31 downto 0);
    signal mem_read_w     : std_logic;
    signal mem_write_w    : std_logic;

    ---------------------------------------------------------------------------
    -- NOTE ON EVERYTHING BELOW LABELED "...DBG...":
    -- these are SignalTap debug trace buffers — small on-chip RAMs that
    -- just record snapshots of internal signals (PC, instruction, bus
    -- activity, interrupt state, etc.) over time so they can be inspected
    -- after running on the real FPGA. They don't feed back into the CPU
    -- or change its behavior at all — think of them as a flight recorder.
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- MUL FPGA trace signals.
    -- No CPU behavior is changed: these are existing RV32I_CORE debug outputs
    -- copied into a separate In-System Memory instance named MULDBG.
    ---------------------------------------------------------------------------
    signal muldbg_rd1_w        : std_logic_vector(31 downto 0);
    signal muldbg_rd2_w        : std_logic_vector(31 downto 0);
    signal muldbg_write_data_w : std_logic_vector(31 downto 0);
    signal muldbg_alu_res_w    : std_logic_vector(31 downto 0);

    signal muldbg_cap_a_q      : std_logic_vector(31 downto 0) := (others => '0');
    signal muldbg_cap_b_q      : std_logic_vector(31 downto 0) := (others => '0');
    signal muldbg_cap_res_q    : std_logic_vector(31 downto 0) := (others => '0');
    signal muldbg_cap_wb_q     : std_logic_vector(31 downto 0) := (others => '0');
    signal muldbg_cap_pc_q     : std_logic_vector(31 downto 0) := (others => '0');
    signal muldbg_cap_inst_q   : std_logic_vector(31 downto 0) := (others => '0');

    signal muldbg_slot_q       : unsigned(2 downto 0) := (others => '0');
    signal muldbg_state_q      : unsigned(2 downto 0) := (others => '0');
    signal muldbg_wren_w       : std_logic := '0';
    signal muldbg_addr_w       : std_logic_vector(5 downto 0) := (others => '0');
    signal muldbg_data_w       : std_logic_vector(31 downto 0) := (others => '0');
    signal muldbg_q_w          : std_logic_vector(31 downto 0);
    signal muldbg_is_mul_w     : std_logic;

    ---------------------------------------------------------------------------
    -- STORE trace: one 128-bit record per DTCM store.
    -- Record format:
    -- [127:96] PC (zero extended)
    -- [95:64]  instruction
    -- [63:32]  DTCM byte address
    -- [31:0]   write data
    ---------------------------------------------------------------------------
    signal swdbg_slot_q        : unsigned(4 downto 0) := (others => '0');
    signal swdbg_wren_w        : std_logic := '0';
    signal swdbg_addr_w        : std_logic_vector(4 downto 0) := (others => '0');
    signal swdbg_data_w        : std_logic_vector(127 downto 0) := (others => '0');
    signal swdbg_q_w           : std_logic_vector(127 downto 0);

    ---------------------------------------------------------------------------
    -- BRDBG: captures the loop-tail dependency around PC=0x5C..0x68.
    -- One 256-bit record per sampled cycle:
    -- [255:224] PC
    -- [223:192] instruction
    -- [191:160] read_data1
    -- [159:128] read_data2
    -- [127:96]  alu_res/write-back path
    -- [95:64]   write_data
    -- [63:32]   flags (bit0=Branch_ctrl, bit1=brTaken)
    -- [31:0]    reserved
    ---------------------------------------------------------------------------
    signal brdbg_branch_w      : std_logic;
    signal brdbg_taken_w       : std_logic;
    signal brdbg_slot_q        : unsigned(4 downto 0) := (others => '0');
    signal brdbg_wren_w        : std_logic := '0';
    signal brdbg_addr_w        : std_logic_vector(4 downto 0) := (others => '0');
    signal brdbg_data_w        : std_logic_vector(255 downto 0) := (others => '0');
    signal brdbg_q_w           : std_logic_vector(255 downto 0);
    signal brdbg_pc32_w        : std_logic_vector(31 downto 0);
    signal brdbg_flags_w       : std_logic_vector(31 downto 0);

    ---------------------------------------------------------------------------
    -- X8DBG: trace every register-file write whose rd = x8 (s0).
    -- One 128-bit record per x8 write:
    -- [127:96] PC
    -- [95:64]  instruction
    -- [63:32]  write_data
    -- [31:0]   info: bit0=RegWrite, bits[5:1]=rd
    ---------------------------------------------------------------------------
    signal x8dbg_regwrite_w    : std_logic;
    signal x8dbg_slot_q        : unsigned(3 downto 0) := (others => '0');
    signal x8dbg_wren_w        : std_logic := '0';
    signal x8dbg_addr_w        : std_logic_vector(3 downto 0) := (others => '0');
    signal x8dbg_data_w        : std_logic_vector(127 downto 0) := (others => '0');
    signal x8dbg_q_w           : std_logic_vector(127 downto 0);

    ---------------------------------------------------------------------------
    -- IODBG: traces memory-mapped I/O writes in the GPIO/HEX range.
    -- One 128-bit record per I/O store:
    -- [127:96] PC
    -- [95:64]  instruction
    -- [63:32]  byte address
    -- [31:0]   write data
    ---------------------------------------------------------------------------
    signal iodbg_slot_q        : unsigned(5 downto 0) := (others => '0');
    signal iodbg_wren_w        : std_logic := '0';
    signal iodbg_addr_w        : std_logic_vector(5 downto 0) := (others => '0');
    signal iodbg_data_w        : std_logic_vector(127 downto 0) := (others => '0');
    signal iodbg_q_w           : std_logic_vector(127 downto 0);
    signal iodbg_pc32_w        : std_logic_vector(31 downto 0);

    ---------------------------------------------------------------------------
    -- HEXDBG: captures the actual 7-segment outputs driven to the FPGA pins.
    -- One 64-bit record per change:
    -- [63:57] HEX5
    -- [56:50] HEX4
    -- [49:43] HEX3
    -- [42:36] HEX2
    -- [35:29] HEX1
    -- [28:22] HEX0
    -- [21:0]  reserved
    ---------------------------------------------------------------------------
    signal hexdbg_prev_q       : std_logic_vector(41 downto 0) := (others => '0');
    signal hexdbg_slot_q       : unsigned(5 downto 0) := (others => '0');
    signal hexdbg_wren_w       : std_logic := '0';
    signal hexdbg_addr_w       : std_logic_vector(5 downto 0) := (others => '0');
    signal hexdbg_data_w       : std_logic_vector(63 downto 0) := (others => '0');
    signal hexdbg_q_w          : std_logic_vector(63 downto 0);

    ---------------------------------------------------------------------------
    -- SWREADDBG: captures activity around PORT_SW (0x2010).
    -- Record layout [127:0]:
    -- [127:96] data_addr_w
    -- [95:64]  sw_data_rd_w
    -- [63:32]  data_rd_w (what CPU sees)
    -- [31:22]  physical SW[9:0]
    -- [21]     mem_read_w
    -- [20:0]   reserved
    ---------------------------------------------------------------------------
    signal swreaddbg_slot_q  : unsigned(5 downto 0) := (others => '0');
    signal swreaddbg_wren_w  : std_logic := '0';
    signal swreaddbg_addr_w  : std_logic_vector(5 downto 0) := (others => '0');
    signal swreaddbg_data_w  : std_logic_vector(127 downto 0) := (others => '0');
    signal swreaddbg_q_w     : std_logic_vector(127 downto 0);
    signal swreaddbg_pc32_w  : std_logic_vector(31 downto 0);

    ---------------------------------------------------------------------------
    -- SWPINDBG: direct physical switch snapshot.
    -- Each 32-bit word is zero-extended SW[9:0].
    -- No CPU/MMIO decoding is involved.
    ---------------------------------------------------------------------------
    signal swpindbg_cnt_q   : unsigned(5 downto 0) := (others => '0');
    signal swpindbg_prev_q  : std_logic_vector(9 downto 0) := (others => '0');
    signal swpindbg_wren_w  : std_logic := '0';
    signal swpindbg_addr_w  : std_logic_vector(5 downto 0) := (others => '0');
    signal swpindbg_data_w  : std_logic_vector(31 downto 0) := (others => '0');
    signal swpindbg_q_w     : std_logic_vector(31 downto 0);

    ---------------------------------------------------------------------------
    -- PATHDBG: unified CPU memory-path trace.
    -- [255:224] PC
    -- [223:192] instruction
    -- [191:160] data_addr_w
    -- [159:128] data_rd_w
    -- [127:96]  data_wr_w
    -- [95]      mem_read_w
    -- [94]      mem_write_w
    -- [93:0]    reserved
    ---------------------------------------------------------------------------
    signal pathdbg_slot_q  : unsigned(5 downto 0) := (others => '0');
    signal pathdbg_wren_w  : std_logic := '0';
    signal pathdbg_addr_w  : std_logic_vector(5 downto 0) := (others => '0');
    signal pathdbg_data_w  : std_logic_vector(255 downto 0) := (others => '0');
    signal pathdbg_q_w     : std_logic_vector(255 downto 0);
    signal pathdbg_pc32_w  : std_logic_vector(31 downto 0);

    ---------------------------------------------------------------------------
    -- HOLDDBG: why the PC is being held
    ---------------------------------------------------------------------------
    signal dbg_irq_hold_w    : std_logic;
    signal dbg_div_hold_w    : std_logic;
    signal dbg_divrem_w      : std_logic_vector(1 downto 0);
    signal dbg_divstate_w    : std_logic_vector(2 downto 0);

    signal holddbg_slot_q    : unsigned(5 downto 0) := (others => '0');
    signal holddbg_wren_w    : std_logic := '0';
    signal holddbg_addr_w    : std_logic_vector(5 downto 0) := (others => '0');
    signal holddbg_data_w    : std_logic_vector(127 downto 0) := (others => '0');
    signal holddbg_q_w       : std_logic_vector(127 downto 0);
    signal dbg_jal_w, dbg_jalr_w, dbg_branch_w, dbg_brtaken_w, dbg_irq_jump_w : std_logic;
    signal dbg_alu_res_w : std_logic_vector(31 downto 0);
    signal pcdbg_slot_q : unsigned(5 downto 0) := (others => '0');
    signal pcdbg_wren_w : std_logic := '0';
    signal pcdbg_addr_w : std_logic_vector(5 downto 0);
    signal pcdbg_data_w : std_logic_vector(255 downto 0);
    signal pcdbg_q_w    : std_logic_vector(255 downto 0);

    signal loopdbg_slot_q  : unsigned(5 downto 0) := (others => '0');
    signal loopdbg_wren_w  : std_logic := '0';
    signal loopdbg_addr_w  : std_logic_vector(5 downto 0) := (others => '0');
    signal loopdbg_data_w  : std_logic_vector(255 downto 0) := (others => '0');
    signal loopdbg_q_w     : std_logic_vector(255 downto 0);
    signal hexdbg_now_w        : std_logic_vector(41 downto 0);
    signal x8dbg_pc32_w        : std_logic_vector(31 downto 0);
    signal x8dbg_info_w        : std_logic_vector(31 downto 0);

    signal dtcm_cs_w      : std_logic;
    signal led_cs_w       : std_logic;
    signal sw_cs_w        : std_logic;
    signal hex_cs_w       : std_logic;
    signal key_cs_w       : std_logic;
    signal timer_cs_w     : std_logic;
    signal intc_cs_w      : std_logic;
    signal divclk_w       : std_logic;
    signal smclk_w        : std_logic;
    signal dtcm_read_w    : std_logic;
    signal dtcm_write_w   : std_logic;
    signal hex_write_w    : std_logic;

    signal dtcm_addr_w    : std_logic_vector(DTCM_ADDR_WIDTH-1 downto 0);
    signal dtcm_data_rd_w : std_logic_vector(31 downto 0);
    signal led_data_rd_w  : std_logic_vector(31 downto 0);
    signal sw_data_rd_w   : std_logic_vector(31 downto 0);
    signal hex_data_rd_w  : std_logic_vector(31 downto 0);
    signal key_data_rd_w  : std_logic_vector(31 downto 0);
    signal timer_data_rd_w  : std_logic_vector(31 downto 0);
    signal intc_data_rd_w   : std_logic_vector(31 downto 0);
    signal btctl1_dbg_w   : std_logic_vector(7 downto 0);
    signal btctl2_dbg_w   : std_logic_vector(7 downto 0);
    signal btcmpr0_dbg_w  : std_logic_vector(31 downto 0);
    signal btcmpr1_dbg_w  : std_logic_vector(31 downto 0);
    signal btcapr_dbg_w   : std_logic_vector(31 downto 0);
    signal btcnt_dbg_w    : std_logic_vector(31 downto 0);
    signal equ0_dbg_w     : std_logic;
    signal equ1_dbg_w     : std_logic;
    signal pwmout_w       : std_logic;
    signal capture_event_dbg_w : std_logic;
    signal timer_btifg_w       : std_logic;
    signal timer_irq_w         : std_logic;
    signal timer_irq_clear_w   : std_logic;

    ---------------------------------------------------------------------------
    -- MCLK -> SMCLK CDC for Basic Timer MMIO writes
    -- CPU holds payload in MCLK domain; a toggle transfers one write event.
    ---------------------------------------------------------------------------
    -- Lossless MCLK->SMCLK write FIFO for Basic Timer MMIO.
    -- Depth 8 is more than enough for the short initialization burst.
    type timer_fifo_t is array (0 to 7) of std_logic_vector(63 downto 0);
    signal timer_fifo_q : timer_fifo_t := (others => (others => '0'));

    signal timer_wbin_q,  timer_wgray_q : unsigned(3 downto 0) := (others => '0');
    signal timer_rbin_q,  timer_rgray_q : unsigned(3 downto 0) := (others => '0');
    signal timer_rgray_m1_q, timer_rgray_m2_q : unsigned(3 downto 0) := (others => '0');
    signal timer_wgray_s1_q, timer_wgray_s2_q : unsigned(3 downto 0) := (others => '0');

    signal timer_fifo_empty_s_w : std_logic;
    signal timer_fifo_full_m_w  : std_logic;
    signal timer_wr_pulse_s_w   : std_logic := '0';

    signal timer_bus_cs_s_w    : std_logic;
    signal timer_bus_rd_s_w    : std_logic;
    signal timer_bus_wr_s_w    : std_logic;
    signal timer_bus_addr_s_w  : std_logic_vector(31 downto 0);
    signal timer_bus_data_s_w  : std_logic_vector(31 downto 0);

    ---------------------------------------------------------------------------
    -- MCLK -> SMCLK CDC for timer IRQ-clear pulse
    ---------------------------------------------------------------------------
    signal timer_clr_toggle_m_q : std_logic := '0';
    signal timer_clr_sync1_s_q  : std_logic := '0';
    signal timer_clr_sync2_s_q  : std_logic := '0';
    signal timer_clr_seen_s_q   : std_logic := '0';
    signal timer_clr_pulse_s_w  : std_logic := '0';
    signal intc_intr_w          : std_logic;
    signal intc_ie_dbg_w        : std_logic_vector(7 downto 0);
    signal intc_ifg_dbg_w       : std_logic_vector(7 downto 0);
    signal intc_type_dbg_w      : std_logic_vector(7 downto 0);
    signal intc_key_event_dbg_w : std_logic_vector(2 downto 0);
    signal key_clean_w          : std_logic_vector(2 downto 0);
    signal btcl0_dbg_w         : std_logic_vector(31 downto 0);
    signal btcl1_dbg_w         : std_logic_vector(31 downto 0);
    signal heu0_dbg_w          : std_logic;
    signal heu1_dbg_w          : std_logic;
    signal cpu_inta_w          : std_logic;
    signal cpu_gie_w           : std_logic;
    signal cpu_irq_state_w     : std_logic_vector(1 downto 0);
    signal cpu_irq_type_reg_w  : std_logic_vector(7 downto 0);
    signal cpu_irq_return_pc_w : std_logic_vector(G_PC_WIDTH-1 downto 0);

    ---------------------------------------------------------------------------
    -- INTDBG: interrupt/timer path snapshot
    -- [127:96] PC
    -- [95:64]  instruction
    -- [63:56]  IE
    -- [55:48]  IFG
    -- [47:40]  TYPE
    -- [39]     timer_irq
    -- [38]     INTR
    -- [37]     INTA
    -- [36]     GIE
    -- [35:34]  CPU IRQ state
    -- [33:26]  CPU latched IRQ TYPE
    -- [25:0]   reserved
    ---------------------------------------------------------------------------
    signal intdbg_slot_q : unsigned(5 downto 0) := (others => '0');
    signal intdbg_wren_w : std_logic := '0';
    signal intdbg_addr_w : std_logic_vector(5 downto 0) := (others => '0');
    signal intdbg_data_w : std_logic_vector(127 downto 0) := (others => '0');
    signal intdbg_q_w    : std_logic_vector(127 downto 0);

    ---------------------------------------------------------------------------
    -- INTBUS: exact CPU bus accesses to interrupt-controller registers.
    -- Captures every read/write to 0x202C..0x202E. Observation only.
    -- [255:224] PC
    -- [223:192] instruction
    -- [191:160] data_addr_w
    -- [159:128] data_wr_w
    -- [127:96]  data_rd_w
    -- [95:88]   IE
    -- [87:80]   IFG
    -- [79:72]   TYPE
    -- [71]      mem_read
    -- [70]      mem_write
    -- [69]      intc_cs
    -- [68]      INTR
    -- [67]      INTA
    -- [66]      GIE
    -- [65:64]   CPU IRQ state
    -- [63:56]   CPU latched IRQ TYPE
    -- [55:53]   KEY event debug
    -- [52:0]    reserved
    ---------------------------------------------------------------------------
    signal intbus_slot_q : unsigned(5 downto 0) := (others => '0');
    signal intbus_wren_w : std_logic := '0';
    signal intbus_addr_w : std_logic_vector(5 downto 0) := (others => '0');
    signal intbus_data_w : std_logic_vector(255 downto 0) := (others => '0');
    signal intbus_q_w    : std_logic_vector(255 downto 0);
    -- BTDBG: live Basic Timer snapshot (one RAM location per clock)
    -- [127:96] BTCNT
    -- [95:64]  BTCMPR0
    -- [63:56]  BTCTL1
    -- [55:48]  BTCTL2
    -- [47]     EQU0
    -- [46]     BTIFG
    -- [45]     timer_irq
    -- [44]     timer_irq_clear
    -- [43]     timer_cs
    -- [42]     mem_write
    -- [41]     mem_read
    -- [40:9]   current CPU data address
    -- [8:0]    reserved
    signal btdbg_data_w : std_logic_vector(127 downto 0) := (others => '0');
    signal btdbg_q_w    : std_logic_vector(127 downto 0);

    ---------------------------------------------------------------------------
    -- TWRDBG: CPU-side Basic Timer MMIO write trace, before any CDC.
    -- [127:96] PC
    -- [95:64]  instruction
    -- [63:32]  data_addr_w
    -- [31:0]   data_wr_w
    ---------------------------------------------------------------------------
    signal twrdbg_slot_q : unsigned(5 downto 0) := (others => '0');
    signal twrdbg_wren_w : std_logic := '0';
    signal twrdbg_addr_w : std_logic_vector(5 downto 0) := (others => '0');
    signal twrdbg_data_w : std_logic_vector(127 downto 0) := (others => '0');
    signal twrdbg_q_w    : std_logic_vector(127 downto 0);
    signal cpu_tp_w            : std_logic_vector(31 downto 0);
	signal mclk_cnt_dbg_w : std_logic_vector(15 downto 0);
    signal led8_w              : std_logic_vector(7 downto 0);

    -- Stable SignalTap debug aliases
    signal DBG_PC           : std_logic_vector(G_PC_WIDTH-1 downto 0);
    signal DBG_INSTRUCTION  : std_logic_vector(31 downto 0);
    signal DBG_MCLK         : std_logic;
    signal DBG_DIVCLK       : std_logic;
    signal DBG_RESET        : std_logic;
    signal DBG_MEM_WRITE    : std_logic;
    signal DBG_MEM_READ     : std_logic;
    signal DBG_DTCM_WRITE   : std_logic;
    signal DBG_DTCM_READ    : std_logic;
    signal DBG_DATA_ADDR    : std_logic_vector(31 downto 0);
    signal DBG_DATA_WR      : std_logic_vector(31 downto 0);
    signal DBG_DATA_RD      : std_logic_vector(31 downto 0);
    signal DBG_DTCM_ADDR    : std_logic_vector(DTCM_ADDR_WIDTH-1 downto 0);
    signal DBG_DTCM_DATA_RD : std_logic_vector(31 downto 0);

    attribute keep : boolean;
    attribute keep of DBG_PC, DBG_INSTRUCTION, DBG_MCLK, DBG_DIVCLK,
                      DBG_RESET, DBG_MEM_WRITE, DBG_MEM_READ,
                      DBG_DTCM_WRITE, DBG_DTCM_READ, DBG_DATA_ADDR,
                      DBG_DATA_WR, DBG_DATA_RD, DBG_DTCM_ADDR,
                      DBG_DTCM_DATA_RD : signal is true;

    -- IRQV: observation-only trace of the CPU interrupt vector fetch.
    signal irqv_slot_q : unsigned(5 downto 0) := (others => '0');
    signal irqv_addr_w : std_logic_vector(5 downto 0);
    signal irqv_wren_w : std_logic;
    signal irqv_data_w : std_logic_vector(255 downto 0);
    signal irqv_q_w    : std_logic_vector(255 downto 0);

    ---------------------------------------------------------------------------
    -- KEYD: direct key-path trace.
    -- Captures changes in physical KEY(3:1), debounced key_clean_w, or
    -- interrupt-controller key_event output.
    ---------------------------------------------------------------------------
    signal keyd_slot_q   : unsigned(5 downto 0) := (others => '0');
    signal keyd_addr_w   : std_logic_vector(5 downto 0);
    signal keyd_wren_w   : std_logic;
    signal keyd_data_w   : std_logic_vector(127 downto 0);
    signal keyd_q_w      : std_logic_vector(127 downto 0);
    signal keyd_prev_raw_q   : std_logic_vector(2 downto 0) := (others => '1');
    signal keyd_prev_clean_q : std_logic_vector(2 downto 0) := (others => '1');
    signal keyd_prev_event_q : std_logic_vector(2 downto 0) := (others => '0');
begin
    -- KEY0 is active-low on the FPGA board.
    rst_w <= not KEY(0);

    DBG_MCLK         <= mclk_w;
    DBG_DIVCLK       <= divclk_w;
    DBG_RESET        <= rst_w;
    DBG_MEM_WRITE    <= mem_write_w;
    DBG_MEM_READ     <= mem_read_w;
    DBG_DTCM_WRITE   <= dtcm_write_w;
    DBG_DTCM_READ    <= dtcm_read_w;
    DBG_DATA_ADDR    <= data_addr_w;
    DBG_DATA_WR      <= data_wr_w;
    DBG_DATA_RD      <= data_rd_w;
    DBG_DTCM_ADDR    <= dtcm_addr_w;
    DBG_DTCM_DATA_RD <= dtcm_data_rd_w;

    ---------------------------------------------------------------------------
    -- Clock tree
    -- Lecturer note: MCLK, DIVCLK/ACCELCLK and SMCLK use separate PLL
    -- instances, all driven from the physical 50-MHz board clock.
    --
    -- MCLK PLL remains inside RV32I_CORE.
    -- DIVCLK PLL: 50 MHz output (1/1) for the fast accelerator clock.
    -- SMCLK PLL: same ratio as MCLK by default; it is a separate PLL instance.
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Final project clocking:
    -- MCLK is generated inside RV32I_CORE with G_PLL_DIV/G_PLL_MUL (25 MHz
    -- with the supplied package values).  The divider accelerator uses its
    -- own 50-MHz PLL (1/1), as required by the lecturer, so DIVCLK is faster
    -- than MCLK and the CDC synchronizers in RV32I_CORE are actually used.
    ---------------------------------------------------------------------------
    FPGA_AUX_CLOCKS_G : if MODELSIM = 0 generate
        DIVCLK_PLL : entity work.PLL
            generic map(
                PLL_DIV => 1,
                PLL_MUL => 1
            )
            port map(
                areset => rst_w,
                inclk0 => CLOCK_50,
                c0     => divclk_w,
                locked => open
            );

        SMCLK_PLL : entity work.PLL
            generic map(
                PLL_DIV => G_PLL_DIV,
                PLL_MUL => G_PLL_MUL
            )
            port map(
                areset => rst_w,
                inclk0 => CLOCK_50,
                c0     => smclk_w,
                locked => open
            );
    end generate;

    SIM_AUX_CLOCKS_G : if MODELSIM /= 0 generate
        -- Functional simulation uses the testbench clock directly.
        -- CDC is still present in the core; the faster physical DIVCLK is
        -- realized by the separate PLL in the FPGA configuration above.
        divclk_w <= CLOCK_50;
        smclk_w  <= CLOCK_50;
    end generate;

    CPU : entity work.RV32I_CORE
        generic map(
            MODELSIM => MODELSIM
        )
        port map(
            rst_i            => rst_w,
            clk_i            => CLOCK_50,
            divclk_i         => divclk_w,
            data_addr_o      => data_addr_w,
            data_wr_o        => data_wr_w,
            data_rd_i        => data_rd_w,
            mem_read_o       => mem_read_w,
            mem_write_o      => mem_write_w,
            mclk_o           => mclk_w,
            intr_i           => intc_intr_w,
            irq_type_i       => intc_type_dbg_w,
            inta_o           => cpu_inta_w,
            gie_o            => cpu_gie_w,
            pc_o             => DBG_PC,
            instruction_o    => DBG_INSTRUCTION,
            RegWrite_ctrl_o  => x8dbg_regwrite_w,
            MemWrite_ctrl_o  => open,
            Branch_ctrl_o    => brdbg_branch_w,
            read_data1_o     => muldbg_rd1_w,
            read_data2_o     => muldbg_rd2_w,
            write_data_o     => muldbg_write_data_w,
            alu_res_o        => muldbg_alu_res_w,
            brTaken_o        => brdbg_taken_w,
            mclk_cnt_o       => mclk_cnt_dbg_w,
            irq_state_o      => cpu_irq_state_w,
            irq_type_reg_o   => cpu_irq_type_reg_w,
            irq_return_pc_o  => cpu_irq_return_pc_w,
            tp_o             => cpu_tp_w,
            dbg_irq_pc_hold_o => dbg_irq_hold_w,
            dbg_div_pc_hold_o => dbg_div_hold_w,
            dbg_divrem_o      => dbg_divrem_w,
            dbg_div_state_o   => dbg_divstate_w,
            dbg_jal_o         => dbg_jal_w,
            dbg_jalr_o        => dbg_jalr_w,
            dbg_branch_o      => dbg_branch_w,
            dbg_brtaken_o     => dbg_brtaken_w,
            dbg_irq_jump_o    => dbg_irq_jump_w,
            dbg_alu_res_o     => dbg_alu_res_w
        );

    ---------------------------------------------------------------------------
    -- MUL TRACE
    --
    -- A MUL instruction is:
    -- opcode = 0110011, funct7 = 0000001, funct3 = 000.
    --
    -- For each of the first 8 MUL instructions, MULDBG stores 6 words:
    --   word +0 : rs1 value / multiplier A
    --   word +1 : rs2 value / multiplier B
    --   word +2 : CORE alu_res_o (MUL result path)
    --   word +3 : CORE write_data_o
    --   word +4 : PC (zero extended)
    --   word +5 : instruction
    --
    -- Slot 0 therefore occupies MULDBG word addresses 0..5,
    -- slot 1 occupies 6..11, etc.
    ---------------------------------------------------------------------------
    muldbg_is_mul_w <= '1' when
        DBG_INSTRUCTION(6 downto 0)   = "0110011" and
        DBG_INSTRUCTION(31 downto 25) = "0000001" and
        DBG_INSTRUCTION(14 downto 12) = "000"
        else '0';

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            muldbg_cap_a_q    <= (others => '0');
            muldbg_cap_b_q    <= (others => '0');
            muldbg_cap_res_q  <= (others => '0');
            muldbg_cap_wb_q   <= (others => '0');
            muldbg_cap_pc_q   <= (others => '0');
            muldbg_cap_inst_q <= (others => '0');
            muldbg_slot_q     <= (others => '0');
            muldbg_state_q    <= (others => '0');

        elsif rising_edge(mclk_w) then
            if muldbg_state_q = "000" then
                if muldbg_is_mul_w = '1' then
                    muldbg_cap_a_q    <= muldbg_rd1_w;
                    muldbg_cap_b_q    <= muldbg_rd2_w;
                    muldbg_cap_res_q  <= muldbg_alu_res_w;
                    muldbg_cap_wb_q   <= muldbg_write_data_w;
                    muldbg_cap_pc_q   <= (31 downto DBG_PC'length => '0') & DBG_PC;
                    muldbg_cap_inst_q <= DBG_INSTRUCTION;
                    muldbg_state_q    <= "001";
                end if;
            elsif muldbg_state_q = "110" then
                muldbg_state_q <= "000";
                if muldbg_slot_q /= "111" then
                    muldbg_slot_q <= muldbg_slot_q + 1;
                end if;
            else
                muldbg_state_q <= muldbg_state_q + 1;
            end if;
        end if;
    end process;

    process(muldbg_state_q, muldbg_slot_q,
            muldbg_cap_a_q, muldbg_cap_b_q, muldbg_cap_res_q,
            muldbg_cap_wb_q, muldbg_cap_pc_q, muldbg_cap_inst_q)
        variable base_v : unsigned(5 downto 0);
    begin
        muldbg_wren_w <= '0';
        muldbg_addr_w <= (others => '0');
        muldbg_data_w <= (others => '0');

        base_v := to_unsigned(to_integer(muldbg_slot_q) * 6, 6);

        case muldbg_state_q is
            when "001" =>
                muldbg_wren_w <= '1';
                muldbg_addr_w <= std_logic_vector(base_v + 0);
                muldbg_data_w <= muldbg_cap_a_q;
            when "010" =>
                muldbg_wren_w <= '1';
                muldbg_addr_w <= std_logic_vector(base_v + 1);
                muldbg_data_w <= muldbg_cap_b_q;
            when "011" =>
                muldbg_wren_w <= '1';
                muldbg_addr_w <= std_logic_vector(base_v + 2);
                muldbg_data_w <= muldbg_cap_res_q;
            when "100" =>
                muldbg_wren_w <= '1';
                muldbg_addr_w <= std_logic_vector(base_v + 3);
                muldbg_data_w <= muldbg_cap_wb_q;
            when "101" =>
                muldbg_wren_w <= '1';
                muldbg_addr_w <= std_logic_vector(base_v + 4);
                muldbg_data_w <= muldbg_cap_pc_q;
            when "110" =>
                muldbg_wren_w <= '1';
                muldbg_addr_w <= std_logic_vector(base_v + 5);
                muldbg_data_w <= muldbg_cap_inst_q;
            when others =>
                null;
        end case;
    end process;

    FPGA_MULDBG_G : if MODELSIM = 0 generate
    begin
        MULDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 32,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = MULDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => muldbg_wren_w,
                clock0    => mclk_w,
                address_a => muldbg_addr_w,
                data_a    => muldbg_data_w,
                q_a       => muldbg_q_w
            );
    end generate FPGA_MULDBG_G;

    ---------------------------------------------------------------------------
    -- DTCM STORE TRACE
    --
    -- This does NOT modify the CPU/DTCM bus. It only observes every write to
    -- the DTCM region and stores a 128-bit snapshot in a separate RAM.
    --
    -- In ISMCE, SWDBG entry 0 is the first DTCM store, entry 1 the second, etc.
    ---------------------------------------------------------------------------
    swdbg_wren_w <= '1' when (mem_write_w = '1' and dtcm_cs_w = '1') else '0';
    swdbg_addr_w <= std_logic_vector(swdbg_slot_q);
    swdbg_data_w <= ((31 downto DBG_PC'length => '0') & DBG_PC) &
                    DBG_INSTRUCTION &
                    data_addr_w &
                    data_wr_w;

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            swdbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if swdbg_wren_w = '1' and swdbg_slot_q /= "11111" then
                swdbg_slot_q <= swdbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_SWDBG_G : if MODELSIM = 0 generate
    begin
        SWDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 128,
                widthad_a              => 5,
                numwords_a             => 32,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = SWDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => swdbg_wren_w,
                clock0    => mclk_w,
                address_a => swdbg_addr_w,
                data_a    => swdbg_data_w,
                q_a       => swdbg_q_w
            );
    end generate FPGA_SWDBG_G;

    ---------------------------------------------------------------------------
    -- LOOP-TAIL / BRANCH DEBUG
    -- Capture only the PCs involved in the final pointer increment, loop
    -- counter decrement, loop branch, and terminal self-loop.
    ---------------------------------------------------------------------------
    brdbg_pc32_w  <= (31 downto DBG_PC'length => '0') & DBG_PC;
    brdbg_flags_w <= (31 downto 2 => '0') & brdbg_taken_w & brdbg_branch_w;

    brdbg_wren_w <= '1' when
        (DBG_PC = std_logic_vector(to_unsigned(16#5C#, DBG_PC'length)) or
         DBG_PC = std_logic_vector(to_unsigned(16#60#, DBG_PC'length)) or
         DBG_PC = std_logic_vector(to_unsigned(16#64#, DBG_PC'length)) or
         DBG_PC = std_logic_vector(to_unsigned(16#68#, DBG_PC'length)))
        and brdbg_slot_q /= "11111"
        else '0';

    brdbg_addr_w <= std_logic_vector(brdbg_slot_q);

    brdbg_data_w <= brdbg_pc32_w &
                    DBG_INSTRUCTION &
                    muldbg_rd1_w &
                    muldbg_rd2_w &
                    muldbg_alu_res_w &
                    muldbg_write_data_w &
                    brdbg_flags_w &
                    x"00000000";

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            brdbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if brdbg_wren_w = '1' then
                brdbg_slot_q <= brdbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_BRDBG_G : if MODELSIM = 0 generate
    begin
        BRDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 256,
                widthad_a              => 5,
                numwords_a             => 32,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = BRDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => brdbg_wren_w,
                clock0    => mclk_w,
                address_a => brdbg_addr_w,
                data_a    => brdbg_data_w,
                q_a       => brdbg_q_w
            );
    end generate FPGA_BRDBG_G;

    ---------------------------------------------------------------------------
    -- X8 / s0 REGISTER-WRITE TRACE
    -- Observes only; does not alter CPU control or register-file behavior.
    ---------------------------------------------------------------------------
    x8dbg_pc32_w <= (31 downto DBG_PC'length => '0') & DBG_PC;
    x8dbg_info_w <= (31 downto 6 => '0') &
                    DBG_INSTRUCTION(11 downto 7) &
                    x8dbg_regwrite_w;

    x8dbg_wren_w <= '1' when
        (x8dbg_regwrite_w = '1' and DBG_INSTRUCTION(11 downto 7) = "01000"
         and x8dbg_slot_q /= "1111")
        else '0';

    x8dbg_addr_w <= std_logic_vector(x8dbg_slot_q);

    x8dbg_data_w <= x8dbg_pc32_w &
                    DBG_INSTRUCTION &
                    muldbg_write_data_w &
                    x8dbg_info_w;

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            x8dbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if x8dbg_wren_w = '1' then
                x8dbg_slot_q <= x8dbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_X8DBG_G : if MODELSIM = 0 generate
    begin
        X8DBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 128,
                widthad_a              => 4,
                numwords_a             => 16,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = X8DBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => x8dbg_wren_w,
                clock0    => mclk_w,
                address_a => x8dbg_addr_w,
                data_a    => x8dbg_data_w,
                q_a       => x8dbg_q_w
            );
    end generate FPGA_X8DBG_G;

    ---------------------------------------------------------------------------
    -- MEMORY-MAPPED I/O STORE TRACE
    -- Captures only writes to 0x2000..0x200D:
    -- LEDR + HEX0..HEX5.  This block observes the bus only and does not change
    -- CPU/GPIO behavior.
    ---------------------------------------------------------------------------
    iodbg_pc32_w <= (31 downto DBG_PC'length => '0') & DBG_PC;

    iodbg_wren_w <= '1' when
        (mem_write_w = '1' and
         unsigned(data_addr_w) >= to_unsigned(16#2000#, data_addr_w'length) and
         unsigned(data_addr_w) <= to_unsigned(16#200D#, data_addr_w'length) and
         iodbg_slot_q /= "111111")
        else '0';

    iodbg_addr_w <= std_logic_vector(iodbg_slot_q);

    iodbg_data_w <= iodbg_pc32_w &
                    DBG_INSTRUCTION &
                    data_addr_w &
                    data_wr_w;

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            iodbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if iodbg_wren_w = '1' then
                iodbg_slot_q <= iodbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_IODBG_G : if MODELSIM = 0 generate
    begin
        IODBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 128,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = IODBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => iodbg_wren_w,
                clock0    => mclk_w,
                address_a => iodbg_addr_w,
                data_a    => iodbg_data_w,
                q_a       => iodbg_q_w
            );
    end generate FPGA_IODBG_G;

    ---------------------------------------------------------------------------
    -- ACTUAL HEX PIN OUTPUT TRACE
    -- Observes HEX0..HEX5 after gpio_hex6 + seven-segment decode.
    -- It does not alter the HEX outputs.
    ---------------------------------------------------------------------------
    hexdbg_now_w <= HEX5 & HEX4 & HEX3 & HEX2 & HEX1 & HEX0;

    hexdbg_wren_w <= '1' when
        (hexdbg_now_w /= hexdbg_prev_q and hexdbg_slot_q /= "111111")
        else '0';

    hexdbg_addr_w <= std_logic_vector(hexdbg_slot_q);

    hexdbg_data_w <= HEX5 & HEX4 & HEX3 & HEX2 & HEX1 & HEX0 &
                     (21 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            hexdbg_prev_q <= (others => '0');
            hexdbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if hexdbg_now_w /= hexdbg_prev_q then
                hexdbg_prev_q <= hexdbg_now_w;
                if hexdbg_slot_q /= "111111" then
                    hexdbg_slot_q <= hexdbg_slot_q + 1;
                end if;
            end if;
        end if;
    end process;

    FPGA_HEXDBG_G : if MODELSIM = 0 generate
    begin
        HEXDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 64,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = HEXDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => hexdbg_wren_w,
                clock0    => mclk_w,
                address_a => hexdbg_addr_w,
                data_a    => hexdbg_data_w,
                q_a       => hexdbg_q_w
            );
    end generate FPGA_HEXDBG_G;

    ---------------------------------------------------------------------------
    -- LOOPDBG: SW0/SW1 decision path trace for test1
    -- [255:224] PC
    -- [223:192] instruction
    -- [191:160] read_data1
    -- [159:128] read_data2
    -- [127:96]  data_rd_w
    -- [95:64]   write_data
    -- [63:32]   ALU result
    -- [31] RegWrite [30] Branch [29] brTaken [28] MemRead [27] MemWrite
    ---------------------------------------------------------------------------
    loopdbg_addr_w <= std_logic_vector(loopdbg_slot_q);

    loopdbg_wren_w <= '1' when
        (SW /= "0000000000" and
         (DBG_PC = std_logic_vector(to_unsigned(16#14#, DBG_PC'length)) or
          DBG_PC = std_logic_vector(to_unsigned(16#18#, DBG_PC'length)) or
          DBG_PC = std_logic_vector(to_unsigned(16#1C#, DBG_PC'length)) or
          DBG_PC = std_logic_vector(to_unsigned(16#20#, DBG_PC'length)) or
          DBG_PC = std_logic_vector(to_unsigned(16#3C#, DBG_PC'length)) or
          DBG_PC = std_logic_vector(to_unsigned(16#40#, DBG_PC'length)) or
          DBG_PC = std_logic_vector(to_unsigned(16#44#, DBG_PC'length)) or
          DBG_PC = std_logic_vector(to_unsigned(16#48#, DBG_PC'length))) and
         loopdbg_slot_q /= "111111")
        else '0';

    loopdbg_data_w <= ((31 downto DBG_PC'length => '0') & DBG_PC) &
                      DBG_INSTRUCTION &
                      muldbg_rd1_w &
                      muldbg_rd2_w &
                      data_rd_w &
                      muldbg_write_data_w &
                      muldbg_alu_res_w &
                      x8dbg_regwrite_w &
                      brdbg_branch_w &
                      brdbg_taken_w &
                      mem_read_w &
                      mem_write_w &
                      (26 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            loopdbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if loopdbg_wren_w = '1' then
                loopdbg_slot_q <= loopdbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_LOOPDBG_G : if MODELSIM = 0 generate
    begin
        LOOPDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 256,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = LOOPDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => loopdbg_wren_w,
                clock0    => mclk_w,
                address_a => loopdbg_addr_w,
                data_a    => loopdbg_data_w,
                q_a       => loopdbg_q_w
            );
    end generate FPGA_LOOPDBG_G;

    ---------------------------------------------------------------------------
    -- PCDBG: redirect controls while PC is 0x14
    -- [255:224] PC, [223:192] instruction, [191:160] ALU result
    -- [159] Jal [158] Jalr [157] Branch [156] brTaken [155] irq_jump
    -- [154] irq_hold [153] div_hold [152:151] DIVREM [150:148] div_state
    ---------------------------------------------------------------------------
    pcdbg_addr_w <= std_logic_vector(pcdbg_slot_q);
    pcdbg_wren_w <= '1' when
        (DBG_PC = std_logic_vector(to_unsigned(16#14#, DBG_PC'length)) and
         pcdbg_slot_q /= "111111") else '0';

    pcdbg_data_w <= ((31 downto DBG_PC'length => '0') & DBG_PC) &
                    DBG_INSTRUCTION & dbg_alu_res_w &
                    dbg_jal_w & dbg_jalr_w & dbg_branch_w & dbg_brtaken_w &
                    dbg_irq_jump_w & dbg_irq_hold_w & dbg_div_hold_w &
                    dbg_divrem_w & dbg_divstate_w & (147 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w='1' then pcdbg_slot_q <= (others=>'0');
        elsif rising_edge(mclk_w) then
            if pcdbg_wren_w='1' then pcdbg_slot_q <= pcdbg_slot_q + 1; end if;
        end if;
    end process;

    FPGA_PCDBG_G : if MODELSIM = 0 generate
    begin
        PCDBG_RAM : altsyncram
          generic map(operation_mode=>"SINGLE_PORT", width_a=>256, widthad_a=>6,
                      numwords_a=>64,
                      lpm_hint=>"ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = PCDBG",
                      lpm_type=>"altsyncram", outdata_reg_a=>"UNREGISTERED",
                      intended_device_family=>"Cyclone V", ram_block_type=>"AUTO")
          port map(wren_a=>pcdbg_wren_w, clock0=>mclk_w, address_a=>pcdbg_addr_w,
                   data_a=>pcdbg_data_w, q_a=>pcdbg_q_w);
    end generate FPGA_PCDBG_G;

    ---------------------------------------------------------------------------
    -- PC HOLD DIAGNOSTIC TRACE
    -- Captures cycles while PC=0x14, plus any cycle where either hold is active.
    --
    -- Record [127:0]:
    -- [127:96] PC
    -- [95:64]  instruction
    -- [63]     irq_pc_hold
    -- [62]     div_pc_hold
    -- [61:60]  DIVREM ctrl
    -- [59:57]  divider FSM state
    -- [56:0]   reserved
    ---------------------------------------------------------------------------
    holddbg_addr_w <= std_logic_vector(holddbg_slot_q);

    holddbg_wren_w <= '1' when
        ((DBG_PC = std_logic_vector(to_unsigned(16#14#, DBG_PC'length)) or
          dbg_irq_hold_w = '1' or dbg_div_hold_w = '1') and
         holddbg_slot_q /= "111111")
        else '0';

    holddbg_data_w <= ((31 downto DBG_PC'length => '0') & DBG_PC) &
                      DBG_INSTRUCTION &
                      dbg_irq_hold_w &
                      dbg_div_hold_w &
                      dbg_divrem_w &
                      dbg_divstate_w &
                      (56 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            holddbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if holddbg_wren_w = '1' then
                holddbg_slot_q <= holddbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_HOLDDBG_G : if MODELSIM = 0 generate
    begin
        HOLDDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 128,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = HOLDDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => holddbg_wren_w,
                clock0    => mclk_w,
                address_a => holddbg_addr_w,
                data_a    => holddbg_data_w,
                q_a       => holddbg_q_w
            );
    end generate FPGA_HOLDDBG_G;

    ---------------------------------------------------------------------------
    -- UNIFIED CPU MEMORY PATH TRACE
    -- Captures only accesses in the MMIO region 0x2000..0x2014 so we see
    -- both switch reads and LED/HEX writes in one ordered trace.
    ---------------------------------------------------------------------------
    pathdbg_pc32_w <= (31 downto DBG_PC'length => '0') & DBG_PC;
    pathdbg_addr_w <= std_logic_vector(pathdbg_slot_q);

    pathdbg_wren_w <= '1' when
        (unsigned(data_addr_w) >= to_unsigned(16#2000#, data_addr_w'length) and
         unsigned(data_addr_w) <= to_unsigned(16#2014#, data_addr_w'length) and
         (mem_read_w = '1' or mem_write_w = '1') and
         pathdbg_slot_q /= "111111")
        else '0';

    pathdbg_data_w <= pathdbg_pc32_w &
                      DBG_INSTRUCTION &
                      data_addr_w &
                      data_rd_w &
                      data_wr_w &
                      mem_read_w &
                      mem_write_w &
                      (93 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            pathdbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if pathdbg_wren_w = '1' then
                pathdbg_slot_q <= pathdbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_PATHDBG_G : if MODELSIM = 0 generate
    begin
        PATHDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 256,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = PATHDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => pathdbg_wren_w,
                clock0    => mclk_w,
                address_a => pathdbg_addr_w,
                data_a    => pathdbg_data_w,
                q_a       => pathdbg_q_w
            );
    end generate FPGA_PATHDBG_G;

    ---------------------------------------------------------------------------
    -- DIRECT PHYSICAL SWITCH TRACE
    -- Entry 0 is written after reset; subsequent entries are written whenever
    -- SW[9:0] changes. Observation-only: it does not alter CPU/GPIO behavior.
    ---------------------------------------------------------------------------
    swpindbg_addr_w <= std_logic_vector(swpindbg_cnt_q);
    swpindbg_data_w <= (31 downto 10 => '0') & SW;

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            swpindbg_cnt_q  <= (others => '0');
            swpindbg_prev_q <= (others => '0');
            swpindbg_wren_w <= '0';
        elsif rising_edge(mclk_w) then
            swpindbg_wren_w <= '0';
            if SW /= swpindbg_prev_q then
                swpindbg_prev_q <= SW;
                if swpindbg_cnt_q /= "111111" then
                    swpindbg_wren_w <= '1';
                    swpindbg_cnt_q <= swpindbg_cnt_q + 1;
                end if;
            end if;
        end if;
    end process;

    FPGA_SWPINDBG_G : if MODELSIM = 0 generate
    begin
        SWPINDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 32,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = SWPINDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => swpindbg_wren_w,
                clock0    => mclk_w,
                address_a => swpindbg_addr_w,
                data_a    => swpindbg_data_w,
                q_a       => swpindbg_q_w
            );
    end generate FPGA_SWPINDBG_G;

    ---------------------------------------------------------------------------
    -- SWITCH READ TRACE
    -- Captures reads performed by the CPU from PORT_SW = 0x2010.
    -- This is observation-only and does not alter CPU/GPIO behavior.
    ---------------------------------------------------------------------------
    swreaddbg_pc32_w <= (31 downto DBG_PC'length => '0') & DBG_PC;

    swreaddbg_wren_w <= '1' when
        (unsigned(data_addr_w) = to_unsigned(16#2010#, data_addr_w'length) and
         mem_read_w = '1' and
         SW /= "0000000000" and
         swreaddbg_slot_q /= "111111")
        else '0';

    swreaddbg_addr_w <= std_logic_vector(swreaddbg_slot_q);

    swreaddbg_data_w <= data_addr_w &
                        sw_data_rd_w &
                        data_rd_w &
                        SW &
                        mem_read_w &
                        (20 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            swreaddbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if swreaddbg_wren_w = '1' then
                swreaddbg_slot_q <= swreaddbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_SWREADDBG_G : if MODELSIM = 0 generate
    begin
        SWREADDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 128,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = SWREADDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => swreaddbg_wren_w,
                clock0    => mclk_w,
                address_a => swreaddbg_addr_w,
                data_a    => swreaddbg_data_w,
                q_a       => swreaddbg_q_w
            );
    end generate FPGA_SWREADDBG_G;

    ---------------------------------------------------------------------------
    -- EXACT INTERRUPT-CONTROLLER BUS TRACE (INTBUS)
    ---------------------------------------------------------------------------
    intbus_addr_w <= std_logic_vector(intbus_slot_q);

    intbus_wren_w <= '1' when
        (unsigned(data_addr_w) >= to_unsigned(16#202C#, data_addr_w'length) and
         unsigned(data_addr_w) <= to_unsigned(16#202E#, data_addr_w'length) and
         (mem_read_w = '1' or mem_write_w = '1') and
         intbus_slot_q /= "111111")
        else '0';

    intbus_data_w <=
        ((31 downto DBG_PC'length => '0') & DBG_PC) &
        DBG_INSTRUCTION &
        data_addr_w &
        data_wr_w &
        data_rd_w &
        intc_ie_dbg_w &
        intc_ifg_dbg_w &
        intc_type_dbg_w &
        mem_read_w &
        mem_write_w &
        intc_cs_w &
        intc_intr_w &
        cpu_inta_w &
        cpu_gie_w &
        cpu_irq_state_w &
        cpu_irq_type_reg_w &
        intc_key_event_dbg_w &
        (52 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            intbus_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if intbus_wren_w = '1' then
                intbus_slot_q <= intbus_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_INTBUS_G : if MODELSIM = 0 generate
    begin
        INTBUS_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 256,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = INTBUS",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => intbus_wren_w,
                clock0    => mclk_w,
                address_a => intbus_addr_w,
                data_a    => intbus_data_w,
                q_a       => intbus_q_w
            );
    end generate FPGA_INTBUS_G;

    ---------------------------------------------------------------------------
    -- INTERRUPT / BASIC TIMER TRACE
    -- Capture whenever timer/interrupt activity exists, and also a few early
    -- setup cycles while IE is non-zero.
    ---------------------------------------------------------------------------
    intdbg_addr_w <= std_logic_vector(intdbg_slot_q);

    intdbg_wren_w <= '1' when
        ((timer_irq_w = '1' or
          intc_ifg_dbg_w /= x"00" or
          intc_intr_w = '1' or
          cpu_inta_w = '1') and
         intdbg_slot_q /= "111111")
        else '0';

    intdbg_data_w <=
        ((31 downto DBG_PC'length => '0') & DBG_PC) &
        DBG_INSTRUCTION &
        intc_ie_dbg_w &
        intc_ifg_dbg_w &
        intc_type_dbg_w &
        timer_irq_w &
        intc_intr_w &
        cpu_inta_w &
        cpu_gie_w &
        cpu_irq_state_w &
        cpu_irq_type_reg_w &
        (25 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            intdbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if intdbg_wren_w = '1' then
                intdbg_slot_q <= intdbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_INTDBG_G : if MODELSIM = 0 generate
    begin
        INTDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 128,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = INTDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => intdbg_wren_w,
                clock0    => mclk_w,
                address_a => intdbg_addr_w,
                data_a    => intdbg_data_w,
                q_a       => intdbg_q_w
            );
    end generate FPGA_INTDBG_G;

    ---------------------------------------------------------------------------
    -- BTDBG: continuously overwrite address 0 with the current timer state.
    -- Read instance BTDBG in In-System Memory Content Editor after waiting.
    ---------------------------------------------------------------------------
    btdbg_data_w <=
        btcnt_dbg_w &
        btcmpr0_dbg_w &
        btctl1_dbg_w &
        btctl2_dbg_w &
        equ0_dbg_w &
        timer_btifg_w &
        timer_irq_w &
        timer_irq_clear_w &
        timer_cs_w &
        mem_write_w &
        mem_read_w &
        data_addr_w &
        (8 downto 0 => '0');

    FPGA_BTDBG_G : if MODELSIM = 0 generate
    begin
        BTDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 128,
                widthad_a              => 1,
                numwords_a             => 2,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = BTDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => '1',
                clock0    => smclk_w,
                address_a => "0",
                data_a    => btdbg_data_w,
                q_a       => btdbg_q_w
            );
    end generate FPGA_BTDBG_G;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- BASIC TIMER CDC BRIDGE
    -- Lossless asynchronous FIFO: CPU writes in MCLK domain, timer consumes
    -- them in SMCLK domain.  This preserves back-to-back writes such as
    -- BTCTL1 -> BTCMPR0 -> BTCTL1.
    ---------------------------------------------------------------------------

    -- Synchronize read pointer into MCLK domain.
    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            timer_rgray_m1_q <= (others => '0');
            timer_rgray_m2_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            timer_rgray_m1_q <= timer_rgray_q;
            timer_rgray_m2_q <= timer_rgray_m1_q;
        end if;
    end process;

    -- FIFO full when next write Gray pointer equals synchronized read pointer
    -- with the two MSBs inverted (standard async-FIFO full test).
    process(timer_wbin_q, timer_rgray_m2_q)
        variable next_bin_v  : unsigned(3 downto 0);
        variable next_gray_v : unsigned(3 downto 0);
        variable full_cmp_v  : unsigned(3 downto 0);
    begin
        next_bin_v  := timer_wbin_q + 1;
        next_gray_v := next_bin_v xor ('0' & next_bin_v(3 downto 1));
        full_cmp_v  := timer_rgray_m2_q;
        full_cmp_v(3 downto 2) := not timer_rgray_m2_q(3 downto 2);

        if next_gray_v = full_cmp_v then
            timer_fifo_full_m_w <= '1';
        else
            timer_fifo_full_m_w <= '0';
        end if;
    end process;

    -- Enqueue every CPU write to the Basic Timer.
    process(mclk_w, rst_w)
        variable next_bin_v : unsigned(3 downto 0);
    begin
        if rst_w = '1' then
            timer_wbin_q  <= (others => '0');
            timer_wgray_q <= (others => '0');
            timer_clr_toggle_m_q <= '0';
        elsif rising_edge(mclk_w) then
            if timer_cs_w = '1' and mem_write_w = '1' and
               timer_fifo_full_m_w = '0' then
                timer_fifo_q(to_integer(timer_wbin_q(2 downto 0))) <=
                    data_addr_w & data_wr_w;
                next_bin_v := timer_wbin_q + 1;
                timer_wbin_q  <= next_bin_v;
                timer_wgray_q <= next_bin_v xor ('0' & next_bin_v(3 downto 1));
            end if;

            if timer_irq_clear_w = '1' then
                timer_clr_toggle_m_q <= not timer_clr_toggle_m_q;
            end if;
        end if;
    end process;

    -- Synchronize write pointer into SMCLK domain.
    process(smclk_w, rst_w)
    begin
        if rst_w = '1' then
            timer_wgray_s1_q <= (others => '0');
            timer_wgray_s2_q <= (others => '0');
        elsif rising_edge(smclk_w) then
            timer_wgray_s1_q <= timer_wgray_q;
            timer_wgray_s2_q <= timer_wgray_s1_q;
        end if;
    end process;

    timer_fifo_empty_s_w <= '1' when timer_rgray_q = timer_wgray_s2_q else '0';

    -- Dequeue exactly one queued write per SMCLK cycle.
    process(smclk_w, rst_w)
        variable next_bin_v : unsigned(3 downto 0);
    begin
        if rst_w = '1' then
            timer_rbin_q       <= (others => '0');
            timer_rgray_q      <= (others => '0');
            timer_wr_pulse_s_w <= '0';
            timer_bus_addr_s_w <= (others => '0');
            timer_bus_data_s_w <= (others => '0');
        elsif rising_edge(smclk_w) then
            timer_wr_pulse_s_w <= '0';

            if timer_fifo_empty_s_w = '0' then
                timer_bus_addr_s_w <=
                    timer_fifo_q(to_integer(timer_rbin_q(2 downto 0)))(63 downto 32);
                timer_bus_data_s_w <=
                    timer_fifo_q(to_integer(timer_rbin_q(2 downto 0)))(31 downto 0);
                timer_wr_pulse_s_w <= '1';

                next_bin_v := timer_rbin_q + 1;
                timer_rbin_q  <= next_bin_v;
                timer_rgray_q <= next_bin_v xor ('0' & next_bin_v(3 downto 1));
            end if;
        end if;
    end process;

    -- Synchronize IRQ-clear event into SMCLK domain.
    process(smclk_w, rst_w)
    begin
        if rst_w = '1' then
            timer_clr_sync1_s_q <= '0';
            timer_clr_sync2_s_q <= '0';
            timer_clr_seen_s_q  <= '0';
        elsif rising_edge(smclk_w) then
            timer_clr_sync1_s_q <= timer_clr_toggle_m_q;
            timer_clr_sync2_s_q <= timer_clr_sync1_s_q;
            timer_clr_seen_s_q  <= timer_clr_sync2_s_q;
        end if;
    end process;

    timer_clr_pulse_s_w <= timer_clr_sync2_s_q xor timer_clr_seen_s_q;

    -- Writes use the stable MCLK-side payload when the synchronized event arrives.
    -- Reads remain combinational because basic_timer data_o is combinational.
    timer_bus_wr_s_w <= timer_wr_pulse_s_w;
    timer_bus_rd_s_w <= timer_cs_w and mem_read_w;
    timer_bus_cs_s_w <= timer_wr_pulse_s_w or (timer_cs_w and mem_read_w);

    ---------------------------------------------------------------------------
    -- TWRDBG: capture every CPU write to Basic Timer register space.
    -- This observes the MCLK-side bus before CDC.
    ---------------------------------------------------------------------------
    twrdbg_addr_w <= std_logic_vector(twrdbg_slot_q);

    twrdbg_wren_w <= '1' when
        (mem_write_w = '1' and
         unsigned(data_addr_w) >= to_unsigned(16#201C#, data_addr_w'length) and
         unsigned(data_addr_w) <= to_unsigned(16#2028#, data_addr_w'length) and
         twrdbg_slot_q /= "111111")
        else '0';

    twrdbg_data_w <= ((31 downto DBG_PC'length => '0') & DBG_PC) &
                     DBG_INSTRUCTION &
                     data_addr_w &
                     data_wr_w;

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            twrdbg_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if twrdbg_wren_w = '1' then
                twrdbg_slot_q <= twrdbg_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_TWRDBG_G : if MODELSIM = 0 generate
    begin
        TWRDBG_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 128,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = TWRDBG",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => twrdbg_wren_w,
                clock0    => mclk_w,
                address_a => twrdbg_addr_w,
                data_a    => twrdbg_data_w,
                q_a       => twrdbg_q_w
            );
    end generate FPGA_TWRDBG_G;

    -- Memory-mapped I/O address decoder:
    -- look at the CPU's address and figure out which device it's talking
    -- to. Only one of these chip-selects (cs_w) should ever be 1 at a time.
    dtcm_cs_w <= '1' when unsigned(data_addr_w) < unsigned'(x"00002000") else '0'; -- below 0x2000: plain memory
    led_cs_w  <= '1' when data_addr_w = x"00002000" else '0';  -- LEDs
    sw_cs_w   <= '1' when data_addr_w = x"00002010" else '0';  -- switches
    key_cs_w  <= '1' when data_addr_w = x"00002014" else '0';  -- pushbuttons

    timer_cs_w <= '1' when                    -- any of the Basic Timer's registers
        data_addr_w = x"0000201C" or
        data_addr_w = x"0000201D" or
        data_addr_w = x"00002020" or
        data_addr_w = x"00002024" or
        data_addr_w = x"00002028"
        else '0';

    intc_cs_w <= '1' when                     -- any of the Interrupt Controller's registers
        data_addr_w = x"0000202C" or
        data_addr_w = x"0000202D" or
        data_addr_w = x"0000202E"
        else '0';


    hex_cs_w <= '1' when                      -- any of the six HEX display registers
        data_addr_w = x"00002004" or
        data_addr_w = x"00002005" or
        data_addr_w = x"00002008" or
        data_addr_w = x"00002009" or
        data_addr_w = x"0000200C" or
        data_addr_w = x"0000200D"
        else '0';


    ---------------------------------------------------------------------------
    -- IRQ VECTOR FETCH TRACE (IRQV) -- observation only, no functional changes
    -- [255:224] PC
    -- [223:192] instruction
    -- [191:160] CPU data address
    -- [159:128] CPU data read
    -- [127:96]  DTCM data read
    -- [95:88]   latched IRQ TYPE
    -- [87:86]   IRQ state
    -- [85]      INTA
    -- [84]      GIE
    -- [83]      MemRead
    -- [82]      DTCM chip-select
    -- [81:0]    reserved
    ---------------------------------------------------------------------------
    irqv_addr_w <= std_logic_vector(irqv_slot_q);

    irqv_wren_w <= '1' when
        (cpu_irq_state_w /= "00" and irqv_slot_q /= "111111")
        else '0';

    irqv_data_w <=
        ((31 downto DBG_PC'length => '0') & DBG_PC) &
        DBG_INSTRUCTION &
        data_addr_w &
        data_rd_w &
        dtcm_data_rd_w &
        cpu_irq_type_reg_w &
        cpu_irq_state_w &
        cpu_inta_w &
        cpu_gie_w &
        mem_read_w &
        dtcm_cs_w &
        (81 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            irqv_slot_q <= (others => '0');
        elsif rising_edge(mclk_w) then
            if irqv_wren_w = '1' then
                irqv_slot_q <= irqv_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_IRQV_G : if MODELSIM = 0 generate
    begin
        IRQV_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 256,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = IRQV",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => irqv_wren_w,
                clock0    => mclk_w,
                address_a => irqv_addr_w,
                data_a    => irqv_data_w,
                q_a       => irqv_q_w
            );
    end generate FPGA_IRQV_G;

    dtcm_read_w  <= mem_read_w and dtcm_cs_w;   -- a real DTCM read: it's a load, AND the address is in range
    dtcm_write_w <= mem_write_w and dtcm_cs_w;  -- a real DTCM write: it's a store, AND the address is in range
    hex_write_w  <= mem_write_w and hex_cs_w;   -- a real HEX write

    -- Full CPU byte address -> DTCM 32-bit word index
    dtcm_addr_w <= data_addr_w(DTCM_ADDR_WIDTH+1 downto 2);
    -- drop the low 2 bits (they just pick a byte within a word) to get a word index

    DTCM : entity work.dmemory
        generic map(
            DATA_BUS_WIDTH  => DATA_BUS_WIDTH,
            DTCM_ADDR_WIDTH => DTCM_ADDR_WIDTH,
            WORDS_NUM       => G_DATA_WORDSNUM
        )
        port map(
            clk_i            => mclk_w,
            rst_i            => rst_w,
            dtcm_addr_i      => dtcm_addr_w,
            dtcm_data_wr_i   => data_wr_w,
            MemRead_ctrl_i   => dtcm_read_w,
            MemWrite_ctrl_i  => dtcm_write_w,
            dtcm_data_rd_o   => dtcm_data_rd_w
        );

    LED_GPIO : entity work.gpio_led
        port map(
            clk_i       => mclk_w,
            rst_i       => rst_w,
            cs_i        => led_cs_w,
            mem_write_i => mem_write_w,
            data_i      => data_wr_w,
            led_o       => led8_w,
            data_o      => led_data_rd_w
        );

        ---------------------------------------------------------------------------
    -- SWITCH READ PATH FIX
    -- Directly expose the physical 10 slide switches to the 32-bit MMIO read
    -- data bus.  SW[0] -> bit 0, ... SW[9] -> bit 9.
    ---------------------------------------------------------------------------
    sw_data_rd_w <= (31 downto 10 => '0') & SW;


    ---------------------------------------------------------------------------
    -- KEYD direct key-path trace (observation only)
    -- [127:96] PC
    -- [95:64]  instruction
    -- [63:61]  physical KEY(3 downto 1)  (active-low)
    -- [60:58]  key_clean_w
    -- [57:55]  intc_key_event_dbg_w
    -- [54:47]  IFG
    -- [46:39]  IE
    -- [38]     INTR
    -- [37]     INTA
    -- [36]     GIE
    -- [35:0]   reserved
    ---------------------------------------------------------------------------
    keyd_addr_w <= std_logic_vector(keyd_slot_q);

    keyd_wren_w <= '1' when
        (keyd_slot_q /= "111111" and
         (KEY(3 downto 1) /= keyd_prev_raw_q or
          key_clean_w /= keyd_prev_clean_q or
          intc_key_event_dbg_w /= keyd_prev_event_q))
        else '0';

    keyd_data_w <=
        ((31 downto DBG_PC'length => '0') & DBG_PC) &
        DBG_INSTRUCTION &
        KEY(3 downto 1) &
        key_clean_w &
        intc_key_event_dbg_w &
        intc_ifg_dbg_w &
        intc_ie_dbg_w &
        intc_intr_w &
        cpu_inta_w &
        cpu_gie_w &
        (35 downto 0 => '0');

    process(mclk_w, rst_w)
    begin
        if rst_w = '1' then
            keyd_slot_q       <= (others => '0');
            keyd_prev_raw_q   <= KEY(3 downto 1);
            keyd_prev_clean_q <= key_clean_w;
            keyd_prev_event_q <= intc_key_event_dbg_w;
        elsif rising_edge(mclk_w) then
            keyd_prev_raw_q   <= KEY(3 downto 1);
            keyd_prev_clean_q <= key_clean_w;
            keyd_prev_event_q <= intc_key_event_dbg_w;
            if keyd_wren_w = '1' then
                keyd_slot_q <= keyd_slot_q + 1;
            end if;
        end if;
    end process;

    FPGA_KEYD_G : if MODELSIM = 0 generate
    begin
        KEYD_RAM : altsyncram
            generic map(
                operation_mode         => "SINGLE_PORT",
                width_a                => 128,
                widthad_a              => 6,
                numwords_a             => 64,
                lpm_hint               => "ENABLE_RUNTIME_MOD = YES,INSTANCE_NAME = KEYD",
                lpm_type               => "altsyncram",
                outdata_reg_a          => "UNREGISTERED",
                intended_device_family => "Cyclone V",
                ram_block_type         => "AUTO"
            )
            port map(
                wren_a    => keyd_wren_w,
                clock0    => mclk_w,
                address_a => keyd_addr_w,
                data_a    => keyd_data_w,
                q_a       => keyd_q_w
            );
    end generate FPGA_KEYD_G;

    ---------------------------------------------------------------------------
    -- KEY1..KEY3 debouncing. Board keys are active-low.
    -- For simulation only a few stable cycles are required; on FPGA use
    -- approximately 10 ms at the default 25-MHz MCLK.
    ---------------------------------------------------------------------------
    SIM_KEY_DEBOUNCE_G : if MODELSIM /= 0 generate
        KEY_DEBOUNCE_SIM : entity work.key_debouncer
            generic map(
                WIDTH           => 3,
                DEBOUNCE_CYCLES => 2
            )
            port map(
                clk_i   => mclk_w,
                rst_i   => rst_w,
                raw_i   => KEY(3 downto 1),
                clean_o => key_clean_w
            );
    end generate;

    FPGA_KEY_DEBOUNCE_G : if MODELSIM = 0 generate
        KEY_DEBOUNCE_FPGA : entity work.key_debouncer
            generic map(
                WIDTH           => 3,
                DEBOUNCE_CYCLES => 250000
            )
            port map(
                clk_i   => mclk_w,
                rst_i   => rst_w,
                raw_i   => KEY(3 downto 1),
                clean_o => key_clean_w
            );
    end generate;

    KEY_GPIO : entity work.gpio_key
        port map(
            key_i  => key_clean_w,
            data_o => key_data_rd_w
        );

    BASIC_TIMER : entity work.basic_timer
        port map(
            clk_i       => smclk_w,
            rst_i       => rst_w,
            cs_i        => timer_bus_cs_s_w,
            mem_read_i  => timer_bus_rd_s_w,
            mem_write_i => timer_bus_wr_s_w,
            address_i   => timer_bus_addr_s_w,
            data_i      => timer_bus_data_s_w,
            data_o      => timer_data_rd_w,
            capin1_i    => CAPIN1,
            capin2_i    => CAPIN2,
            pwmout_o    => pwmout_w,
            btctl1_o    => btctl1_dbg_w,
            btctl2_o    => btctl2_dbg_w,
            btcmpr0_o   => btcmpr0_dbg_w,
            btcmpr1_o   => btcmpr1_dbg_w,
            btcapr_o    => btcapr_dbg_w,
            btcnt_o     => btcnt_dbg_w,
            equ0_o      => equ0_dbg_w,
            equ1_o      => equ1_dbg_w,
            capture_event_o => capture_event_dbg_w,
            btifg_o         => timer_btifg_w,
            irq_o           => timer_irq_w,
            irq_clear_i     => timer_clr_pulse_s_w,
            btcl0_o         => btcl0_dbg_w,
            btcl1_o         => btcl1_dbg_w,
            heu0_o          => heu0_dbg_w,
            heu1_o          => heu1_dbg_w
        );

    INTERRUPT_CONTROLLER : entity work.interrupt_controller
        port map(
            clk_i          => mclk_w,
            rst_i          => rst_w,
            cs_i           => intc_cs_w,
            mem_read_i     => mem_read_w,
            mem_write_i    => mem_write_w,
            address_i      => data_addr_w,
            data_i         => data_wr_w,
            data_o         => intc_data_rd_w,
            timer_irq_i     => timer_irq_w,
            key_clean_i     => key_clean_w,
            gie_i           => cpu_gie_w,
            inta_i         => cpu_inta_w,
            intr_o         => intc_intr_w,
            timer_clear_o  => timer_irq_clear_w,
            ie_o           => intc_ie_dbg_w,
            ifg_o          => intc_ifg_dbg_w,
            type_o         => intc_type_dbg_w,
            key_event_o    => intc_key_event_dbg_w
        );

    HEX_GPIO : entity work.gpio_hex6
        port map(
            clk_i       => mclk_w,
            rst_i       => rst_w,
            mem_write_i => hex_write_w,
            address_i   => data_addr_w,
            data_i      => data_wr_w,
            hex0_o      => HEX0,
            hex1_o      => HEX1,
            hex2_o      => HEX2,
            hex3_o      => HEX3,
            hex4_o      => HEX4,
            hex5_o      => HEX5,
            data_o      => hex_data_rd_w
        );

    -- readback mux: whichever device's chip-select is active on a read,
    -- that device's data goes back to the CPU
    data_rd_w <= dtcm_data_rd_w when dtcm_cs_w = '1' and mem_read_w = '1' else
                 led_data_rd_w  when led_cs_w  = '1' and mem_read_w = '1' else
                 sw_data_rd_w   when sw_cs_w   = '1' and mem_read_w = '1' else
                 key_data_rd_w   when key_cs_w   = '1' and mem_read_w = '1' else
                 timer_data_rd_w   when timer_cs_w   = '1' and mem_read_w = '1' else
                 intc_data_rd_w    when intc_cs_w    = '1' and mem_read_w = '1' else
                 hex_data_rd_w     when hex_cs_w     = '1' and mem_read_w = '1' else
                 (others => '0');

    LEDR(7 downto 0) <= led8_w;
    LEDR(9 downto 8) <= (others => '0');
    PWMOUT <= pwmout_w;
end architecture;