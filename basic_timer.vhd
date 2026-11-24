--============================================================================
-- Basic Timer - Stage 3.4
-- Counter + Output Compare PWM + Input Capture
--
-- Address map:
--   BTCTL1  = 0x201C
--   BTCTL2  = 0x201D
--   BTCMPR0 = 0x2020
--   BTCMPR1 = 0x2024
--   BTCAPR  = 0x2028
--
-- BTCTL1:
--   bit 7     BTOUTMD : 0=Set/Reset, 1=Reset/Set
--   bit 6     BTOUTEN : 1=PWM updates enabled, 0=hold PWM value
--   bit 5     BTHOLD  : 1=hold BTCNT
--   bits 4:3  BTSSEL  : 00=/1, 01=/2, 10=/4, 11=/8
--   bit 2     BTCLR   : write-one clear; self-clearing
--   bits 1:0  BTINT   : used later for interrupt-source selection
--
-- BTCTL2:
--   bits 3:2  CAPMD
--       00,11 = capture disabled
--       01    = rising-edge capture
--       10    = falling-edge capture
--   bits 1:0  CAPISEL
--       00 = CAPIN1
--       01 = CAPIN2
--       10 = VCC ('1')
--       11 = GND ('0')
--
-- On a selected capture event:
--   BTCAPR <= BTCNT
--============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity basic_timer is
    port(
        clk_i       : in  std_logic;
        rst_i       : in  std_logic;
        cs_i        : in  std_logic;
        mem_read_i  : in  std_logic;
        mem_write_i : in  std_logic;
        address_i   : in  std_logic_vector(31 downto 0);
        data_i      : in  std_logic_vector(31 downto 0);
        data_o      : out std_logic_vector(31 downto 0);

        capin1_i    : in  std_logic;
        capin2_i    : in  std_logic;
        pwmout_o    : out std_logic;

        -- Debug / verification
        btctl1_o       : out std_logic_vector(7 downto 0);
        btctl2_o       : out std_logic_vector(7 downto 0);
        btcmpr0_o      : out std_logic_vector(31 downto 0);
        btcmpr1_o      : out std_logic_vector(31 downto 0);
        btcapr_o       : out std_logic_vector(31 downto 0);
        btcnt_o        : out std_logic_vector(31 downto 0);
        equ0_o         : out std_logic;
        equ1_o         : out std_logic;
        capture_event_o: out std_logic;
        btifg_o        : out std_logic;
        irq_o          : out std_logic;
        irq_clear_i    : in  std_logic;
        btcl0_o        : out std_logic_vector(31 downto 0);
        btcl1_o        : out std_logic_vector(31 downto 0);
        heu0_o         : out std_logic;
        heu1_o         : out std_logic
    );
end entity;

architecture rtl of basic_timer is
    constant ADDR_BTCTL1  : std_logic_vector(31 downto 0) := x"0000201C";
    constant ADDR_BTCTL2  : std_logic_vector(31 downto 0) := x"0000201D";
    constant ADDR_BTCMPR0 : std_logic_vector(31 downto 0) := x"00002020";
    constant ADDR_BTCMPR1 : std_logic_vector(31 downto 0) := x"00002024";
    constant ADDR_BTCAPR  : std_logic_vector(31 downto 0) := x"00002028";

    signal btctl1_q    : std_logic_vector(7 downto 0)  := (others => '0'); -- BTCTL1 register (mode, enable, prescaler...)
    signal btctl2_q    : std_logic_vector(7 downto 0)  := (others => '0'); -- BTCTL2 register (capture settings)
    signal btcmpr0_q   : std_logic_vector(31 downto 0) := (others => '0'); -- BTCMPR0: sets the PWM period
    signal btcmpr1_q   : std_logic_vector(31 downto 0) := (others => '0'); -- BTCMPR1: sets the PWM duty point
    signal btcl0_q     : std_logic_vector(31 downto 0) := (others => '0'); -- latched copy of BTCMPR0, actually used for compare
    signal btcl1_q     : std_logic_vector(31 downto 0) := (others => '0'); -- latched copy of BTCMPR1, actually used for compare
    signal btcapr_q    : std_logic_vector(31 downto 0) := (others => '0'); -- BTCAPR: last captured counter value
    signal btcnt_q     : unsigned(31 downto 0) := (others => '0');        -- the running counter itself
    signal prescaler_q : unsigned(2 downto 0)  := (others => '0');        -- slows the counter down, per BTSSEL
    signal pwmout_q    : std_logic := '0';                                -- the PWM output pin value

    signal timer_tick_w     : std_logic; -- 1 = counter is allowed to step this cycle (after prescaling)
    signal equ0_w           : std_logic; -- 1 = counter just matched btcl0_q (period end)
    signal equ1_w           : std_logic; -- 1 = counter just matched btcl1_q (duty point)
    signal capture_source_w : std_logic; -- the chosen capture input, picked by CAPISEL
    signal capture_prev_q   : std_logic := '0'; -- capture_source_w from last cycle, to see edges
    signal capture_event_w  : std_logic; -- 1 = the chosen edge (rising/falling) just happened
    signal capture_event_q  : std_logic := '0'; -- capture_event_w, delayed one cycle for other logic to use
    signal btifg_q          : std_logic := '0'; -- the Basic Timer's own pending-interrupt flag
    signal irq_source_w     : std_logic; -- whichever event (EQ0/EQ1/capture) is set as the interrupt trigger
    constant HEU0_C         : std_logic := '1';
    constant HEU1_C         : std_logic := '1';
begin
    ---------------------------------------------------------------------------
    -- Clock divider
    ---------------------------------------------------------------------------
    process(btctl1_q, prescaler_q)
    begin
        case btctl1_q(4 downto 3) is             -- BTSSEL picks how much to slow the counter down
            when "00" =>
                timer_tick_w <= '1';              -- /1: tick every cycle
            when "01" =>
                if prescaler_q(0) = '1' then
                    timer_tick_w <= '1';          -- /2: tick every 2 cycles
                else
                    timer_tick_w <= '0';
                end if;
            when "10" =>
                if prescaler_q(1 downto 0) = "11" then
                    timer_tick_w <= '1';          -- /4: tick every 4 cycles
                else
                    timer_tick_w <= '0';
                end if;
            when others =>
                if prescaler_q = "111" then
                    timer_tick_w <= '1';          -- /8: tick every 8 cycles
                else
                    timer_tick_w <= '0';
                end if;
        end case;
    end process;

    ---------------------------------------------------------------------------
    -- Output-compare comparators
    ---------------------------------------------------------------------------
    equ0_w <= '1' when timer_tick_w = '1' and
                         btcl0_q /= x"00000000" and
                         std_logic_vector(btcnt_q) = btcl0_q
              else '0';
    -- 1 = counter just reached the period value (btcl0_q), and it's not zero/disabled

    equ1_w <= '1' when timer_tick_w = '1' and
                         btcl1_q /= x"00000000" and
                         std_logic_vector(btcnt_q) = btcl1_q
              else '0';
    -- 1 = counter just reached the duty-point value (btcl1_q)

    ---------------------------------------------------------------------------
    -- Capture source selector
    ---------------------------------------------------------------------------
    with btctl2_q(1 downto 0) select                  -- CAPISEL picks the capture source
        capture_source_w <= capin1_i when "00",        -- external pin CAPIN1
                            capin2_i when "01",         -- external pin CAPIN2
                            '1'      when "10",         -- tied high
                            '0'      when others;       -- tied low

    ---------------------------------------------------------------------------
    -- Capture event detector
    ---------------------------------------------------------------------------
    capture_event_w <=
        '1' when btctl2_q(3 downto 2) = "01" and                -- CAPMD = rising edge
                 capture_prev_q = '0' and capture_source_w = '1' -- and we just saw 0 -> 1
        else
        '1' when btctl2_q(3 downto 2) = "10" and                -- CAPMD = falling edge
                 capture_prev_q = '1' and capture_source_w = '0' -- and we just saw 1 -> 0
        else
        '0';

    ---------------------------------------------------------------------------
    -- Timer interrupt-source selector
    -- BTINT = BTCTL1(1 downto 0)
    --   00 -> EQ0
    --   01 -> EQ1
    --   10 -> capture event
    --   11 -> disabled
    ---------------------------------------------------------------------------
    with btctl1_q(1 downto 0) select              -- BTINT picks what raises the interrupt
        irq_source_w <= equ0_w          when "00", -- period-end match
                        equ1_w          when "01", -- duty-point match
                        capture_event_q when "10", -- a capture just happened
                        '0'             when others; -- disabled

    ---------------------------------------------------------------------------
    -- Registers, counter, PWM and capture
    ---------------------------------------------------------------------------
    process(clk_i)
        variable btctl1_write_v : boolean;
        variable clear_v        : boolean;
        variable next_mode_v    : std_logic;
        variable next_enable_v  : std_logic;
    begin
        if rising_edge(clk_i) then
            if rst_i = '1' then
                btctl1_q       <= (others => '0');
                btctl2_q       <= (others => '0');
                btcmpr0_q      <= (others => '0');
                btcmpr1_q      <= (others => '0');
                btcl0_q        <= (others => '0');
                btcl1_q        <= (others => '0');
                btcapr_q       <= (others => '0');
                btcnt_q        <= (others => '0');
                prescaler_q    <= (others => '0');
                pwmout_q       <= '0';
                capture_prev_q  <= '0';
                capture_event_q <= '0';
                btifg_q         <= '0';
            else
                prescaler_q      <= prescaler_q + 1;         -- always counts, feeds the clock divider above
                capture_prev_q   <= capture_source_w;         -- remember it, to catch the next edge
                capture_event_q  <= capture_event_w;          -- delay the capture event by one cycle

                -- BTCMPRx is what software writes; btcl0_q/btcl1_q are the
                -- values actually used for comparing. Here they're just
                -- copied straight through every cycle.
                if HEU0_C = '1' then
                    btcl0_q <= btcmpr0_q;
                end if;

                if HEU1_C = '1' then
                    btcl1_q <= btcmpr1_q;
                end if;

                -- clearing wins over setting: if both happen the same cycle,
                -- the flag ends up cleared.
                if irq_clear_i = '1' then
                    btifg_q <= '0';               -- CPU/software just handled it: clear the flag
                elsif irq_source_w = '1' then
                    btifg_q <= '1';               -- chosen event fired: raise the flag
                end if;

                btctl1_write_v := (cs_i = '1' and mem_write_i = '1' and
                                   address_i = ADDR_BTCTL1);        -- 1 = software is writing BTCTL1 this cycle
                clear_v       := btctl1_write_v and data_i(2) = '1'; -- 1 = that write also asked to clear BTCNT (BTCLR)

                next_mode_v   := btctl1_q(7);      -- PWM mode (Set/Reset or Reset/Set), before this write
                next_enable_v := btctl1_q(6);       -- PWM enable, before this write
                if btctl1_write_v then
                    next_mode_v   := data_i(7);     -- if writing now, use the new value instead
                    next_enable_v := data_i(6);
                end if;

                if cs_i = '1' and mem_write_i = '1' then
                    case address_i is                             -- which register software is writing
                        when ADDR_BTCTL1 =>
                            btctl1_q <= data_i(7 downto 3) & '0' &
                                        data_i(1 downto 0);         -- BTCLR bit itself is not stored, it self-clears
                        when ADDR_BTCTL2 =>
                            btctl2_q <= data_i(7 downto 0);
                        when ADDR_BTCMPR0 =>
                            btcmpr0_q <= data_i;                    -- new PWM period
                        when ADDR_BTCMPR1 =>
                            btcmpr1_q <= data_i;                    -- new PWM duty point
                        when ADDR_BTCAPR =>
                            btcapr_q <= data_i;                     -- software can also write BTCAPR directly
                        when others =>
                            null;
                    end case;
                end if;

                if capture_event_w = '1' then
                    btcapr_q <= std_logic_vector(btcnt_q);          -- a real capture wins over a software write
                end if;

                -- if software just changed the period (BTCMPR0), restart the
                -- counter fresh so we don't miss/overshoot the new period
                if cs_i = '1' and mem_write_i = '1' and
                   address_i = ADDR_BTCMPR0 then
                    btcnt_q     <= (others => '0');
                    prescaler_q <= (others => '0');
                elsif clear_v then
                    btcnt_q <= (others => '0');                    -- software asked for a manual clear (BTCLR)
                elsif btctl1_q(5) = '0' and timer_tick_w = '1' then -- not held, and this cycle is a valid tick
                    if btcl0_q /= x"00000000" and
                       std_logic_vector(btcnt_q) = btcl0_q then
                        btcnt_q <= (others => '0');                 -- reached the period end: wrap back to 0
                    else
                        btcnt_q <= btcnt_q + 1;                     -- otherwise just keep counting up
                    end if;
                end if;

                -- PWM output: a manual clear with PWM enabled forces the
                -- output to its mode's starting level; otherwise the output
                -- flips at the two compare points (period end / duty point).
                if clear_v and next_enable_v = '1' then
                    if next_mode_v = '0' then
                        pwmout_q <= '0';         -- mode 0 (Set/Reset) starts low
                    else
                        pwmout_q <= '1';         -- mode 1 (Reset/Set) starts high
                    end if;
                elsif btctl1_q(6) = '1' then     -- PWM updates enabled
                    if btctl1_q(7) = '0' then    -- mode 0: Set at duty point, Reset at period end
                        if equ0_w = '1' then
                            pwmout_q <= '0';
                        elsif equ1_w = '1' then
                            pwmout_q <= '1';
                        end if;
                    else                          -- mode 1: opposite of mode 0
                        if equ0_w = '1' then
                            pwmout_q <= '1';
                        elsif equ1_w = '1' then
                            pwmout_q <= '0';
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Read-data mux
    ---------------------------------------------------------------------------
    process(cs_i, mem_read_i, address_i, btctl1_q, btctl2_q,
            btcmpr0_q, btcmpr1_q, btcapr_q)
    begin
        data_o <= (others => '0');

        if cs_i = '1' and mem_read_i = '1' then
            case address_i is
                when ADDR_BTCTL1 =>
                    data_o <= x"000000" & btctl1_q;
                when ADDR_BTCTL2 =>
                    data_o <= x"000000" & btctl2_q;
                when ADDR_BTCMPR0 =>
                    data_o <= btcmpr0_q;
                when ADDR_BTCMPR1 =>
                    data_o <= btcmpr1_q;
                when ADDR_BTCAPR =>
                    data_o <= btcapr_q;
                when others =>
                    data_o <= (others => '0');
            end case;
        end if;
    end process;

    pwmout_o       <= pwmout_q;
    btctl1_o       <= btctl1_q;
    btctl2_o       <= btctl2_q;
    btcmpr0_o      <= btcmpr0_q;
    btcmpr1_o      <= btcmpr1_q;
    btcapr_o       <= btcapr_q;
    btcnt_o        <= std_logic_vector(btcnt_q);
    equ0_o         <= equ0_w;
    equ1_o         <= equ1_w;
    capture_event_o <= capture_event_q;
    btifg_o         <= btifg_q;
    irq_o           <= btifg_q;
    btcl0_o         <= btcl0_q;
    btcl1_o         <= btcl1_q;
    heu0_o          <= HEU0_C;
    heu1_o          <= HEU1_C;
end architecture;
