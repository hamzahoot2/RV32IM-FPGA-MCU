--============================================================================
-- Interrupt Controller for RV32IM MCU (Basic Timer + KEY1..KEY3; no UART bonus)
--
-- IE/IFG bit mapping:
--   bits1..0 reserved (UART bonus not implemented)
--   bit2 BTIE/BTIFG
--   bit3 KEY1IE/KEY1IFG
--   bit4 KEY2IE/KEY2IFG
--   bit5 KEY3IE/KEY3IFG
--
-- TYPE priority/content:
--   Basic Timer       = 0x10
--   KEY1              = 0x14
--   KEY2              = 0x18
--   KEY3              = 0x1C
--============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity interrupt_controller is
    port(
        clk_i           : in  std_logic;
        rst_i           : in  std_logic;

        cs_i            : in  std_logic;                     -- 1 = CPU is talking to this block (chip select)
        mem_read_i      : in  std_logic;                     -- 1 = CPU is doing a load from this block
        mem_write_i     : in  std_logic;                     -- 1 = CPU is doing a store to this block
        address_i       : in  std_logic_vector(31 downto 0); -- address the CPU put on the bus
        data_i          : in  std_logic_vector(31 downto 0); -- data the CPU is writing
        data_o          : out std_logic_vector(31 downto 0); -- data sent back to the CPU on a read

        timer_irq_i     : in  std_logic;                     -- "time's up" flag coming from the Basic Timer
        key_clean_i     : in  std_logic_vector(2 downto 0);  -- debounced KEY1/KEY2/KEY3: 0=pressed, 1=released

        gie_i           : in  std_logic;                     -- GIE: master on/off switch for all interrupts
        inta_i          : in  std_logic;                     -- 1 = CPU is acknowledging/servicing an interrupt now
        intr_o          : out std_logic;                     -- request line sent to the CPU
        timer_clear_o   : out std_logic;                     -- tells the Basic Timer its flag was handled

        ie_o            : out std_logic_vector(7 downto 0);  -- IE register, mirrored out for debug
        ifg_o           : out std_logic_vector(7 downto 0);  -- IFG register, mirrored out for debug
        type_o          : out std_logic_vector(7 downto 0);  -- TYPE code, mirrored out for debug
        key_event_o     : out std_logic_vector(2 downto 0)   -- key_event_w, mirrored out for debug
    );
end entity;

architecture rtl of interrupt_controller is
    constant ADDR_IE   : std_logic_vector(31 downto 0) := x"0000202C"; -- address of the IE register
    constant ADDR_IFG  : std_logic_vector(31 downto 0) := x"0000202D"; -- address of the IFG register
    constant ADDR_TYPE : std_logic_vector(31 downto 0) := x"0000202E"; -- address of the TYPE register

    signal ie_q            : std_logic_vector(7 downto 0) := (others => '0'); -- IE register: which sources are allowed
    signal ifg_q           : std_logic_vector(7 downto 0) := (others => '0'); -- IFG register: which sources are pending
    signal key_prev_q      : std_logic_vector(2 downto 0) := (others => '1'); -- key_clean_i value from last clock
    signal key_event_w     : std_logic_vector(2 downto 0);                    -- pulses 1 when a key was just released
    signal pending_w       : std_logic_vector(7 downto 0);                    -- sources that are pending AND allowed
    signal type_w          : std_logic_vector(7 downto 0);                    -- code telling the CPU which source to handle
    signal timer_served_w  : std_logic;                                       -- 1 = CPU is servicing the Timer right now

    -- timer_irq_i comes from another clock (SMCLK), so we pass it through two
    -- flip-flops first (sync1_q, sync2_q) to make it safe to use here.
    -- Then we compare it to its own last value (prev_q) to catch just the
    -- moment it turns on, instead of the whole time it stays on.
    signal timer_irq_sync1_q : std_logic := '0'; -- first copy of timer_irq_i, can be unstable for a moment
    signal timer_irq_sync2_q : std_logic := '0'; -- second copy, one cycle later, now stable
    signal timer_irq_prev_q  : std_logic := '0'; -- timer_irq_sync2_q from last clock, to detect a change
    signal timer_irq_event_w : std_logic;        -- pulses 1 for one cycle when the timer flag just turned on
    signal timer_holdoff_q   : std_logic := '0'; -- 1 = ignore the timer flag for a bit, just handled it
    signal timer_sw_clear_w  : std_logic;        -- 1 = software just cleared the timer bit by writing IFG
begin
    key_event_w <= key_clean_i and (not key_prev_q);
    -- key: 0=pressed, 1=released. we want just the release moment (0->1),
    -- key_clean_i = key state NOW, key_prev_q = key state one cycle AGO
    -- we want event=1 only on the exact cycle it changes 0 -> 1

    timer_irq_event_w <= timer_irq_sync2_q and (not timer_irq_prev_q);
    -- same fix, for the timer flag (which also stays high, not a pulse).

    timer_sw_clear_w <= '1' when cs_i = '1' and mem_write_i = '1' and
                                address_i = ADDR_IFG and data_i(2) = '0'
                        else '0';
    -- 1 = software just wrote 0 into IFG bit 2, meaning "clear the timer flag"

    pending_w   <= ie_q and ifg_q;
    -- a source only counts if: it's pending AND it's allowed

    process(pending_w)
    begin
        if pending_w(2) = '1' then          -- priority 1 (highest): Timer
            type_w <= x"10";
        elsif pending_w(3) = '1' then       -- priority 2: KEY1
            type_w <= x"14";
        elsif pending_w(4) = '1' then       -- priority 3: KEY2
            type_w <= x"18";
        elsif pending_w(5) = '1' then       -- priority 4 (lowest): KEY3
            type_w <= x"1C";
        else
            type_w <= x"00";                -- nothing pending
        end if;
    end process;

    intr_o <= gie_i when pending_w(5 downto 2) /= "0000" else '0';
    -- ask the CPU only if: something is pending AND the master switch is on

    timer_served_w   <= '1' when inta_i = '1' and type_w = x"10" else '0';
    -- 1 = the CPU is acknowledging an interrupt AND it's the Timer's turn

    timer_clear_o    <= timer_served_w or timer_sw_clear_w;
    -- tell the Timer its flag is handled, either because the CPU serviced it
    -- or because software cleared it by hand

    process(clk_i)
        variable next_ifg_v : std_logic_vector(7 downto 0);
    begin
        if rising_edge(clk_i) then
            if rst_i = '1' then
                ie_q              <= (others => '0');
                ifg_q             <= (others => '0');
                key_prev_q        <= (others => '1');
                timer_irq_sync1_q <= '0';
                timer_irq_sync2_q <= '0';
                timer_irq_prev_q  <= '0';
                timer_holdoff_q    <= '0';
            else
                key_prev_q        <= key_clean_i;         -- remember key state, to catch the next release
                timer_irq_sync1_q <= timer_irq_i;         -- first copy: can be unstable for a moment
                timer_irq_sync2_q <= timer_irq_sync1_q;   -- second copy: now stable, safe to use
                timer_irq_prev_q  <= timer_irq_sync2_q;   -- remember it, to catch the next "just turned on"
                next_ifg_v := ifg_q;                      -- start from the current IFG value, change only what's needed

                if cs_i = '1' and mem_write_i = '1' then
                    case address_i is
                        when ADDR_IE =>
                            ie_q <= "00" & data_i(5 downto 2) & "00"; -- software sets which sources are allowed
                        when ADDR_IFG =>
                            next_ifg_v := "00" & data_i(5 downto 2) & "00"; -- software can clear a flag by hand
                        when others =>
                            null;
                    end case;
                end if;

                -- The timer flag stays "on" until we say otherwise (it's a level,
                -- not a one-time pulse). So after we service it, we ignore it for
                -- a bit (timer_holdoff_q) until it actually goes back to 0 —
                -- otherwise we'd think a new interrupt just happened right away.
                if timer_served_w = '1' or timer_sw_clear_w = '1' then
                    next_ifg_v(2) := '0';           -- just handled it: clear the flag
                    timer_holdoff_q <= '1';         -- and start ignoring it for now
                elsif timer_holdoff_q = '1' then
                    next_ifg_v(2) := '0';           -- still ignoring
                    if timer_irq_sync2_q = '0' then
                        timer_holdoff_q <= '0';     -- flag finally went low: stop ignoring
                    end if;
                elsif timer_irq_sync2_q = '1' then
                    next_ifg_v(2) := '1';           -- genuinely new timer event: raise the flag
                end if;

                -- KEY1/KEY2/KEY3 flags are only cleared by software (writing
                -- IFG), never automatically. Hardware only ever sets them,
                -- one bit per key, when that key is released.
                if key_event_w(0) = '1' then
                    next_ifg_v(3) := '1';           -- KEY1 was released: set its flag
                end if;

                if key_event_w(1) = '1' then
                    next_ifg_v(4) := '1';           -- KEY2 was released: set its flag
                end if;

                if key_event_w(2) = '1' then
                    next_ifg_v(5) := '1';           -- KEY3 was released: set its flag
                end if;

                next_ifg_v(7 downto 6) := "00";     -- these bits aren't used, keep them 0
                next_ifg_v(1 downto 0) := "00";     -- UART bits removed, keep them 0
                ifg_q <= next_ifg_v;                -- store the updated IFG value
            end if;
        end if;
    end process;

    process(cs_i, mem_read_i, address_i, ie_q, ifg_q, type_w)
    begin
        data_o <= (others => '0');                  -- default: nothing to send back
        if cs_i = '1' and mem_read_i = '1' then      -- CPU is doing a load from us
            case address_i is
                when ADDR_IE   => data_o <= x"000000" & ie_q;   -- CPU asked for IE
                when ADDR_IFG  => data_o <= x"000000" & ifg_q;  -- CPU asked for IFG
                when ADDR_TYPE => data_o <= x"000000" & type_w; -- CPU asked for TYPE
                when others    => data_o <= (others => '0');
            end case;
        end if;
    end process;

    ie_o        <= ie_q;
    ifg_o       <= ifg_q;
    type_o      <= type_w;
    key_event_o <= key_event_w;
end architecture;

