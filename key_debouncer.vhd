--============================================================================
-- Synchronous debouncer for active-low FPGA pushbuttons.
-- The output changes only after the synchronized input has remained different
-- from the current clean value for DEBOUNCE_CYCLES consecutive clock cycles.
--============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity key_debouncer is
    generic(
        WIDTH           : positive := 3;
        DEBOUNCE_CYCLES : positive := 250000
    );
    port(
        clk_i   : in  std_logic;
        rst_i   : in  std_logic;
        raw_i   : in  std_logic_vector(WIDTH-1 downto 0);   -- straight from the physical buttons, still bouncy
        clean_o : out std_logic_vector(WIDTH-1 downto 0)    -- steady, bounce-free version of raw_i
    );
end entity;

architecture rtl of key_debouncer is
    type count_array_t is array (natural range <>) of natural range 0 to DEBOUNCE_CYCLES;
    signal count_q : count_array_t(0 to WIDTH-1) := (others => 0);        -- how long each button has held a new value
    signal sync1_q : std_logic_vector(WIDTH-1 downto 0) := (others => '1'); -- first copy of raw_i, can be unstable
    signal sync2_q : std_logic_vector(WIDTH-1 downto 0) := (others => '1'); -- second copy, one cycle later, now stable
    signal clean_q : std_logic_vector(WIDTH-1 downto 0) := (others => '1'); -- the accepted, bounce-free value
begin
    process(clk_i)
    begin
        if rising_edge(clk_i) then
            if rst_i = '1' then
                sync1_q <= (others => '1');
                sync2_q <= (others => '1');
                clean_q <= (others => '1');
                count_q <= (others => 0);
            else
                sync1_q <= raw_i;     -- first copy: can be unstable for a moment (metastable)
                sync2_q <= sync1_q;   -- second copy: now stable, safe to use

                for i in 0 to WIDTH-1 loop
                    if sync2_q(i) = clean_q(i) then
                        count_q(i) <= 0;                    -- same as before: no bounce happening, reset the counter
                    elsif count_q(i) = DEBOUNCE_CYCLES-1 then
                        clean_q(i) <= sync2_q(i);            -- held the new value long enough: accept it as real
                        count_q(i) <= 0;
                    else
                        count_q(i) <= count_q(i) + 1;        -- different, but not long enough yet: keep counting
                    end if;
                end loop;
            end if;
        end if;
    end process;

    clean_o <= clean_q;
end architecture;
