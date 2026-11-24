library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity cdc_sync is
    port(
        clk_i  : in  std_logic;  -- the clock we want the signal to be safe in
        rst_i  : in  std_logic;  -- reset
        din_i  : in  std_logic;  -- input signal coming from another clock, not safe to use as-is
        dout_o : out std_logic   -- the safe, stable output. this is what the rest of the design uses
    );
end entity;

architecture rtl of cdc_sync is

    signal ff1_q : std_logic := '0'; -- first copy of the input. can be unstable for a moment (metastable)
    signal ff2_q : std_logic := '0'; -- second copy, taken one cycle later. by now ff1_q is stable

begin

    process(clk_i)
    begin
        if rising_edge(clk_i) then

            if rst_i = '1' then
                ff1_q <= '0';
                ff2_q <= '0';

            else
                ff1_q <= din_i;   -- first copy of the input. can be unstable for a moment (metastable)
                ff2_q <= ff1_q;   -- second copy, taken one cycle later. by now ff1_q is stable
            end if;

        end if;
    end process;

    dout_o <= ff2_q;
    -- safe, stable output — this is what the rest of the design uses

end architecture;
