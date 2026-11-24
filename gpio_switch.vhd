library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity gpio_switch is
    port(
        sw_i : in STD_LOGIC_VECTOR(7 downto 0);      -- live state of the physical switches
        data_o : out STD_LOGIC_VECTOR(31 downto 0)   -- switch state, ready for the CPU to read
    );
end entity;

architecture rtl of gpio_switch is
begin
    data_o <= x"000000" & sw_i;
    -- input-only port: no latch needed, just always show the current switches
end architecture;
