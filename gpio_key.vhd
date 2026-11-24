--============================================================================
-- Memory-mapped pushbutton GPIO
-- PORT_PB address: 0x2014
-- KEY0 is not included because it is reserved for system reset.
-- FPGA pushbuttons are active-low; data_o reports pressed buttons as logic '1'.
--
-- data_o(0) = KEY1 pressed
-- data_o(1) = KEY2 pressed
-- data_o(2) = KEY3 pressed
--============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity gpio_key is
    port(
        key_i  : in  std_logic_vector(3 downto 1); -- raw KEY3/2/1 pins: 0=pressed, 1=released
        data_o : out std_logic_vector(31 downto 0) -- pressed state, ready for the CPU to read
    );
end entity;

architecture rtl of gpio_key is
    signal pressed_w : std_logic_vector(2 downto 0); -- flipped so 1=pressed, easier for software to read
begin
    pressed_w(0) <= not key_i(1); -- KEY1 pressed = 1
    pressed_w(1) <= not key_i(2); -- KEY2 pressed = 1
    pressed_w(2) <= not key_i(3); -- KEY3 pressed = 1

    data_o <= x"0000000" & '0' & pressed_w;
    -- input-only port: no latch needed, just always show the current keys
end architecture;
