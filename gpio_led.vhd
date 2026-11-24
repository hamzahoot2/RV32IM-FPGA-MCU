library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity gpio_led is
    port(
        clk_i : in STD_LOGIC;
        rst_i : in STD_LOGIC;
        cs_i : in STD_LOGIC;                          -- 1 = CPU is talking to this block (chip select)
        mem_write_i : in STD_LOGIC;                   -- 1 = CPU is doing a store to this block
        data_i : in STD_LOGIC_VECTOR(31 downto 0);    -- data the CPU is writing
        led_o : out STD_LOGIC_VECTOR(7 downto 0);     -- goes straight out to the physical LEDs
        data_o : out STD_LOGIC_VECTOR(31 downto 0)    -- data sent back to the CPU on a read
    );
end entity;

architecture rtl of gpio_led is
    signal led_q : STD_LOGIC_VECTOR(7 downto 0); -- stored LED value, kept until the next write
begin
    process(clk_i, rst_i)
    begin
        if rst_i = '1' then
            led_q <= (others => '0');           -- LEDs off at reset
        elsif rising_edge(clk_i) then
            if cs_i = '1' and mem_write_i = '1' then
                led_q <= data_i(7 downto 0);    -- CPU store: latch the low byte as the new LED value
            end if;
        end if;
    end process;
    led_o <= led_q;
    data_o <= x"000000" & led_q;
    -- readback: give the CPU the same value back if it reads this address
end architecture;
