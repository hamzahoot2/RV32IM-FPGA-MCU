library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity gpio_hex6 is
    port(
        clk_i : in STD_LOGIC;
        rst_i : in STD_LOGIC;
        mem_write_i : in STD_LOGIC;                     -- 1 = CPU is doing a store
        address_i : in STD_LOGIC_VECTOR(31 downto 0);   -- which HEX register the CPU is targeting
        data_i : in STD_LOGIC_VECTOR(31 downto 0);      -- data the CPU is writing
        hex0_o, hex1_o, hex2_o, hex3_o, hex4_o, hex5_o : out STD_LOGIC_VECTOR(6 downto 0); -- 7-seg display pins
        data_o : out STD_LOGIC_VECTOR(31 downto 0)      -- data sent back to the CPU on a read
    );
end entity;

architecture rtl of gpio_hex6 is
    signal h0_q, h1_q, h2_q, h3_q, h4_q, h5_q : STD_LOGIC_VECTOR(7 downto 0); -- stored value for each digit
    signal selected_q : STD_LOGIC_VECTOR(7 downto 0); -- picked digit value, for the CPU readback
begin
    process(clk_i, rst_i)
    begin
        if rst_i = '1' then
            h0_q <= (others => '0'); h1_q <= (others => '0');
            h2_q <= (others => '0'); h3_q <= (others => '0');
            h4_q <= (others => '0'); h5_q <= (others => '0');
        elsif rising_edge(clk_i) then
            if mem_write_i = '1' then
                case address_i is                              -- each HEX digit has its own address
                    when x"00002004" => h0_q <= data_i(7 downto 0);
                    when x"00002005" => h1_q <= data_i(7 downto 0);
                    when x"00002008" => h2_q <= data_i(7 downto 0);
                    when x"00002009" => h3_q <= data_i(7 downto 0);
                    when x"0000200C" => h4_q <= data_i(7 downto 0);
                    when x"0000200D" => h5_q <= data_i(7 downto 0);
                    when others => null;
                end case;
            end if;
        end if;
    end process;

    with address_i select selected_q <=              -- pick which digit's value to send back on a read
        h0_q when x"00002004",
        h1_q when x"00002005",
        h2_q when x"00002008",
        h3_q when x"00002009",
        h4_q when x"0000200C",
        h5_q when x"0000200D",
        (others => '0') when others;

    data_o <= x"000000" & selected_q;

    -- each digit's low nibble drives its own 7-segment decoder
    D0: entity work.seven_seg_decoder port map(value_i => h0_q(3 downto 0), hex_o => hex0_o);
    D1: entity work.seven_seg_decoder port map(value_i => h1_q(3 downto 0), hex_o => hex1_o);
    D2: entity work.seven_seg_decoder port map(value_i => h2_q(3 downto 0), hex_o => hex2_o);
    D3: entity work.seven_seg_decoder port map(value_i => h3_q(3 downto 0), hex_o => hex3_o);
    D4: entity work.seven_seg_decoder port map(value_i => h4_q(3 downto 0), hex_o => hex4_o);
    D5: entity work.seven_seg_decoder port map(value_i => h5_q(3 downto 0), hex_o => hex5_o);
end architecture;
