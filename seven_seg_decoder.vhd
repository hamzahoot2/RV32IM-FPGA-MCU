library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity seven_seg_decoder is
    port(
        value_i : in STD_LOGIC_VECTOR(3 downto 0);  -- one hex digit, 0 to F
        hex_o : out STD_LOGIC_VECTOR(6 downto 0)    -- which segments to light up, 0 = segment on
    );
end entity;

architecture rtl of seven_seg_decoder is
begin
    process(value_i)
    begin
        -- lookup table: for each digit 0-F, which of the 7 segments to light
        case value_i is
            when "0000" => hex_o <= "1000000";
            when "0001" => hex_o <= "1111001";
            when "0010" => hex_o <= "0100100";
            when "0011" => hex_o <= "0110000";
            when "0100" => hex_o <= "0011001";
            when "0101" => hex_o <= "0010010";
            when "0110" => hex_o <= "0000010";
            when "0111" => hex_o <= "1111000";
            when "1000" => hex_o <= "0000000";
            when "1001" => hex_o <= "0010000";
            when "1010" => hex_o <= "0001000";
            when "1011" => hex_o <= "0000011";
            when "1100" => hex_o <= "1000110";
            when "1101" => hex_o <= "0100001";
            when "1110" => hex_o <= "0000110";
            when others => hex_o <= "0001110";
        end case;
    end process;
end architecture;
