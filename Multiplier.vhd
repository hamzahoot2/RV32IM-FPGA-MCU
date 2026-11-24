LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY Multiplier IS
    PORT(
        A_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);  -- first number to multiply
        B_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);  -- second number to multiply
        Res  : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)  -- the product, A_in * B_in
    );
END Multiplier;

-- 16x16 multiply, built from four smaller 8x8 multiplies (like doing
-- long multiplication by hand: split each number into a high half and
-- a low half, multiply every pair, then add the pieces back together
-- shifted to the right place)
ARCHITECTURE structural OF Multiplier IS
    SIGNAL A_low, A_high, B_low, B_high : unsigned(7 DOWNTO 0);  -- each input split into two 8-bit halves
    SIGNAL P0, P1, P2, P3 : unsigned(15 DOWNTO 0); -- the four partial products
    SIGNAL M : unsigned(16 DOWNTO 0);              -- the two "middle" partial products, added together
BEGIN
    A_low  <= unsigned(A_in(7 DOWNTO 0));
    A_high <= unsigned(A_in(15 DOWNTO 8));
    B_low  <= unsigned(B_in(7 DOWNTO 0));
    B_high <= unsigned(B_in(15 DOWNTO 8));

    P0 <= A_low * B_low;    -- lowest part of the result, no shift needed
    P1 <= A_low * B_high;   -- middle part, needs to be shifted left by 8
    P2 <= A_high * B_low;   -- middle part, also shifted left by 8
    P3 <= A_high * B_high;  -- highest part, shifted left by 16

    M <= ('0' & P1) + ('0' & P2);
    -- add the two middle parts together first (extra bit so it can't overflow)

    Res <= std_logic_vector( (x"0000" & P0) + 
                             ("0000000" & M & x"00") + 
                             (P3 & x"0000") );
    -- shift each piece to its correct position and add them all up:
    -- P0 as-is, M shifted left by 8, P3 shifted left by 16
END structural;
