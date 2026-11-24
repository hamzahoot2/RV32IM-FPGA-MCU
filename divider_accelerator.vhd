--=============================================================================
-- Unsigned Binary Multicycle Division Accelerator
-- Final project version aligned with lecturer notes / Figure 9.
--
-- DIVRST:
--   Synchronous LOAD/initialize pulse in DIVCLK domain.
--   Loads DIVIDEND and DIVISOR into the internal divider registers,
--   clears the upper half of the dividend shift-register and clears quotient.
--
-- DIVENA:
--   Starts the already-loaded division.
--
-- Operation:
--   Exactly N=32 DIVCLK iterations.
--   Uses shift + subtract + non-negative test (no "/" or "mod").
--
-- Divide by zero:
--   Lecturer note: division result is all ones when exceptions are unsupported.
--   QUOTIENT = 0xFFFFFFFF for N=32.
--   RESIDUE is kept as the loaded dividend (RISC-V-compatible implementation).
--=============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity divider_accelerator is
    generic(
        N : positive := 32
    );
    port(
        DIVCLK     : in  std_logic;                            -- this block's own clock, separate from the CPU's
        DIVRST     : in  std_logic;                             -- pulse: load DIVIDEND/DIVISOR and get ready
        DIVENA     : in  std_logic;                             -- pulse: start dividing the loaded values
        DIVIDEND   : in  std_logic_vector(N-1 downto 0);        -- number to be divided
        DIVISOR    : in  std_logic_vector(N-1 downto 0);        -- number to divide by
        DIVBUSY    : out std_logic;                             -- 1 = still dividing, result not ready yet
        RESIDUE    : out std_logic_vector(N-1 downto 0);        -- remainder, valid once DIVBUSY goes back to 0
        QUOTIENT   : out std_logic_vector(N-1 downto 0)         -- result, valid once DIVBUSY goes back to 0
    );
end entity divider_accelerator;

architecture rtl of divider_accelerator is

    signal dividend_shift_q  : unsigned(2*N-1 downto 0) := (others => '0'); -- working register: shifts left each cycle
    signal dividend_orig_q   : unsigned(N-1 downto 0)   := (others => '0'); -- original dividend, kept for the divide-by-zero case
    signal divisor_q         : unsigned(N-1 downto 0)   := (others => '0'); -- the number we're dividing by, loaded once
    signal quotient_q        : unsigned(N-1 downto 0)   := (others => '0'); -- result being built up bit by bit

    signal busy_q            : std_logic := '0';               -- 1 while the N-cycle division is running
    signal loaded_q          : std_logic := '0';                -- 1 = values are loaded, ready to start
    signal divzero_q         : std_logic := '0';                -- 1 = the divisor was zero when we started
    signal count_q           : natural range 0 to N := 0;       -- how many of the N cycles have run so far

    signal residue_q         : unsigned(N-1 downto 0) := (others => '0'); -- final remainder, held after done
    signal quotient_out_q    : unsigned(N-1 downto 0) := (others => '0'); -- final quotient, held after done

begin

    process(DIVCLK)
        variable shifted_v      : unsigned(2*N-1 downto 0); -- working register after shifting left by one
        variable y_v            : unsigned(N-1 downto 0);   -- the upper half: current partial remainder
        variable subtract_ext_v : signed(N downto 0);       -- y_v minus the divisor, one extra bit to catch sign
        variable next_div_v     : unsigned(2*N-1 downto 0); -- working register's value for the next cycle
        variable next_quot_v    : unsigned(N-1 downto 0);   -- quotient's value for the next cycle
    begin
        if rising_edge(DIVCLK) then

            if DIVRST = '1' then
                -- load everything fresh: put DIVIDEND in the lower half of
                -- the working register (upper half starts at 0), and
                -- remember the original dividend for the divide-by-zero case
                dividend_shift_q               <= (others => '0');
                dividend_shift_q(N-1 downto 0) <= unsigned(DIVIDEND);
                dividend_orig_q                <= unsigned(DIVIDEND);
                divisor_q                      <= unsigned(DIVISOR);
                quotient_q                     <= (others => '0');

                residue_q                      <= (others => '0');
                quotient_out_q                 <= (others => '0');

                busy_q                         <= '0';
                count_q                        <= 0;
                divzero_q                      <= '0';
                loaded_q                       <= '1';       -- now ready, waiting for DIVENA

            elsif DIVENA = '1' and busy_q = '0' and loaded_q = '1' then
                busy_q    <= '1';               -- start the N-cycle division now
                count_q   <= 0;
                loaded_q  <= '0';

                if divisor_q = 0 then
                    divzero_q <= '1';            -- remember: dividing by zero, special result at the end
                else
                    divzero_q <= '0';
                end if;

            -- one step of long division per cycle: shift, try to subtract
            -- the divisor, keep the result only if it didn't go negative
            elsif busy_q = '1' then

                shifted_v := shift_left(dividend_shift_q, 1); -- make room for the next quotient bit                          DIVIDEND/DIVISOR
                                                                                                                        -- each cycle: shift left, try subtracting the divisor.
                y_v := shifted_v(2*N-1 downto N);              -- current partial remainder (upper half)                   fits (not negative)? keep it, quotient bit = 1.
                subtract_ext_v :=                                                                                       -- doesn't fit? undo it, quotient bit = 0.
                    signed('0' & y_v) - signed('0' & divisor_q); -- try: does the divisor fit into it?                     after 32 cycles: quotient register has the answer,
                                                                                                                        -- upper half of the working register has the remainder.
                next_div_v  := shifted_v;
                next_quot_v := shift_left(quotient_q, 1);

                if subtract_ext_v(N) = '0' then      -- result not negative: the divisor fit
                    next_div_v(2*N-1 downto N) :=
                        unsigned(subtract_ext_v(N-1 downto 0));  -- keep the subtracted value
                    next_quot_v(0) := '1';                       -- and record a "1" quotient bit
                else
                    next_quot_v(0) := '0';                       -- didn't fit: leave it as-is, "0" quotient bit
                end if;

                dividend_shift_q <= next_div_v;
                quotient_q       <= next_quot_v;

                if count_q = N-1 then           -- this was the last of the N steps
                    if divzero_q = '1' then
                        quotient_out_q <= (others => '1');  -- divide-by-zero: quotient is all 1s
                        residue_q      <= dividend_orig_q;  -- and remainder is just the original dividend
                    else
                        quotient_out_q <= next_quot_v;      -- normal case: publish the final results
                        residue_q      <= next_div_v(2*N-1 downto N);
                    end if;

                    busy_q    <= '0';           -- done: DIVBUSY drops, results are now valid
                    count_q   <= 0;
                    divzero_q <= '0';
                else
                    count_q <= count_q + 1;     -- not done yet, one more step next cycle
                end if;

            end if;
        end if;
    end process;

    DIVBUSY  <= busy_q;
    RESIDUE  <= std_logic_vector(residue_q);
    QUOTIENT <= std_logic_vector(quotient_out_q);

end architecture rtl;
