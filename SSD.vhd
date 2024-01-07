
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;


entity SSD is
    Port ( clock : in STD_LOGIC;
           digs : in std_logic_vector (15 downto 0);
           catod : out STD_LOGIC_VECTOR (6 downto 0);
           anod : out STD_LOGIC_VECTOR (3 downto 0));
end SSD;

architecture Behavioral of SSD is
signal cnt: std_logic_vector(15 downto 0) := (others => '0');
signal sel: std_logic_vector(1 downto 0);
signal MuxO: std_logic_vector(3 downto 0);

begin
 counter: process(clock)
     begin
       if rising_edge(clock) then
          cnt <= cnt+1;
       end if;
       end process;
       
       sel<=cnt(15 downto 14);
     MUX1:  process (sel, digs)
          begin
            case(sel) is
              when "00" => MuxO <= digs(3 downto 0);
              when "01" => MuxO <= digs(7 downto 4);
              when "10" => MuxO <= digs(11 downto 8);
              when "11" => MuxO <= digs(15 downto 12);
              when others => MuxO <= (others => 'X');
            end case;
          end process;
          
      MUX2: process(sel)
             begin
               case(sel) is
                 when "00" => anod <= "1110";
                 when "01" => anod <= "1101";
                 when "10" => anod <= "1011";
                 when "11" => anod <= "0111";
                 when others => anod <= (others => 'X');
               end case;
            end process;
            
        with MuxO SELect
               catod <= "1111001" when "0001",   --1
                     "0100100" when "0010",   --2
                     "0110000" when "0011",   --3
                     "0011001" when "0100",   --4
                     "0010010" when "0101",   --5
                     "0000010" when "0110",   --6
                     "1111000" when "0111",   --7
                     "0000000" when "1000",   --8
                     "0010000" when "1001",   --9
                     "0001000" when "1010",   --A
                     "0000011" when "1011",   --b
                     "1000110" when "1100",   --C
                     "0100001" when "1101",   --d
                     "0000110" when "1110",   --E
                     "0001110" when "1111",   --F
                     "1000000" when others;   --0

end Behavioral;
