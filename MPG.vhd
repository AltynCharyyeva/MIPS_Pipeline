library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity MPG is
    Port ( clock : in STD_LOGIC;
           input : in STD_LOGIC;
           Enable : out STD_LOGIC);
end MPG;

architecture Behavioral of MPG is
signal cnt: std_logic_vector(17 downto 0):= (others => '0');
signal Q1: std_logic := '0';
signal Q2: std_logic := '0';
signal Q3: std_logic := '0';
signal En: std_logic_vector(17 downto 0);
begin
   
   counter: process(clock)
     begin
       if rising_edge(clock) then
          cnt<=cnt+1;
       end if;
    end process;
    
    En <= ("111111111111111111" and cnt);
    
    REG1: process(clock)
       begin
          if rising_edge (clock) then
             if En = "111111111111111111" then
                Q1<=input;
              end if;
          end if;
       end process;
       
    REG2And3: process(clock)
       begin
          if rising_edge (clock) then
             Q2<=Q1;
             Q3<=Q2;
           end if;
         end process;
  
       Enable <= (not Q3) and Q2;  
                
end Behavioral;
