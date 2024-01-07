library IEEE;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL; 
use IEEE.STD_LOGIC_1164.ALL;

entity MEM is
    Port ( MemWrite : in STD_LOGIC;
           ALUResIn : in STD_LOGIC_VECTOR (15 downto 0);
           RD2 : in STD_LOGIC_VECTOR (15 downto 0);
           clk : in STD_LOGIC;
           EN : in STD_LOGIC;
           MemData : out STD_LOGIC_VECTOR (15 downto 0);
           ALUResOut : out STD_LOGIC_VECTOR (15 downto 0));
end MEM;

architecture Behavioral of MEM is
type MemRam is array (0 to 31) of std_logic_vector(15 downto 0);
signal Ram : MemRam := (
       x"0002",
       x"0003",
       x"0004",
       x"0005",
       x"0006",
       x"0007",
       x"0008",
       others => x"0000");
begin
  writeProcess: process(clk)
            begin
             if rising_edge(clk) then
               if EN = '1' and MemWrite = '1' then
                  Ram(conv_integer(ALUResIn(4 downto 0))) <= RD2;
               end if;
             end if;
           end process;
           
           MemData <= Ram(conv_integer(ALUResIn(4 downto 0)));
           ALUResOut <= ALUResIn;
         

end Behavioral;
