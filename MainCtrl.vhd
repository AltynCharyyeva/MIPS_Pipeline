
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;


entity MainCtrl is
    Port ( Instr : in STD_LOGIC_VECTOR (15 downto 13);
           RegDst : out STD_LOGIC;
           ExtOp : out STD_LOGIC;
           ALUSrc : out STD_LOGIC;
           Branch : out STD_LOGIC;
           Jump : out STD_LOGIC;
           ALUOp : out STD_LOGIC_VECTOR (1 downto 0);
           MemWrite : out STD_LOGIC;
           MemToReg : out STD_LOGIC;
           RegWr : out STD_LOGIC);
end MainCtrl;

architecture Behavioral of MainCtrl is

begin

  MainControl: process(Instr)
                begin
                RegDst <= '0'; ExtOp <= '0'; ALUSrc <= '0'; Branch <= '0';
                Jump <= '0'; MemWrite <= '0';  ALUOp <= "00"; MemToReg <= '0'; RegWr <= '0';
                  case(Instr(15 downto 13)) is
                     when "000" => RegDst <= '1'; RegWr <= '1';
                     when "001" => ExtOp <= '1'; ALUSrc <= '1'; RegWr <= '1'; ALUOp <= "01";
                     when "010" =>  ExtOp <= '1'; ALUSrc <= '1'; RegWr <= '1'; MemToReg <= '1'; ALUOp <= "01";
                     when "011" => ExtOp <= '1'; ALUSrc <= '1'; MemWrite <= '1'; ALUOp <= "01";
                     when "100" => ExtOp <= '1'; Branch <= '1'; ALUOp <= "10";
                     when "101" => ExtOp <= '1'; ALUSrc <= '1'; RegWr <= '1'; ALUOp <= "11";
                     when "111" => Jump <= '1'; ALUOp <= "01";
                     when others => RegDst <= '0'; ExtOp <= '0'; ALUSrc <= '0'; Branch <= '0';
                     Jump <= '0'; ALUOp <= "00"; MemWrite <= '0'; MemToReg <= '0'; RegWr <= '0';
                   end case;    
                 end process;                  
                     
end Behavioral;
