library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity ID_pipeline is
    Port ( RegWr : in STD_LOGIC;
       Instr : in STD_LOGIC_VECTOR (15 downto 0);
       CLK : in STD_LOGIC;
       EN : in STD_LOGIC;
       ExtOp : in STD_LOGIC;
       WD : in STD_LOGIC_VECTOR(15 downto 0);
       RD1: out STD_LOGIC_VECTOR(15 downto 0);
       RD2  : out STD_LOGIC_VECTOR(15 downto 0);
       Ext_Imm : out STD_LOGIC_VECTOR(15 downto 0);
       func : out STD_LOGIC_VECTOR(2 downto 0);
       wa : in STD_LOGIC_VECTOR(2 downto 0);
       rd: out STD_LOGIC_VECTOR(2 downto 0);
       rt: out STD_LOGIC_VECTOR(2 downto 0);
       sa : out STD_LOGIC);
end ID_pipeline;

architecture Behavioral of ID_pipeline is
type Reg is array (0 to 7) of std_logic_vector (15 downto 0);
signal reg_file : Reg; 
                    --(
                    -- x"0000",
                    -- x"0000",
                    -- x"0000",
                    -- x"0000",
                    -- x"0000",
                    -- x"0000",         
                    -- x"0000",
                    -- others => x"0000");
                     
       
signal WriteAddr: std_logic_vector(2 downto 0);
signal ExtZero, ExtSign : std_logic_vector(15 downto 0);
begin


   process(clk)
     begin
        if falling_edge(clk) then
          if EN = '1' and RegWr = '1' then
             reg_file(conv_integer(wa)) <= WD;
           end if;
        end if;
   end process;
   
       RD1 <= reg_file(conv_integer(Instr(12 downto 10)));   -- citire $rs
       RD2 <= reg_file(conv_integer(Instr(9 downto 7)));    -- citire  $rt
        
        
  --MuxWriteAddr:  WriteAddr <= Instr(9 downto 7) when RegDst = '0' else Instr(6 downto 4);
       
       ExtZero <= "000000000"&(Instr(6 downto 0));
       ExtSign <= Instr(6)&Instr(6)&Instr(6)&Instr(6)&Instr(6)&Instr(6)&Instr(6)&Instr(6)&Instr(6)&Instr(6 downto 0);
       Ext_Imm <= ExtZero when ExtOp = '0' else ExtSign;
       func <= Instr(2 downto 0);
       sa <= Instr(3);
       
       rt <= instr(9 downto 7);
       rd <= instr(6 downto 4);


end Behavioral;
