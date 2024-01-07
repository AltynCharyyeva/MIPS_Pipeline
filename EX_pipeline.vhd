library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity EX_pipeline is
    Port ( RD1 : in STD_LOGIC_VECTOR (15 downto 0);
       RD2 : in STD_LOGIC_VECTOR (15 downto 0);
       ALUSrc : in STD_LOGIC;
       Ext_Imm : in STD_LOGIC_VECTOR(15 downto 0);
       PC_plus : in STD_LOGIC_VECTOR(15 downto 0);
       sa : in STD_LOGIC;
       RegDst : in STD_LOGIC;
       func : in STD_LOGIC_VECTOR(2 downto 0);
       ALUOp : in STD_LOGIC_VECTOR (1 downto 0);
       rd : in STD_LOGIC_VECTOR (2 downto 0);
       rt : in STD_LOGIC_VECTOR (2 downto 0);
       Zero : out STD_LOGIC;
       ALURes : out STD_LOGIC_VECTOR (15 downto 0);
       WA : out STD_LOGIC_VECTOR (2 downto 0);
       BranchAddress : out STD_LOGIC_VECTOR(15 downto 0));
end EX_pipeline;

architecture Behavioral of EX_pipeline is
signal ALUCtrl : std_logic_vector(2 downto 0);
signal A, B: std_logic_vector(15 downto 0);
begin

 ALUControl: process(ALUOp, func)
             begin
             case ALUOp is
             when "00" =>
               case func is
                  when "001" => ALUCtrl <= "000"; --add
                  when "010" => ALUCtrl <= "001"; --sub
                  when "011" => ALUCtrl <= "010"; --sll
                  when "100" => ALUCtrl <= "011"; --srl
                  when "101" => ALUCtrl <= "100"; --and
                  when "110" => ALUCtrl <= "101"; --or
                  when "111" => ALUCtrl <= "110"; --xor
                  when "000" => ALUCtrl <= "111"; --sra
                  when others => ALUCtrl <= (others => 'X');
                  end case;
             when "01" => ALUCtrl <= "000"; --add(addi, sw, lw, j)
             when "10" => ALUCtrl <= "001"; --sub(beq)
             when "11" => ALUCtrl <= "101"; --or(ori)
             when others => ALUCtrl <= (others => 'X'); 
             end case;  
             end process;
            
            A<=RD1;
            B<=RD2 when ALUSrc = '0' else Ext_Imm;

 ALU: process(A, B, ALUCtrl, sa)
             begin
             case ALUCtrl is 
             when "000" => ALURes <= A+B;
             when "001" => ALURes <= A-B;
             when "010" => if sa = '1' then 
                              ALURes <= A(14 downto 0)&'0';
                           end if;
             when "011" => if sa = '1' then 
                              ALURes <= '0'&A(15 downto 1);
                           end if;
             when "100" => ALURes <= A and B;
             when "101" => ALURes <= A or B;
             when "110" => ALURes <= A xor B;
             when others => ALURes <= (others => 'X');
             end case;
             end process;
         Zero <= '1' when A=B else '0';
         BranchAddress <= Ext_Imm + PC_plus;
         
         WA <= rt when RegDst = '0' else rd;
      

end Behavioral;
