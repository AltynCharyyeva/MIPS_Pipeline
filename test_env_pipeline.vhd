library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity test_env_pipeline is
Port( clk: in std_logic;
      btn: in std_logic_vector(4 downto 0);
      sw: in std_logic_vector(15 downto 0);
      led: out std_logic_vector(15 downto 0);
      an: out std_logic_vector(3 downto 0);
      cat: out std_logic_vector(6 downto 0));
end test_env_pipeline;


architecture Behavioral of test_env_pipeline is

component MPG is
   Port ( clock : in STD_LOGIC;
           input : in STD_LOGIC;
           Enable : out STD_LOGIC);
end component;

component SSD is
    Port ( clock : in STD_LOGIC;
           digs : in std_logic_vector (15 downto 0);
           catod : out STD_LOGIC_VECTOR (6 downto 0);
           anod : out STD_LOGIC_VECTOR (3 downto 0));
end component;

component ROM is
    Port ( clock : in STD_LOGIC;
           RST : in STD_LOGIC;
           En : in STD_LOGIC;
           Jump : in STD_LOGIC;
           PCSrc : in STD_LOGIC;
           JumpAdress : std_logic_vector(15 downto 0); 
           BranchAdress : std_logic_vector(15 downto 0);
           instruction : out STD_LOGIC_VECTOR (15 downto 0);
           PCinc : out STD_LOGIC_VECTOR (15 downto 0));
end component;

component ID_pipeline is
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
end component;


component MainCtrl is
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
end component;


component EX_pipeline is
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
end component;


component MEM is
    Port ( MemWrite : in STD_LOGIC;
           ALUResIn : in STD_LOGIC_VECTOR (15 downto 0);
           RD2 : in STD_LOGIC_VECTOR (15 downto 0);
           clk : in STD_LOGIC;
           EN : in STD_LOGIC;
           MemData : out STD_LOGIC_VECTOR (15 downto 0);
           ALUResOut : out STD_LOGIC_VECTOR (15 downto 0));
end component;

signal enable,reset, jump, PCSrc, ExtOp, RegDst, AluSrc, branch, MemWr, MemToReg, RegWr, sa, zero : std_logic;
signal instr, PCInc, JumpAddress, WD, RD1, RD2, Ext_imm, ALURes, BranchAddress,
Mem_Data, ALUResOut, digits : std_logic_vector(15 downto 0);
signal AluOp : std_logic_vector(1 downto 0);
signal func, rt, rd, wa: std_logic_vector(2 downto 0);


----semnale IF_ID
signal instr_IF_ID, PCinc_IF_ID: std_logic_vector(15 downto 0);

---semnale ID_EX
signal RegDst_ID_EX, AluSrc_ID_EX, branch_ID_EX, MemWr_ID_EX, RegWr_ID_EX, sa_ID_EX, MemToReg_ID_EX : std_logic;
signal RD1_ID_EX, RD2_ID_EX, Ext_imm_ID_EX, PCinc_ID_EX : std_logic_vector(15 downto 0);
signal rd_ID_EX, rt_ID_EX, func_ID_EX : std_logic_vector(2 downto 0);
signal ALUOp_ID_EX: std_logic_vector(1 downto 0);

-----semnale EX_MEM
signal branch_EX_MEM, MemWr_EX_MEM, MemToReg_EX_MEM, RegWr_EX_MEM, zero_EX_MEM : std_logic;
signal BranchAddress_EX_MEM, AluRes_EX_MEM, RD2_EX_MEM : std_logic_vector(15 downto 0); 
signal rd_EX_MEM: std_logic_vector(2 downto 0);

-----semnale MEM_WB
signal MemToReg_MEM_WB, RegWr_MEM_WB: std_logic;
signal AluRes_MEM_WB, MemData_MEM_WB: std_logic_vector(15 downto 0);
signal rd_MEM_WB : std_logic_vector(2 downto 0);
begin


--------------------------------MIPS_pipeline------------------------------------------------------------
debouncer1: MPG port map (clock=>clk, input=>btn(0), enable=>enable);
debouncer2: MPG port map (clock=>clk, input=>btn(1), enable=>reset);
-------componenente
Instruction_Fetch: ROM port map (clk, reset, enable, jump, PCSrc, JumpAddress, BranchAddress_EX_MEM, instr, PCInc);
MainControl: MainCtrl port map (instr_IF_ID(15 downto 13), RegDst, ExtOp, ALUSrc, Branch, Jump, ALUOp, MemWr, 
MemToReg, RegWr);
InstructionDecoder: ID_pipeline port map (RegWr_MEM_WB, instr_IF_ID, clk, enable, ExtOp, WD, RD1, 
RD2, Ext_imm, func, rd_MEM_WB, rd, rt, sa);
Execution_Unit: EX_pipeline port map (RD1_ID_EX, RD2_ID_EX, AluSrc_ID_EX, Ext_imm_ID_EX, PCinc_ID_EX, sa_ID_EX,
 RegDst_ID_EX, func_ID_EX, ALUOp_ID_EX, rd_ID_EX, rt_ID_EX, zero, ALURes, wa, BranchAddress);
Memory: MEM port map(MemWr_EX_MEM, AluRes_EX_MEM, RD2_EX_MEM, clk, enable, Mem_Data, ALUResOut);
display: SSD port map (clock=>clk, digs=>digits,anod=>an,catod=>cat);

  
  PCSrc <= branch_EX_MEM and zero_EX_MEM;
  JumpAddress <= PCinc_IF_ID(15 downto 13)&instr_IF_ID(12 downto 0);
  WD <= ALURes_MEM_WB when MemToReg_MEM_WB = '0' else MemData_MEM_WB;
  
  
  
  process(clk)
  begin
       if rising_edge(clk) then
         if enable = '1' then
         ---IF_ID_Process
         
         instr_IF_ID <= instr;
         PCinc_IF_ID <= PCInc;
         
         ----ID_EX_Process
         RegDst_ID_EX <= RegDst;
         AluSrc_ID_EX <= ALUSrc;
         branch_ID_EX <= branch;
         MemWr_ID_EX <= MemWr;
         RegWr_ID_EX <= RegWr;
         ALUOp_ID_EX <= ALUOp;
         MemToReg_ID_EX <= MemToReg;
         
         RD1_ID_EX <= RD1;
         RD2_ID_EX <= RD2;
         Ext_imm_ID_EX <= Ext_imm;
         PCinc_ID_EX <= PCinc_IF_ID;
         rd_ID_EX <= rd;
         rt_ID_EX <= rt;
         func_ID_EX <= func;
         sa_ID_EX <= sa;
         
         -----EX_MEM_Process
         
         branch_EX_MEM <= branch_ID_EX; 
         MemWr_EX_MEM <= MemWr_ID_EX;
         MemToReg_EX_MEM <= MemToReg_ID_EX;
         RegWr_EX_MEM <= RegWr_ID_EX;
         zero_EX_MEM  <= zero;
         
         BranchAddress_EX_MEM <= BranchAddress;
         AluRes_EX_MEM <= ALURes;
         RD2_EX_MEM <= RD2_ID_EX;
         rd_EX_MEM <= wa;
         
         -----MEM_WB_Process
         
         MemToReg_MEM_WB <= MemToReg_EX_MEM;
         RegWr_MEM_WB  <= RegWr_EX_MEM;
         AluRes_MEM_WB <= ALUResOut;
         MemData_MEM_WB <= Mem_Data;
         rd_MEM_WB <=  rd_EX_MEM;      
         
         end if;
       end if;
  end process; 
  
  
   -- sel <= sw(7 downto 5);
  MUX8_1: process(sw(7 downto 5), instr, PCInc, RD1_ID_EX, RD2_ID_EX, WD, Ext_imm_ID_EX, Mem_Data, ALURes)
          begin
           case sw(7 downto 5) is
              when "000" => digits <= instr;
              when "001" => digits <= PCInc;
              when "010" => digits <= RD1_ID_EX;
              when "011" => digits <= RD2_ID_EX;
              when "100" => digits <= Ext_imm_ID_EX;
              when "101" => digits <= ALURes;
              when "110" => digits <= Mem_Data;
              when "111" => digits <= WD;
           end case;
         end process; 
                
           led(7) <= RegDst;
           led(6) <= ExtOP;
           led(5) <= ALUSrc;
           led(4) <= Branch;
           led(3) <= Jump;
           led(2) <= MemWr;
           led(1) <= MemToReg;
           led(0) <= RegWr;
           led(9 downto 8) <= ALUOp;
end architecture;