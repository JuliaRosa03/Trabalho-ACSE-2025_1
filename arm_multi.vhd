---------------------------------------------------------------
-- arm_multi.vhd
-- David_Harris@hmc.edu, Sarah.Harris@unlv.edu 20 June 2015
-- Multicycle implementation of a subset of ARMv4
--
-- Compile in ModelSim at the command line with the command 
-- vcom -2008 arm_multi.vhd
-- Expect plenty of simulation warnings of metavalues detected
-- run 760
-- Expect at time 755 ns a message of
-- Failure: NO ERRORS: Simulation succeeded
-- when the value 7 is written to address 100 (0x64)
---------------------------------------------------------------

library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
entity testbench is
end;

architecture test of testbench is
  component top
    port(clk, reset:          in  STD_LOGIC;
         WriteData, Adr:      out STD_LOGIC_VECTOR(31 downto 0);
         MemWrite:            out STD_LOGIC);
  end component;
  signal WriteData, DataAdr:     STD_LOGIC_VECTOR(31 downto 0);
  signal clk, reset,  MemWrite:  STD_LOGIC;
begin

  -- instantiate device to be tested
  dut: top port map(clk, reset, WriteData, DataAdr, MemWrite);

  -- Generate clock with 10 ns period
  process begin
    clk <= '1';
    wait for 5 ns; 
    clk <= '0';
    wait for 5 ns;
  end process;

  -- Generate reset for first two clock cycles
  process begin
    reset <= '1';
    wait for 22 ns;
    reset <= '0';
    wait;
  end process;

  -- check that 7 gets written to address 84 
  -- at end of program
  process (clk) begin
    if (clk'event and clk = '0' and MemWrite = '1') then
      if (to_integer(DataAdr) = 100 and 
          to_integer(WriteData) = 7) then 
        report "NO ERRORS: Simulation succeeded" severity failure;
      elsif (DataAdr /= 96) then 
        report "Simulation failed" severity failure;
      end if;
    end if;
  end process;
end;


library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
entity top is -- top-level design for testing
  port(clk, reset:           in     STD_LOGIC;
       WriteData, Adr:       buffer STD_LOGIC_VECTOR(31 downto 0);
       MemWrite:             buffer STD_LOGIC);
end;

architecture test of top is
  component arm 
    port(clk, reset:        in  STD_LOGIC;
         MemWrite:          out STD_LOGIC;
         Adr, WriteData:    out STD_LOGIC_VECTOR(31 downto 0);
         ReadData:          in  STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component mem
    port(clk, we:  in  STD_LOGIC;
         a, wd:    in  STD_LOGIC_VECTOR(31 downto 0);
         rd:       out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  signal ReadData: STD_LOGIC_VECTOR(31 downto 0);
begin
  -- instantiate processor and memories
  i_arm: arm port map(clk, reset, MemWrite, Adr, 
                      WriteData, ReadData);
  i_mem: mem port map(clk, MemWrite, Adr, 
                      WriteData, ReadData);
end;

library IEEE; 
use IEEE.STD_LOGIC_1164.all; use STD.TEXTIO.all;
use IEEE.NUMERIC_STD_UNSIGNED.all; 
entity mem is -- memory
  port(clk, we:  in STD_LOGIC;
       a, wd:    in STD_LOGIC_VECTOR(31 downto 0);
       rd:       out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of mem is -- instruction and data memory
begin
  process is
    file mem_file: TEXT;
    variable L: line;
    variable ch: character;
    variable i, index, result: integer;

    type ramtype is array (63 downto 0) of 
                    STD_LOGIC_VECTOR(31 downto 0);
    variable ram: ramtype;
  begin 
    -- initialize memory from file
    for i in 0 to 63 loop -- set all contents low
      ram(i) := (others => '0'); 
    end loop;
    index := 0; 
    FILE_OPEN(mem_file, "memfile.dat", READ_MODE);
    while not endfile(mem_file) loop
      readline(mem_file, L);
      result := 0;
      for i in 1 to 8 loop
        read(L, ch);
        if '0' <= ch and ch <= '9' then 
            result := character'pos(ch) - character'pos('0');
        elsif 'a' <= ch and ch <= 'f' then
           result := character'pos(ch) - character'pos('a')+10;
        elsif 'A' <= ch and ch <= 'F' then
           result := character'pos(ch) - character'pos('A')+10;
        else report "Format error on line " & integer'image(index)
             severity error;
        end if;
        ram(index)(35-i*4 downto 32-i*4) := 
          to_std_logic_vector(result,4);
      end loop;
      index := index + 1;
    end loop;


  -- read or write memory
    loop
      if clk'event and clk = '1' then
          if (we = '1') then 
            ram(to_integer(a(7 downto 2))) := wd;
          end if;
      end if;
      rd <= ram(to_integer(a(7 downto 2))); 
      wait on clk, a;
    end loop;
  end process;
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;
entity arm is -- multicycle processor
  port(clk, reset:        in  STD_LOGIC;
       MemWrite:          out STD_LOGIC;
       Adr, WriteData:    out STD_LOGIC_VECTOR(31 downto 0);
       ReadData:          in  STD_LOGIC_VECTOR(31 downto 0));
end;

architecture struct of arm is
  component controller
    port(clk, reset:        in  STD_LOGIC;
         Instr:             in  STD_LOGIC_VECTOR(31 downto 12);
         ALUFlags:          in  STD_LOGIC_VECTOR(3 downto 0);
         PCWrite:           out STD_LOGIC;
         MemWrite:          out STD_LOGIC;
         RegWrite:          out STD_LOGIC;
         IRWrite:           out STD_LOGIC;
         AdrSrc:            out STD_LOGIC;
         RegSrc:            out STD_LOGIC_VECTOR(1 downto 0);
         ALUSrcA:           out STD_LOGIC;
         ALUSrcB:           out STD_LOGIC_VECTOR(1 downto 0);
         ResultSrc:         out STD_LOGIC_VECTOR(1 downto 0);
         ImmSrc:            out STD_LOGIC_VECTOR(1 downto 0);
         ALUControl:        out STD_LOGIC_VECTOR(1 downto 0));
  end component;
  component datapath
    port(clk, reset:        in  STD_LOGIC;
         Adr:               out STD_LOGIC_VECTOR(31 downto 0);
         WriteData:         out STD_LOGIC_VECTOR(31 downto 0);
         ReadData:          in  STD_LOGIC_VECTOR(31 downto 0);
         Instr:             out STD_LOGIC_VECTOR(31 downto 0);
         ALUFlags:          out STD_LOGIC_VECTOR(3 downto 0);
         PCWrite:           in  STD_LOGIC;
         RegWrite:          in  STD_LOGIC;
         IRWrite:           in  STD_LOGIC;
         AdrSrc:            in  STD_LOGIC;
         RegSrc:            in  STD_LOGIC_VECTOR(1 downto 0);
         ALUSrcA:           in  STD_LOGIC;
         ALUSrcB:           in  STD_LOGIC_VECTOR(1 downto 0);
         ResultSrc:         in  STD_LOGIC_VECTOR(1 downto 0);
         ImmSrc:            in  STD_LOGIC_VECTOR(1 downto 0);
         ALUControl:        in  STD_LOGIC_VECTOR(1 downto 0));
  end component;
  signal Instr: STD_LOGIC_VECTOR(31 downto 0);
  signal ALUFlags: STD_LOGIC_VECTOR(3 downto 0);
  signal PCWrite, RegWrite, IRWrite: STD_LOGIC;
  signal AdrSrc, ALUSrcA: STD_LOGIC;
  signal RegSrc, ALUSrcB: STD_LOGIC_VECTOR(1 downto 0);
  signal ImmSrc, ALUControl, ResultSrc: STD_LOGIC_VECTOR(1 downto 0);

begin
  cont: controller port map(clk, reset, Instr(31 downto 12), 
                            ALUFlags, PCWrite, MemWrite, RegWrite,     
                            IRWrite, AdrSrc, RegSrc, ALUSrcA, 
                            ALUSrcB, ResultSrc, ImmSrc, ALUControl);

  dp: datapath port map(clk, reset, Adr, WriteData, ReadData, 
                        Instr, ALUFlags,
                        PCWrite, RegWrite, IRWrite,
                        AdrSrc, RegSrc, ALUSrcA, ALUSrcB, ResultSrc,
                        ImmSrc, ALUControl);
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;
entity controller is -- single cycle control decoder
  port(clk, reset:        in  STD_LOGIC;
       Instr:             in  STD_LOGIC_VECTOR(31 downto 12);
       ALUFlags:          in  STD_LOGIC_VECTOR(3 downto 0);
       PCWrite:           out STD_LOGIC;
       MemWrite:          out STD_LOGIC;
       RegWrite:          out STD_LOGIC;
       IRWrite:           out STD_LOGIC;
       AdrSrc:            out STD_LOGIC;
       RegSrc:            out STD_LOGIC_VECTOR(1 downto 0);            
       ALUSrcA:           out STD_LOGIC;
       ALUSrcB:           out STD_LOGIC_VECTOR(1 downto 0);
       ResultSrc:         out STD_LOGIC_VECTOR(1 downto 0);
       ImmSrc:            out STD_LOGIC_VECTOR(1 downto 0);
       ALUControl:        out STD_LOGIC_VECTOR(1 downto 0));
end;
architecture struct of controller is
  component decoder
    port(clk, reset:       in  STD_LOGIC;
         Op:               in  STD_LOGIC_VECTOR(1 downto 0);
         Funct:            in  STD_LOGIC_VECTOR(5 downto 0);
         Rd:               in  STD_LOGIC_VECTOR(3 downto 0);
         FlagW:            out STD_LOGIC_VECTOR(1 downto 0);
         PCS, NextPC:      out STD_LOGIC; 
         RegW, MemW:       out STD_LOGIC;
         IRWrite, AdrSrc:  out STD_LOGIC;
         ResultSrc:        out STD_LOGIC_VECTOR(1 downto 0);
         ALUSrcA:          out STD_LOGIC;
         ALUSrcB, ImmSrc:  out STD_LOGIC_VECTOR(1 downto 0);
         RegSrc:           out STD_LOGIC_VECTOR(1 downto 0);
         ALUControl:       out STD_LOGIC_VECTOR(1 downto 0));
  end component;
  component condlogic
    port(clk, reset:        in  STD_LOGIC;
         Cond:              in  STD_LOGIC_VECTOR(3 downto 0);
         ALUFlags:          in  STD_LOGIC_VECTOR(3 downto 0);
         FlagW:             in  STD_LOGIC_VECTOR(1 downto 0);
         PCS, NextPC:       in  STD_LOGIC;
         RegW, MemW:        in  STD_LOGIC;
         PCWrite, RegWrite: out STD_LOGIC;
         MemWrite:          out STD_LOGIC);
  end component;
  signal FlagW: STD_LOGIC_VECTOR(1 downto 0);
  signal PCS, NextPC, RegW, MemW: STD_LOGIC;
begin
  dec: decoder port map(clk, reset, Instr(27 downto 26), Instr(25 downto 
20),
                       Instr(15 downto 12), FlagW, PCS, 
                       NextPC, RegW, MemW,
                       IRWrite, AdrSrc, ResultSrc, 
                       ALUSrcA, ALUSrcB, ImmSrc, RegSrc, ALUControl);
  cl: condlogic port map(clk, reset, Instr(31 downto 28), 
                         ALUFlags, FlagW, PCS, NextPC, RegW, MemW,
                         PCWrite, RegWrite, MemWrite);                
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;
entity decoder is -- main control decoder
  port(clk, reset:       in  STD_LOGIC;
       Op:               in  STD_LOGIC_VECTOR(1 downto 0);
       Funct:            in  STD_LOGIC_VECTOR(5 downto 0);
       Rd:               in  STD_LOGIC_VECTOR(3 downto 0);
       FlagW:            out STD_LOGIC_VECTOR(1 downto 0);
       PCS, NextPC:      out STD_LOGIC; 
       RegW, MemW:       out STD_LOGIC;
       IRWrite, AdrSrc:  out STD_LOGIC;
       ResultSrc:        out STD_LOGIC_VECTOR(1 downto 0);
       ALUSrcA:          out STD_LOGIC;
       ALUSrcB, ImmSrc:  out STD_LOGIC_VECTOR(1 downto 0);
       RegSrc:           out STD_LOGIC_VECTOR(1 downto 0);
       ALUControl:       out STD_LOGIC_VECTOR(1 downto 0));
end;

architecture behave of decoder is
  component mainfsm
    port(clk, reset:       in  STD_LOGIC;
         Op:               in  STD_LOGIC_VECTOR(1 downto 0);
         Funct:            in  STD_LOGIC_VECTOR(5 downto 0);
         IRWrite:          out STD_LOGIC;
         AdrSrc, ALUSrcA:  out STD_LOGIC;
         ALUSrcB:          out STD_LOGIC_VECTOR(1 downto 0);
         ResultSrc:        out STD_LOGIC_VECTOR(1 downto 0);
         NextPC, RegW:     out STD_LOGIC;
         MemW, Branch:     out STD_LOGIC;
         ALUOp:            out STD_LOGIC);
    end component;
  signal Branch, ALUOp: STD_LOGIC;
begin
  -- Main FSM
  fsm: mainfsm port map(clk, reset, Op, Funct, 
                        IRWrite, AdrSrc, 
                        ALUSrcA, ALUSrcB, ResultSrc,
                        NextPC, RegW, MemW, Branch, ALUOp);

  process(all) begin -- ALU Decoder
    if (ALUOp) then
      case Funct(4 downto 1) is
        when "0100" => ALUControl <= "00"; -- ADD
        when "0010" => ALUControl <= "01"; -- SUB
        when "0000" => ALUControl <= "10"; -- AND
        when "1100" => ALUControl <= "11"; -- ORR
        when others => ALUControl <= "--"; -- unimplemented
      end case;
      FlagW(1) <= Funct(0);
      FlagW(0) <= Funct(0) and (not ALUControl(1));
    else 
      ALUControl <= "00";
      FlagW <= "00";
    end if;
  end process;
  
  -- PC Logic
  PCS <= ((and Rd) and RegW) or Branch;

  -- Instr Decoder
  ImmSrc <= Op;
  RegSrc(0) <= '1' when (Op = 2B"10") else '0';
  RegSrc(1) <= '1' when (Op = 2B"01") else '0';
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;
entity mainfsm is
  port(clk, reset:       in  STD_LOGIC;
       Op:               in  STD_LOGIC_VECTOR(1 downto 0);
       Funct:            in  STD_LOGIC_VECTOR(5 downto 0);
       IRWrite:          out STD_LOGIC;
       AdrSrc, ALUSrcA:  out STD_LOGIC;
       ALUSrcB:          out STD_LOGIC_VECTOR(1 downto 0);
       ResultSrc:        out STD_LOGIC_VECTOR(1 downto 0);
       NextPC, RegW:     out STD_LOGIC;
       MemW, Branch:     out STD_LOGIC;
       ALUOp:            out STD_LOGIC);
end;

architecture synth of mainfsm is
  type statetype is (FETCH, DECODE, MEMADR, MEMRD, MEMWB, MEMWR, 
                     EXECUTER, EXECUTEI, ALUWB, BR, UNKNOWN);
  signal state, nextstate: statetype;
  signal controls: STD_LOGIC_VECTOR(11 downto 0);
begin
  --state register
  process(clk, reset) begin
    if reset then state <= FETCH;
    elsif rising_edge(clk) then
      state <= nextstate;
    end if;
  end process;

  -- next state logic
  process(all) begin
    case state is
      when FETCH  =>       nextstate <= DECODE;
      when DECODE =>
        case Op is
          when "00" =>     nextstate <= ExecuteI when (Funct(5) = '1') 
                                   else EXECUTER;
          when "01" =>     nextstate <= MEMADR;
          when "10" =>     nextstate <= BR;
          when others =>   nextstate <= UNKNOWN;
        end case;
      when EXECUTER =>     nextstate <= ALUWB;
      when EXECUTEI =>     nextstate <= ALUWB;
      when MEMADR   =>     nextstate <= MEMRD when (Funct(0) = '1')
                                   else MEMWR;
      when MEMRD    =>     nextstate <= MEMWB;
      when others   =>     nextstate <= FETCH;
    end case;
  end process;

  -- state-dependent output logic
  process(all) begin
    case state is
      when FETCH => 	controls <= 12B"100010101100"; 
      when DECODE => 	controls <= 12B"000000101100";      
      when EXECUTER =>	controls <= 12B"000000000001";
      when EXECUTEI =>	controls <= 12B"000000000011";
      when ALUWB =>   	controls <= 12B"000100000000";
      when MEMADR =>  	controls <= 12B"000000000010";
      when MEMWR =>   	controls <= 12B"001001000000";
      when MEMRD =>   	controls <= 12B"000001000000";
      when MEMWB =>   	controls <= 12B"000100010000";
      when BR =>    	controls <= 12B"010000100010";
      when others => 	controls <= "XXXXXXXXXXXX";
    end case;
  end process;

  (NextPC, Branch, MemW, RegW, IRWrite,
   AdrSrc, ResultSrc,   
   ALUSrcA, ALUSrcB, ALUOp) <= controls;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;
entity condlogic is -- Conditional logic
    port(clk, reset:        in  STD_LOGIC;
         Cond:              in  STD_LOGIC_VECTOR(3 downto 0);
         ALUFlags:          in  STD_LOGIC_VECTOR(3 downto 0);
         FlagW:             in  STD_LOGIC_VECTOR(1 downto 0);
         PCS, NextPC:       in  STD_LOGIC;
         RegW, MemW:        in  STD_LOGIC;
         PCWrite, RegWrite: out STD_LOGIC;
         MemWrite:          out STD_LOGIC);
end;

architecture behave of condlogic is
  component condcheck
    port(Cond:           in  STD_LOGIC_VECTOR(3 downto 0);
         Flags:          in  STD_LOGIC_VECTOR(3 downto 0);
         CondEx:         out STD_LOGIC);
  end component;
  component flopenr generic(width: integer);
    port(clk, reset, en: in  STD_LOGIC;
         d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
         q:          out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;
  component flopr generic(width: integer);
    port(clk, reset: in  STD_LOGIC;
         d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
         q:          out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;

  signal FlagWrite:     STD_LOGIC_VECTOR(1 downto 0);
  signal Flags:         STD_LOGIC_VECTOR(3 downto 0);
  signal CondEx:        STD_LOGIC_VECTOR(0 downto 0);
  signal CondExDelayed: STD_LOGIC_VECTOR(0 downto 0);
begin
  flagreg1: flopenr generic map(2)
    port map(clk, reset, FlagWrite(1), 
             ALUFlags(3 downto 2), Flags(3 downto 2));
  flagreg0: flopenr generic map(2)
    port map(clk, reset, FlagWrite(0), 
             ALUFlags(1 downto 0), Flags(1 downto 0));
  cc: condcheck port map(Cond, Flags, CondEx(0));
  condreg: flopr generic map(1)
    port map(clk, reset, CondEx, CondExDelayed); 


  FlagWrite <= FlagW and (CondEx(0), CondEx(0)); 
  RegWrite  <= RegW  and CondExDelayed(0);
  MemWrite  <= MemW  and CondExDelayed(0);
  PCWrite   <= (PCS  and CondExDelayed(0)) or NextPC;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;
entity condcheck is 
  port(Cond:           in  STD_LOGIC_VECTOR(3 downto 0);
       Flags:          in  STD_LOGIC_VECTOR(3 downto 0);
       CondEx:         out STD_LOGIC);
end;

architecture behave of condcheck is
  signal neg, zero, carry, overflow, ge: STD_LOGIC;
begin
  (neg, zero, carry, overflow) <= Flags;
  ge <= (neg xnor overflow);
  
  process(all) begin -- Condition checking
    case Cond is
      when "0000" => CondEx <= zero;
      when "0001" => CondEx <= not zero;
      when "0010" => CondEx <= carry;
      when "0011" => CondEx <= not carry;
      when "0100" => CondEx <= neg;
      when "0101" => CondEx <= not neg;
      when "0110" => CondEx <= overflow;
      when "0111" => CondEx <= not overflow;
      when "1000" => CondEx <= carry and (not zero);
      when "1001" => CondEx <= not(carry and (not zero));
      when "1010" => CondEx <= ge;
      when "1011" => CondEx <= not ge;
      when "1100" => CondEx <= (not zero) and ge;
      when "1101" => CondEx <= not ((not zero) and ge);
      when "1110" => CondEx <= '1';
      when others => CondEx <= '-';
    end case;
  end process;
end;
     
library IEEE; use IEEE.STD_LOGIC_1164.all; 
entity datapath is  
  port(clk, reset:        in  STD_LOGIC;
       Adr:               out STD_LOGIC_VECTOR(31 downto 0);
       WriteData:         out STD_LOGIC_VECTOR(31 downto 0);
       ReadData:          in  STD_LOGIC_VECTOR(31 downto 0);
       Instr:             out STD_LOGIC_VECTOR(31 downto 0);
       ALUFlags:          out STD_LOGIC_VECTOR(3 downto 0);
       PCWrite:           in  STD_LOGIC;
       RegWrite:          in  STD_LOGIC;
       IRWrite:           in  STD_LOGIC;
       AdrSrc:            in  STD_LOGIC;
       RegSrc:            in  STD_LOGIC_VECTOR(1 downto 0);
       ALUSrcA:           in  STD_LOGIC;
       ALUSrcB:           in  STD_LOGIC_VECTOR(1 downto 0);
       ResultSrc:         in  STD_LOGIC_VECTOR(1 downto 0);
       ImmSrc:            in  STD_LOGIC_VECTOR(1 downto 0);
       ALUControl:        in  STD_LOGIC_VECTOR(1 downto 0));
end;

architecture struct of datapath is
  component alu
    port(a, b:       in  STD_LOGIC_VECTOR(31 downto 0);
         ALUControl: in  STD_LOGIC_VECTOR(1 downto 0);
         Result:     buffer STD_LOGIC_VECTOR(31 downto 0);
         ALUFlags:      out STD_LOGIC_VECTOR(3 downto 0));
  end component;
  component regfile
    port(clk:           in  STD_LOGIC;
         we3:           in  STD_LOGIC;
         ra1, ra2, wa3: in  STD_LOGIC_VECTOR(3 downto 0);
         wd3, r15:      in  STD_LOGIC_VECTOR(31 downto 0);
         rd1, rd2:      out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component adder
    port(a, b: in  STD_LOGIC_VECTOR(31 downto 0);
         y:    out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component extend
    port(Instr:  in  STD_LOGIC_VECTOR(23 downto 0);
         ImmSrc: in  STD_LOGIC_VECTOR(1 downto 0);
         ExtImm: out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component flopenr generic(width: integer);
    port(clk, reset, en: in  STD_LOGIC;
         d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
         q:          out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;
  component flopr generic(width: integer);
    port(clk, reset: in  STD_LOGIC;
         d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
         q:          out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;
  component mux2 generic(width: integer);
    port(d0, d1: in  STD_LOGIC_VECTOR(width-1 downto 0);
         s:      in  STD_LOGIC;
         y:      out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;
  component mux3 generic(width: integer);
    port(d0, d1, d2: in  STD_LOGIC_VECTOR(width-1 downto 0);
         s:          in  STD_LOGIC_VECTOR(1 downto 0);
         y:          out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;  signal PCNext, PC: STD_LOGIC_VECTOR(31 downto 0);
  signal ExtImm, SrcA, SrcB:       STD_LOGIC_VECTOR(31 downto 0);
  signal Result:                   STD_LOGIC_VECTOR(31 downto 0);
  signal Data, RD1, RD2, A:        STD_LOGIC_VECTOR(31 downto 0);
  signal ALUResult, ALUOut:        STD_LOGIC_VECTOR(31 downto 0);
  signal RA1, RA2:                 STD_LOGIC_VECTOR(3 downto 0);
begin
  -- next PC logic
  pcreg: flopenr generic map(32) 
    port map(clk, reset, PCWrite, Result, PC);

  -- memory logic
  adrmux: mux2 generic map(32)
    port map(PC, ALUOut, AdrSrc, Adr);
  ir: flopenr generic map(32) 
    port map(clk, reset, IRWrite, ReadData, Instr);
  datareg: flopr generic map(32) 
    port map(clk, reset, ReadData, Data);

  -- register file logic
  ra1mux: mux2 generic map (4)
    port map(Instr(19 downto 16), "1111", RegSrc(0), RA1);
  ra2mux: mux2 generic map (4) port map(Instr(3 downto 0), 
             Instr(15 downto 12), RegSrc(1), RA2);
  rf: regfile port map(clk, RegWrite, RA1, RA2, 
                      Instr(15 downto 12), Result, Result, 
                      RD1, RD2);
  srcareg: flopr generic map(32)
    port map(clk, reset, RD1, A);
  wdreg: flopr generic map(32) 
    port map(clk, reset, RD2, WriteData);
  ext: extend port map(Instr(23 downto 0), ImmSrc, ExtImm);

  -- ALU logic
  srcamux: mux2 generic map(32) 
    port map(A, PC, ALUSrcA, SrcA);
  srcbmux: mux3 generic map(32) 
    port map(WriteData, ExtImm, 32D"4", ALUSrcB, SrcB);
  i_alu: alu port map(SrcA, SrcB, ALUControl, ALUResult, ALUFlags);
  aluoutreg: flopr generic map(32) 
    port map(clk, reset, ALUResult, ALUOut);
  resmux: mux3 generic map(32) 
    port map(ALUOut, Data, ALUResult, ResultSrc, Result);
end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 
use IEEE.NUMERIC_STD_UNSIGNED.all;
entity regfile is -- three-port register file
  port(clk:           in  STD_LOGIC;
       we3:           in  STD_LOGIC;
       ra1, ra2, wa3: in  STD_LOGIC_VECTOR(3 downto 0);
       wd3, r15:      in  STD_LOGIC_VECTOR(31 downto 0);
       rd1, rd2:      out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of regfile is
  type ramtype is array (31 downto 0) of 
    STD_LOGIC_VECTOR(31 downto 0);
  signal mem: ramtype;
begin
  process(clk) begin
    if rising_edge(clk) then
       if we3 = '1' then mem(to_integer(wa3)) <= wd3;
       end if;
    end if;
  end process;
  process(all) begin
    if (to_integer(ra1) = 15) then rd1 <= r15; 
    else rd1 <= mem(to_integer(ra1));
    end if;
    if (to_integer(ra2) = 15) then rd2 <= r15; 
    else rd2 <= mem(to_integer(ra2));
    end if;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 
use IEEE.NUMERIC_STD_UNSIGNED.all;
entity adder is -- adder
  port(a, b: in  STD_LOGIC_VECTOR(31 downto 0);
       y:    out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of adder is
begin
  y <= a + b;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;
entity extend is 
  port(Instr:  in  STD_LOGIC_VECTOR(23 downto 0);
       ImmSrc: in  STD_LOGIC_VECTOR(1 downto 0);
       ExtImm: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of extend is
begin
  process(all) begin
    case ImmSrc is
      when "00"   => ExtImm <= (X"000000", Instr(7 downto 0));
      when "01"   => ExtImm <= (X"00000", Instr(11 downto 0));
      when "10"   => ExtImm <= (Instr(23), Instr(23), Instr(23), 
        Instr(23), Instr(23), Instr(23), Instr(23 downto 0), "00");
      when others => ExtImm <= X"--------";
    end case;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;  
entity flopenr is -- flip-flop with enable and asynchronous reset
  generic(width: integer);
  port(clk, reset, en: in  STD_LOGIC;
       d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
       q:          out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture asynchronous of flopenr is
begin
  process(clk, reset) begin
    if reset then q <= (others => '0');
    elsif rising_edge(clk) then
      if en then 
        q <= d;
      end if;
    end if;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;  
entity flopr is -- flip-flop with asynchronous reset
  generic(width: integer);
  port(clk, reset: in  STD_LOGIC;
       d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
       q:          out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture asynchronous of flopr is
begin
  process(clk, reset) begin
    if reset then  q <= (others => '0');
    elsif rising_edge(clk) then
      q <= d;
    end if;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;
entity mux2 is -- two-input multiplexer
  generic(width: integer);
  port(d0, d1: in  STD_LOGIC_VECTOR(width-1 downto 0);
       s:      in  STD_LOGIC;
       y:      out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture behave of mux2 is
begin
  y <= d1 when s else d0;
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;
entity mux3 is -- three-input multiplexer
  generic(width: integer);
  port(d0, d1, d2: in  STD_LOGIC_VECTOR(width-1 downto 0);
       s:          in  STD_LOGIC_VECTOR(1 downto 0);
       y:          out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture behave of mux3 is
begin
  process(all) begin
    case s is
      when "00"    => y <= d0;
      when "01"    => y <= d1;
      when "10"    => y <= d2;
      when others  => y <= d0;
    end case;
  end process;
end;



library IEEE; use IEEE.STD_LOGIC_1164.all; 
use IEEE.NUMERIC_STD_UNSIGNED.all;
entity alu is 
  port(a, b:       in  STD_LOGIC_VECTOR(31 downto 0);
       ALUControl: in  STD_LOGIC_VECTOR(1 downto 0);
       Result:     buffer STD_LOGIC_VECTOR(31 downto 0);
       ALUFlags:      out STD_LOGIC_VECTOR(3 downto 0));
end;

architecture behave of alu is
  signal condinvb: STD_LOGIC_VECTOR(31 downto 0);
  signal sum:      STD_LOGIC_VECTOR(32 downto 0);
  signal neg, zero, carry, overflow: STD_LOGIC;
begin
  condinvb <= not b when ALUControl(0) else b;
  sum <= ('0', a) + ('0', condinvb) + ALUControl(0);

  process(all) begin
    case? ALUControl(1 downto 0) is
      when "0-"   => result <= sum(31 downto 0); 
      when "10"   => result <= a and b; 
      when "11"   => result <= a or b; 
      when others => result <= (others => '-');
    end case?;
  end process;

  neg      <= Result(31);
  zero     <= '1' when (Result = 0) else '0';
  carry    <= (not ALUControl(1)) and sum(32);
  overflow <= (not ALUControl(1)) and
             (not (a(31) xor b(31) xor ALUControl(0))) and
             (a(31) xor sum(31));
  ALUFlags    <= (neg, zero, carry, overflow);
end;


