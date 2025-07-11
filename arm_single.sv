module testbench();

  logic        clk, clkTimer;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite, clkTimer);
  
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

always
    begin
      clkTimer <= 1; # 500000; clkTimer <= 0; # 500000;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
         $display("Wrote %d to address %d", WriteData, DataAdr);
        end
      end
    //always @(negedge clk)
  //   begin
  //     if(MemWrite) begin
  //       if(DataAdr === 100 & WriteData === 7) begin
  //         $display("Simulation succeeded");
  //         $stop;
  //       end else if (DataAdr !== 96) begin
  //         $display("Simulation failed");
  //         $stop;
  //       end
  //     end
  //   end

endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite,
           input logic         clkTimer);

  logic [31:0] PC, Instr, ReadData;
  logic [31:0] timestamp, CountTrigger, CountCheck;
  
  // instantiate processor and memories
  arm arm(clk, reset, PC, Instr, MemWrite, DataAdr, 
          WriteData, ReadData);
  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData, CountCheck, timestamp, CountTrigger);

  timer timer(clkTimer, timestamp, CountTrigger, CountCheck);
endmodule

module timer(input logic clkTimer,
             input logic [31:0] timestamp,
             input logic [31:0] CountTrigger,
             output logic [31:0] CountCheck);

  logic [31:0] count = 32'b0;

  always_ff @(posedge clkTimer) begin
    if (CountTrigger[1] & count < timestamp)
      count++;
        
    else if (CountTrigger[1] === 1'b0 || count === timestamp) begin
      count <= 32'b0;
      CountCheck = 32'd9;
      end
  end

endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd,
            input logic [31:0] outTimer,
             output logic [31:0] Timestamp,
             output logic [31:0] TimeTrigger);

  logic [31:0] RAM[63:0];

  assign rd = RAM[a[31:2]]; // word aligned
  assign Timestamp = RAM[40]; //time-stamp para o timer
  assign TimeTrigger = RAM[41]; //flag que diz ao timer para comecar a contagem

  always_ff @(posedge clk) begin
    if (we)
     RAM[a[31:2]] <= wd;

    if (outTimer[0] === 1'b1)
      RAM[41] <= outTimer; //flag do timer que diz que contagem foi completada
   end


endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("memfile2.dat",RAM);
      //$readmemh("memfileT.dat",RAM);

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module arm(input  logic        clk, reset,
           output logic [31:0] PC,
           input  logic [31:0] Instr,
           output logic        MemWrite,
           output logic [31:0] ALUResult, WriteData,
           input  logic [31:0] ReadData);

  logic [3:0] ALUFlags;
  logic       RegWrite, 
              ALUSrc, MemtoReg, PCSrc, MOVFlag, BLFlag;
  logic [1:0] RegSrc, ImmSrc;
  logic [2:0] ALUControl;

  controller c(clk, reset, Instr[31:12], ALUFlags, 
               RegSrc, RegWrite, ImmSrc, 
               ALUSrc, ALUControl,
               MemWrite, MemtoReg, PCSrc, MOVFlag, BLFlag);
  datapath dp(clk, reset, 
              RegSrc, RegWrite, ImmSrc,
              ALUSrc, ALUControl,
              MemtoReg, PCSrc, MOVFlag, BLFlag,
              ALUFlags, PC, Instr,
              ALUResult, WriteData, ReadData);
endmodule

module controller(input  logic         clk, reset,
                  input  logic [31:12] Instr,
                  input  logic [3:0]   ALUFlags,
                  output logic [1:0]   RegSrc,
                  output logic         RegWrite,
                  output logic [1:0]   ImmSrc,
                  output logic         ALUSrc, 
                  output logic [2:0]   ALUControl,
                  output logic         MemWrite, MemtoReg,
                  output logic         PCSrc,
                  output logic         MOVFlag, BLFlag);

  logic [1:0] FlagW;
  logic       PCS, RegW, MemW, NoWrite;
  
  decoder dec(Instr[27:26], Instr[25:20], Instr[15:12],
              FlagW, PCS, RegW, MemW,
              MemtoReg, ALUSrc, NoWrite, MOVFlag, BLFlag, ImmSrc, RegSrc, ALUControl);
  condlogic cl(clk, reset, Instr[31:28], ALUFlags,
               FlagW, PCS, RegW, MemW, NoWrite,
               PCSrc, RegWrite, MemWrite);
endmodule

module decoder(input  logic [1:0] Op,
               input  logic [5:0] Funct,
               input  logic [3:0] Rd,
               output logic [1:0] FlagW,
               output logic       PCS, RegW, MemW,
               output logic       MemtoReg, ALUSrc, NoWrite, MOVFlag, BLFlag,
               output logic [1:0] ImmSrc, RegSrc,
               output logic [2:0] ALUControl);

  logic [9:0] controls;
  logic       Branch, ALUOp;

  // Main Decoder
  
  always_comb
  	case(Op)
  	                        // Data processing immediate
  	  2'b00:  if (Funct[5]) begin 
            controls = 10'b0000101001;
            BLFlag = 1'b0;
          end 

  	                        // Data processing register
  	         else begin           
              controls = 10'b0000001001;
              BLFlag = 1'b0; 
          end
          
  	                        // LDR
  	  2'b01: if (Funct[0]) begin 
              controls = 10'b0001111000; 
              BLFlag = 1'b0;
          end
  	                        // STR
  	         else begin         
              controls = 10'b1001110100; 
              BLFlag = 1'b0;
          end

  	                        // B
  	  2'b10:  if (Funct[5:4] === 2'b00) begin               
              controls = 10'b0110100010; 
              BLFlag = 1'b0;
          end
                          //BL
          else begin
              controls = 10'b0110101010; 
              BLFlag = 1'b1;
          end
  	                        // Unimplemented
  	  default: begin             
              controls = 10'bx;  
              BLFlag = 1'b0;  
      end      
  	endcase

  assign {RegSrc, ImmSrc, ALUSrc, MemtoReg, 
          RegW, MemW, Branch, ALUOp} = controls; 
          
  // ALU Decoder             
  always_comb
    if (ALUOp) begin                 // which DP Instr?
      case(Funct[4:1]) 
  	    4'b0100: begin
        ALUControl = 3'b000; // ADD
        MOVFlag = 1'b0;
        NoWrite = 1'b1;
      end
  	    4'b0010: begin
        ALUControl = 3'b001; // SUB
        MOVFlag = 1'b0;
        NoWrite = 1'b1;
      end
        4'b0000: begin 
        ALUControl = 3'b010; // AND
        MOVFlag = 1'b0;
        NoWrite = 1'b1;
      end
  	    4'b1100: begin
        ALUControl = 3'b011; // ORR
        MOVFlag = 1'b0;
        NoWrite = 1'b1;
      end
        4'b1010: begin
        ALUControl = 3'b001; //CMP (com base em SUB)
        MOVFlag = 1'b0;
        NoWrite = 1'b0; //se for CMP nao escreve
      end
        4'b1000: begin
        ALUControl = 3'b010; //TST (com base em AND)
        MOVFlag = 1'b0;
        NoWrite = 1'b0; //se for TST nao escreve
      end
        4'b0001: begin
        ALUControl = 3'b100; //EOR
        MOVFlag = 1'b0;
        NoWrite = 1'b1;
      end
        // MOV
        4'b1101: begin
        ALUControl = 3'b000; //ADD (com base em soma)
        MOVFlag = 1'b1;
        NoWrite = 1'b1;
      end
  	    default: begin 
        ALUControl = 3'bx;  // unimplemented
        MOVFlag = 1'b0;
        NoWrite = 1'b1;
      end
    endcase
      // update flags if S bit is set 
	// (C & V only updated for arith instructions)
      FlagW[1]      = Funct[0]; // FlagW[1] = S-bit
	// FlagW[0] = S-bit & (ADD | SUB)
      FlagW[0]      = Funct[0] & 
        (ALUControl == 3'b000 | ALUControl == 3'b001); 
    end else begin
      ALUControl = 3'b000; // add for non-DP instructions
      FlagW      = 3'b000; // don't update Flags
    end
              
  // PC Logic
  assign PCS  = ((Rd == 4'b1111) & RegW) | Branch; 
endmodule

module condlogic(input  logic       clk, reset,
                 input  logic [3:0] Cond,
                 input  logic [3:0] ALUFlags,
                 input  logic [1:0] FlagW,
                 input  logic       PCS, RegW, MemW, NoWrite,
                 output logic       PCSrc, RegWrite, MemWrite);
                 
  logic [1:0] FlagWrite;
  logic [3:0] Flags;
  logic       CondEx;

  flopenr #(2)flagreg1(clk, reset, FlagWrite[1], 
                       ALUFlags[3:2], Flags[3:2]);
  flopenr #(2)flagreg0(clk, reset, FlagWrite[0], 
                       ALUFlags[1:0], Flags[1:0]);

  // write controls are conditional
  condcheck cc(Cond, Flags, CondEx);
  assign FlagWrite = FlagW & {2{CondEx}};
  assign RegWrite  = NoWrite & RegW  & CondEx;
  assign MemWrite  = MemW  & CondEx;
  assign PCSrc     = PCS   & CondEx;
endmodule    

module condcheck(input  logic [3:0] Cond,
                 input  logic [3:0] Flags,
                 output logic       CondEx);
  
  logic neg, zero, carry, overflow, ge;
  
  assign {neg, zero, carry, overflow} = Flags;
  assign ge = (neg == overflow);
                  
  always_comb
    case(Cond)
      4'b0000: CondEx = zero;             // EQ
      4'b0001: CondEx = ~zero;            // NE
      4'b0010: CondEx = carry;            // CS
      4'b0011: CondEx = ~carry;           // CC
      4'b0100: CondEx = neg;              // MI
      4'b0101: CondEx = ~neg;             // PL
      4'b0110: CondEx = overflow;         // VS
      4'b0111: CondEx = ~overflow;        // VC
      4'b1000: CondEx = carry & ~zero;    // HI
      4'b1001: CondEx = ~(carry & ~zero); // LS
      4'b1010: CondEx = ge;               // GE
      4'b1011: CondEx = ~ge;              // LT
      4'b1100: CondEx = ~zero & ge;       // GT
      4'b1101: CondEx = ~(~zero & ge);    // LE
      4'b1110: CondEx = 1'b1;             // Always
      default: CondEx = 1'bx;             // undefined
    endcase
endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  RegSrc,
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic        ALUSrc,
                input  logic [2:0]  ALUControl,
                input  logic        MemtoReg,
                input  logic        PCSrc,
                input  logic        MOVFlag, BLFlag,
                output logic [3:0]  ALUFlags,
                output logic [31:0] PC,
                input  logic [31:0] Instr,
                output logic [31:0] ALUResult, WriteData,
                input  logic [31:0] ReadData);

  logic [31:0] PCNext, PCPlus4, PCPlus8;
  logic [31:0] ExtImm, SrcA, SrcB, Result, movSrcAresult;
  logic [3:0]  RA1, RA2;
  logic [31:0] ShiftResult;
  logic [3:0] a3result;
  logic [31:0] wd3result;

  // next PC logic
  mux2 #(32)  pcmux(PCPlus4, Result, PCSrc, PCNext);
  flopr #(32) pcreg(clk, reset, PCNext, PC);
  adder #(32) pcadd1(PC, 32'b100, PCPlus4);
  adder #(32) pcadd2(PCPlus4, 32'b100, PCPlus8);

  // register file logic
  mux2 #(4)   ra1mux(Instr[19:16], 4'b1111, RegSrc[0], RA1);
  mux2 #(4)   ra2mux(Instr[3:0], Instr[15:12], RegSrc[1], RA2);

  mux2 #(4)   a3mux(Instr[15:12], 4'b1110, BLFlag, a3result); // se for BL, envia 14 (LR) para A3
  mux2 #(32)  wd3mux(Result, PCPlus4, BLFlag, wd3result);   // se for BL, envia 'PC + 4' para WD3
  regfile     rf(clk, RegWrite, RA1, RA2,
                 a3result, wd3result, PCPlus8, 
                 SrcA, WriteData); 
  mux2 #(32)  resmux(ALUResult, ReadData, MemtoReg, Result);

  mux2 #(32) movSrcAmux(SrcA, 32'b0, MOVFlag, movSrcAresult); // Escolhe Src ou 0 a partir de MOVFlag
  extend      ext(Instr[23:0], ImmSrc, ExtImm);


  logic [31:0] RmValue;
  assign RmValue = WriteData;  // WriteData = saída de Rm (RA2)
  
  //calculo do shift
  shifter shift(Instr[11:7], Instr[6:5], RmValue, ShiftResult);

  // ALU logic
  mux2 #(32)  srcbmux(ShiftResult, ExtImm, ALUSrc, SrcB);
  alu         alu(movSrcAresult, SrcB, ALUControl, 
                  ALUResult, ALUFlags);
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [3:0]  ra1, ra2, wa3, 
               input  logic [31:0] wd3, r15,
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[14:0];

  // three ported register file
  // read two ports combinationally
  // write third port on rising edge of clock
  // register 15 reads PC+8 instead

  always_ff @(posedge clk)
    if (we3) rf[wa3] <= wd3;	

  assign rd1 = (ra1 == 4'b1111) ? r15 : rf[ra1];
  assign rd2 = (ra2 == 4'b1111) ? r15 : rf[ra2];
endmodule

module extend(input  logic [23:0] Instr,
              input  logic [1:0]  ImmSrc,
              output logic [31:0] ExtImm);
 
  always_comb
    case(ImmSrc) 
               // 8-bit unsigned immediate
      2'b00:   ExtImm = {24'b0, Instr[7:0]};  
               // 12-bit unsigned immediate 
      2'b01:   ExtImm = {20'b0, Instr[11:0]}; 
               // 24-bit two's complement shifted branch 
      2'b10:   ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00}; 
      default: ExtImm = 32'bx; // undefined
    endcase             
endmodule

module adder #(parameter WIDTH=8)
              (input  logic [WIDTH-1:0] a, b,
               output logic [WIDTH-1:0] y);
             
  assign y = a + b;
endmodule

module flopenr #(parameter WIDTH = 8)
                (input  logic             clk, reset, en,
                 input  logic [WIDTH-1:0] d, 
                 output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)   q <= 0;
    else if (en) q <= d;
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule


module alu(input  logic [31:0] a, b,
           input  logic [2:0]  ALUControl,
           output logic [31:0] Result,
           output logic [3:0]  ALUFlags);

  logic        neg, zero, carry, overflow;
  logic [31:0] condinvb;
  logic [32:0] sum;

  assign condinvb = ALUControl[0] ? ~b : b;
  assign sum = a + condinvb + ALUControl[0];

  always_comb
    casex (ALUControl[2:0])
      3'b00?: Result = sum;
      3'b010: Result = a & b;
      3'b011: Result = a | b;
      3'b100: Result = a ^ b; //EOR
    endcase

  assign neg      = Result[31];
  assign zero     = (Result == 32'b0);
  assign carry    = (ALUControl[1] == 1'b0) & sum[32];
  assign overflow = (ALUControl[1] == 1'b0) & 
                    ~(a[31] ^ b[31] ^ ALUControl[0]) & 
                    (a[31] ^ sum[31]); 
  assign ALUFlags    = {neg, zero, carry, overflow};
  endmodule

// Modulo para funcoes shift
module shifter(input logic [4:0] shamt5,
               input logic [1:0] sh,
               input logic [31:0] Rm,
               output logic [31:0] Rmshifted);

  logic [31:0] shamt32;
  logic signed [31:0] RmSigned;

  assign RmSigned = Rm;
  
  assign shamt32 = {27'b0, shamt5};

  always_comb
    casex (sh[1:0]) // tipo de shift
      2'b00: Rmshifted = Rm << shamt32; //LSL
      2'b01: Rmshifted = Rm >> shamt32; //LSR
      2'b10: Rmshifted = RmSigned >>> shamt32; //ASR
      2'b11: Rmshifted = Rmshifted;
    endcase

endmodule