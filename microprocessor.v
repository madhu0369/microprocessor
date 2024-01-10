//Design of 16-bit MICROPROCESSOR

`timescale 1ns / 1ps
 
//Fields of INSTRUCTION REGISTER
`define opcode    IR[31:27]   // OPCODE SIZE - 5bit
`define rdst      IR[26:22]  // Destination Register - 5bit
`define rsrc1     IR[21:17]  // Address of first operand...size - 5bit
`define imm_mode  IR[16]     // Determines the addressing mode...MODE - 0 = Register addresssing mode...MODE - 1 = Immediate addressing mode..size = 1bit
`define rsrc2     IR[15:11]  //Address of second operand...size = 5bit
`define isrc      IR[15:0]   // Specifies the operand value in immediate addressing mode when mode = 1...size = 16bit
 
//INSTRUCTION SET

//MOVE OPERATIONS
`define movsgpr        5'b00000   //useful while doing multiplication operation
`define mov            5'b00001   //To copy the content of one register to another


//ARTHIMETIC OPERATIONS
`define add            5'b00010
`define sub            5'b00011
`define mul            5'b00100
`define div            5'b00101

//LOGICAL OPERATIONS
`define ror            5'b00110
`define rand           5'b00111
`define rxor           5'b01000
`define rxnor          5'b01001
`define rnand          5'b01010
`define rnor           5'b01011
`define rnot           5'b01100

//LOAD & STORE INSTRUCTIONS
 
`define storereg       5'b01101   //store content of register in data memory
`define storedin       5'b01110   // store content of din bus in data memory
`define sendreg        5'b01111   // send data from DM to register
 
// Jump and Branch instructions
//Opcode - 16 is a JUMP INSTRUCTION...No condition will be satisfied to jump
//From Opcode - 17 to Opcode - 24 are Branch instructions...Condition must satisfy to jump
`define jump           5'b10000  //jump to address (direct jump)
`define jcarry         5'b10001  //jump if carry is set
`define jnocarry       5'b10010  //jump if carry bit is low
`define jsign          5'b10011  //jump if sign is set
`define jnosign        5'b10100  //jump if sign bit is low
`define jzero          5'b10101  //jump if zero is set
`define jnozero        5'b10110  //jump if zero bit is low
`define joverflow      5'b10111 //jump if overflow is set
`define jnooverflow    5'b11000 //jump if overlow bit is low

//HALT INSTRUCTION
`define halt           5'b11001
 
module microprocessor(input clk,sys_rst,
input [15:0] din,
output reg [15:0] dout);

//adding program and data memory(Harvard Architecture)
reg [31:0] inst_mem [31:0]; //program memory...width of memory = 32 bit...Depth of Memory = 32 (from location 0 to 31)
reg [15:0] data_mem [15:0]; //data memory...width of memory = 16 bit...Depth of memory = 16 (from location 0 to 15) 


                          
                          
reg [31:0] IR;            //Instruction Register of size 32-bit
                          // instruction register  <--ir[31:27]--><--ir[26:22]--><--ir[21:17]--><--ir[16]--><--ir[15:11]--><--ir[10:0]-->
                          //fields                 <---  oper  --><--   rdest --><--   rsrc1 --><--modesel--><--  rsrc2 --><--unused  -->             
                          //fields                 <---  oper  --><--   rdest --><--   rsrc1 --><--modesel--><--  immediate_data      -->      
 
reg [15:0] GPR [31:0] ;  // 32 General Purpose Registers of size 16-bit... from GPR[0] to GPR[31]...
 
 
 
reg [15:0] SGPR ;     //To store MSB of multiplication
 
reg [31:0] mul_res;  // To store the result of multiplication
 
 //condition flags
reg sign = 0, zero = 0, overflow = 0, carry = 0;
reg [16:0] temp_sum; //Temporarily storing the result of Addition 

reg jmp_flag = 0;
reg stop = 0;
 
task decode_inst();
begin

jmp_flag = 1'b0;
stop     = 1'b0;

case(`opcode)

`movsgpr: begin
   GPR[`rdst] = SGPR;   
end
 

`mov : begin
   if(`imm_mode)
        GPR[`rdst]  = `isrc;
   else
       GPR[`rdst]   = GPR[`rsrc1];
end
 

`add : begin
      if(`imm_mode)
        GPR[`rdst]   = GPR[`rsrc1] + `isrc;
     else
        GPR[`rdst]   = GPR[`rsrc1] + GPR[`rsrc2];
end
 

 
`sub : begin
      if(`imm_mode)
        GPR[`rdst]  = GPR[`rsrc1] - `isrc;
     else
       GPR[`rdst]   = GPR[`rsrc1] - GPR[`rsrc2];
end
 

 
`mul : begin
      if(`imm_mode)
        mul_res   = GPR[`rsrc1] * `isrc;
     else
        mul_res   = GPR[`rsrc1] * GPR[`rsrc2];
        
     GPR[`rdst]   =  mul_res[15:0];
     SGPR         =  mul_res[31:16];
end

`div : begin
     if(`imm_mode)
     GPR[`rdst] = GPR[`rsrc1]/`isrc;
     else
      GPR[`rdst] = GPR[`rsrc1]/GPR[`rsrc2];
 end

//Bitwise OR Operation
 
`ror : begin
      if(`imm_mode)
        GPR[`rdst]  = GPR[`rsrc1] | `isrc;
     else
       GPR[`rdst]   = GPR[`rsrc1] | GPR[`rsrc2];
end
 
//Bitwise AND Operation
 
`rand : begin
      if(`imm_mode)
        GPR[`rdst]  = GPR[`rsrc1] & `isrc;
     else
       GPR[`rdst]   = GPR[`rsrc1] & GPR[`rsrc2];
end
 
//Bitwise XOR Operation
 
`rxor : begin
      if(`imm_mode)
        GPR[`rdst]  = GPR[`rsrc1] ^ `isrc;
     else
       GPR[`rdst]   = GPR[`rsrc1] ^ GPR[`rsrc2];
end
 
//Bitwise XNOR Operation
 
`rxnor : begin
      if(`imm_mode)
        GPR[`rdst]  = GPR[`rsrc1] ~^ `isrc;
     else
        GPR[`rdst]   = GPR[`rsrc1] ~^ GPR[`rsrc2];
end
 
//Bitwise NAND Operation
 
`rnand : begin
      if(`imm_mode)
        GPR[`rdst]  = ~(GPR[`rsrc1] & `isrc);
     else
       GPR[`rdst]   = ~(GPR[`rsrc1] & GPR[`rsrc2]);
end
 
//Bitwise NOR Operation
`rnor : begin
      if(`imm_mode)
        GPR[`rdst]  = ~(GPR[`rsrc1] | `isrc);
     else
       GPR[`rdst]   = ~(GPR[`rsrc1] | GPR[`rsrc2]);
end
 
//NOT OPERATION
 
`rnot : begin
      if(`imm_mode)
        GPR[`rdst]  = ~(`isrc);
     else
        GPR[`rdst]   = ~(GPR[`rsrc1]);
end        
        

`storedin: begin
   data_mem[`isrc] = din;
end
 
 
`storereg: begin
   data_mem[`isrc] = GPR[`rsrc1];
end
 
`sendreg: begin
  GPR[`rdst] =  data_mem[`isrc];
end
 
 //JUMP & BRANCH INSTRUCTIONS
 
 `jump: begin
 jmp_flag = 1'b1;
end
 
`jcarry: begin
  if(carry == 1'b1)
     jmp_flag = 1'b1;
   else
     jmp_flag = 1'b0; 
end
 
`jsign: begin
  if(sign == 1'b1)
     jmp_flag = 1'b1;
   else
     jmp_flag = 1'b0; 
end
 
`jzero: begin
  if(zero == 1'b1)
     jmp_flag = 1'b1;
   else
     jmp_flag = 1'b0; 
end
 
 
`joverflow: begin
  if(overflow == 1'b1)
     jmp_flag = 1'b1;
   else
     jmp_flag = 1'b0; 
end
 
`jnocarry: begin
  if(carry == 1'b0)
     jmp_flag = 1'b1;
   else
     jmp_flag = 1'b0; 
end
 
`jnosign: begin
  if(sign == 1'b0)
     jmp_flag = 1'b1;
   else
     jmp_flag = 1'b0; 
end
 
`jnozero: begin
  if(zero == 1'b0)
     jmp_flag = 1'b1;
   else
     jmp_flag = 1'b0; 
end
 
`jnooverflow: begin
  if(overflow == 1'b0)
     jmp_flag = 1'b1;
   else
     jmp_flag = 1'b0; 
end
 
`halt : begin
stop = 1'b1;
end
endcase
end
endtask

 
task decode_condflag();
begin
 
//SIGN BIT FLAG
if(`opcode == `mul)
  sign = SGPR[15];  //For multiplication result
else
  sign = GPR[`rdst][15]; // For either addition or subtraction result
 
//CARRY BIT FLAG
 
if(`opcode == `add)
   begin
      if(`imm_mode)
         begin
         temp_sum = GPR[`rsrc1] + `isrc;
         carry    = temp_sum[16]; 
         end
      else
         begin
         temp_sum = GPR[`rsrc1] + GPR[`rsrc2];
         carry    = temp_sum[16]; 
         end   end
      else
      begin
        carry  = 1'b0;
      end
 
 //ZERO BIT FLAG
if(`opcode == `mul)
  zero =  ~((|SGPR[15:0]) | (|GPR[`rdst]));
else
  zero =  ~(|GPR[`rdst]); 
 
 
//OVERFLOW BIT FLAG 
 
if(`opcode == `add)
     begin
     if(`imm_mode)
         overflow = ( (~GPR[`rsrc1][15] & ~IR[15] & GPR[`rdst][15] ) | (GPR[`rsrc1][15] & IR[15] & ~GPR[`rdst][15]) );
     else
         overflow = ( (~GPR[`rsrc1][15] & ~GPR[`rsrc2][15] & GPR[`rdst][15]) | (GPR[`rsrc1][15] & GPR[`rsrc2][15] & ~GPR[`rdst][15]));
     end
     else if(`opcode == `sub)
     begin
       if(`imm_mode)
         overflow = ( (~GPR[`rsrc1][15] & IR[15] & GPR[`rdst][15] ) | (GPR[`rsrc1][15] & ~IR[15] & ~GPR[`rdst][15]) );
       else
         overflow = ( (~GPR[`rsrc1][15] & GPR[`rsrc2][15] & GPR[`rdst][15]) | (GPR[`rsrc1][15] & ~GPR[`rsrc2][15] & ~GPR[`rdst][15]));
     end 
     else
     begin
     overflow = 1'b0;
     end
 
end
endtask

//READING THE PROGRAM from PROGRAM MEMORY
 
initial begin
$readmemb("inst.mem",inst_mem);
end

//CONTROL UNIT design using FINITE STATE MACHINE
//FSM STATES
parameter    idle = 0;                  //idle : check reset state
parameter    fetch_inst = 1;            // fetch_inst : load instrcution from Program memory
parameter    dec_exec_inst = 2;         // dec_exec_inst : execute instruction + update condition flag
parameter    next_inst = 3;             // next_inst : next instruction to be fetched
parameter    sense_halt = 4;            //checking the halt condition
parameter    delay_next_inst = 5;       //state used to delay the next instruction by 4 clock ticks

reg [2:0] state = idle, next_state = idle;
reg [2:0] count = 0;
integer PC = 0;

//RESET DECODER ( To sense the reset condition)
always@(posedge clk)
begin
 if(sys_rst)
   state <= idle;
 else
   state <= next_state; 
end
 
//NEXT DECODER + OUTPUT DECODER
 
always@(*)
begin
  case(state)
   idle: begin
     IR         = 32'h0;
     PC         = 0;
     next_state = fetch_inst;
   end
 
  fetch_inst: begin
    IR          =  inst_mem[PC];   
    next_state  = dec_exec_inst;
  end
  
  dec_exec_inst: begin
    decode_inst();
    decode_condflag();
    next_state  = delay_next_inst;   
  end
  
  
  delay_next_inst:begin
  if(count < 4)
       next_state  = delay_next_inst;       
     else
       next_state  = next_inst;
  end
  
  next_inst: begin
      next_state = sense_halt;
      if(jmp_flag == 1'b1)
        PC = `isrc;
      else
        PC = PC + 1;
  end
  
  
 sense_halt: begin
    if(stop == 1'b0)
      next_state = fetch_inst;
    else if(sys_rst == 1'b1)
      next_state = idle;
    else
      next_state = sense_halt;
 end
  
  default : next_state = idle;
  
endcase  
end

// COUNT UPDATE
 
always@(posedge clk)
begin
case(state)
 
 idle : begin
    count <= 0;
 end
 
 fetch_inst: begin
   count <= 0;
 end
 
 dec_exec_inst : begin
   count <= 0;    
 end  
 
 delay_next_inst: begin
   count  <= count + 1;
 end
 
  next_inst : begin
    count <= 0;
 end
 
  sense_halt : begin
    count <= 0;
 end
 
 default : count <= 0;
 
  
endcase
end
endmodule