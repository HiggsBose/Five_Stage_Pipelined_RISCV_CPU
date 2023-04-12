# Five_Stage_Pipelined_RSICV_CPU
A CPU with 5-stage Pipeline feature based on RISC-V RV32I ISA


# CPU Architecture
This CPU is implemented as the most common five-stage pipelined CPUs, with five discrete stages (IF, ID, EX, MEM, and WB). And it solves most of the hazard like data hazard and control hazard. The structural hazard is naturally avoided because we choose to seperate instruction memory and data memory, and we write the register file at the positive edge of the clock signal while reading the register file at the negative edge of the clock signal. 

## Instruct Fetch Stage (IF)
to be completed...

# Testbench
