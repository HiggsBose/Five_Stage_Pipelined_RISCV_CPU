`include "src/defines.v"
`include "src/five_stages/if.v"
`include "src/five_stages/id.v"
`include "src/five_stages/ex.v"
`include "src/five_stages/mem.v"
`include "src/five_stages/wb.v"
`include "src/templates/pipe_dff.v"
`include "src/pipe_regs/if_id.v"
`include "src/pipe_regs/id_ex.v"
`include "src/pipe_regs/ex_mem.v"
`include "src/pipe_regs/mem_wb.v"
`include "src/hazard_unit/forward_unit_id.v"
`include "src/hazard_unit/forward_unit_ex.v"
`include "src/hazard_unit/hazard_detect_unit.v"

module RISCVPipeline (
    input clk, rst,
    input [31:0] instr, mem_read_data,
    output ram_write,
    output [2:0] write_type,
    output [31:0] instr_addr, mem_addr, mem_write_data
);

    wire [31:0] pc_if, pc_plus4_if, pc_id, pc_plus4_id, pc_ex, pc_plus4_ex, pc_plus4_mem, nxpc_wb;
    wire [31:0] instr_if, instr_id;
    wire [1:0] reg_src_id, reg_src_ex, reg_src_mem, reg_src_wb;
    wire [2:0] instr_funct3_id, instr_funct3_ex, instr_funct3_mem;
    wire [3:0] alu_type_id, alu_type_ex;

    wire [4:0] rs1_id, rs2_id, rs1_ex, rs2_ex;
    wire [4:0] rd_id, rd_ex, rd_mem, rd_wb;
    wire [31:0] rs1_data_id, rs2_data_id, rs1_data_ex, rs2_data_ex, real_rs2_data_ex, rs2_data_mem; 
    wire [31:0] imm_id, imm_ex, imm_mem, imm_wb; 

    wire [31:0] reg_write_data_mem, reg_write_data_wb;

    wire [31:0] alu_result_ex, alu_result_mem, alu_result_wb;

    wire [31:0] mem2reg_data, mem2reg_data_wb;

    wire [31:0] new_pc;

    wire [1:0] rs1_fwd_ex, rs1_fwd_id, rs2_fwd_ex, rs2_fwd_id;


    IF instrcution_fetch_stage(
        .clk(clk),
        .rst(rst),
        .stall_if(stall_if),
        .bubble_if(bubble_if),
        .pc_src(pc_src_id),
        .pc_new(new_pc),
        .pc_if(pc_if),
        .pc_plus4_if(pc_plus4_if),
        .instr_addr(instr_addr)
    );

    IF_ID_REG if_id_register(
        .stall_id(stall_id),
        .bubble_id(bubble_id),
        .clk(clk),
        .instr_if(instr_if),
        .pc_if(pc_if),
        .pc_plus4_if(pc_plus4_if),
        .instr_id(instr_id),   
        .pc_id(pc_id),      
        .pc_plus4_id(pc_plus4_id)
    );

    assign instr_if = (bubble_if) ? `INST_NOP : instr; 
    // assign instr_if = instr;

    ID instruction_decode_stage(
        .clk(clk),
        .rst(rst),
        .instr_id(instr_id),
        .pc_id(pc_id),
        .reg_write_wb(reg_write_wb),
        .rd_wb(rd_wb),

        .reg_write_data_mem(reg_write_data_mem), // not determined
        .reg_write_data_wb(reg_write_data_wb), // not determined

        .rs1_fwd_id(rs1_fwd_id), // not determined
        .rs2_fwd_id(rs2_fwd_id), // not determined

        .branch_id(branch_id),
        .jal_id(jal_id),
        .jalr_id(jalr_id),

        .instr_funct3_id(instr_funct3_id),
        .pc_src(pc_src_id),
        .new_pc(new_pc),

        .mem_read_id(mem_read_id),
        .mem_write_id(mem_write_id),

        .alu_src1_id(alu_src1_id),
        .alu_src2_id(alu_src2_id),
        .alu_type_id(alu_type_id),

        .reg_src_id(reg_src_id),
        .reg_write_id(reg_write_id),
        
        .rs1_id(rs1_id),
        .rs2_id(rs2_id),
        .rd_id(rd_id),

        .rs1_data_id(rs1_data_id),
        .rs2_data_id(rs2_data_id),

        .imm_id(imm_id)
    );

    ID_EX_REG id_ex_register(
        .clk(clk),
        .stall_ex(stall_ex),
        .bubble_ex(bubble_ex),

        .branch_id(branch_id),
        .jal_id(jal_id),
        .jalr_id(jalr_id),
        .mem_read_id(mem_read_id),
        .mem_write_id(mem_write_id),
        .alu_src1_id(alu_src1_id),
        .alu_src2_id(alu_src2_id),

        .reg_write_id(reg_write_id),
        .reg_src_id(reg_src_id),
        .instr_funct3_id(instr_funct3_id),
        .alu_type_id(alu_type_id),

        .rs1_id(rs1_id),
        .rs2_id(rs2_id),
        .rd_id(rd_id),
        .rs1_data_id(rs1_data_id),
        .rs2_data_id(rs2_data_id),
        
        .imm_id(imm_id),
        .pc_id(pc_id),
        .pc_plus4_id(pc_plus4_id),

        .mem_read_ex(mem_read_ex),
        .mem_write_ex(mem_write_ex),

        .imm_ex(imm_ex),
        .reg_write_ex(reg_write_ex),
        .instr_funct3_ex(instr_funct3_ex),
        .reg_src_ex(reg_src_ex),
        .rd_ex(rd_ex),

        .pc_ex(pc_ex),
        .pc_plus4_ex(pc_plus4_ex),

        .alu_src1_ex(alu_src1_ex),
        .alu_src2_ex(alu_src2_ex),
        .alu_type_ex(alu_type_ex),

        .rs1_ex(rs1_ex),
        .rs2_ex(rs2_ex),

        .rs1_data_ex(rs1_data_ex),
        .rs2_data_ex(rs2_data_ex)

    );

    EX exexute_stage(
        .alu_src1_ex(alu_src1_ex),
        .alu_src2_ex(alu_src2_ex),
        .alu_type_ex(alu_type_ex),
        .pc_ex(pc_ex),
        .rs1_data_ex(rs1_data_ex),
        .rs2_data_ex(rs2_data_ex),
        .imm_ex(imm_ex),
        .reg_write_data_wb(reg_write_data_wb),
        .reg_write_data_mem(reg_write_data_mem),
        .rs1_ex(rs1_ex),
        .rs2_ex(rs2_ex),
        .rs1_fwd_ex(rs1_fwd_ex),
        .rs2_fwd_ex(rs2_fwd_ex),

        .alu_result_ex(alu_result_ex),
        .real_rs2_data_ex(real_rs2_data_ex)

    );

    EX_MEM_REG ex_mem_register(
        .clk(clk),
        .stall_mem(stall_mem),
        .bubble_mem(bubble_mem),

        .mem_read_ex(mem_read_ex),
        .mem_write_ex(mem_write_ex),
        .reg_write_ex(reg_write_ex),
        .imm_ex(imm_ex),
        .instr_funct3_ex(instr_funct3_ex),
        .reg_src_ex(reg_src_ex),
        .rd_ex(rd_ex),
        .pc_plus4_ex(pc_plus4_ex),
        .rs2_data_ex(real_rs2_data_ex),
        .alu_result_ex(alu_result_ex),

        .reg_write_mem(reg_write_mem),
        .reg_src_mem(reg_src_mem),
        .rd_mem(rd_mem),
        .alu_result_mem(alu_result_mem),
        .imm_mem(imm_mem),
        .pc_plus4_mem(pc_plus4_mem),

        .mem_read_mem(mem_read_mem),
        .mem_write_mem(mem_write_mem),
        .instr_funct3_mem(instr_funct3_mem),
        .rs2_data_mem(rs2_data_mem)
    );

    MEM_MODULE memory_stage(
        .mem_read_mem(mem_read_mem),
        .mem_write_mem(mem_write_mem),
        .instr_funct3_mem(instr_funct3_mem),
        .rs2_data_mem(rs2_data_mem),
        .alu_result_mem(alu_result_mem),
        .mem_read_data(mem_read_data),
        .reg_src_mem(reg_src_mem),
        .imm_mem(imm_mem),
        .pc_plus4_mem(pc_plus4_mem),

        .mem_write(ram_write),
        .write_type(write_type),
        .mem_addr(mem_addr),
        .write_data(mem_write_data),
        .mem2reg_data(mem2reg_data),
        .reg_write_data_mem(reg_write_data_mem)
    );

    MEM_WB_REG mem_wb_register(
        .clk(clk),
        .bubble_wb(bubble_wb),
        .stall_wb(stall_wb),

        .mem2reg_data(mem2reg_data),

        .reg_write_mem(reg_write_mem),

        .reg_src_mem(reg_src_mem),
        .rd_mem(rd_mem),
        .alu_result_mem(alu_result_mem),
        .imm_mem(imm_mem),
        .pc_plus4_mem(pc_plus4_mem),

        .reg_src_wb(reg_src_wb),
        .alu_result_wb(alu_result_wb),
        .mem2reg_data_wb(mem2reg_data_wb),
        .imm_wb(imm_wb),
        .nxpc_wb(nxpc_wb),
        .reg_write_wb(reg_write_wb),
        .rd_wb(rd_wb)
    );

    WB write_back_stage(
        .reg_src_wb(reg_src_wb),
        .alu_result_wb(alu_result_wb),
        .mem2reg_data_wb(mem2reg_data_wb),
        .imm_wb(imm_wb),
        .nxpc_wb(nxpc_wb),

        .reg_write_data_wb(reg_write_data_wb)
    );

    FWD_EX forward_unit_ex(
        .reg_write_wb(reg_write_wb),
        .rd_wb(rd_wb),
        .rs1_ex(rs1_ex),
        .rs2_ex(rs2_ex),

        .reg_write_mem(reg_write_mem),
        .rd_mem(rd_mem),

        .rs1_fwd_ex(rs1_fwd_ex),
        .rs2_fwd_ex(rs2_fwd_ex)
    );

    FWD_ID forward_unit_id(
        .reg_write_mem(reg_write_mem),
        .rd_mem(rd_mem),
        .branch_id(branch_id),
        .jal_id(jal_id),
        .jalr_id(jalr_id),

        .rs1_id(rs1_id),
        .rs2_id(rs2_id),

        .rs1_fwd_id(rs1_fwd_id),
        .rs2_fwd_id(rs2_fwd_id)
    );

    HazardDetect hazard_detect_unit(
        .clk(clk),
        .rst(rst),
        .pc_src_id(pc_src_id),
        .jal_id(jal_id),
        .jalr_id(jalr_id),
        .branch_id(branch_id),
        .rs1_id(rs1_id),
        .rs2_id(rs2_id),
        .mem_read_ex(mem_read_ex),
        .rd_ex(rd_ex),
        .rd_mem(rd_mem),
        .mem_read_mem(mem_read_mem),

        .stall_if(stall_if),
        .stall_id(stall_id),
        .stall_ex(stall_ex),
        .stall_mem(stall_mem),
        .stall_wb(stall_wb),
        
        .bubble_if(bubble_if),
        .bubble_id(bubble_id),
        .bubble_ex(bubble_ex),
        .bubble_mem(bubble_mem),
        .bubble_wb(bubble_wb)
    );
endmodule