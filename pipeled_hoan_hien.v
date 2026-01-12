module top(
    input         clk, reset,
    output [31:0] WriteData, DataAdr,
    output        MemWrite
);
    wire [31:0] PC, Instr, ReadData;

    // SỬA: Gọi module riscvpipeline thay vì riscvsingle
    riscvpipeline rv(
        .clk(clk), 
        .reset(reset), 
        .PC(PC), 
        .Instr(Instr), 
        .MemWrite(MemWrite), 
        .ALUResult(DataAdr), 
        .WriteData(WriteData), 
        .ReadData(ReadData)
    );

    imem imem(PC, Instr);
    dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule


module imem(
    input  [31:0] a,
    output [31:0] rd
);
    reg [31:0] RAM[63:0]; // Mảng nhớ 64 từ (word)

    initial begin
        
        $readmemh("riscvtest.txt", RAM);
    end

    assign rd = RAM[a[31:2]]; // Word aligned (Bỏ 2 bit cuối của địa chỉ)
endmodule



module dmem(
    input         clk, we,
    input  [31:0] a, wd,
    output [31:0] rd
);
    reg [31:0] RAM[63:0];

    assign rd = RAM[a[31:2]]; // Đọc bất đồng bộ

    always @(posedge clk) begin
        if (we) RAM[a[31:2]] <= wd; // Ghi đồng bộ
    end
endmodule

module hazard_unit(
    input  [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
    input        RegWriteM, RegWriteW,
    input        ResultSrcE0, 
    input        PCSrcE,      
    output reg [1:0] ForwardAE, ForwardBE,
    output       StallF, StallD, FlushD, FlushE
);

    wire lwStall;

    // Forwarding
    always @(*) begin
        if ((Rs1E == RdM) && RegWriteM && (Rs1E != 0)) ForwardAE = 2'b10;
        else if ((Rs1E == RdW) && RegWriteW && (Rs1E != 0)) ForwardAE = 2'b01;
        else ForwardAE = 2'b00;
        
        if ((Rs2E == RdM) && RegWriteM && (Rs2E != 0)) ForwardBE = 2'b10;
        else if ((Rs2E == RdW) && RegWriteW && (Rs2E != 0)) ForwardBE = 2'b01;
        else ForwardBE = 2'b00;
    end

    // Stall Logic (Load Hazard)
    assign lwStall = ResultSrcE0 & ((Rs1D == RdE) | (Rs2D == RdE));
    assign StallF = lwStall;
    assign StallD = lwStall;

    // Flush Logic (Control Hazard)
    assign FlushD = PCSrcE;
    assign FlushE = lwStall | PCSrcE; 

endmodule





module riscvpipeline(
    input         clk, reset,
    output [31:0] PC,
    input  [31:0] Instr,
    output        MemWrite,
    output [31:0] ALUResult, WriteData,
    input  [31:0] ReadData
);

    wire       ALUSrcD, RegWriteD, MemWriteD, JumpD, BranchD;
    wire [1:0] ResultSrcD, ImmSrcD;
    wire [2:0] ALUControlD;
    wire [1:0] ForwardAE, ForwardBE;
    wire       StallF, StallD, FlushD, FlushE;
    wire [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;
    wire       RegWriteM, RegWriteW;
    wire [1:0] ResultSrcE;
    wire       PCSrcE; // Dây kết nối quan trọng

    wire [31:0] InstrD; 
    wire        ZeroE;
    wire        MemWriteM;

    controller c(
        .op(InstrD[6:0]), 
        .funct3(InstrD[14:12]), 
        .funct7b5(InstrD[30]), 
        .Zero(ZeroE),
        .ResultSrc(ResultSrcD), 
        .MemWrite(MemWriteD), 
        .PCSrc(), 
        .ALUSrc(ALUSrcD), 
        .RegWrite(RegWriteD), 
        .Jump(JumpD),
        .ImmSrc(ImmSrcD), 
        .ALUControl(ALUControlD),
        .Branch(BranchD)
    );

    datapath dp(
        .clk(clk), .reset(reset),
        .InstrF(Instr), .ReadDataM(ReadData),
        .RegWriteD(RegWriteD), .MemWriteD(MemWriteD), .JumpD(JumpD), .BranchD(BranchD), 
        .ALUSrcD(ALUSrcD), .ResultSrcD(ResultSrcD), .ImmSrcD(ImmSrcD), .ALUControlD(ALUControlD),
        .ForwardAE(ForwardAE), .ForwardBE(ForwardBE), 
        .StallF(StallF), .StallD(StallD), .FlushD(FlushD), .FlushE(FlushE),
        .Rs1D(Rs1D), .Rs2D(Rs2D), .Rs1E(Rs1E), .Rs2E(Rs2E), 
        .RdE(RdE), .RdM(RdM), .RdW(RdW),
        .RegWriteM(RegWriteM), .RegWriteW(RegWriteW), .ResultSrcE(ResultSrcE),
        .InstrD(InstrD), .ZeroE(ZeroE), .PCF(PC),
        .ALUResultM(ALUResult), .WriteDataM(WriteData), .MemWriteM(MemWriteM),
        .PCSrcE(PCSrcE)
    );
    
    hazard_unit hu(
        .Rs1D(Rs1D), .Rs2D(Rs2D), .Rs1E(Rs1E), .Rs2E(Rs2E), 
        .RdE(RdE), .RdM(RdM), .RdW(RdW),
        .RegWriteM(RegWriteM), .RegWriteW(RegWriteW),
        .ResultSrcE0(ResultSrcE[0]),
        .PCSrcE(PCSrcE), 
        .ForwardAE(ForwardAE), .ForwardBE(ForwardBE),
        .StallF(StallF), .StallD(StallD), .FlushD(FlushD), .FlushE(FlushE)
    );

    assign MemWrite = MemWriteM;

endmodule



module controller(
    input  [6:0] op,
    input  [2:0] funct3,
    input        funct7b5,
    input        Zero,
    output [1:0] ResultSrc,
    output       MemWrite,
    output       PCSrc, ALUSrc,
    output       RegWrite, Jump,
    output [1:0] ImmSrc,
    output [2:0] ALUControl,
    output       Branch // Thêm output này
);

    wire [1:0] ALUOp;

    maindec md(
        .op(op), 
        .ResultSrc(ResultSrc), 
        .MemWrite(MemWrite), 
        .Branch(Branch),
        .ALUSrc(ALUSrc), 
        .RegWrite(RegWrite), 
        .Jump(Jump), 
        .ImmSrc(ImmSrc), 
        .ALUOp(ALUOp)
    );

    aludec ad(
        .opb5(op[5]), 
        .funct3(funct3), 
        .funct7b5(funct7b5), 
        .ALUOp(ALUOp), 
        .ALUControl(ALUControl)
    );
    
    // Logic PCSrc này thực tế sẽ được tính ở Datapath tầng Execute
    assign PCSrc = (Branch & Zero) | Jump; 

endmodule

module maindec(
    input  [6:0] op,          
    output [1:0] ResultSrc,   
    output       MemWrite,    
    output       Branch, ALUSrc, 
    output       RegWrite, Jump, 
    output [1:0] ImmSrc, 
    output [1:0] ALUOp 
);

    reg [10:0] controls;

    // Gán các wire output bằng giá trị của biến reg controls
    assign {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = controls;

    always @(*) begin
        case(op)
            // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
            7'b0_000_011: controls = 11'b1_00_1_0_01_0_00_0; // lw
            7'b0_100_011: controls = 11'b0_01_1_1_00_0_00_0; // sw
            7'b0_110_011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type
            7'b1_100_011: controls = 11'b0_10_0_0_00_1_01_0; // beq
            7'b0_010_011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
            7'b1_101_111: controls = 11'b1_11_0_0_10_0_00_1; // jal
            default:    controls = 11'b0_00_0_0_00_0_00_0; // Default case an toàn
        endcase
    end
endmodule


module aludec(
    input        opb5,
    input  [2:0] funct3,
    input        funct7b5,
    input  [1:0] ALUOp,
    output reg [2:0] ALUControl // Phải là reg vì nằm trong always
);

    wire RtypeSub;
    assign RtypeSub = funct7b5 & opb5; // TRUE for R-type subtract

    always @(*) begin
        case(ALUOp)
            2'b00: ALUControl = 3'b000; // addition
            2'b01: ALUControl = 3'b001; // subtraction
            default: case(funct3) // R-type or I-type ALU
                3'b000: if (RtypeSub) 
                            ALUControl = 3'b001; // sub
                        else 
                            ALUControl = 3'b000; // add, addi

                3'b010: ALUControl = 3'b101; // slt, slti
                3'b110: ALUControl = 3'b011; // or, ori
                3'b111: ALUControl = 3'b010; // and, andi
                default: ALUControl = 3'bxxx; 
            endcase

        endcase
    end
endmodule

module datapath(
    input         clk, reset,
    // Stage F
    input  [31:0] InstrF,
    input  [31:0] ReadDataM,
    // Stage D
    input         RegWriteD, MemWriteD, JumpD, BranchD, ALUSrcD,
    input  [1:0]  ResultSrcD, ImmSrcD,
    input  [2:0]  ALUControlD,
    // Hazard Inputs
    input  [1:0]  ForwardAE, ForwardBE,
    input         StallF, StallD, FlushD, FlushE,
    // Outputs
    output [4:0]  Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
    output [31:0] InstrD,
    output        ZeroE,
    output reg [31:0] PCF,
    output [31:0] ALUResultM, WriteDataM,
    output        MemWriteM, RegWriteM, RegWriteW,
    output [1:0]  ResultSrcE,
    output        PCSrcE
);

    // --- Wiring ---
    wire [31:0] PCNextF, PCPlus4F;
    wire [31:0] PCPlus4D, PCD;
    wire [31:0] ImmExtD, RD1D, RD2D;
    wire [31:0] SrcAE, SrcBE, PCTargetE, ALUResultE, WriteDataE;
    wire [31:0] SrcAE_Forwarded, WriteDataE_Forwarded;
    wire [31:0] ResultW;
    
    // --- Pipeline Registers ---
    reg [31:0] InstrD_reg, PCD_reg, PCPlus4D_reg; 
    
    reg [31:0] RD1E, RD2E, PCE, ImmExtE, PCPlus4E;
    reg        RegWriteE, MemWriteE, JumpE, BranchE, ALUSrcE;
    reg [2:0]  ALUControlE;
    reg [1:0]  ResultSrcE_reg;
    reg [4:0]  Rs1E_reg, Rs2E_reg, RdE_reg;

    reg [31:0] ALUResultM_reg, WriteDataM_reg, PCPlus4M;
    reg [4:0]  RdM_reg;
    reg        RegWriteM_reg, MemWriteM_reg;
    reg [1:0]  ResultSrcM;

    reg [31:0] ALUResultW, ReadDataW, PCPlus4W;
    reg [4:0]  RdW_reg;
    reg        RegWriteW_reg;
    reg [1:0]  ResultSrcW;

    // --- Assignments ---
    assign Rs1D = InstrD[19:15];
    assign Rs2D = InstrD[24:20];
    assign Rs1E = Rs1E_reg;
    assign Rs2E = Rs2E_reg;
    assign RdE  = RdE_reg;
    assign RdM  = RdM_reg;
    assign RdW  = RdW_reg;
    assign RegWriteM = RegWriteM_reg;
    assign RegWriteW = RegWriteW_reg;
    assign ResultSrcE = ResultSrcE_reg;

    // ==========================================================
    // 1. FETCH STAGE
    // ==========================================================
    mux2 #(32) pcmux(PCPlus4F, PCTargetE, PCSrcE, PCNextF);

    // PC Register (Enable = !StallF)
    always @(posedge clk or posedge reset) begin
        if (reset) PCF <= 0;
        else if (!StallF) PCF <= PCNextF; 
    end
    adder pcadd4(PCF, 32'd4, PCPlus4F);

    // ==========================================================
    // 2. IF/ID REGISTER (Ưu tiên: Reset > Flush > Stall)
    // ==========================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            InstrD_reg <= 0; PCD_reg <= 0; PCPlus4D_reg <= 0;
        end 
        else if (FlushD) begin // CLR
            InstrD_reg <= 0; PCD_reg <= 0; PCPlus4D_reg <= 0;
        end 
        else if (!StallD) begin // EN
            InstrD_reg <= InstrF;
            PCD_reg    <= PCF;
            PCPlus4D_reg <= PCPlus4F;
        end
    end
    assign InstrD = InstrD_reg;
    assign PCD = PCD_reg;
    assign PCPlus4D = PCPlus4D_reg;

    // ==========================================================
    // 3. DECODE STAGE
    // ==========================================================
    regfile rf(clk, RegWriteW, InstrD[19:15], InstrD[24:20], RdW, ResultW, RD1D, RD2D);
    extend ext(InstrD[31:7], ImmSrcD, ImmExtD);

    // ==========================================================
    // 4. ID/EX REGISTER (Ưu tiên: Reset > Flush)
    // ==========================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            RegWriteE <= 0; MemWriteE <= 0; JumpE <= 0; BranchE <= 0;
            ALUSrcE <= 0; ResultSrcE_reg <= 0; ALUControlE <= 0;
            RD1E <= 0; RD2E <= 0; PCE <= 0; ImmExtE <= 0; PCPlus4E <= 0;
            Rs1E_reg <= 0; Rs2E_reg <= 0; RdE_reg <= 0;
        end 
        else if (FlushE) begin // CLR
            RegWriteE <= 0; MemWriteE <= 0; JumpE <= 0; BranchE <= 0;
            ALUSrcE <= 0; ResultSrcE_reg <= 0; ALUControlE <= 0;
            RD1E <= 0; RD2E <= 0; PCE <= 0; ImmExtE <= 0; PCPlus4E <= 0;
            Rs1E_reg <= 0; Rs2E_reg <= 0; RdE_reg <= 0;
        end 
        else begin
            RegWriteE <= RegWriteD; MemWriteE <= MemWriteD; 
            JumpE <= JumpD; BranchE <= BranchD;
            ALUSrcE <= ALUSrcD; ResultSrcE_reg <= ResultSrcD; 
            ALUControlE <= ALUControlD;
            RD1E <= RD1D; RD2E <= RD2D; 
            PCE <= PCD; ImmExtE <= ImmExtD; PCPlus4E <= PCPlus4D;
            Rs1E_reg <= InstrD[19:15];
            Rs2E_reg <= InstrD[24:20];
            RdE_reg <= InstrD[11:7];
        end
    end

    // ==========================================================
    // 5. EXECUTE STAGE
    // ==========================================================
    mux3 #(32) forwardAmux(RD1E, ResultW, ALUResultM, ForwardAE, SrcAE_Forwarded);
    mux3 #(32) forwardBmux(RD2E, ResultW, ALUResultM, ForwardBE, WriteDataE_Forwarded);

    assign SrcAE = SrcAE_Forwarded; 
    assign WriteDataE = WriteDataE_Forwarded;

    mux2 #(32) srcbmux(WriteDataE, ImmExtE, ALUSrcE, SrcBE);
    alu alu_unit(SrcAE, SrcBE, ALUControlE, ALUResultE, ZeroE);
    adder pcaddbranch(PCE, ImmExtE, PCTargetE);
    
    // Logic PCSrcE: Nhảy khi (Branch & Zero) HOẶC Jump
    assign PCSrcE = (BranchE & ZeroE) | JumpE;

    // ==========================================================
    // 6. EX/MEM REGISTER
    // ==========================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            RegWriteM_reg <= 0; MemWriteM_reg <= 0; ResultSrcM <= 0;
            ALUResultM_reg <= 0; WriteDataM_reg <= 0; PCPlus4M <= 0; RdM_reg <= 0;
        end else begin
            RegWriteM_reg <= RegWriteE; MemWriteM_reg <= MemWriteE; ResultSrcM <= ResultSrcE_reg;
            ALUResultM_reg <= ALUResultE; WriteDataM_reg <= WriteDataE; 
            PCPlus4M <= PCPlus4E; RdM_reg <= RdE_reg;
        end
    end

    assign ALUResultM = ALUResultM_reg;
    assign WriteDataM = WriteDataM_reg;
    assign MemWriteM  = MemWriteM_reg;

    // ==========================================================
    // 7. MEM/WB REGISTER
    // ==========================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            RegWriteW_reg <= 0; ResultSrcW <= 0;
            ALUResultW <= 0; ReadDataW <= 0; PCPlus4W <= 0; RdW_reg <= 0;
        end else begin
            RegWriteW_reg <= RegWriteM_reg; ResultSrcW <= ResultSrcM;
            ALUResultW <= ALUResultM_reg; ReadDataW <= ReadDataM; 
            PCPlus4W <= PCPlus4M; RdW_reg <= RdM_reg;
        end
    end
    mux3 #(32) resultmux(ALUResultW, ReadDataW, PCPlus4W, ResultSrcW, ResultW);

endmodule


module adder(
    input  [31:0] a, b,
    output [31:0] y
);
    assign y = a + b;
endmodule

module extend(
    input  [31:7] instr,
    input  [1:0]  immsrc,
    output reg [31:0] immext // Output reg cho always block
);
    always @(*) begin
        case(immsrc)
            // I-type
            2'b00: immext = {{20{instr[31]}}, instr[31:20]};
            // S-type (stores)
            2'b01: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            // B-type (branches) - Sửa lỗi cú pháp 1'b0
            2'b10: immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
            // J-type (jal)
            2'b11: immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
            default: immext = 32'bx;
        endcase
    end
endmodule


module flopr #(parameter WIDTH = 32) (
    input              clk, reset,
    input  [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q // Output reg cho always ff
);
    // Reset bất đồng bộ (Asynchronous Reset) là chuẩn mực
    always @(posedge clk or posedge reset) begin
        if (reset) q <= 0;
        else       q <= d;
    end
endmodule


module flopenr #(parameter WIDTH = 32) (
    input              clk, reset, en,
    input  [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q
);
    always @(posedge clk or posedge reset) begin
        if (reset)      q <= 0;
        else if (en)    q <= d;
    end
endmodule


module mux2 #(parameter WIDTH = 32) (
    input  [WIDTH-1:0] d0, d1,
    input              s,
    output [WIDTH-1:0] y
);
    assign y = s ? d1 : d0;
endmodule


module mux3 #(parameter WIDTH = 32) (
    input  [WIDTH-1:0] d0, d1, d2,
    input  [1:0]       s,
    output [WIDTH-1:0] y
);
    assign y = s[1] ? d2 : (s[0] ? d1 : d0);

endmodule


module regfile(
    input         clk,
    input         we3, 
    input  [4:0]  ra1, ra2, wa3, 
    input  [31:0] wd3, 
    output [31:0] rd1, rd2 
);
    reg [31:0] rf[31:0];
    integer i;

    // Khởi tạo
    initial begin
        for (i=0; i<32; i=i+1) rf[i] = 0;
    end

    // SỬA: Đổi posedge -> negedge (Ghi ở sườn xuống)
    // Điều này giúp việc ghi hoàn tất trước khi lệnh tiếp theo đọc ở sườn lên
    always @(negedge clk) begin
        if (we3) rf[wa3] <= wd3;
    end

    assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
    assign rd2 = (ra2 != 0) ? rf[ra2] : 0;
endmodule

module alu(
    input  [31:0] SrcA, SrcB,
    input  [2:0]  ALUControl,
    output reg [31:0] ALUResult, // Dùng reg vì nằm trong always
    output        Zero
);

    always @(*) begin
        case (ALUControl)
            3'b000: ALUResult = SrcA + SrcB;             // ADD (Cộng)
            3'b001: ALUResult = SrcA - SrcB;             // SUB (Trừ)
            3'b010: ALUResult = SrcA & SrcB;             // AND
            3'b011: ALUResult = SrcA | SrcB;             // OR
            3'b101: ALUResult = ($signed(SrcA) < $signed(SrcB)) ? 32'd1 : 32'd0; // SLT (So sánh nhỏ hơn)
            default: ALUResult = 32'b0;
        endcase
    end

    // Cờ Zero bật lên 1 khi kết quả bằng 0
    assign Zero = (ALUResult == 32'b0);

endmodule
