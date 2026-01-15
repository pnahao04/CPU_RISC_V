module top(
    input         clk, reset,
    output [31:0] WriteData, DataAdr,
    output        MemWrite
);
    wire [31:0] PC, Instr, ReadData;

    
    riscvsingle rvsingle(
        .clk(clk), 
        .reset(reset), 
        .PC(PC), 
        .Instr(Instr), 
        .MemWrite(MemWrite), 
        .ALUResult(DataAdr), // Nối ALUResult ra DataAdr
        .WriteData(WriteData), 
        .ReadData(ReadData)
    );

    // Gọi bộ nhớ lệnh
    imem imem(PC, Instr);

    // Gọi bộ nhớ dữ liệu
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





module riscvsingle(
    input         clk, reset,
    output [31:0] PC,
    input  [31:0] Instr,
    output        MemWrite,
    output [31:0] ALUResult, WriteData,
    input  [31:0] ReadData
);

    wire       ALUSrc, RegWrite, Jump, Zero;
    wire [1:0] ResultSrc, ImmSrc;
    wire [2:0] ALUControl;
    wire       PCSrc;      // Khai báo thêm dây nối PCSrc

    controller c(
        .op(Instr[6:0]),           // 
        .funct3(Instr[14:12]),     // lệnh thuộc loại 
        .funct7b5(Instr[30]),      // loại R  phân loại add and sub
        .Zero(Zero), 
        .ResultSrc(ResultSrc), 
        .MemWrite(MemWrite), 
        .PCSrc(PCSrc),
        .ALUSrc(ALUSrc), 
        .RegWrite(RegWrite), 
        .Jump(Jump),
        .ImmSrc(ImmSrc), 
        .ALUControl(ALUControl)
    );

    datapath dp(
        .clk(clk), 
        .reset(reset), 
        .ResultSrc(ResultSrc), 
        .PCSrc(PCSrc),
        .ALUSrc(ALUSrc), 
        .RegWrite(RegWrite),
        .ImmSrc(ImmSrc), 
        .ALUControl(ALUControl),
        .Zero(Zero), 
        .PC(PC), 
        .Instr(Instr),
        .ALUResult(ALUResult), 
        .WriteData(WriteData), 
        .ReadData(ReadData)
    );

endmodule



module controller(
    input  [6:0] op,
    input  [2:0] funct3,
    input        funct7b5,
    input        Zero,
    output [1:0] ResultSrc,        // chọn ngõ ra cho result ( mux3)
    output       MemWrite,         // cho phép ghi giá trị vào Memory
    output       PCSrc, ALUSrc,    // điều kiển Pc có tăng 4 hay thực hiện lệnh nhảy ( mux2 ) , chọn giá trị từ thanh ghi hay từ immm của Extend ( mux2) 
    output       RegWrite, Jump,   //  cho phép ghi vào  thanh ghi ,  lệnh này dùng để chọn xem tính hiệu +4 hay nhảy
    output [1:0] ImmSrc,           // chon bit ngõ ra chi Extend 
    output [2:0] ALUControl        // điều kiển alu 
);

    wire [1:0] ALUOp;     // chọn xem ALU sẽ thực hiện lệnh gì
    wire       Branch;    // xác nhận đó là lệnh beq thuộc loại câu lệnh so sánh

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
    input  [1:0]  ResultSrc,
    input         PCSrc, ALUSrc,
    input         RegWrite,
    input  [1:0]  ImmSrc,
    input  [2:0]  ALUControl,
    output        Zero,
    output [31:0] PC,
    input  [31:0] Instr,
    output [31:0] ALUResult, WriteData,
    input  [31:0] ReadData
);

    wire [31:0] PCNext, PCPlus4, PCTarget;
    wire [31:0] ImmExt;
    wire [31:0] SrcA, SrcB;
    wire [31:0] Result;

    // --- Next PC logic ---
    
    // PC Register
    flopr #(32) pcreg(
        .clk(clk), 
        .reset(reset), 
        .d(PCNext), 
        .q(PC)
    );

    // PC + 4 Adder
    adder pcadd4(
        .a(PC), 
        .b(32'd4), 
        .y(PCPlus4)
    );

    // PC Target Adder (Branch)
    adder pcaddbranch(
        .a(PC), 
        .b(ImmExt), 
        .y(PCTarget)
    );

    // PC Mux
    mux2 #(32) pcmux(
        .d0(PCPlus4), 
        .d1(PCTarget), 
        .s(PCSrc), 
        .y(PCNext)
    );

    // --- Register file logic ---
    regfile rf(
        .clk(clk), 
        .we3(RegWrite), 
        .ra1(Instr[19:15]), 
        .ra2(Instr[24:20]), 
        .wa3(Instr[11:7]), 
        .wd3(Result), 
        .rd1(SrcA), 
        .rd2(WriteData)
    );

    // Sign Extension
    extend ext(
        .instr(Instr[31:7]), 
        .immsrc(ImmSrc), 
        .immext(ImmExt)
    );

    // --- ALU logic ---
    
    // ALU Source B Mux
    mux2 #(32) srcbmux(
        .d0(WriteData), 
        .d1(ImmExt), 
        .s(ALUSrc), 
        .y(SrcB)
    );
    
    // ALU Unit
    alu alu_unit(
        .SrcA(SrcA), 
        .SrcB(SrcB), 
        .ALUControl(ALUControl), 
        .ALUResult(ALUResult), 
        .Zero(Zero)
    );

    // Result Mux (Writeback Mux)
    mux3 #(32) resultmux(
        .d0(ALUResult), 
        .d1(ReadData), 
        .d2(PCPlus4), 
        .s(ResultSrc), 
        .y(Result)
    );

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
    input         we3,           // Write Enable
    input  [4:0]  ra1, ra2, wa3, // 
    input  [31:0] wd3,           // Write Data
    output [31:0] rd1, rd2       // Read Data
);
    reg [31:0] rf[31:0]; // Mảng 32 thanh ghi 32-bit

    // 1. Hoạt động GHI (Đồng bộ xung Clock)
    // Write third port on rising edge of clock (wa3/wd3/we3)
    always @(posedge clk) begin
        if (we3) rf[wa3] <= wd3;
    end

    // 2. Hoạt động ĐỌC (Tổ hợp - Bất đồng bộ)
    // Register 0 hardwired to 0
    // Nếu địa chỉ đọc là 0 thì trả về 0, ngược lại thì đọc từ mảng rf
    assign rd1 = (ra1 != 0) ? rf[ra1] : 32'b0;
    assign rd2 = (ra2 != 0) ? rf[ra2] : 32'b0;

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
