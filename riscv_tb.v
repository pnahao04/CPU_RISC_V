module testbench();
    reg         clk;
    reg         reset;
    wire [31:0] WriteData, DataAdr;
    wire        MemWrite;

    // Instantiate device to be tested
    top dut(clk, reset, WriteData, DataAdr, MemWrite);

    // --- PHẦN THÊM MỚI ĐỂ HIỆN SÓNG TRÊN EDA PLAYGROUND ---
    initial begin
        $dumpfile("dump.vcd"); // Tạo file lưu sóng
        $dumpvars(0, testbench); // Lưu tất cả tín hiệu trong testbench
    end
  
  
  initial begin
      #400; // Chờ 500 đơn vị thời gian (đủ cho chuong trình nhỏ chạy)
      $finish; // Bắt buộc dừng mô phỏng
   end
  
  
  
    // -----------------------------------------------------

    // Initialize test
    initial begin
        reset <= 1; #22; reset <= 0;
    end

    // Generate clock
    always begin
        clk <= 1; #5; clk <= 0; #5;
    end


endmodule