`timescale 1ns/1ps
module async_fifo_tb;
  reg clk_a=0, clk_b=0;
  reg [15:0] data_in=16'h57a3;
  reg [31:0] random_state=32'h12345678;
  wire [11:0] data_out;
  top #(.UNITS(2)) dut(.*);
  always #4 clk_a=~clk_a;
  // Different period and phase; avoid coincident scoreboard sampling edges.
  initial begin #1; forever #6 clk_b=~clk_b; end
  always @(negedge clk_a) begin
    random_state=random_state ^ (random_state << 13);
    random_state=random_state ^ (random_state >> 17);
    random_state=random_state ^ (random_state << 5);
    data_in=random_state[15:0];
  end
  for(genvar i=0;i<2;i=i+1) begin: check_bank
    reg [11:0] expected [0:8191];
    reg [11:0] next_read;
    integer written=0, read_count=0, saw_full=0, saw_empty=0;
    always @(posedge clk_a) begin
      if(dut.bank[i].full) saw_full=saw_full+1;
      if(dut.bank[i].push) begin
        if(written-read_count >= 8) $fatal(1,"FIFO overflow in bank %0d",i);
        expected[written]=data_in+dut.bank[i].producer;
        written=written+1;
      end
    end
    always @(posedge clk_b) begin
      if(dut.bank[i].empty) saw_empty=saw_empty+1;
      if(dut.bank[i].pop) begin
        if(read_count>=written) $fatal(1,"FIFO underflow in bank %0d",i);
        next_read=expected[read_count];
        read_count=read_count+1;
        #0.1;
        if(dut.tree[2+i] !== next_read)
          $fatal(1,"FIFO ordering/data mismatch in bank %0d at word %0d",i,read_count);
      end
    end
    initial begin
      #20000;
      if(read_count<100 || saw_full==0 || saw_empty==0)
        $fatal(1,"insufficient FIFO wrap/full/empty coverage in bank %0d",i);
      $display("FIFO bank %0d: written=%0d read=%0d full=%0d empty=%0d",i,written,read_count,saw_full,saw_empty);
    end
  end
  initial begin #20001; $display("Dual-clock FIFO functional check passed"); $finish; end
endmodule
