
module test (input wire clk,
             input wire rst,
             input wire[7:0] data_in,
             output reg[7:0] data_out);

always @(posedge clk)
begin
    data_out <= data_in + data_in;
    if (rst) begin
        data_out <= 0;
    end
end

endmodule

