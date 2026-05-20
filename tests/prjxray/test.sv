
module test (input wire clk,
             input wire rst,
             input wire[7:0] data_in,
             output reg[7:0] data_out);

(* keep = "true" *) reg [31:0] lane0;
(* keep = "true" *) reg [31:0] lane1;

wire [31:0] din = {4{data_in}};
wire [31:0] rot0 = {lane0[22:0], lane0[31:23]};
wire [31:0] rot1 = {lane1[18:0], lane1[31:19]};

wire [31:0] sum0 = lane0 + din + {lane1[15:0], lane0[31:16]};
wire [31:0] sum1 = lane1 + rot0 + {din[23:0], din[31:24]};

wire [31:0] next0 = sum0 ^ rot1 ^ (lane1 | din);
wire [31:0] next1 = sum1 ^ rot0 ^ (lane0 & {din[15:0], din[31:16]});

wire [31:0] fold = next0 ^ {next1[15:0], next1[31:16]};

always @(posedge clk)
begin
    if (rst) begin
        lane0 <= 32'h0123_4567;
        lane1 <= 32'hfedc_ba98;
        data_out <= 0;
    end else begin
        lane0 <= next0;
        lane1 <= next1;
        data_out <= {
            ^fold[31:24],
            ^fold[23:16],
            ^fold[15:8],
            ^fold[7:0],
            ^(fold[31:16] & fold[15:0]),
            ^(next0[31:16] | next1[15:0]),
            ^(next0[15:0] ^ next1[31:16]),
            ^(next0 + next1)
        };
    end
end

endmodule
