// Exercise the original encoder, not the ECP5-specific camera/serializer top.
module tmds_example (
    input clk, input rst, input [1:0] state, input [7:0] pixel,
    input [1:0] sync, output [9:0] encoded
);
    TMDS_Encoder encoder (
        .clklow(clk), .reset(rst), .state(state), .pix_data(pixel),
        .H_VSync_Ctr(sync), .aux_data(4'b0), .q_out(encoded)
    );
endmodule
