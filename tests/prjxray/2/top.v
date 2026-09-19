// Boardless controller-core test. No DDR PHY is instantiated or emulated.
// A byte shift input exposes every active controller input with few package pins.
// An indexed byte output makes the controller/PHY interface observable.
module ddr_controller_example (
    input clk, input rst, input load, input [7:0] data_in,
    input [5:0] read_index, output [7:0] data_out
);
    reg [191:0] inputs = 0;
    always @(posedge clk)
        if (rst) inputs <= 0;
        else if (load) inputs <= {inputs[183:0], data_in};
    wire [63:0] wb_data, phy_data;
    wire [95:0] phy_cmd;
    wire [7:0] phy_dm;
    wire [3:0] aux;
    wire [31:0] debug_data;
    wire stall, ack, error, calibrated, uart_tx;
    wire dq_tri, dqs_tri, toggle_dqs, phy_reset, leveling;
    wire [4:0] odelay_data, odelay_dqs, idelay_data, idelay_dqs;
    wire odelay_data_ld, odelay_dqs_ld, idelay_data_ld, idelay_dqs_ld, bitslip;
    ddr3_controller #(
        .LANES(1), .AUX_WIDTH(4), .ODELAY_SUPPORTED(0),
        .SECOND_WISHBONE(0), .ECC_ENABLE(0), .BIST_MODE(1)
    ) controller (
        .i_controller_clk(clk), .i_rst_n(!rst),
        .i_wb_cyc(inputs[0]), .i_wb_stb(inputs[1]), .i_wb_we(inputs[2]),
        .i_wb_addr(inputs[26:3]), .i_wb_data(inputs[90:27]),
        .i_wb_sel(inputs[98:91]), .i_aux(inputs[102:99]),
        .o_wb_stall(stall), .o_wb_ack(ack), .o_wb_err(error),
        .o_wb_data(wb_data), .o_aux(aux),
        .i_wb2_cyc(1'b0), .i_wb2_stb(1'b0), .i_wb2_we(1'b0),
        .i_wb2_addr(7'b0), .i_wb2_sel(4'b0), .i_wb2_data(32'b0),
        .i_phy_iserdes_data(inputs[166:103]),
        .i_phy_iserdes_dqs(inputs[174:167]),
        .i_phy_iserdes_bitslip_reference(inputs[182:175]),
        .i_phy_idelayctrl_rdy(inputs[183]), .i_user_self_refresh(inputs[184]),
        .o_phy_cmd(phy_cmd), .o_phy_dqs_tri_control(dqs_tri),
        .o_phy_dq_tri_control(dq_tri), .o_phy_toggle_dqs(toggle_dqs),
        .o_phy_data(phy_data), .o_phy_dm(phy_dm),
        .o_phy_odelay_data_cntvaluein(odelay_data),
        .o_phy_odelay_dqs_cntvaluein(odelay_dqs),
        .o_phy_idelay_data_cntvaluein(idelay_data),
        .o_phy_idelay_dqs_cntvaluein(idelay_dqs),
        .o_phy_odelay_data_ld(odelay_data_ld), .o_phy_odelay_dqs_ld(odelay_dqs_ld),
        .o_phy_idelay_data_ld(idelay_data_ld), .o_phy_idelay_dqs_ld(idelay_dqs_ld),
        .o_phy_bitslip(bitslip), .o_phy_write_leveling_calib(leveling),
        .o_phy_reset(phy_reset), .o_calib_complete(calibrated),
        .o_debug1(debug_data), .uart_tx(uart_tx)
    );
    wire [511:0] outputs = {debug_data, bitslip, idelay_dqs_ld, idelay_data_ld,
        odelay_dqs_ld, odelay_data_ld, idelay_dqs, idelay_data, odelay_dqs,
        odelay_data, leveling, phy_reset, toggle_dqs, dq_tri, dqs_tri,
        uart_tx, calibrated, error, ack, stall, aux, phy_dm, phy_cmd, phy_data, wb_data};
    assign data_out = outputs[read_index * 8 +: 8];
endmodule
