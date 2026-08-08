`include "efb_define_def.v"

module top
(
    inout SCL,
    inout SDA,
    output LED_OD,
    input FPGA_CLK,
    output I2S_BCK,
    output I2S_DAT,
    output I2S_WS,
    input TUNER_CLK,
    input D0,
    input D1,
    input D2,
    input D3,
    input D4,
    input D5,
    input D6,
    input D7,
    input DATA_VALID,
    input SYNC
);
    wire osc;
    reg [7:0] rst_ctr = 8'd0;
    wire rst_n = (rst_ctr & 8'hf0) != 0;
    wire [7:0] tuner_stream_data;
    wire tuner_stream_data_valid;
    wire tuner_stream_sync_pulse;
    wire tuner_stream_sync_error;
    wire [7:0] tuner_stream_byte_index;
    wire tuner_stream_pid_valid;
    wire [12:0] tuner_stream_pid;
    wire tuner_led_toggle;
    wire [7:0] frame_data;
    wire frame_data_valid;
    wire frame_data_ready;

    defparam i_osc.NOM_FREQ = "8.31";
    OSCH i_osc (
        .STDBY(1'b0),
        .OSC(osc),
        .SEDSTDBY()
    );

    always @(posedge osc) begin
        if (rst_ctr == 8'hfe)
            rst_ctr <= 8'h80;
        else
            rst_ctr <= rst_ctr + 1'b1;
    end

    assign LED_OD = tuner_led_toggle ? 1'b0 : 1'bz;

    tuner u_tuner (
        .clk(TUNER_CLK),
        .rst_n(rst_n),
        .valid(DATA_VALID),
        .sync(SYNC),
        .d0(D0),
        .d1(D1),
        .d2(D2),
        .d3(D3),
        .d4(D4),
        .d5(D5),
        .d6(D6),
        .d7(D7),
        .synced_packet_led_toggle(tuner_led_toggle),
        .stream_data_out(tuner_stream_data),
        .stream_data_valid_out(tuner_stream_data_valid),
        .stream_sync_pulse_out(tuner_stream_sync_pulse),
        .stream_sync_error_out(tuner_stream_sync_error),
        .stream_byte_index_out(tuner_stream_byte_index),
        .stream_pid_valid_out(tuner_stream_pid_valid),
        .stream_pid_out(tuner_stream_pid)
    );

    telemetry u_telemetry (
        .tuner_clk(TUNER_CLK),
        .fpga_clk(FPGA_CLK),
        .rst_n(rst_n),
        .ts_data(tuner_stream_data),
        .ts_data_valid(tuner_stream_data_valid),
        .ts_sync_pulse(tuner_stream_sync_pulse),
        .ts_sync_error_pulse(tuner_stream_sync_error),
        .ts_byte_index(tuner_stream_byte_index),
        .ts_pid_valid(tuner_stream_pid_valid),
        .ts_pid(tuner_stream_pid),
        .frame_data(frame_data),
        .frame_data_valid(frame_data_valid),
        .frame_data_ready(frame_data_ready)
    );

    efb UUT (
        .wb_clk_i(osc),
        .wb_rst_i(~rst_n),
        .wb_cyc_i(1'b0),
        .wb_stb_i(1'b0),
        .wb_we_i(1'b0),
        .wb_adr_i(8'd0),
        .wb_dat_i(8'd0),
        .i2c1_scl(SCL),
        .i2c1_sda(SDA)
    );

    i2s_tx u_i2s (
        .clk(FPGA_CLK),
        .rst_n(rst_n),
        .stream_data_in(frame_data),
        .stream_data_valid_in(frame_data_valid),
        .stream_data_ready_out(frame_data_ready),
        .i2s_bck(I2S_BCK),
        .i2s_dat(I2S_DAT),
        .i2s_ws(I2S_WS)
    );
endmodule