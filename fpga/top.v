/* Dummy top for MachXO2
 * All external pins except LED_OD are declared as inputs.
 * Inputs are XOR-reduced to avoid �unused� warnings.
 */

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
	
    wire tuner_led_toggle;
	wire osc;
	
	/* -- internal oscillator ------------------------------------------- */
	defparam i_osc.NOM_FREQ = "8.31";
	OSCH i_osc ( .STDBY (1'b0), .OSC (osc), .SEDSTDBY () );

    /* -- LED fade submodule ------------------------------- */
    wire blinky_led_signal; /* Intermediate wire for blinky output */
    blinky u_led (
		.clk (osc),
        .led (blinky_led_signal)
    );

	reg [7:0] rst_ctr = 0;
	wire rst_n;

	assign rst_n = (rst_ctr & 8'b11110000) != 0;

	always @(posedge osc) begin
		if (rst_ctr == 8'b11111110) begin
			rst_ctr <= 8'b10000000;
		end else begin
			rst_ctr <= rst_ctr + 1;
		end
	end


	wire enable;
    /* Combine blinky output with unused inputs and sync toggle to drive LED_OD */
    wire led_signal = !enable & (!blinky_led_signal ^ SYNC ^ tuner_led_toggle);
    assign LED_OD = led_signal ? 1'b0 : 1'bz;
	
    i2c_gpio u_i2c_gpio (
		.SCL (SCL),
		.SDA (SDA),
        .Enable (enable),
        .Status (rst_ctr),
		.RST_N (rst_n),
		.CLK (osc)
    );
	
	
    /* Wires for streaming data from tuner to i2s_tx */
    wire [7:0] tuner_stream_data;
    wire       tuner_stream_data_valid;
    wire       tuner_stream_sync_pulse;
 
    /* -- MPEG-TS tuner submodule -------------------------- */
    tuner u_tuner
    (
        .clk     (TUNER_CLK),   /* sample clock                */
        .valid   (DATA_VALID),  /* byte is valid               */
        .sync    (SYNC),        /* start of new packet (original sync, mpeg_ts_sync_pid uses data_byte_in) */
        .d0      (D0),
        .d1      (D1),
        .d2      (D2),
        .d3      (D3),
        .d4      (D4),
        .d5      (D5),
        .d6      (D6),
        .d7      (D7),
        .synced_packet_led_toggle (tuner_led_toggle),
        .stream_data_out (tuner_stream_data),
        .stream_data_valid_out (tuner_stream_data_valid),
        .stream_sync_pulse_out (tuner_stream_sync_pulse)
    );
	 
    /* -- I2S transmitter ----------------------------------- */
    i2s_tx u_i2s (
        .clk      (FPGA_CLK),           /* i2s_tx master clock */
        .stream_data_in (tuner_stream_data),
        .stream_data_valid_in (tuner_stream_data_valid),
        .stream_sync_pulse_in (tuner_stream_sync_pulse),
        .i2s_bck  (I2S_BCK),
        .i2s_dat  (I2S_DAT),
        .i2s_ws   (I2S_WS)
    );


endmodule

