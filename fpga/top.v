/*
 * Module: top
 * Author: Original Author
 * Date: Original Date
 * Version: 1.0
 *
 * Description:
 * This is the top-level module for the MachXO2 FPGA.
 * It instantiates an internal oscillator, a blinky LED module, reset generation logic,
 * an I2C GPIO interface, an MPEG-TS tuner, and an I2S transmitter.
 * The main functions include:
 *  - Generating a system clock using an internal oscillator.
 *  - Providing a power-on reset signal.
 *  - Interfacing with an I2C bus for control/status.
 *  - Receiving MPEG-TS data from an external tuner.
 *  - Processing and streaming this data via an I2S interface.
 *  - Indicating status via an LED.
 *
 * Note on unused inputs:
 * The original code mentioned XOR-reducing inputs to avoid 'unused' warnings.
 * This is implicitly handled by the LED driving logic which incorporates the 'SYNC' input
 * and 'tuner_led_toggle' (derived from other tuner inputs).
 */
 
`include "efb_define_def.v"

module top
(
    /* I2C Interface */
    inout SCL,                      /* I2C Clock line */
    inout SDA,                      /* I2C Data line */

    /* LED Output */
    output LED_OD,                  /* Open-drain LED output. Active low (0 = ON, Z = OFF). */

    /* FPGA Master Clock Input */
    input FPGA_CLK,                 /* Main clock for I2S transmitter and potentially other synchronous logic. */

    /* I2S Output Interface */
    output I2S_BCK,                 /* I2S Bit Clock */
    output I2S_DAT,                 /* I2S Data */
    output I2S_WS,                  /* I2S Word Select (Left/Right Channel Clock) */

    /* Tuner Data Input Interface */
    input TUNER_CLK,                /* Clock signal from the tuner, data is synchronous to this clock. */
    input D0,                       /* Tuner Data Bit 0 (LSB) */
    input D1,                       /* Tuner Data Bit 1 */
    input D2,                       /* Tuner Data Bit 2 */
    input D3,                       /* Tuner Data Bit 3 */
    input D4,                       /* Tuner Data Bit 4 */
    input D5,                       /* Tuner Data Bit 5 */
    input D6,                       /* Tuner Data Bit 6 */
    input D7,                       /* Tuner Data Bit 7 (MSB) */
    input DATA_VALID,               /* Signal from tuner indicating D0-D7 are valid. */
    input SYNC                      /* Signal from tuner indicating start of a new packet. */
);

    /*------------------------------------------------------------------------*/
    /* Internal Wires and Registers                                           */
    /*------------------------------------------------------------------------*/
    wire osc;                       /* Output of the internal oscillator, used for reset, I2C, and blinky. */
    wire blinky_led_signal;         /* Signal from the blinky module to toggle the LED. */
    wire tuner_led_toggle;          /* Signal from the tuner module, toggles on synced packet. */
    wire enable;                    /* Enable signal, typically controlled by I2C GPIO. */
    wire rst_n;                     /* Active-low reset signal, generated internally. */

    /* Data bus from tuner to I2S transmitter */
    wire [7:0] tuner_stream_data;       /* Parallel data from tuner module. */
    wire       tuner_stream_data_valid; /* Validity signal for tuner_stream_data. */
    wire       tuner_stream_sync_pulse; /* Sync pulse associated with tuner_stream_data. */

    /* Register for reset generation counter */
    reg [7:0] rst_ctr = 8'd0;       /* Counter used to generate the initial power-on reset pulse. */
    reg [31:0] fpga_clock_cycles = 32'd0;
    reg [31:0] tuner_clock_cycles = 32'd0;


    /*------------------------------------------------------------------------*/
    /* Internal Oscillator (OSCH)                                             */
    /*------------------------------------------------------------------------*/
    /* Instantiates the MachXO2's internal high-frequency oscillator.
     * NOM_FREQ: Specifies the nominal frequency (e.g., "8.31" for 8.31 MHz).
     * STDBY: Standby control. 1'b0 means oscillator is active.
     * OSC: Output of the oscillator.
     */
    defparam i_osc.NOM_FREQ = "8.31"; /* Sets nominal frequency of the oscillator */
    OSCH i_osc (
        .STDBY    (1'b0),         /* Oscillator is enabled */
        .OSC      (osc),          /* Oscillator output wire */
        .SEDSTDBY ()              /* Standby control for SED (not used) */
    );


    /*------------------------------------------------------------------------*/
    /* LED Blinker Module (blinky)                                            */
    /*------------------------------------------------------------------------*/
    /* This module generates a periodic signal to make the LED blink.
     * clk: Clock input, driven by the internal oscillator 'osc'.
     * led: Output signal that toggles periodically.
     */
    blinky u_led (
        .clk (osc),                 /* Clock input for the blinker */
        .led (blinky_led_signal)    /* Output signal for LED blinking pattern */
    );


    /*------------------------------------------------------------------------*/
    /* Reset Generation Logic                                                 */
    /*------------------------------------------------------------------------*/
    /* This logic generates an active-low reset pulse (rst_n) upon power-up.
     * The reset is asserted (rst_n = 0) for the first 16 cycles of the 'osc' clock.
     * After these initial 16 cycles, rst_n is de-asserted (rst_n = 1) and remains de-asserted.
     */

    /* rst_n is active-low.
     * It is 0 (asserted) if rst_ctr[7:4] are all 0 (i.e., rst_ctr is 0-15).
     * It is 1 (de-asserted) if any of rst_ctr[7:4] are 1 (i.e., rst_ctr is 16 or greater, up to 254, then 128 onwards).
     */
    assign rst_n = (rst_ctr & 8'b11110000) != 0;

    /* Counter logic for rst_ctr. */
    always @(posedge osc) begin
        if (rst_ctr == 8'b11111110) begin   /* When rst_ctr reaches 254 */
            rst_ctr <= 8'b10000000;         /* It rolls over to 128. For rst_ctr = 128, rst_ctr[7:4] = 1000, so rst_n remains 1 (de-asserted). */
        end else begin
            rst_ctr <= rst_ctr + 1;         /* Increment counter */
        end
    end


    /*------------------------------------------------------------------------*/
    /* LED Driving Logic                                                      */
    /*------------------------------------------------------------------------*/
    /* The LED_OD is an open-drain output.
     * It is turned ON by driving it to '0' and OFF by driving it to 'Z' (high-impedance).
     * The led_signal combines the blinky pattern with SYNC and tuner_led_toggle,
     * and is gated by the 'enable' signal (from I2C).
     * The XOR combination helps ensure inputs are "used".
     */
    wire led_signal_intermediate;
    assign led_signal_intermediate = !blinky_led_signal ^ SYNC ^ tuner_led_toggle;
    assign LED_OD = (!enable & led_signal_intermediate) ? 1'b0 : 1'b1; /* LED on if condition true, else high-Z */

    /*------------------------------------------------------------------------*/
    /* I2C GPIO Interface (i2c_gpio)                                          */
    /*------------------------------------------------------------------------*/
    /* This module provides an I2C interface that can be used to control
     * signals like 'enable' and read status like 'rst_ctr'.
     * SCL, SDA: I2C bus lines.
     * Enable: An output from this module, used here to gate the LED.
     * Status: An input to this module, connected to rst_ctr to make it readable via I2C.
     * RST_N: Active-low reset for this I2C module.
     * CLK: Clock for this module, driven by 'osc'.
     
    i2c_gpio u_i2c_gpio (
        .SCL    (SCL),
        .SDA    (SDA),
        .Enable (enable),           
        .Status (rst_ctr),          
        .RST_N  (rst_n),            
        .CLK    (osc),              
        .fpga_clock_cycles (tuner_clock_cycles)
    );*/


    /*------------------------------------------------------------------------*/
    /* MPEG-TS Tuner Interface Logic (tuner)                                  */
    /*------------------------------------------------------------------------*/
    /* This module interfaces with the external tuner, processes the incoming
     * MPEG-TS stream (D0-D7, DATA_VALID, SYNC, clocked by TUNER_CLK),
     * and outputs a processed stream.
     * synced_packet_led_toggle: Toggles to indicate packet synchronization.
     * stream_data_out, stream_data_valid_out, stream_sync_pulse_out: Processed output stream.
    tuner u_tuner (
        .clk                     (TUNER_CLK),    
        .valid                   (DATA_VALID),   
        .sync                    (SYNC),         
        .d0                      (D0),           
        .d1                      (D1),           
        .d2                      (D2),           
        .d3                      (D3),           
        .d4                      (D4),           
        .d5                      (D5),           
        .d6                      (D6),           
        .d7                      (D7),           
        .synced_packet_led_toggle(tuner_led_toggle),       
        .stream_data_out         (tuner_stream_data),      
        .stream_data_valid_out   (tuner_stream_data_valid),
        .stream_sync_pulse_out   (tuner_stream_sync_pulse) 
    );
     */

    /* Counter logic for tuner clock. */
    always @(negedge TUNER_CLK) begin
        tuner_clock_cycles <= tuner_clock_cycles + 1; 
        if(DATA_VALID) begin
            //tuner_clock_cycles_valid <= tuner_clock_cycles_valid + 1; 
        end
    end

    /* Counter logic for fpga clock. */
    always @(negedge FPGA_CLK) begin
        fpga_clock_cycles <= fpga_clock_cycles + 1;         /* Increment counter */
    end

    efb UUT  ( .wb_clk_i (osc), .i2c1_scl (SCL), .i2c1_sda (SDA) );

    /*------------------------------------------------------------------------*/
    /* I2S Transmitter (i2s_tx)                                               */
    /*------------------------------------------------------------------------*/
    /* This module takes the processed data stream from the 'tuner' module
     * and transmits it over an I2S interface.
     * clk: Master clock for I2S logic (FPGA_CLK).
     * stream_data_in, stream_data_valid_in, stream_sync_pulse_in: Input data stream.
     * i2s_bck, i2s_dat, i2s_ws: I2S output signals.
     */
     
    i2s_tx u_i2s (
        .clk                   (FPGA_CLK),                
        .stream_data_in        (8'b10100101),       
        .stream_data_valid_in  (fpga_clock_cycles[15]), 
        .i2s_bck               (I2S_BCK),                 
        .i2s_dat               (I2S_DAT),                 
        .i2s_ws                (I2S_WS)                   
    ); 
endmodule

