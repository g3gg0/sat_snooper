/*  Simplified I2S transmitter
    ----------------------------------------------------------
     Drives BCK, WS, DAT.
     On positive edge of stream_data_valid_in, it captures stream_data_in.
     Transmits the 8-bit data MSB-first.
     i2s_ws toggles after every 188 bytes are transmitted.
*/

module i2s_tx
(
    input             clk,          /* master / bit clock                 */

    /* Inputs for receiving streamed data */
    input      [7:0]  stream_data_in,
    input             stream_data_valid_in,

    /* I2S lines */
    output            i2s_bck,
    output reg        i2s_dat = 1'b0,
    output reg        i2s_ws  = 1'b0  /* Initial state of WS */
);

    assign i2s_bck = is_transmitting ? clk : 1'b0;

    reg [7:0] shift_reg;                     /* Holds the byte currently being transmitted */
    reg [2:0] bit_pos_counter;               /* Counts bits from 7 down to 0 (MSB first) */
    reg [7:0] byte_tx_counter = 8'd0;        /* Counts transmitted bytes from 0 to 187 */
    reg       is_transmitting = 1'b0;        /* Flag: true if a byte is currently being shifted out */

    reg prev_stream_data_valid_in_ff = 1'b0; /* Flip-flop to detect posedge */

    always @(posedge clk) begin
        prev_stream_data_valid_in_ff <= stream_data_valid_in;
    end

    wire stream_data_valid_posedge = stream_data_valid_in && !prev_stream_data_valid_in_ff;

    always @(posedge clk) begin
        if (!is_transmitting) begin
            if (stream_data_valid_posedge) begin
                shift_reg         <= stream_data_in;
                is_transmitting   <= 1'b1;
                bit_pos_counter   <= 3'd7; /* Start with MSB */
            end
        end else begin /* is_transmitting is true */
            i2s_dat <= shift_reg[bit_pos_counter];

            if (bit_pos_counter == 3'd0) begin /* Current bit is LSB, byte transmission finished this cycle */
                is_transmitting <= 1'b0; /* Ready for next byte capture in the next cycle */

                if (byte_tx_counter == 8'd187) begin /* 188 bytes (0-187) have been transmitted */
                    byte_tx_counter <= 8'd0;
                    i2s_ws          <= ~i2s_ws; /* Toggle WS after 188 bytes */
                end else begin
                    byte_tx_counter <= byte_tx_counter + 1'b1;
                end
            end else begin
                bit_pos_counter <= bit_pos_counter - 1'b1;
            end
        end
    end
endmodule
