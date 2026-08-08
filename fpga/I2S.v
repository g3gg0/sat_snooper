/* Continuous 8-bit MSB I2S transmitter for the ESP32-S3 receiver. */

module i2s_tx
(
    input             clk,          /* master / bit clock                 */
    input             rst_n,

    /* Inputs for receiving streamed data */
    input      [7:0]  stream_data_in,
    input             stream_data_valid_in,
    output            stream_data_ready_out,

    /* I2S lines */
    output reg        i2s_bck,
    output reg        i2s_dat,
    output reg        i2s_ws
);

    reg [7:0] shift_reg;
    reg [2:0] bit_pos_counter;
    reg [2:0] clock_divider;

    assign stream_data_ready_out = bit_pos_counter == 3'd0 &&
                                   clock_divider == 3'd4 && i2s_bck;

    /* BCK is clk/5. DAT and WS change only when BCK falls. */
    always @(negedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 8'hff;
            bit_pos_counter <= 3'd0;
            clock_divider <= 3'd0;
            i2s_bck <= 1'b0;
            i2s_dat <= 1'b1;
            i2s_ws <= 1'b0;
        end else begin
            if (clock_divider == 3'd4)
                clock_divider <= 3'd0;
            else
                clock_divider <= clock_divider + 1'b1;

            if (clock_divider == 3'd1) begin
                i2s_bck <= 1'b1;
            end else if (clock_divider == 3'd4) begin
                i2s_bck <= 1'b0;
                if (bit_pos_counter == 3'd0) begin
                    bit_pos_counter <= 3'd7;
                    i2s_ws <= ~i2s_ws;
                    if (stream_data_valid_in) begin
                        shift_reg <= stream_data_in;
                        i2s_dat <= stream_data_in[7];
                    end else begin
                        shift_reg <= 8'hff;
                        i2s_dat <= 1'b1;
                    end
                end else begin
                    bit_pos_counter <= bit_pos_counter - 1'b1;
                    i2s_dat <= shift_reg[bit_pos_counter - 1'b1];
                end
            end
        end
    end
endmodule
