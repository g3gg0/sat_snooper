/* MPEG-TS Sync and PID Matcher (Simplified for Sync)
 * ----------------------------------------------------------------
 * - Waits for 0x47 (SYNC_BYTE) to appear when not in sync.
 * - Once 0x47 is found, it assumes sync and starts counting 188 bytes.
 * - If the 0th byte of a subsequent packet is not 0x47, it goes out of sync.
 * - Toggles an LED signal on the 0th byte of every synced packet.
 * - Provides a byte_enable signal for writing to RAM.
 */
module mpeg_ts_sync_pid
(
    input           clk,                /* System clock */
    input           data_valid,         /* Input data byte is valid */
    input   [7:0]   data_byte_in,       /* Input data byte */

    output  reg     sync_pulse,         /* High for one cycle at the start of a synced packet */
    output  reg     byte_enable_out,    /* High when data should be written to RAM */
    output  reg     is_in_sync,         /* High when currently in sync */
    output  reg     led_toggle_signal   /* Toggles on 0th byte of a synced packet */
);

    localparam SYNC_BYTE        = 8'h47;
    localparam BYTES_PER_PKT    = 188;

    /* State machine */
    localparam S_UNSYNCED       = 2'd0;
    localparam S_FIRST_SYNC     = 2'd1; /* Found first 0x47, waiting to confirm next packet */
    localparam S_SYNCED         = 2'd2;

    reg [1:0] current_state = S_UNSYNCED;
    reg [7:0] byte_counter  = 8'd0;   /* 0 to 187 */

    always @(posedge clk) begin
        sync_pulse <= 1'b0; /* Default to low */
        byte_enable_out <= 1'b0; /* Default to low */

        case (current_state)
            S_UNSYNCED: begin
                is_in_sync <= 1'b0;
                byte_counter <= 8'd0;
                if (data_valid && (data_byte_in == SYNC_BYTE)) begin
                    current_state <= S_FIRST_SYNC;
                    is_in_sync <= 1'b1; /* Tentatively in sync */
                    sync_pulse <= 1'b1; /* Pulse for the first sync byte */
                    led_toggle_signal <= ~led_toggle_signal; /* Toggle on first sync */
                    byte_enable_out <= 1'b1;
                    byte_counter <= 8'd1; /* Start counting from the next byte */
                end
            end

            S_FIRST_SYNC: begin
                is_in_sync <= 1'b1;
                if (data_valid) begin
                    byte_enable_out <= 1'b1;
                    if (byte_counter == (BYTES_PER_PKT - 1)) begin
                        current_state <= S_SYNCED;
                        byte_counter <= 8'd0;
                    end else begin
                        byte_counter <= byte_counter + 1'b1;
                    end
                end
            end

            S_SYNCED: begin
                is_in_sync <= 1'b1;
                if (data_valid) begin
                    byte_enable_out <= 1'b1;
                    if (byte_counter == 8'd0) begin /* Check 0th byte of new packet */
                        if (data_byte_in == SYNC_BYTE) begin
                            sync_pulse <= 1'b1;
                            led_toggle_signal <= ~led_toggle_signal;
                            byte_counter <= 8'd1;
                        end else begin
                            current_state <= S_UNSYNCED; /* Lost sync */
                            is_in_sync <= 1'b0;
                            byte_enable_out <= 1'b0; /* Do not write this invalid packet's start */
                        end
                    end else if (byte_counter == (BYTES_PER_PKT - 1)) begin
                        byte_counter <= 8'd0;
                    end else begin
                        byte_counter <= byte_counter + 1'b1;
                    end
                end
            end
            default: current_state <= S_UNSYNCED;
        endcase
    end

endmodule
