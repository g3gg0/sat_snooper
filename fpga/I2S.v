/*  I2S transmitter
    -----------------------------------------------------------
     Drives BCK, WS, DAT.
     Reads bytes sequentially from tuner RAM (1504-byte ring).
     Sends each byte MSB-first; WS toggles every byte
      (left / right 8-bit samples).                           */

module i2s_tx
(
    input             clk,          /* master / bit clock                 */

    /* Inputs for receiving streamed data from tuner */
    input      [7:0]  stream_data_in,
    input             stream_data_valid_in,
    input             stream_sync_pulse_in,

    /* I2S lines */
    output            i2s_bck,
    output reg        i2s_dat = 1'b0,
    output reg        i2s_ws  = 1'b0
);

    /* -- clock mapping ------------------------------------- */
    assign i2s_bck = clk;

    /* -- FIFO Buffer ----------------------------------------- */
    localparam FIFO_DEPTH_BITS = 4; /* e.g., 4 bits = 16 entries */
    localparam FIFO_DEPTH      = 1 << FIFO_DEPTH_BITS;
    reg [7:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [FIFO_DEPTH_BITS-1:0] fifo_wr_ptr = {FIFO_DEPTH_BITS{1'b0}};
    reg [FIFO_DEPTH_BITS-1:0] fifo_rd_ptr = {FIFO_DEPTH_BITS{1'b0}};
    reg [FIFO_DEPTH_BITS:0]   fifo_count  = {(FIFO_DEPTH_BITS+1){1'b0}}; /* To detect full/empty */

    wire fifo_full  = (fifo_count == FIFO_DEPTH);
    wire fifo_empty = (fifo_count == 0);

    /* -- I2S bit-level timing and shift register control --- */
    reg [7:0] shift_reg = 8'd0; /* Loaded from FIFO */
    reg [2:0] bit_pos   = 3'd7; /* 7-0 MSB first */
    wire load_next_byte_for_i2s = (bit_pos == 3'd7); /* Signal to FIFO logic */

    always @(posedge clk) begin
        i2s_dat <= shift_reg[bit_pos]; /* Output current bit */

        if (bit_pos == 3'd0) begin
            bit_pos   <= 3'd7;  /* Reset bit counter */
            i2s_ws    <= ~i2s_ws; /* Toggle word select */
        end else begin
            bit_pos <= bit_pos - 1'b1;
        end
    end

    /* -- Combined FIFO Read/Write Logic -------------------- */
    always @(posedge clk) begin
        /* Priority 1: FIFO Reset on sync pulse if full */
        if (stream_sync_pulse_in && fifo_full) begin
            fifo_wr_ptr <= {FIFO_DEPTH_BITS{1'b0}};
            fifo_rd_ptr <= {FIFO_DEPTH_BITS{1'b0}};
            fifo_count  <= {(FIFO_DEPTH_BITS+1){1'b0}};
            /* shift_reg is not cleared here, will be loaded if new data comes or cleared if FIFO empty */
        end else begin
            /* FIFO Write operation */
            if (stream_data_valid_in) begin
                if (!fifo_full) begin
                    fifo_mem[fifo_wr_ptr] <= stream_data_in;
                    fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
                    fifo_count  <= fifo_count + 1'b1;
                end
                /* else: Data is dropped if FIFO is full and no sync pulse */
            end

            /* FIFO Read operation (loading shift_reg) */
            if (load_next_byte_for_i2s) begin
                if (!fifo_empty) begin
                    shift_reg   <= fifo_mem[fifo_rd_ptr];
                    fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
                    fifo_count  <= fifo_count - 1'b1;
                end else begin
                    shift_reg <= 8'h00; /* Output zeros if FIFO is empty and I2S needs data */
                end
            end
        end
    end
endmodule
