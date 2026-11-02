/* EFB I2C slave receiver for atomic PID filter configuration updates. */
`include "efb_define_def.v"

module i2c_pid_filter
(
    input clk,
    input rst_n,
    inout scl,
    inout sda,
    output reg [3:0] filter_enable,
    output reg [12:0] filter_pid_0,
    output reg [12:0] filter_mask_0,
    output reg [12:0] filter_pid_1,
    output reg [12:0] filter_mask_1,
    output reg [12:0] filter_pid_2,
    output reg [12:0] filter_mask_2,
    output reg [12:0] filter_pid_3,
    output reg [12:0] filter_mask_3,
    output reg config_toggle
);
    localparam [7:0] CONFIG_COMMAND = 8'hf0;
    localparam [3:0] STATE_ENABLE = 4'd0;
    localparam [3:0] STATE_RESET_RECEIVER = 4'd1;
    localparam [3:0] STATE_WAIT_IDLE = 4'd2;
    localparam [3:0] STATE_DISCARD_0 = 4'd3;
    localparam [3:0] STATE_DISCARD_1 = 4'd4;
    localparam [3:0] STATE_WAIT_COMMAND = 4'd5;
    localparam [3:0] STATE_READ_COMMAND = 4'd6;
    localparam [3:0] STATE_ACK_COMMAND = 4'd7;
    localparam [3:0] STATE_WAIT_DATA = 4'd8;
    localparam [3:0] STATE_READ_DATA = 4'd9;
    localparam [3:0] STATE_NACK = 4'd10;
    localparam [3:0] STATE_WAIT_STOP = 4'd11;

    reg [3:0] state = STATE_ENABLE;
    reg wb_pending = 1'b0;
    reg command_valid = 1'b0;
    reg [4:0] data_index = 5'd0;
    reg [7:0] received_data [0:15];
    reg [7:0] wb_address;
    reg [7:0] wb_write_data;
    reg wb_write;
    wire [7:0] wb_read_data;
    wire wb_ack;
    wire wb_strobe = wb_pending;

    efb u_efb (
        .wb_clk_i(clk),
        .wb_rst_i(~rst_n),
        .wb_cyc_i(wb_strobe),
        .wb_stb_i(wb_strobe),
        .wb_we_i(wb_write),
        .wb_adr_i(wb_address),
        .wb_dat_i(wb_write_data),
        .wb_dat_o(wb_read_data),
        .wb_ack_o(wb_ack),
        .i2c1_scl(scl),
        .i2c1_sda(sda),
        .i2c1_irqo()
    );

    always @(*) begin
        wb_address = `MICO_EFB_I2C_SR;
        wb_write_data = 8'h00;
        wb_write = 1'b0;
        case (state)
            STATE_ENABLE: begin
                wb_address = `MICO_EFB_I2C_CR;
                wb_write_data = `MICO_EFB_I2C_CR_I2CEN;
                wb_write = 1'b1;
            end
            STATE_RESET_RECEIVER: begin
                wb_address = `MICO_EFB_I2C_CMDR;
                wb_write_data = `MICO_EFB_I2C_CMDR_CKSDIS;
                wb_write = 1'b1;
            end
            STATE_DISCARD_0,
            STATE_DISCARD_1,
            STATE_READ_COMMAND,
            STATE_READ_DATA: begin
                wb_address = `MICO_EFB_I2C_RXDR;
            end
            STATE_ACK_COMMAND: begin
                wb_address = `MICO_EFB_I2C_CMDR;
                wb_write_data = command_valid ?
                                `MICO_EFB_I2C_CMDR_CKSDIS :
                                (`MICO_EFB_I2C_CMDR_NACK | `MICO_EFB_I2C_CMDR_CKSDIS);
                wb_write = 1'b1;
            end
            STATE_NACK: begin
                wb_address = `MICO_EFB_I2C_CMDR;
                wb_write_data = `MICO_EFB_I2C_CMDR_NACK | `MICO_EFB_I2C_CMDR_CKSDIS;
                wb_write = 1'b1;
            end
            default: begin
                wb_address = `MICO_EFB_I2C_SR;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_ENABLE;
            wb_pending <= 1'b0;
            command_valid <= 1'b0;
            data_index <= 5'd0;
            filter_enable <= 4'd0;
            filter_pid_0 <= 13'd0;
            filter_mask_0 <= 13'd0;
            filter_pid_1 <= 13'd0;
            filter_mask_1 <= 13'd0;
            filter_pid_2 <= 13'd0;
            filter_mask_2 <= 13'd0;
            filter_pid_3 <= 13'd0;
            filter_mask_3 <= 13'd0;
            config_toggle <= 1'b0;
        end else if (!wb_pending) begin
            wb_pending <= 1'b1;
        end else if (wb_ack) begin
            wb_pending <= 1'b0;
            case (state)
                STATE_ENABLE: state <= STATE_RESET_RECEIVER;
                STATE_RESET_RECEIVER: state <= STATE_WAIT_IDLE;
                STATE_WAIT_IDLE: begin
                    if (!(wb_read_data & `MICO_EFB_I2C_SR_BUSY))
                        state <= STATE_DISCARD_0;
                end
                STATE_DISCARD_0: state <= STATE_DISCARD_1;
                STATE_DISCARD_1: state <= STATE_WAIT_COMMAND;
                STATE_WAIT_COMMAND: begin
                    if (wb_read_data & `MICO_EFB_I2C_SR_TRRDY)
                        state <= STATE_READ_COMMAND;
                    else if (!(wb_read_data & `MICO_EFB_I2C_SR_BUSY))
                        state <= STATE_RESET_RECEIVER;
                end
                STATE_READ_COMMAND: begin
                    command_valid <= wb_read_data == CONFIG_COMMAND;
                    data_index <= 5'd0;
                    state <= STATE_ACK_COMMAND;
                end
                STATE_ACK_COMMAND: begin
                    state <= command_valid ? STATE_WAIT_DATA : STATE_WAIT_STOP;
                end
                STATE_WAIT_DATA: begin
                    if (wb_read_data & `MICO_EFB_I2C_SR_TRRDY)
                        state <= STATE_READ_DATA;
                    else if (!(wb_read_data & `MICO_EFB_I2C_SR_BUSY))
                        state <= STATE_RESET_RECEIVER;
                end
                STATE_READ_DATA: begin
                    if (data_index < 5'd16)
                        received_data[data_index] <= wb_read_data;
                    if (data_index == 5'd16) begin
                        filter_enable <= received_data[0][3:0];
                        filter_pid_0 <= {received_data[1][4:0], received_data[2]};
                        filter_mask_0 <= {received_data[3][4:0], received_data[4]};
                        filter_pid_1 <= {received_data[5][4:0], received_data[6]};
                        filter_mask_1 <= {received_data[7][4:0], received_data[8]};
                        filter_pid_2 <= {received_data[9][4:0], received_data[10]};
                        filter_mask_2 <= {received_data[11][4:0], received_data[12]};
                        filter_pid_3 <= {received_data[13][4:0], received_data[14]};
                        filter_mask_3 <= {received_data[15][4:0], wb_read_data};
                        config_toggle <= ~config_toggle;
                        state <= STATE_NACK;
                    end else begin
                        data_index <= data_index + 1'b1;
                        state <= STATE_WAIT_DATA;
                    end
                end
                STATE_NACK: state <= STATE_WAIT_STOP;
                STATE_WAIT_STOP: begin
                    if (!(wb_read_data & `MICO_EFB_I2C_SR_BUSY))
                        state <= STATE_RESET_RECEIVER;
                end
                default: state <= STATE_ENABLE;
            endcase
        end
    end
endmodule