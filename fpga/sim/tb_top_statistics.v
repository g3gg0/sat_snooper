`timescale 1ns/1ps

module OSCH
#(
    parameter NOM_FREQ = "8.31"
)
(
    input STDBY,
    output reg OSC = 1'b0,
    output SEDSTDBY
);
    assign SEDSTDBY = 1'b0;
    always #3 OSC = ~OSC;
endmodule

module efb
(
    input wb_clk_i,
    input wb_rst_i,
    input wb_cyc_i,
    input wb_stb_i,
    input wb_we_i,
    input [7:0] wb_adr_i,
    input [7:0] wb_dat_i,
    output [7:0] wb_dat_o,
    output wb_ack_o,
    inout i2c1_scl,
    inout i2c1_sda,
    output i2c1_irqo
);
    assign wb_dat_o = 8'd0;
    assign wb_ack_o = 1'b0;
    assign i2c1_irqo = 1'b0;
endmodule

module tb_top_statistics;
    tri scl;
    tri sda;
    wire led_od;
    reg fpga_clk = 1'b0;
    wire i2s_bck;
    wire i2s_dat;
    wire i2s_ws;
    reg tuner_clk = 1'b0;
    reg [7:0] tuner_data = 8'd0;

    reg [7:0] payload [0:187];
    reg [7:0] received_type = 8'd0;
    reg [7:0] received_length = 8'd0;
    reg [7:0] expected_filtered_last = 8'd0;
    integer parser_state = 0;
    integer payload_index = 0;
    integer led_transition_count = 0;
    reg monitor_led = 1'b0;
    reg statistics_seen = 1'b0;
    reg pid_report_seen = 1'b0;
    reg pat_seen = 1'b0;
    integer filtered_packet_count = 0;
    reg bck_period_seen = 1'b0;
    time previous_bck_rise = 0;

    always #5 fpga_clk = ~fpga_clk;
    always #7 tuner_clk = ~tuner_clk;

    top dut (
        .SCL(scl),
        .SDA(sda),
        .LED_OD(led_od),
        .FPGA_CLK(fpga_clk),
        .I2S_BCK(i2s_bck),
        .I2S_DAT(i2s_dat),
        .I2S_WS(i2s_ws),
        .TUNER_CLK(tuner_clk),
        .D0(tuner_data[0]),
        .D1(tuner_data[1]),
        .D2(tuner_data[2]),
        .D3(tuner_data[3]),
        .D4(tuner_data[4]),
        .D5(tuner_data[5]),
        .D6(tuner_data[6]),
        .D7(tuner_data[7])
    );

    always @(posedge i2s_bck) begin
        if (dut.rst_n) begin
            if (previous_bck_rise != 0) begin
                if ($time - previous_bck_rise != 50) begin
                    $display("FAIL: expected BCK period 50 ns, got %0t ns",
                             $time - previous_bck_rise);
                    $finish(1);
                end
                bck_period_seen = 1'b1;
            end
            previous_bck_rise = $time;
        end
    end

    task send_tuner_byte;
        input [7:0] value;
        begin
            @(negedge tuner_clk);
            tuner_data = value;
        end
    endtask

    task send_packet;
        input [12:0] pid;
        input [7:0] seed;
        integer byte_index;
        begin
            @(negedge tuner_clk);
            tuner_data = 8'h47;
            @(negedge tuner_clk);
            tuner_data = {3'b010, pid[12:8]};
            @(negedge tuner_clk);
            tuner_data = pid[7:0];
            for (byte_index = 3; byte_index < 188; byte_index = byte_index + 1) begin
                @(negedge tuner_clk);
                tuner_data = seed + byte_index[7:0];
            end
        end
    endtask

    task verify_frame;
        begin
            case (received_type)
                8'd0: begin
                    if (received_length != 8'd14 ||
                        payload[0] !== 8'h05 || payload[1] !== 8'h00 ||
                        payload[2] !== 8'h00 || payload[3] !== 8'h00 ||
                        payload[4] !== 8'hac || payload[5] !== 8'h03 ||
                        payload[6] !== 8'h00 || payload[7] !== 8'h00 ||
                        payload[8] !== 8'h01 || payload[9] !== 8'h00 ||
                        payload[10] !== 8'h00 || payload[11] !== 8'h00 ||
                        payload[12] !== 8'h00 || payload[13] !== 8'h00) begin
                        $display("FAIL: statistics frame is incorrect: sync=%02x%02x%02x%02x bytes=%02x%02x%02x%02x errors=%02x%02x%02x%02x drops=%02x%02x",
                                 payload[3], payload[2], payload[1], payload[0],
                                 payload[7], payload[6], payload[5], payload[4],
                                 payload[11], payload[10], payload[9], payload[8],
                                 payload[13], payload[12]);
                        $finish(1);
                    end
                    statistics_seen = 1'b1;
                end
                8'd1: begin
                    if (received_length != 8'd5 ||
                        payload[0] !== 8'h00 ||
                        payload[1] !== 8'h00 || payload[2] !== 8'h00 ||
                        payload[3] !== 8'h23 || payload[4] !== 8'h01) begin
                        $display("FAIL: PID report frame is incorrect");
                        $finish(1);
                    end
                    pid_report_seen = 1'b1;
                end
                8'd2: begin
                    if (received_length != 8'd188 ||
                        payload[0] !== 8'h47 || payload[1] !== 8'h40 ||
                        payload[2] !== 8'h00 || payload[187] !== 8'hcb) begin
                        $display("FAIL: PAT frame is incorrect: %02x %02x %02x ... %02x",
                                 payload[0], payload[1], payload[2], payload[187]);
                        $finish(1);
                    end
                    pat_seen = 1'b1;
                end
                8'd3: begin
                    expected_filtered_last = 8'hdb + filtered_packet_count * 8'h10;
                    if (received_length != 8'd188 ||
                        payload[0] !== 8'h47 || payload[1] !== 8'h41 ||
                        payload[2] !== 8'h23 ||
                        payload[187] !== expected_filtered_last) begin
                        $display("FAIL: filtered TS packet is incorrect: %02x %02x %02x ... %02x",
                                 payload[0], payload[1], payload[2], payload[187]);
                        $finish(1);
                    end
                    filtered_packet_count = filtered_packet_count + 1;
                end
                default: begin
                    $display("FAIL: unexpected frame type %0d", received_type);
                    $finish(1);
                end
            endcase

            if (statistics_seen && pid_report_seen && pat_seen &&
                filtered_packet_count == 4) begin
                if (!bck_period_seen) begin
                    $display("FAIL: no complete BCK period observed");
                    $finish(1);
                end
                if (led_transition_count != 5 || led_od !== 1'b0) begin
                    $display("FAIL: expected five LED transitions ending in the ON state");
                    $finish(1);
                end
                $display("PASS: statistics, PID report, PAT, sync error, and LED verified");
                $finish(0);
            end
        end
    endtask

    always @(posedge fpga_clk) begin
        if (dut.frame_data_valid) begin
            case (parser_state)
                0: begin
                    received_type = dut.frame_data;
                    parser_state = 1;
                end
                1: begin
                    received_length = dut.frame_data;
                    payload_index = 0;
                    parser_state = 2;
                end
                2: begin
                    payload[payload_index] = dut.frame_data;
                    payload_index = payload_index + 1;
                    if (payload_index == received_length) begin
                        parser_state = 0;
                        verify_frame();
                    end
                end
            endcase
        end
    end

    always @(led_od) begin
        if (monitor_led)
            led_transition_count = led_transition_count + 1;
    end

    initial begin
        wait (dut.rst_n == 1'b1);
        #1 monitor_led = 1'b1;
        dut.u_telemetry.filter_enable_tuner = 4'b0001;
        dut.u_telemetry.filter_pid_0_tuner = 13'h0120;
        dut.u_telemetry.filter_mask_0_tuner = 13'h1ff0;
        send_packet(13'h0000, 8'h10);
        send_packet(13'h0123, 8'h20);
        send_packet(13'h0123, 8'h30);
        send_packet(13'h0123, 8'h40);
        send_packet(13'h0123, 8'h50);
        send_tuner_byte(8'h00);

        repeat (10) @(posedge fpga_clk);
        @(negedge fpga_clk);
        dut.u_telemetry.pid_interval_counter = 32'd39999999;
        dut.u_telemetry.stats_interval_counter = 32'd3999999;

        #2000000;
        $display("FAIL: timed out waiting for all telemetry frames");
        $finish(1);
    end
endmodule