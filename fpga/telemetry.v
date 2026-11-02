/* FPGA telemetry collection and framed-stream scheduler. */
module telemetry
(
    input tuner_clk,
    input fpga_clk,
    input rst_n,
    input [7:0] ts_data,
    input ts_data_valid,
    input ts_sync_pulse,
    input ts_sync_error_pulse,
    input [7:0] ts_byte_index,
    input ts_pid_valid,
    input [12:0] ts_pid,
    input [3:0] filter_enable,
    input [12:0] filter_pid_0,
    input [12:0] filter_mask_0,
    input [12:0] filter_pid_1,
    input [12:0] filter_mask_1,
    input [12:0] filter_pid_2,
    input [12:0] filter_mask_2,
    input [12:0] filter_pid_3,
    input [12:0] filter_mask_3,
    input filter_config_toggle,
    output reg [7:0] frame_data,
    output reg frame_data_valid,
    input frame_data_ready
);
    localparam [31:0] STATS_INTERVAL_CYCLES = 32'd4000000;
    localparam [31:0] PID_FILTERED_INTERVAL_CYCLES = 32'd40000000;
    localparam [31:0] PID_UNFILTERED_INTERVAL_CYCLES = 32'd4000000;
    localparam [7:0] FRAME_TYPE_STATISTICS = 8'd0;
    localparam [7:0] FRAME_TYPE_PID_REPORT = 8'd1;
    localparam [7:0] FRAME_TYPE_PAT = 8'd2;
    localparam [7:0] FRAME_TYPE_FILTERED_TS = 8'd3;
    localparam [7:0] STATISTICS_LENGTH = 8'd14;
    localparam [6:0] MAX_REPORTED_PIDS = 7'd127;
    localparam [9:0] FILTERED_PACKET_DATA_SIZE = 10'd185;

    reg [31:0] tuner_sync_count = 32'd0;
    reg [31:0] tuner_data_byte_count = 32'd0;
    reg [31:0] tuner_sync_error_count = 32'd0;
    reg [15:0] tuner_filtered_drop_count = 16'd0;
    wire [31:0] tuner_sync_count_gray = tuner_sync_count ^ (tuner_sync_count >> 1);
    wire [31:0] tuner_data_byte_count_gray = tuner_data_byte_count ^ (tuner_data_byte_count >> 1);
    wire [31:0] tuner_sync_error_count_gray = tuner_sync_error_count ^ (tuner_sync_error_count >> 1);
    wire [15:0] tuner_filtered_drop_count_gray = tuner_filtered_drop_count ^ (tuner_filtered_drop_count >> 1);

    reg [15:0] pid_memory [0:126];
    reg [6:0] pid_live_count = 7'd0;
    reg [6:0] pid_snapshot_count = 7'd0;
    reg pid_live_overflow = 1'b0;
    reg pid_snapshot_overflow = 1'b0;
    reg pid_collection_paused = 1'b0;
    reg [12:0] pid_candidate = 13'd0;
    reg [6:0] pid_scan_address = 7'd0;
    reg [15:0] pid_scan_data = 16'd0;
    reg pid_scan_active = 1'b0;
    reg pid_scan_wait = 1'b0;
    reg [15:0] pid_read_data;
    reg pid_report_request_meta = 1'b0;
    reg pid_report_request_tuner = 1'b0;
    reg pid_report_request_seen = 1'b0;
    reg pid_report_ack = 1'b0;
    reg pid_report_done = 1'b0;
    reg pid_report_done_meta = 1'b0;
    reg pid_report_done_tuner = 1'b0;

    reg [7:0] pat_data [3:187];
    reg [7:0] pat_candidate_header_1 = 8'd0;
    reg [7:0] pat_header_1 = 8'd0;
    reg [7:0] pat_header_2 = 8'd0;
    reg pat_capture_active = 1'b0;
    reg pat_event_toggle = 1'b0;
    reg pat_ack_meta = 1'b0;
    reg pat_ack_tuner = 1'b0;
    reg [7:0] pat_read_data;

    reg filter_config_toggle_meta = 1'b0;
    reg filter_config_toggle_tuner = 1'b0;
    reg filter_config_toggle_seen = 1'b0;
    reg [3:0] filter_enable_tuner = 4'd0;
    reg [12:0] filter_pid_0_tuner = 13'd0;
    reg [12:0] filter_mask_0_tuner = 13'd0;
    reg [12:0] filter_pid_1_tuner = 13'd0;
    reg [12:0] filter_mask_1_tuner = 13'd0;
    reg [12:0] filter_pid_2_tuner = 13'd0;
    reg [12:0] filter_mask_2_tuner = 13'd0;
    reg [12:0] filter_pid_3_tuner = 13'd0;
    reg [12:0] filter_mask_3_tuner = 13'd0;
    reg filter_config_toggle_fpga_meta = 1'b0;
    reg filter_config_toggle_fpga = 1'b0;
    reg filter_config_toggle_fpga_seen = 1'b0;
    reg [3:0] filter_enable_fpga = 4'd0;
    wire [31:0] pid_interval_cycles = filter_enable_fpga == 4'd0 ?
        PID_UNFILTERED_INTERVAL_CYCLES : PID_FILTERED_INTERVAL_CYCLES;
    wire filter_pid_match =
        (filter_enable_tuner[0] &&
         ((ts_pid & filter_mask_0_tuner) == (filter_pid_0_tuner & filter_mask_0_tuner))) ||
        (filter_enable_tuner[1] &&
         ((ts_pid & filter_mask_1_tuner) == (filter_pid_1_tuner & filter_mask_1_tuner))) ||
        (filter_enable_tuner[2] &&
         ((ts_pid & filter_mask_2_tuner) == (filter_pid_2_tuner & filter_mask_2_tuner))) ||
        (filter_enable_tuner[3] &&
         ((ts_pid & filter_mask_3_tuner) == (filter_pid_3_tuner & filter_mask_3_tuner)));
    reg [7:0] filtered_packet_data [0:739];
    reg [7:0] filtered_header_1 [0:3];
    reg [7:0] filtered_header_2 [0:3];
    reg filtered_capture_active = 1'b0;
    reg [2:0] filtered_write_pointer_bin = 3'd0;
    reg [2:0] filtered_write_pointer_gray = 3'd0;
    reg [2:0] filtered_read_pointer_gray_meta = 3'd0;
    reg [2:0] filtered_read_pointer_gray_tuner = 3'd0;
    wire [2:0] filtered_write_pointer_next_bin = filtered_write_pointer_bin + 1'b1;
    wire [2:0] filtered_write_pointer_next_gray =
        filtered_write_pointer_next_bin ^ (filtered_write_pointer_next_bin >> 1);
    wire filtered_fifo_full = filtered_write_pointer_gray ==
        {~filtered_read_pointer_gray_tuner[2:1], filtered_read_pointer_gray_tuner[0]};
    wire [9:0] filtered_write_base =
        filtered_write_pointer_bin[1:0] * FILTERED_PACKET_DATA_SIZE;
    reg [7:0] filtered_read_data;

    reg pid_report_request = 1'b0;
    reg pid_report_ack_meta = 1'b0;
    reg pid_report_ack_fpga = 1'b0;
    reg pid_report_ack_seen = 1'b0;
    reg pat_event_meta = 1'b0;
    reg pat_event_fpga = 1'b0;
    reg pat_event_seen = 1'b0;
    reg pat_ack = 1'b0;
    reg [2:0] filtered_write_pointer_gray_meta = 3'd0;
    reg [2:0] filtered_write_pointer_gray_fpga = 3'd0;
    reg [2:0] filtered_read_pointer_bin = 3'd0;
    reg [2:0] filtered_read_pointer_gray = 3'd0;
    wire [2:0] filtered_read_pointer_next_bin = filtered_read_pointer_bin + 1'b1;
    wire [2:0] filtered_read_pointer_next_gray =
        filtered_read_pointer_next_bin ^ (filtered_read_pointer_next_bin >> 1);
    wire filtered_fifo_empty =
        filtered_read_pointer_gray == filtered_write_pointer_gray_fpga;
    wire [9:0] filtered_read_base =
        filtered_read_pointer_bin[1:0] * FILTERED_PACKET_DATA_SIZE;
    reg [7:0] filtered_active_header_1 = 8'd0;
    reg [7:0] filtered_active_header_2 = 8'd0;

    reg [31:0] sync_count_gray_meta = 32'd0;
    reg [31:0] sync_count_gray_fpga = 32'd0;
    reg [31:0] data_byte_count_gray_meta = 32'd0;
    reg [31:0] data_byte_count_gray_fpga = 32'd0;
    reg [31:0] sync_error_count_gray_meta = 32'd0;
    reg [31:0] sync_error_count_gray_fpga = 32'd0;
    reg [15:0] filtered_drop_count_gray_meta = 16'd0;
    reg [15:0] filtered_drop_count_gray_fpga = 16'd0;

    reg [31:0] stats_interval_counter = 32'd0;
    reg [31:0] pid_interval_counter = 32'd0;
    reg stats_pending = 1'b0;
    reg pid_report_pending = 1'b0;
    reg pat_pending = 1'b0;
    reg [31:0] stats_sync_snapshot = 32'd0;
    reg [31:0] stats_data_byte_snapshot = 32'd0;
    reg [31:0] stats_sync_error_snapshot = 32'd0;
    reg [15:0] stats_filtered_drop_snapshot = 16'd0;
    reg [6:0] report_pid_count = 7'd0;
    reg report_pid_overflow = 1'b0;
    reg [7:0] active_frame_type = 8'd0;
    reg [7:0] active_payload_length = 8'd0;
    reg [8:0] frame_byte_index = 9'd0;
    reg frame_active = 1'b0;
    reg [7:0] frame_byte_value;
    wire [8:0] pid_read_offset = frame_byte_index >= 9'd3 ?
                                 (frame_byte_index - 9'd3) >> 1 : 9'd0;
    wire [6:0] pid_read_address = pid_read_offset[6:0];
    wire [7:0] pat_read_address = frame_byte_index > 9'd5 ?
                                  frame_byte_index - 9'd2 : 8'd3;
    wire [9:0] filtered_read_address = filtered_read_base +
        (frame_byte_index > 9'd5 ? frame_byte_index - 9'd5 : 9'd0);

    function [31:0] gray_to_binary;
        input [31:0] gray;
        integer bit_index;
        begin
            gray_to_binary[31] = gray[31];
            for (bit_index = 30; bit_index >= 0; bit_index = bit_index - 1)
                gray_to_binary[bit_index] = gray_to_binary[bit_index + 1] ^ gray[bit_index];
        end
    endfunction

    always @(posedge tuner_clk or negedge rst_n) begin
        if (!rst_n) begin
            tuner_sync_count <= 32'd0;
            tuner_data_byte_count <= 32'd0;
            tuner_sync_error_count <= 32'd0;
            tuner_filtered_drop_count <= 16'd0;
            pid_live_count <= 7'd0;
            pid_snapshot_count <= 7'd0;
            pid_live_overflow <= 1'b0;
            pid_snapshot_overflow <= 1'b0;
            pid_collection_paused <= 1'b0;
            pid_candidate <= 13'd0;
            pid_scan_address <= 7'd0;
            pid_scan_data <= 16'd0;
            pid_scan_active <= 1'b0;
            pid_scan_wait <= 1'b0;
            pid_report_request_meta <= 1'b0;
            pid_report_request_tuner <= 1'b0;
            pid_report_request_seen <= 1'b0;
            pid_report_ack <= 1'b0;
            pid_report_done_meta <= 1'b0;
            pid_report_done_tuner <= 1'b0;
            pat_candidate_header_1 <= 8'd0;
            pat_header_1 <= 8'd0;
            pat_header_2 <= 8'd0;
            pat_capture_active <= 1'b0;
            pat_event_toggle <= 1'b0;
            pat_ack_meta <= 1'b0;
            pat_ack_tuner <= 1'b0;
            filter_config_toggle_meta <= 1'b0;
            filter_config_toggle_tuner <= 1'b0;
            filter_config_toggle_seen <= 1'b0;
            filter_enable_tuner <= 4'd0;
            filter_pid_0_tuner <= 13'd0;
            filter_mask_0_tuner <= 13'd0;
            filter_pid_1_tuner <= 13'd0;
            filter_mask_1_tuner <= 13'd0;
            filter_pid_2_tuner <= 13'd0;
            filter_mask_2_tuner <= 13'd0;
            filter_pid_3_tuner <= 13'd0;
            filter_mask_3_tuner <= 13'd0;
            filtered_capture_active <= 1'b0;
            filtered_write_pointer_bin <= 3'd0;
            filtered_write_pointer_gray <= 3'd0;
            filtered_read_pointer_gray_meta <= 3'd0;
            filtered_read_pointer_gray_tuner <= 3'd0;
        end else begin
            pid_report_request_meta <= pid_report_request;
            pid_report_request_tuner <= pid_report_request_meta;
            pid_report_done_meta <= pid_report_done;
            pid_report_done_tuner <= pid_report_done_meta;
            pat_ack_meta <= pat_ack;
            pat_ack_tuner <= pat_ack_meta;
            filtered_read_pointer_gray_meta <= filtered_read_pointer_gray;
            filtered_read_pointer_gray_tuner <= filtered_read_pointer_gray_meta;
            filter_config_toggle_meta <= filter_config_toggle;
            filter_config_toggle_tuner <= filter_config_toggle_meta;

            if (filter_config_toggle_tuner != filter_config_toggle_seen) begin
                filter_config_toggle_seen <= filter_config_toggle_tuner;
                filter_enable_tuner <= filter_enable;
                filter_pid_0_tuner <= filter_pid_0;
                filter_mask_0_tuner <= filter_mask_0;
                filter_pid_1_tuner <= filter_pid_1;
                filter_mask_1_tuner <= filter_mask_1;
                filter_pid_2_tuner <= filter_pid_2;
                filter_mask_2_tuner <= filter_mask_2;
                filter_pid_3_tuner <= filter_pid_3;
                filter_mask_3_tuner <= filter_mask_3;
            end

            if (ts_data_valid)
                tuner_data_byte_count <= tuner_data_byte_count + 1'b1;
            if (ts_sync_pulse)
                tuner_sync_count <= tuner_sync_count + 1'b1;
            if (ts_sync_error_pulse)
                tuner_sync_error_count <= tuner_sync_error_count + 1'b1;

            if (pid_collection_paused &&
                pid_report_done_tuner == pid_report_request_seen) begin
                pid_collection_paused <= 1'b0;
                pid_live_count <= 7'd0;
                pid_live_overflow <= 1'b0;
            end else if (!pid_collection_paused && !pid_scan_active &&
                         pid_report_request_tuner != pid_report_request_seen) begin
                pid_snapshot_count <= pid_live_count;
                pid_snapshot_overflow <= pid_live_overflow;
                pid_collection_paused <= 1'b1;
                pid_report_request_seen <= pid_report_request_tuner;
                pid_report_ack <= pid_report_request_tuner;
            end else if (!pid_collection_paused) begin
                if (!pid_scan_active && ts_pid_valid) begin
                    pid_candidate <= ts_pid;
                    pid_scan_address <= 5'd0;
                    pid_scan_active <= 1'b1;
                    pid_scan_wait <= 1'b1;
                end else if (pid_scan_active) begin
                    pid_scan_data <= pid_memory[pid_scan_address];
                    if (pid_scan_wait) begin
                        pid_scan_wait <= 1'b0;
                    end else if (pid_scan_data[12:0] == pid_candidate) begin
                        pid_scan_active <= 1'b0;
                    end else if (pid_scan_address + 1'b1 >= pid_live_count) begin
                        if (pid_live_count < MAX_REPORTED_PIDS) begin
                            pid_memory[pid_live_count] <= {3'd0, pid_candidate};
                            pid_live_count <= pid_live_count + 1'b1;
                        end else begin
                            pid_live_overflow <= 1'b1;
                        end
                        pid_scan_active <= 1'b0;
                    end else begin
                        pid_scan_address <= pid_scan_address + 1'b1;
                        pid_scan_wait <= 1'b1;
                    end
                end
            end

            if (ts_data_valid) begin
                if (ts_sync_pulse) begin
                    pat_capture_active <= 1'b0;
                    filtered_capture_active <= 1'b0;
                end
                if (ts_pid_valid) begin
                    if (ts_pid != 13'd0 && filter_pid_match) begin
                        if (filtered_fifo_full && tuner_filtered_drop_count != 16'hffff)
                            tuner_filtered_drop_count <= tuner_filtered_drop_count + 1'b1;
                    end
                    if (ts_pid == 13'd0 && pat_event_toggle == pat_ack_tuner) begin
                        pat_header_1 <= pat_candidate_header_1;
                        pat_header_2 <= ts_data;
                    end
                    pat_capture_active <= ts_pid == 13'd0 && pat_event_toggle == pat_ack_tuner;
                    if (ts_pid != 13'd0 && filter_pid_match && !filtered_fifo_full) begin
                        filtered_header_1[filtered_write_pointer_bin[1:0]] <=
                            pat_candidate_header_1;
                        filtered_header_2[filtered_write_pointer_bin[1:0]] <= ts_data;
                        filtered_capture_active <= 1'b1;
                    end
                end else if (ts_byte_index == 8'd1) begin
                    pat_candidate_header_1 <= ts_data;
                end else if (pat_capture_active && ts_byte_index >= 8'd3) begin
                    pat_data[ts_byte_index] <= ts_data;
                    if (ts_byte_index == 8'd187) begin
                        pat_capture_active <= 1'b0;
                        pat_event_toggle <= ~pat_event_toggle;
                    end
                end
                if (filtered_capture_active && ts_byte_index >= 8'd3) begin
                    filtered_packet_data[filtered_write_base + ts_byte_index - 8'd3] <= ts_data;
                    if (ts_byte_index == 8'd187) begin
                        filtered_capture_active <= 1'b0;
                        filtered_write_pointer_bin <= filtered_write_pointer_next_bin;
                        filtered_write_pointer_gray <= filtered_write_pointer_next_gray;
                    end
                end
            end
        end
    end

    always @(posedge fpga_clk) begin
        pat_read_data <= pat_data[pat_read_address];
        filtered_read_data <= filtered_packet_data[filtered_read_address];
        pid_read_data <= pid_memory[pid_read_address];
    end

    always @(*) begin
        if (frame_byte_index == 9'd0)
            frame_byte_value = active_frame_type;
        else if (frame_byte_index == 9'd1)
            frame_byte_value = active_payload_length;
        else if (active_frame_type == FRAME_TYPE_STATISTICS) begin
            case (frame_byte_index)
                8'd2: frame_byte_value = stats_sync_snapshot[7:0];
                8'd3: frame_byte_value = stats_sync_snapshot[15:8];
                8'd4: frame_byte_value = stats_sync_snapshot[23:16];
                8'd5: frame_byte_value = stats_sync_snapshot[31:24];
                8'd6: frame_byte_value = stats_data_byte_snapshot[7:0];
                8'd7: frame_byte_value = stats_data_byte_snapshot[15:8];
                8'd8: frame_byte_value = stats_data_byte_snapshot[23:16];
                8'd9: frame_byte_value = stats_data_byte_snapshot[31:24];
                8'd10: frame_byte_value = stats_sync_error_snapshot[7:0];
                8'd11: frame_byte_value = stats_sync_error_snapshot[15:8];
                8'd12: frame_byte_value = stats_sync_error_snapshot[23:16];
                8'd13: frame_byte_value = stats_sync_error_snapshot[31:24];
                8'd14: frame_byte_value = stats_filtered_drop_snapshot[7:0];
                8'd15: frame_byte_value = stats_filtered_drop_snapshot[15:8];
                default: frame_byte_value = 8'd0;
            endcase
        end else if (active_frame_type == FRAME_TYPE_PID_REPORT) begin
            if (frame_byte_index == 8'd2)
                frame_byte_value = {7'd0, report_pid_overflow};
            else if (frame_byte_index[0])
                frame_byte_value = pid_read_data[7:0];
            else
                frame_byte_value = {3'd0, pid_read_data[12:8]};
        end else if (active_frame_type == FRAME_TYPE_PAT) begin
            case (frame_byte_index)
                8'd2: frame_byte_value = 8'h47;
                8'd3: frame_byte_value = pat_header_1;
                8'd4: frame_byte_value = pat_header_2;
                default: frame_byte_value = pat_read_data;
            endcase
        end else if (active_frame_type == FRAME_TYPE_FILTERED_TS) begin
            case (frame_byte_index)
                8'd2: frame_byte_value = 8'h47;
                8'd3: frame_byte_value = filtered_active_header_1;
                8'd4: frame_byte_value = filtered_active_header_2;
                default: frame_byte_value = filtered_read_data;
            endcase
        end else begin
            frame_byte_value = 8'd0;
        end
    end

    always @(posedge fpga_clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_count_gray_meta <= 32'd0;
            sync_count_gray_fpga <= 32'd0;
            data_byte_count_gray_meta <= 32'd0;
            data_byte_count_gray_fpga <= 32'd0;
            sync_error_count_gray_meta <= 32'd0;
            sync_error_count_gray_fpga <= 32'd0;
            filtered_drop_count_gray_meta <= 16'd0;
            filtered_drop_count_gray_fpga <= 16'd0;
            pid_report_ack_meta <= 1'b0;
            pid_report_ack_fpga <= 1'b0;
            pat_event_meta <= 1'b0;
            pat_event_fpga <= 1'b0;
            filter_config_toggle_fpga_meta <= 1'b0;
            filter_config_toggle_fpga <= 1'b0;
            filter_config_toggle_fpga_seen <= 1'b0;
            filter_enable_fpga <= 4'd0;
            filtered_write_pointer_gray_meta <= 3'd0;
            filtered_write_pointer_gray_fpga <= 3'd0;
        end else begin
            sync_count_gray_meta <= tuner_sync_count_gray;
            sync_count_gray_fpga <= sync_count_gray_meta;
            data_byte_count_gray_meta <= tuner_data_byte_count_gray;
            data_byte_count_gray_fpga <= data_byte_count_gray_meta;
            sync_error_count_gray_meta <= tuner_sync_error_count_gray;
            sync_error_count_gray_fpga <= sync_error_count_gray_meta;
            filtered_drop_count_gray_meta <= tuner_filtered_drop_count_gray;
            filtered_drop_count_gray_fpga <= filtered_drop_count_gray_meta;
            pid_report_ack_meta <= pid_report_ack;
            pid_report_ack_fpga <= pid_report_ack_meta;
            pat_event_meta <= pat_event_toggle;
            pat_event_fpga <= pat_event_meta;
            filter_config_toggle_fpga_meta <= filter_config_toggle;
            filter_config_toggle_fpga <= filter_config_toggle_fpga_meta;
            if (filter_config_toggle_fpga != filter_config_toggle_fpga_seen) begin
                filter_config_toggle_fpga_seen <= filter_config_toggle_fpga;
                filter_enable_fpga <= filter_enable;
            end
            filtered_write_pointer_gray_meta <= filtered_write_pointer_gray;
            filtered_write_pointer_gray_fpga <= filtered_write_pointer_gray_meta;
        end
    end

    always @(posedge fpga_clk or negedge rst_n) begin
        if (!rst_n) begin
            stats_interval_counter <= 32'd0;
            pid_interval_counter <= 32'd0;
            stats_pending <= 1'b0;
            pid_report_pending <= 1'b0;
            pat_pending <= 1'b0;
            pid_report_request <= 1'b0;
            pid_report_ack_seen <= 1'b0;
            pid_report_done <= 1'b0;
            pat_event_seen <= 1'b0;
            pat_ack <= 1'b0;
            filtered_read_pointer_bin <= 3'd0;
            filtered_read_pointer_gray <= 3'd0;
            filtered_active_header_1 <= 8'd0;
            filtered_active_header_2 <= 8'd0;
            frame_data <= 8'd0;
            frame_data_valid <= 1'b0;
            frame_byte_index <= 8'd0;
            frame_active <= 1'b0;
        end else begin
            frame_data_valid <= 1'b0;

            if (stats_interval_counter == STATS_INTERVAL_CYCLES - 1'b1) begin
                stats_interval_counter <= 32'd0;
                stats_pending <= 1'b1;
            end else begin
                stats_interval_counter <= stats_interval_counter + 1'b1;
            end
            if (pid_interval_counter >= pid_interval_cycles - 1'b1) begin
                pid_interval_counter <= 32'd0;
                pid_report_request <= ~pid_report_request;
            end else begin
                pid_interval_counter <= pid_interval_counter + 1'b1;
            end
            if (pid_report_ack_fpga != pid_report_ack_seen) begin
                pid_report_ack_seen <= pid_report_ack_fpga;
                report_pid_count <= pid_snapshot_count;
                report_pid_overflow <= pid_snapshot_overflow;
                pid_report_pending <= 1'b1;
            end
            if (pat_event_fpga != pat_event_seen) begin
                pat_event_seen <= pat_event_fpga;
                pat_pending <= 1'b1;
            end
            if (!frame_active) begin
                if (pat_pending) begin
                    active_frame_type <= FRAME_TYPE_PAT;
                    active_payload_length <= 8'd188;
                    frame_byte_index <= 8'd0;
                    frame_active <= 1'b1;
                    pat_pending <= 1'b0;
                end else if (!filtered_fifo_empty) begin
                    active_frame_type <= FRAME_TYPE_FILTERED_TS;
                    active_payload_length <= 8'd188;
                    frame_byte_index <= 8'd0;
                    frame_active <= 1'b1;
                    filtered_active_header_1 <=
                        filtered_header_1[filtered_read_pointer_bin[1:0]];
                    filtered_active_header_2 <=
                        filtered_header_2[filtered_read_pointer_bin[1:0]];
                end else if (pid_report_pending) begin
                    active_frame_type <= FRAME_TYPE_PID_REPORT;
                    active_payload_length <= 8'd1 + {report_pid_count, 1'b0};
                    frame_byte_index <= 8'd0;
                    frame_active <= 1'b1;
                    pid_report_pending <= 1'b0;
                end else if (stats_pending) begin
                    stats_sync_snapshot <= gray_to_binary(sync_count_gray_fpga);
                    stats_data_byte_snapshot <= gray_to_binary(data_byte_count_gray_fpga);
                    stats_sync_error_snapshot <= gray_to_binary(sync_error_count_gray_fpga);
                    stats_filtered_drop_snapshot <= filtered_drop_count_gray_fpga ^
                        (filtered_drop_count_gray_fpga >> 1) ^
                        (filtered_drop_count_gray_fpga >> 2) ^
                        (filtered_drop_count_gray_fpga >> 3) ^
                        (filtered_drop_count_gray_fpga >> 4) ^
                        (filtered_drop_count_gray_fpga >> 5) ^
                        (filtered_drop_count_gray_fpga >> 6) ^
                        (filtered_drop_count_gray_fpga >> 7) ^
                        (filtered_drop_count_gray_fpga >> 8) ^
                        (filtered_drop_count_gray_fpga >> 9) ^
                        (filtered_drop_count_gray_fpga >> 10) ^
                        (filtered_drop_count_gray_fpga >> 11) ^
                        (filtered_drop_count_gray_fpga >> 12) ^
                        (filtered_drop_count_gray_fpga >> 13) ^
                        (filtered_drop_count_gray_fpga >> 14) ^
                        (filtered_drop_count_gray_fpga >> 15);
                    active_frame_type <= FRAME_TYPE_STATISTICS;
                    active_payload_length <= STATISTICS_LENGTH;
                    frame_byte_index <= 8'd0;
                    frame_active <= 1'b1;
                    stats_pending <= 1'b0;
                end
            end else if (frame_data_ready) begin
                frame_data <= frame_byte_value;
                frame_data_valid <= 1'b1;
                if (frame_byte_index == {1'b0, active_payload_length} + 1'b1) begin
                    frame_active <= 1'b0;
                    if (active_frame_type == FRAME_TYPE_PAT)
                        pat_ack <= pat_event_seen;
                    else if (active_frame_type == FRAME_TYPE_FILTERED_TS) begin
                        filtered_read_pointer_bin <= filtered_read_pointer_next_bin;
                        filtered_read_pointer_gray <= filtered_read_pointer_next_gray;
                    end
                    else if (active_frame_type == FRAME_TYPE_PID_REPORT)
                        pid_report_done <= pid_report_ack_seen;
                end else begin
                    frame_byte_index <= frame_byte_index + 1'b1;
                end
            end
        end
    end
endmodule