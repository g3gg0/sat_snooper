/* LCMXO2-256HC-4SG32I
   Fade LED on PL5C (open-drain) ~1 Hz using on-chip OSCH           */

module blinky
(
	input clk,
    output led
);


/* -- fade logic: 8-bit PWM + 512-step ramp (~1 Hz) ----------------- */

reg [7:0]  pwm_counter   = 0;            /* free-running PWM counter   */
reg [7:0]  brightness    = 0;            /* duty-cycle 0�255           */
reg        fade_up       = 1'b1;         /* ramp direction             */
reg [12:0] step_timer    = 0;            /* counts 0�8191              */


always @(posedge clk) begin
    pwm_counter <= pwm_counter + 1'b1;

    if (step_timer == 0) begin
        if (fade_up) begin
            if (~brightness != 8'd0) begin
                brightness <= brightness + 1'b1;
            end else begin
                fade_up <= 1'b0;
            end
        end else begin
            if (brightness != 8'd0) begin
                brightness <= brightness - 1'b1;
            end else begin
                fade_up <= 1'b1;
            end
        end
    end
    step_timer <= step_timer + 1'b1;
end

/* -- open-drain drive (0 = sink, Z = off) ------------------------- */
assign led = (pwm_counter < brightness);

endmodule
