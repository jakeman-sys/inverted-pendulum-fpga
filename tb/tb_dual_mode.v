`timescale 1ns/1ps
module tb_dual_mode;
    reg clk=0; always #10 clk=~clk;
    reg active=0, started=0, s1=0, s2=0, f2=0, f1=0, f1rep=0;
    reg e3=0, e3rep=0;
    reg signed [31:0] actual_pos=32'sd123;
    wire [1:0] page;
    wire speed_mode, zero_position;
    wire signed [31:0] target_pos, edit_pos;
    wire signed [15:0] target_speed, edit_speed;
    dual_mode_ui #(.SPEED_LIMIT_DPS(20)) ui (
        .clk(clk), .active(active), .started(started),
        .s1_press(s1), .s2_press(s2), .f2_press(f2),
        .f1_press(f1), .f1_repeat(f1rep),
        .e3_press(e3), .e3_repeat(e3rep),
        .actual_position_deci(actual_pos),
        .page(page), .speed_mode(speed_mode), .zero_position(zero_position),
        .target_position_deci(target_pos), .edit_position_deci(edit_pos),
        .target_speed_dps(target_speed), .edit_speed_dps(edit_speed)
    );
    task pulse_s1; begin s1=1; @(negedge clk); s1=0; end endtask
    task pulse_s2; begin s2=1; @(negedge clk); s2=0; end endtask
    task pulse_f2; begin f2=1; @(negedge clk); f2=0; end endtask
    task pulse_f1; begin f1=1; @(negedge clk); f1=0; end endtask
    task pulse_e3; begin e3=1; @(negedge clk); e3=0; end endtask

    reg signed [47:0] raw_position=48'sd0;
    wire signed [31:0] measured_speed;
    encoder_speed_estimator #(.PERIOD_CYCLES(10),.FILTER_SHIFT(1)) speed_measure (
        .clk(clk),.zero_position(1'b0),.position_q16(raw_position),
        .speed_deci_dps(measured_speed)
    );
    reg speed_active=0;
    reg signed [15:0] pid_target=0;
    reg signed [31:0] pid_actual=0;
    wire signed [15:0] speed_offset;
    speed_pid_outer #(.KP_X100(100),.KI_X100(0),.KD_X100(100),
        .MAX_OFFSET_ADC(20),.PERIOD_CYCLES(12)) speed_pid (
        .clk(clk),.active(speed_active),.target_speed_dps(pid_target),
        .actual_speed_deci_dps(pid_actual),.offset_adc(speed_offset)
    );

    initial begin
        repeat (3) @(negedge clk);
        if (page !== 0 || speed_mode !== 0 || target_speed !== 0)
            $fatal(1,"startup mode is not position/zero speed");
        active=1; started=1; @(negedge clk); started=0;
        pulse_s2();
        if (page !== 2) $fatal(1,"S2 did not open speed page");
        pulse_e3(); pulse_e3();
        if (edit_speed !== 10 || target_speed !== 0 || speed_mode !== 0)
            $fatal(1,"speed draft changed running mode prematurely");
        pulse_f2();
        if (target_speed !== 10 || speed_mode !== 1)
            $fatal(1,"F2 did not enter speed mode");
        pulse_f1(); pulse_f1();
        if (edit_speed !== 0 || speed_mode !== 1)
            $fatal(1,"speed zero draft changed mode before apply");
        actual_pos=32'sd177;
        pulse_f2();
        if (target_speed !== 0 || speed_mode !== 0 || target_pos !== 177)
            $fatal(1,"speed zero did not hold current position");
        pulse_s1();
        if (page !== 1 || edit_pos !== 177)
            $fatal(1,"S1 did not open position page at held target");
        pulse_e3(); pulse_f2();
        if (target_pos !== 227 || speed_mode !== 0)
            $fatal(1,"position edit/apply failed");
        pulse_s1(); pulse_f1();
        if (!zero_position || target_pos !== 0)
            $fatal(1,"home F1 did not zero output-shaft position");
        @(negedge clk);
        if (zero_position) $fatal(1,"zero pulse was not one cycle");
        active=0; @(negedge clk);
        if (page !== 0 || speed_mode !== 0 || target_speed !== 0)
            $fatal(1,"stop did not restore default mode");
        $display("PASS page controls, speed/position switching and F1 zero");

        // 速度估计只读未置零的原始位置。位置增加约 1 度后应得到正速度。
        raw_position=48'sd655360;
        repeat (11) @(negedge clk);
        if (measured_speed <= 0) $fatal(1,"encoder speed sign positive failed");
        raw_position=48'sd0;
        repeat (10) @(negedge clk);
        if (measured_speed >= 0) $fatal(1,"encoder speed sign negative failed");
        $display("PASS signed encoder speed estimator");

        pid_target=16'sd10; speed_active=1;
        repeat (13) @(negedge clk);
        if (!speed_pid.skip_derivative)
            $fatal(1,"first speed PID sample should suppress derivative");
        repeat (4) @(negedge clk);
        if (speed_offset !== 10)
            $fatal(1,"speed PID positive P output incorrect: %0d",speed_offset);
        pid_target=16'sd20;
        repeat (12) @(negedge clk);
        if (!speed_pid.skip_derivative)
            $fatal(1,"target change did not set C=0");
        repeat (12) @(negedge clk);
        if (speed_pid.skip_derivative)
            $fatal(1,"C did not return to 1 at next PID sample");
        pid_target=-16'sd30;
        repeat (16) @(negedge clk);
        if (speed_offset !== -20)
            $fatal(1,"speed PID negative clamp incorrect: %0d",speed_offset);
        $display("PASS speed PID signs, ADC clamp and derivative suppression");
        $finish;
    end
endmodule
