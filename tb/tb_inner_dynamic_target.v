`timescale 1ns/1ps
module tb_inner_dynamic_target;
    reg clk=0; always #10 clk=~clk;
    reg button=1;
    reg [11:0] target_code=12'd646;
    reg [11:0] adc_code=12'd640;
    wire active,pwma,ain1,ain2,stby;
    pid_angle_ctrl #(
        .CENTER_CODE(646),.WINDOW_CODE(50),.KP_X100(100),
        .KI_X100(0),.KD_X100(0),.MOTOR_INVERT(0),
        .PID_PERIOD_CYCLES(20),.PWM_PERIOD(100),
        .MAX_PWM_TICKS(20),.DEBOUNCE_CYCLES(4)
    ) dut (
        .clk(clk),.button(button),.target_code(target_code),
        .adc_code(adc_code),.adc_valid(1'b1),.adc_otr(1'b0),
        .pid_active(active),.tb_pwma(pwma),.tb_ain1(ain1),
        .tb_ain2(ain2),.tb_stby(stby)
    );
    initial begin
        repeat (15) @(negedge clk);
        button=0; repeat (12) @(negedge clk);
        button=1; repeat (38) @(negedge clk);
        if (!active || !stby || !ain1 || ain2)
            $fatal(1,"inner did not drive toward target 646");
        target_code=12'd630; // ADC 不变，仅外环偏移改变目标
        repeat (55) @(negedge clk);
        if (!active || !stby || ain1 || !ain2)
            $fatal(1,"inner did not reverse after target offset");
        adc_code=12'd700; #1;
        if (stby || pwma) $fatal(1,"fixed center safety window failed");
        $display("PASS dynamic inner target and center-based safety window");
        $finish;
    end
endmodule
