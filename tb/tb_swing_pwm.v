`timescale 1ns/1ps
module tb_swing_pwm;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,enable=0,forward=1;
    wire p,a,b,s,limited_pwm;
    swing_pulse_pwm dut(.clk(clk),.rst_n(rst),.enable(enable),.forward(forward),.pwma(p),.ain1(a),.ain2(b),.stby(s));
    swing_pulse_pwm #(.DUTY_PERCENT(99)) limiter(.clk(clk),.rst_n(rst),.enable(enable),.forward(forward),.pwma(limited_pwm),.ain1(),.ain2(),.stby());
    integer i,highs,limited_highs;
    initial begin
        repeat(3)@(negedge clk);rst=1;enable=1;highs=0;limited_highs=0;
        for(i=0;i<5000;i=i+1)begin #1;if(p)highs=highs+1;if(limited_pwm)limited_highs=limited_highs+1;@(negedge clk);end
        if(highs!=2250 || limited_highs!=3000)$fatal(1,"FAIL45%%/60%% pulse PWM");
        forward=0;#1;if(!s || a || !b)$fatal(1,"FAIL direct reversal must not stop drive");
        enable=0;#1;if(s || p || a || b)$fatal(1,"FAIL immediate pulse disable");
        $display("PASS45%% configured PWM,60%% hard ceiling and zero-delay reversal");$finish;
    end
endmodule
