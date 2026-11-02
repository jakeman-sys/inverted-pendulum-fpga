`timescale 1ns/1ps
// 未改动的用户基线作参考，逐周期比较自动授权版本的PID数值和输出。
module tb_pid_baseline_equivalence;
    reg clk=0;always #10 clk=~clk;
    reg button=0;reg [11:0] adc=1393,target=1403;reg valid=1,otr=0;
    wire old_active,new_active,op,o1,o2,os,np,n1,n2,ns;
    reference_pid_angle_ctrl #(.CENTER_CODE(1403),.WINDOW_CODE(350),.KP_X100(2000),.KI_X100(45),.KD_X100(3500),
        .MOTOR_INVERT(1),.PID_PERIOD_CYCLES(40),.PWM_PERIOD(100),.MAX_PWM_TICKS(60),.DEBOUNCE_CYCLES(4))
        original(.clk(clk),.button(button),.target_code(target),.adc_code(adc),.adc_valid(valid),.adc_otr(otr),
        .pid_active(old_active),.tb_pwma(op),.tb_ain1(o1),.tb_ain2(o2),.tb_stby(os));
    wire external_request=old_active || (original.button_press && original.release_seen);
    pid_angle_ctrl #(.CENTER_CODE(1403),.WINDOW_CODE(350),.KP_X100(2000),.KI_X100(45),.KD_X100(3500),
        .MOTOR_INVERT(1),.PID_PERIOD_CYCLES(40),.PWM_PERIOD(100),.MAX_PWM_TICKS(60),.DEBOUNCE_CYCLES(4),.EXTERNAL_CONTROL(1))
        updated(.clk(clk),.button(1'b0),.external_enable(external_request),.target_code(target),.adc_code(adc),
        .adc_valid(valid),.adc_otr(otr),.pid_active(new_active),.tb_pwma(np),.tb_ain1(n1),.tb_ain2(n2),.tb_stby(ns));
    always @(posedge clk)begin #1;
        if({old_active,op,o1,o2,os} !== {new_active,np,n1,n2,ns} ||
           original.integral!==updated.integral || original.command_q8!==updated.command_q8 || original.duty!==updated.duty)
            $fatal(1,"FAIL PID differs from untouched dual_mode_fix reference");
    end
    initial begin
        repeat(12)@(negedge clk);button=1;repeat(180)@(negedge clk);
        if(!old_active)$fatal(1,"FAIL reference entry");
        adc=1413;repeat(180)@(negedge clk);target=1423;repeat(180)@(negedge clk);
        adc=1402;repeat(180)@(negedge clk);adc=1404;repeat(180)@(negedge clk);
        adc=1800;repeat(12)@(negedge clk);adc=1403;repeat(100)@(negedge clk);
        if(new_active)$fatal(1,"FAIL disabled baseline auto restarted");
        $display("PASS exact PID math/output equivalence to untouched user baseline");$finish;
    end
endmodule
