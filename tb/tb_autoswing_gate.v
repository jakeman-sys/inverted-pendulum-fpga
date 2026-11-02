`timescale 1ns/1ps
`include "gate_defaults.vh"
// 真正的GW5A综合网表：只加速长计时器，检查实际PWM及原PID符号。
module tb_autoswing_gate;
    reg clk=0;always #10 clk=~clk;
    GSR GSR(.GSRI(1'b1));
    wire pwm,a1,a2,stby;
    inverted_pendulum_autoswing_top dut(.clk(clk),.button(1'b0),.button_dec(1'b0),.button_inc(1'b0),
        .button_page(1'b0),.button_speed_page(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),.adc_data(12'd1403),.adc_otr(1'b0),
        .adc_clk(),.tft_cs_n(),.tft_sclk(),.tft_mosi(),.tft_dc(),.tft_rst_n(),.tft_bl(),.tb_pwma(pwm),.tb_ain1(a1),.tb_ain2(a2),.tb_stby(stby));
    reg [11:0] adc=3;reg good=0,pid_good=0,otr=1,low=1;
    reg signed [47:0] raw=0;
    integer i,highs;
    localparam integer CONFIG_DUTY=`TEST_SWING_DUTY_PERCENT>60 ? 60 : `TEST_SWING_DUTY_PERCENT;
    reg [31:0] held_position;
    reg signed [47:0] held_raw;
    // 上限25码内检查符号/低位，并以实际PID目标码验证控制效果。
    function [5:0] used_outer;
        input [15:0] value;
        begin used_outer={value[15],value[4:0]};end
    endfunction
    task frame;input [11:0] code;input ok;begin
        @(negedge clk);adc=code;good=ok;pid_good=ok;force dut.frame_tick=1;@(negedge clk);force dut.frame_tick=0;repeat(8)@(negedge clk);end endtask
    task f2;begin @(negedge clk);force dut.f2_press=1;@(negedge clk);force dut.f2_press=0;repeat(4)@(negedge clk);end endtask
    task end_leg;begin
        @(negedge clk);force dut.u_swing.pulse_count=(`TEST_SWING_PULSE_CYCLES-1);@(negedge clk);force dut.u_swing.pulse_count=0;
        @(negedge clk);release dut.u_swing.pulse_count;repeat(3)@(negedge clk);end endtask
    task end_gap;begin
        if(dut.control_state!=2 || stby || pwm || a1 || a2)$fatal(1,"FAIL gate reverse coast must disable all outputs");
        repeat(100)@(negedge clk);if(stby || pwm)$fatal(1,"FAIL gate short reverse coast");
        force dut.u_swing.gap_count=(`TEST_SWING_GAP_CYCLES-1);@(negedge clk);force dut.u_swing.gap_count=0;
        @(negedge clk);release dut.u_swing.gap_count;repeat(3)@(negedge clk);
        if(dut.control_state!=3 || !stby)$fatal(1,"FAIL gate10ms gap boundary");end endtask
    task pid_update;begin
        repeat(12)@(negedge clk);force dut.u_angle_pid.pid_count=249999;
        @(negedge clk);force dut.u_angle_pid.pid_count=0;@(negedge clk);release dut.u_angle_pid.pid_count;
        repeat(12)@(negedge clk);end endtask
    task duty_check;begin
        highs=0;for(i=0;i<5000;i=i+1)begin if(pwm)highs=highs+1;@(negedge clk);end
        if(highs!=50*CONFIG_DUTY)$fatal(1,"FAIL configured gate PWM high=%d expected=%d",highs,50*CONFIG_DUTY);end endtask
    initial begin
        force dut.startup=0;force dut.frame_tick=0;force dut.adc_valid=1;force dut.adc_average=adc;
        force dut.adc_overrange=otr;force dut.low_otr_only=low;
        force dut.u_adc.average_good=good;force dut.u_adc.raw_bad=!good;
        force dut.u_adc.pid_average_good=pid_good;
        force dut.raw_position_q16=raw;force dut.f1_press=0;force dut.f2_press=0;
        #100;force dut.startup=50000;repeat(5)@(negedge clk);f2();
        if(!stby || !a1 || a2)$fatal(1,"FAIL gate initial escape");duty_check();end_leg();
        end_gap();if(a1 || !a2)$fatal(1,"FAIL configured pulse/coast reversal");
        duty_check();end_leg();if(dut.control_state!=4 || stby || pwm)$fatal(1,"FAIL gate peak wait");
        otr=0;low=0;frame(0,0);frame(2700,0);frame(0,0);
        if(dut.control_state!=4)$fatal(1,"FAIL gate deadzone wrap triggered");
        frame(500,1);frame(600,1);frame(550,1);
        if(dut.control_state!=1 || !a1 || a2)$fatal(1,"FAIL gate left maximum");end_leg();end_gap();end_leg();
        frame(3,0);frame(2300,1);frame(2100,1);frame(2150,1);
        if(dut.control_state!=1 || a1 || !a2)$fatal(1,"FAIL gate right minimum");
        frame(1403,0);frame(1403,0);frame(1403,0);if(dut.control_state==6)$fatal(1,"FAIL gate invalid capture");
        frame(1302,1);frame(1302,1);frame(1302,1);
        if(dut.control_state==6)$fatal(1,"FAIL gate100 capture lower boundary");
        frame(1504,1);frame(1504,1);frame(1504,1);
        if(dut.control_state==6)$fatal(1,"FAIL gate100 capture upper boundary");
        frame(1303,1);frame(1503,1);if(dut.control_state==6)$fatal(1,"FAIL gate early capture");frame(1403,1);
        if(dut.control_state!=6 || !dut.u_angle_pid.pid_active_Z)$fatal(1,"FAIL gate three-frame PID capture");
        adc=1393;pid_update();pid_update();if(!stby || a1 || !a2)$fatal(1,"FAIL gate original PID below target");
        adc=1413;pid_update();pid_update();if(!stby || !a1 || a2)$fatal(1,"FAIL gate original PID above target");
        good=0;pid_good=0;adc=1403;repeat(6400)@(negedge clk);
        if(dut.control_state!=6 || !dut.u_angle_pid.pid_active_Z || !stby)
            $fatal(1,"FAIL gate brief ADC quality loss killed PID");
        good=1;pid_good=1;repeat(8)@(negedge clk);
        good=0;repeat(100010)@(negedge clk);
        if(dut.control_state!=6 || !dut.u_angle_pid.pid_active_Z)$fatal(1,"FAIL gate strict swing validity leaked into PID");
        good=1;
        adc=1754;#1;if(stby || pwm)$fatal(1,"FAIL gate immediate fall stop");repeat(5)@(negedge clk);
        if(dut.control_state!=10 || dut.fault_code!=4 || dut.fault_detail!=1 || dut.fault_adc!=1754)
            $fatal(1,"FAIL gate fall fault snapshot");
        frame(1403,1);frame(1403,1);frame(1403,1);
        if(dut.control_state!=10 || stby || pwm)$fatal(1,"FAIL gate immediate fall cannot bypass stop check");
        force dut.actual_speed_deci_dps=3000;adc=1754;
        force dut.u_swing.recover_stop_count=499998;@(negedge clk);
        if(dut.control_state!=10 || stby || pwm)$fatal(1,"FAIL gate short10ms fall stop");
        force dut.u_swing.recover_stop_count=499999;@(negedge clk);release dut.u_swing.recover_stop_count;
        repeat(12)@(negedge clk);
        if(dut.control_state!=10 || dut.u_swing.recover_ready_count!=0 || stby || pwm)
            $fatal(1,"FAIL gate fast arm must stop before new swing");
        force dut.actual_speed_deci_dps=-21;repeat(8)@(negedge clk);
        if(dut.control_state!=10 || dut.u_swing.recover_ready_count!=0 || stby)
            $fatal(1,"FAIL gate negative2.1dps must wait");
        force dut.actual_speed_deci_dps=0;
        force dut.u_swing.recover_ready_count=4999998;@(negedge clk);
        if(dut.control_state!=10 || stby || pwm)$fatal(1,"FAIL gate short100ms stopped qualification");
        force dut.u_swing.recover_ready_count=4999999;@(negedge clk);release dut.u_swing.recover_ready_count;
        repeat(8)@(negedge clk);
        if(dut.control_state!=1 || dut.recovery_attempts_Z!==4'd1 || !stby)$fatal(1,"FAIL gate stopped arm restart without down wait");
        duty_check();otr=0;low=0;frame(1403,1);frame(1403,1);frame(1403,1);
        if(dut.control_state!=6 || dut.speed_mode_Z || dut.outer_ready_Z || dut.recovery_attempts_Z!==4'd1)
            $fatal(1,"FAIL gate recovered PID position mode/angle-only");
        adc=1754;repeat(6)@(negedge clk);
        if(dut.control_state!=10)$fatal(1,"FAIL gate second fall restart task");
        force dut.u_swing.recover_wait_count=249999999;@(negedge clk);release dut.u_swing.recover_wait_count;
        if(dut.control_state!=7 || dut.fault_code!=5 || dut.recovery_fault_reason_Z!==2'd2 || stby || pwm)
            $fatal(1,"FAIL gate5s stop wait timeout must latch F5 T");
        release dut.actual_speed_deci_dps;adc=1403;good=1;pid_good=1;repeat(8)@(negedge clk);
        f2();frame(1403,1);frame(1403,1);frame(1403,1);good=0;pid_good=0;repeat(100010)@(negedge clk);
        if(dut.control_state!=7 || dut.fault_code!=4 || dut.fault_detail!=2 || stby || pwm)
            $fatal(1,"FAIL gate persistent sample loss F4 A");
        good=1;pid_good=1;repeat(8)@(negedge clk);
        f2();frame(1403,1);frame(1403,1);frame(1403,1);otr=1;low=1;good=0;repeat(5)@(negedge clk);
        if(dut.control_state!=7 || dut.fault_code!=1 || stby || pwm)$fatal(1,"FAIL gate PID low OTR stop");
        force dut.f1_press=1;#1;if(stby || pwm)$fatal(1,"FAIL gate F1 disable");
        @(negedge clk);force dut.f1_press=0;frame(3,0);frame(3,0);
        // 最终综合网表也验证正负限位恢复，不能只在RTL中通过。
        otr=1;low=1;raw=48'sd472514560;repeat(12)@(negedge clk);f2();
        if(dut.control_state!=7 || dut.fault_code!=2 || stby || pwm)$fatal(1,"FAIL gate +721 limit stop");
        f2();if(dut.control_state!=7 || stby)$fatal(1,"FAIL gate limit bypass without acknowledgement");
        force dut.f1_press=1;@(negedge clk);force dut.f1_press=0;repeat(8)@(negedge clk);
        frame(3,0);frame(3,0);f2();
        if(dut.control_state!=1 || !stby || dut.raw_position_q16!=48'sd472514560)
            $fatal(1,"FAIL gate F1 recover without raw encoder reset");
        raw=-48'sd472514560;repeat(12)@(negedge clk);
        if(dut.control_state!=7 || dut.fault_code!=2 || stby)$fatal(1,"FAIL gate negative limit stop");
        force dut.f1_press=1;@(negedge clk);force dut.f1_press=0;repeat(8)@(negedge clk);
        frame(3,0);frame(3,0);f2();
        if(dut.control_state!=1 || !stby || dut.raw_position_q16!=-48'sd472514560)
            $fatal(1,"FAIL gate repeated negative F2 recovery");
        // 实际综合网表在非零POS接管，检查启动目标与默认300ms/1ADC过渡。
        f2();raw=-48'sd452853760;otr=0;low=0;repeat(12)@(negedge clk);
        frame(1393,1);f2();frame(1393,1);frame(1393,1);frame(1393,1);
        if(dut.control_state!=6 || dut.position_target_deci!=300 || dut.actual_position_deci!=300 ||
           dut.outer_ready_Z || dut.angle_target_code!=1403)
            $fatal(1,"FAIL gate handoff must lock current +30deg and delay outer PID");
        force dut.u_outer_start.delay_count=14999999;@(negedge clk);
        force dut.u_outer_start.delay_count=0;@(negedge clk);release dut.u_outer_start.delay_count;
        repeat(20)@(negedge clk);
        if(!dut.outer_ready_Z || used_outer(dut.outer_offset_adc)!==6'd0 || dut.angle_target_code!=1403)
            $fatal(1,"FAIL gate300ms handoff boundary ready=%b offset=%h target=%d",dut.outer_ready_Z,dut.outer_offset_adc,dut.angle_target_code);
        force dut.u_position_pid.period_count=2499999;@(negedge clk);
        force dut.u_position_pid.period_count=0;@(negedge clk);release dut.u_position_pid.period_count;
        repeat(12)@(negedge clk);
        if(dut.position_offset_adc!=0)$fatal(1,"FAIL gate position controller pulled to zero at capture");
        force dut.position_offset_adc=25;
        force dut.u_outer_start.ramp_count=2499999;@(negedge clk);
        force dut.u_outer_start.ramp_count=0;repeat(3)@(negedge clk);
        if(used_outer(dut.outer_offset_adc)!==6'd1 || dut.angle_target_code!=1404)$fatal(1,"FAIL gate first1ADC outer authority");
        force dut.position_offset_adc=-25;repeat(3)@(negedge clk);
        // 综合网表总线声明无signed属性，负值用同位宽补码比较。
        if(used_outer(dut.outer_offset_adc)!==6'h3f || dut.angle_target_code!=1402)
            $fatal(1,"FAIL gate negative outer authority offset=%h target=%d request=%h sign_input=%b",
                dut.outer_offset_adc,dut.angle_target_code,dut.position_offset_adc,dut.u_outer_start.position_offset_adc_15);
        force dut.f1_press=1;#1;if(stby || pwm)$fatal(1,"FAIL gate F1 after handoff");
        @(negedge clk);force dut.f1_press=0;repeat(8)@(negedge clk);
        if(dut.outer_ready_Z || used_outer(dut.outer_offset_adc)!==6'd0 || dut.angle_target_code!=1403)
            $fatal(1,"FAIL gate handoff clear on stop");
        release dut.position_offset_adc;release dut.u_outer_start.ramp_count;
        frame(1403,1);frame(1403,1);f2();frame(1403,1);frame(1403,1);frame(1403,1);
        force dut.s2_press=1;@(negedge clk);force dut.s2_press=0;repeat(4)@(negedge clk);
        // 30 次 × 5 dps = 150 dps。
        force dut.e3_repeat=1;repeat(30)@(negedge clk);force dut.e3_repeat=0;f2();
        if(!dut.speed_mode_Z || dut.speed_target_dps!=150)$fatal(1,"FAIL gate apply150dps");
        raw=48'sd1966080000;repeat(20)@(negedge clk); // 原始位置+3000°。
        if(dut.control_state!=6 || !dut.u_angle_pid.pid_active_Z || dut.fault_code!=0)
            $fatal(1,"FAIL gate multiple speed rotations still tripped720deg F2");
        held_position=dut.actual_position_deci;held_raw=raw;
        // 30 次 × 5 dps：150→0，速度设0后回到位置模式。
        force dut.f1_repeat=1;repeat(30)@(negedge clk);force dut.f1_repeat=0;f2();repeat(12)@(negedge clk);
        if(dut.speed_mode_Z || dut.control_state!=6 || dut.position_target_deci!=held_position ||
           dut.actual_position_deci!=held_position || dut.raw_position_q16!=held_raw)
            $fatal(1,"FAIL gate speed zero hold current without POS reset or immediate F2");
        raw=held_raw+48'sd471859200;repeat(16)@(negedge clk); // 新参考+720°边界。
        if(dut.control_state!=6)$fatal(1,"FAIL gate position720deg boundary should be allowed");
        raw=held_raw+48'sd472514560;repeat(16)@(negedge clk); // 新参考+721°。
        if(dut.control_state!=7 || dut.fault_code!=2 || stby || pwm)
            $fatal(1,"FAIL gate position positive720deg protection removed");
        force dut.f1_press=1;@(negedge clk);force dut.f1_press=0;repeat(8)@(negedge clk);
        frame(1403,1);frame(1403,1);f2();frame(1403,1);frame(1403,1);frame(1403,1);
        force dut.s2_press=1;@(negedge clk);force dut.s2_press=0;repeat(4)@(negedge clk);
        // 30 次 × 5 dps = -150 dps。
        force dut.f1_repeat=1;repeat(30)@(negedge clk);force dut.f1_repeat=0;f2();
        if(!dut.speed_mode_Z || dut.speed_target_dps!=16'hff6a)$fatal(1,"FAIL gate apply negative150dps");
        raw=-48'sd1966080000;repeat(20)@(negedge clk);
        if(dut.control_state!=6 || dut.fault_code!=0)$fatal(1,"FAIL gate negative multiple rotations F2");
        held_position=dut.actual_position_deci;held_raw=raw;
        force dut.s1_press=1;@(negedge clk);force dut.s1_press=0;repeat(4)@(negedge clk);
        force dut.e3_press=1;@(negedge clk);force dut.e3_press=0;repeat(4)@(negedge clk);f2();repeat(12)@(negedge clk);
        if(dut.speed_mode_Z || dut.control_state!=6 || dut.position_target_deci!=held_position+50 ||
           dut.actual_position_deci!=held_position || dut.raw_position_q16!=held_raw)
            $fatal(1,"FAIL gate S1 explicit position switch reset POS or used old guard origin");
        raw=held_raw-48'sd472514560;repeat(16)@(negedge clk);
        if(dut.control_state!=7 || dut.fault_code!=2 || stby || pwm)
            $fatal(1,"FAIL gate position negative720deg protection removed");
        $display("PASS gate PWM=%0d%% pulse_cycles=%0d,100-code capture,PID signs,immediate restart task,10ms off,2dps/100ms stopped arm,auto recapture,F5 timeout,hard faults,continuous speed and position protection",CONFIG_DUTY,`TEST_SWING_PULSE_CYCLES);$finish;
    end
    initial begin #20000000;$fatal(1,"FAIL autoswing gate timeout");end
endmodule
