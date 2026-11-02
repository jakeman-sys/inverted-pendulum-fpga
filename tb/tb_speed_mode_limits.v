`timescale 1ns/1ps
// +150/-150速度模式允许累计多圈；回位置模式重建保护参考，POS连续。
module tb_speed_mode_limits;
    reg clk=0;always #10 clk=~clk;
    reg signed [47:0] raw=0;
    wire pwm,a1,a2,stby;
    inverted_pendulum_autoswing_top dut(.clk(clk),.button(1'b0),.button_dec(1'b0),.button_inc(1'b0),
        .button_page(1'b0),.button_speed_page(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),
        .adc_data(12'd1403),.adc_otr(1'b0),.adc_clk(),.tft_cs_n(),.tft_sclk(),.tft_mosi(),
        .tft_dc(),.tft_rst_n(),.tft_bl(),.tb_pwma(pwm),.tb_ain1(a1),.tb_ain2(a2),.tb_stby(stby));
    task pulse_f2;begin @(negedge clk);force dut.f2_press=1;@(negedge clk);force dut.f2_press=0;repeat(8)@(negedge clk);end endtask
    task frame;begin @(negedge clk);force dut.frame_tick=1;@(negedge clk);force dut.frame_tick=0;repeat(8)@(negedge clk);end endtask
    task speed_page;begin force dut.s2_press=1;@(negedge clk);force dut.s2_press=0;repeat(4)@(negedge clk);end endtask
    task apply_speed;input signed [15:0] target;begin
        force dut.u_ui.edit_speed_dps=target;@(negedge clk);pulse_f2();release dut.u_ui.edit_speed_dps;
        if(dut.speed_mode!=(target!=0) || dut.speed_target_dps!=target || dut.control_state!=6)
            $fatal(1,"FAIL applied speed mode target=%d",target);
    end endtask
    task healthy;begin
        repeat(12)@(negedge clk);
        if(dut.control_state!=6 || !dut.pid_active || dut.fault_code!=0 || dut.position_limit)
            $fatal(1,"FAIL continuous speed/return position tripped F2 state=%d POS=%d",dut.control_state,dut.actual_position_deci);
    end endtask
    initial begin
        force dut.startup=0;force dut.frame_tick=0;force dut.f1_press=0;force dut.f2_press=0;
        force dut.u_encoder.pos_q16=raw;force dut.adc_valid=1;force dut.adc_average=1403;
        force dut.feedback_valid=1;force dut.pid_adc_valid=1;force dut.adc_overrange=0;force dut.low_otr_only=0;
        #100;force dut.startup=50000;repeat(8)@(negedge clk);pulse_f2();frame();frame();frame();
        speed_page();apply_speed(150);force dut.actual_speed_deci_dps=1500;
        raw=48'sd472514560;healthy(); // +721°
        raw=48'sd1966080000;healthy(); // +3000°
        if(dut.actual_position_deci!=30000 || dut.raw_position_q16!=raw)$fatal(1,"FAIL multiple rotations changed encoder origin");
        apply_speed(0);healthy();
        if(dut.position_target_deci!=30000 || dut.actual_position_deci!=30000)
            $fatal(1,"FAIL speed zero must hold current3000deg without resetting POS");
        raw=48'sd2031616000;healthy(); // 仅再移+100°，位置模式保护参考是切换位置。
        raw=48'sd2438594560;repeat(12)@(negedge clk); // +3721°，相对切换点+721°。
        if(dut.control_state!=7 || dut.fault_code!=2 || pwm || stby)$fatal(1,"FAIL position mode lost720deg protection");
        force dut.f1_press=1;@(negedge clk);force dut.f1_press=0;repeat(8)@(negedge clk);frame();frame();
        pulse_f2();frame();frame();frame();speed_page();apply_speed(-150);force dut.actual_speed_deci_dps=-1500;
        raw=-48'sd1966080000;healthy();
        if(dut.actual_position_deci>=0)$fatal(1,"FAIL negative continuous rotation");
        // 从速度页切到位置页应用目标，也不能因旧零点超限立即停机。
        force dut.s1_press=1;@(negedge clk);force dut.s1_press=0;repeat(4)@(negedge clk);
        pulse_f2();healthy();
        if(dut.speed_mode || dut.position_target_deci!=dut.actual_position_deci)
            $fatal(1,"FAIL S1 apply must switch to held position");
        $display("PASS +/-150 speed reached stays active beyond720deg/multiple turns,zero speed/S1 hold continuous POS,position limit retained");$finish;
    end
    initial begin #2000000;$fatal(1,"FAIL speed mode limit timeout");end
endmodule
