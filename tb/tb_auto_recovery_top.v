`timescale 1ns/1ps
// 当前默认计时参数边界使用force加速；检查死区资格、速度门槛、大POS恢复和按键停机。
module tb_auto_recovery_top;
    reg clk=0;always #10 clk=~clk;
    wire pwm,a1,a2,stby;
    reg [11:0] adc=1403;reg good=1,otr=0,low=0;
    reg signed [47:0] raw=0;reg signed [31:0] speed=0;
    inverted_pendulum_autoswing_top #(.RECOVER_IMMEDIATE(0)) dut(.clk(clk),.button(1'b0),.button_dec(1'b0),.button_inc(1'b0),
        .button_page(1'b0),.button_speed_page(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),
        .adc_data(12'd1403),.adc_otr(1'b0),.adc_clk(),.tft_cs_n(),.tft_sclk(),.tft_mosi(),
        .tft_dc(),.tft_rst_n(),.tft_bl(),.tb_pwma(pwm),.tb_ain1(a1),.tb_ain2(a2),.tb_stby(stby));
    task f2;begin @(negedge clk);force dut.f2_press=1;@(negedge clk);force dut.f2_press=0;repeat(8)@(negedge clk);end endtask
    task frame;begin @(negedge clk);force dut.frame_tick=1;@(negedge clk);force dut.frame_tick=0;repeat(8)@(negedge clk);end endtask
    task capture;begin adc=1403;good=1;otr=0;low=0;frame();frame();frame();
        if(dut.control_state!=6 || !dut.pid_active)$fatal(1,"FAIL recovery top capture");end endtask
    task all_off;begin if(pwm || stby || a1 || a2)$fatal(1,"FAIL recovery must disable all driver outputs");end endtask
    task fall;begin adc=1754;good=1;#1;all_off();repeat(12)@(negedge clk);
        if(dut.control_state!=8 || !dut.recovery_pending || dut.position_limit)$fatal(1,"FAIL fall converted to wrong fault/old position limit");end endtask
    task end_stop;begin
        force dut.u_swing.recover_stop_count=14999998;@(negedge clk);
        if(dut.control_state!=8)$fatal(1,"FAIL300ms stop before boundary");
        force dut.u_swing.recover_stop_count=14999999;@(negedge clk);release dut.u_swing.recover_stop_count;
        if(dut.control_state!=9)$fatal(1,"FAIL300ms stop boundary");end endtask
    initial begin
        force dut.startup=0;force dut.frame_tick=0;force dut.f1_press=0;force dut.f2_press=0;
        force dut.adc_valid=1;force dut.adc_average=adc;force dut.feedback_valid=good;force dut.pid_adc_valid=good;
        force dut.adc_overrange=otr;force dut.low_otr_only=low;
        force dut.u_encoder.pos_q16=raw;force dut.actual_speed_deci_dps=speed;
        #100;force dut.startup=50000;repeat(8)@(negedge clk);f2();capture();
        force dut.s2_press=1;@(negedge clk);force dut.s2_press=0;repeat(4)@(negedge clk);
        force dut.u_ui.edit_speed_dps=150;f2();release dut.u_ui.edit_speed_dps;
        raw=48'sd1966080000;speed=3000;repeat(16)@(negedge clk);fall();
        if(dut.actual_position_deci!=30000 || dut.raw_position_q16!=raw)$fatal(1,"FAIL fall reset POS");
        adc=3;good=0;otr=1;low=1;end_stop();repeat(30)@(negedge clk);all_off();
        if(dut.control_state!=9 || dut.u_swing.recover_ready_count!=0)$fatal(1,"FAIL fast arm in deadzone must wait");
        speed=-3000;repeat(12)@(negedge clk);
        if(dut.recovery_down_ready || dut.u_swing.recover_ready_count!=0)$fatal(1,"FAIL negative fast arm must wait");
        adc=2678;otr=0;low=0;speed=-200;repeat(8)@(negedge clk);
        if(!dut.recovery_down_ready || dut.control_state!=9)$fatal(1,"FAIL upper down endpoint and negative20deg/s qualification");
        adc=35;speed=0;repeat(8)@(negedge clk);
        if(dut.recovery_down_ready || dut.u_swing.recover_ready_count!=0)$fatal(1,"FAIL outside down window must reset qualification");
        adc=3;otr=1;low=1;
        speed=200; // 20deg/s含边界。
        force dut.u_swing.recover_ready_count=9999998;@(negedge clk);
        if(dut.control_state!=9)$fatal(1,"FAIL qualification shorter than200ms");
        force dut.u_swing.recover_ready_count=9999999;@(negedge clk);release dut.u_swing.recover_ready_count;
        repeat(8)@(negedge clk);
        if(dut.control_state!=1 || !stby || dut.recovery_attempts!=1 || dut.actual_position_deci!=30000)
            $fatal(1,"FAIL deadzone lowOTR auto swing at continuous POS");
        speed=0;capture();
        if(dut.speed_mode || dut.speed_target_dps!=0 || dut.position_target_deci!=30000 || dut.outer_ready)
            $fatal(1,"FAIL recovered PID must hold current position with angle-only delay");
        force dut.u_swing.stable_count=49999998;@(negedge clk);
        if(dut.recovery_attempts!=1)$fatal(1,"FAIL retry count cleared before1s stable");
        force dut.u_swing.stable_count=49999999;@(negedge clk);release dut.u_swing.stable_count;
        if(dut.recovery_attempts!=0)$fatal(1,"FAIL1s stable retry reset");
        fall();f2();adc=3;good=0;otr=1;low=1;repeat(40)@(negedge clk);all_off();
        if(dut.control_state!=0 || dut.recovery_attempts!=0)$fatal(1,"FAIL F2 cancel pending restart");
        f2();capture();fall();force dut.f1_press=1;@(negedge clk);force dut.f1_press=0;repeat(8)@(negedge clk);
        if(dut.control_state!=0 || dut.actual_position_deci!=0 || dut.raw_position_q16!=raw)$fatal(1,"FAIL F1 stop/zero while pending");
        frame();frame();f2();capture();good=0;repeat(100010)@(negedge clk);
        if(dut.control_state!=7 || dut.fault_code!=4 || dut.fault_detail!=2)$fatal(1,"FAIL bad ADC auto recovery allowed");
        good=1;f2();capture();adc=4095;repeat(8)@(negedge clk);all_off();
        if(dut.control_state!=7 || dut.fault_code!=1)$fatal(1,"FAIL implausible ADC should not pump automatically");
        $display("PASS top immediate stop,300ms/200ms boundaries,down lowOTR,20deg/s gate,large POS speed fall,no coordinate reset,position hold,1s count reset,F2/F1 stop and hard ADC faults");$finish;
    end
    initial begin #20000000;$fatal(1,"FAIL auto recovery top timeout");end
endmodule
