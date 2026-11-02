`timescale 1ns/1ps
// 默认立即恢复：只加速长计时器边界，保留真实状态切换、PID、UI和电机门控。
module tb_immediate_recovery;
    reg clk=0;always #10 clk=~clk;
    wire pwm,a1,a2,stby;
    reg [11:0] adc=1403;reg good=1,otr=0,low=0;
    reg signed [47:0] raw=0;reg signed [31:0] speed=0;
    inverted_pendulum_autoswing_top dut(.clk(clk),.button(1'b0),.button_dec(1'b0),.button_inc(1'b0),
        .button_page(1'b0),.button_speed_page(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),
        .adc_data(12'd1403),.adc_otr(1'b0),.adc_clk(),.tft_cs_n(),.tft_sclk(),.tft_mosi(),
        .tft_dc(),.tft_rst_n(),.tft_bl(),.tb_pwma(pwm),.tb_ain1(a1),.tb_ain2(a2),.tb_stby(stby));
    task f2;begin @(negedge clk);force dut.f2_press=1;@(negedge clk);force dut.f2_press=0;repeat(8)@(negedge clk);end endtask
    task frame;begin @(negedge clk);force dut.frame_tick=1;@(negedge clk);force dut.frame_tick=0;repeat(8)@(negedge clk);end endtask
    task capture;begin adc=1403;good=1;otr=0;low=0;frame();frame();frame();
        if(dut.control_state!=6 || !dut.pid_active)$fatal(1,"FAIL immediate capture");end endtask
    task off;begin if(pwm || stby || a1 || a2)$fatal(1,"FAIL immediate recovery outputs not off");end endtask
    task fall;begin
        @(negedge clk);adc=1754;#1;off();
        @(negedge clk);
        if(dut.control_state!=10 || !dut.recovery_pending || !dut.running || dut.position_limit || dut.pid_request)
            $fatal(1,"FAIL window exit must immediately enter restart task");
        repeat(12)@(negedge clk);off();
    end endtask
    task restart;input integer expected;begin
        // 摆杆仍在窗口外，不要求回到下垂区；但横杆高速时必须停驱等待。
        adc=1754;speed=3000;good=1;otr=0;low=0;
        force dut.u_swing.recover_stop_count=499998;@(negedge clk);
        if(dut.control_state!=10 || dut.recovery_arm_stopped)$fatal(1,"FAIL short10ms gap");off();
        force dut.u_swing.recover_stop_count=499999;@(negedge clk);
        release dut.u_swing.recover_stop_count;
        repeat(30)@(negedge clk);off();
        if(dut.control_state!=10 || dut.u_swing.recover_ready_count!=0)$fatal(1,"FAIL moving arm restarted before stopping");
        speed=21;repeat(8)@(negedge clk);off();
        if(dut.recovery_arm_stopped || dut.u_swing.recover_ready_count!=0)$fatal(1,"FAIL +2.1dps must wait");
        speed=-21;repeat(8)@(negedge clk);off();
        if(dut.recovery_arm_stopped || dut.u_swing.recover_ready_count!=0)$fatal(1,"FAIL -2.1dps must wait");
        speed=-20;repeat(8)@(negedge clk);
        if(!dut.recovery_arm_stopped)$fatal(1,"FAIL -2dps stopped threshold inclusive");
        speed=20;
        force dut.u_swing.recover_ready_count=4999998;@(negedge clk);
        if(dut.control_state!=10)$fatal(1,"FAIL stopped qualification shorter than100ms");off();
        force dut.u_swing.recover_ready_count=4999999;@(negedge clk);release dut.u_swing.recover_ready_count;
        repeat(8)@(negedge clk);
        if(dut.control_state!=1 || !stby || !a1 || a2 || dut.recovery_attempts!=expected ||
           dut.u_swing.start_position!=dut.actual_position_deci)
            $fatal(1,"FAIL stopped arm must start at current POS without waiting down");
    end endtask
    initial begin
        force dut.startup=0;force dut.frame_tick=0;force dut.f1_press=0;force dut.f2_press=0;
        force dut.adc_valid=1;force dut.adc_average=adc;force dut.feedback_valid=good;force dut.pid_adc_valid=good;
        force dut.adc_overrange=otr;force dut.low_otr_only=low;
        force dut.u_encoder.pos_q16=raw;force dut.actual_speed_deci_dps=speed;
        #100;force dut.startup=50000;repeat(10)@(negedge clk);
        if(dut.running)$fatal(1,"FAIL powerup must not auto arm");
        f2();capture();
        force dut.s2_press=1;@(negedge clk);force dut.s2_press=0;repeat(4)@(negedge clk);
        force dut.u_ui.edit_speed_dps=150;f2();release dut.u_ui.edit_speed_dps;
        raw=48'sd1966080000;speed=3000;repeat(20)@(negedge clk);fall();
        if(dut.actual_position_deci!=30000 || dut.raw_position_q16!=raw)$fatal(1,"FAIL immediate fall changed POS");
        adc=1403;frame();frame();frame();off();
        if(dut.control_state!=10)$fatal(1,"FAIL upright frames bypassed electrical gap");
        restart(1);capture();
        if(dut.speed_mode || dut.speed_target_dps!=0 || dut.position_target_deci!=30000 || dut.outer_ready)
            $fatal(1,"FAIL recovered PID must hold current POS with original outer delay");
        fall();restart(2);capture();fall();restart(3);capture();
        adc=1052;repeat(8)@(negedge clk);off();
        if(dut.control_state!=7 || dut.fault_code!=5 || dut.recovery_fault_reason!=1)
            $fatal(1,"FAIL fourth consecutive fall must latch F5 R");
        repeat(50)@(negedge clk);off();if(dut.running)$fatal(1,"FAIL exhausted retry auto restarted");
        adc=1403;f2();capture();fall();restart(1);
        force dut.u_swing.total_count=749999999;@(negedge clk);release dut.u_swing.total_count;
        if(dut.control_state!=10 || dut.fault_code!=3)$fatal(1,"FAIL auto swing timeout must return to immediate gap");
        restart(2);capture();
        force dut.u_swing.stable_count=49999998;@(negedge clk);
        if(dut.recovery_attempts!=2)$fatal(1,"FAIL stable retry reset early");
        force dut.u_swing.stable_count=49999999;@(negedge clk);release dut.u_swing.stable_count;
        if(dut.recovery_attempts!=0)$fatal(1,"FAIL1s stable retry reset");
        fall();force dut.u_swing.recover_wait_count=249999999;@(negedge clk);release dut.u_swing.recover_wait_count;off();
        if(dut.control_state!=7 || dut.fault_code!=5 || dut.recovery_fault_reason!=2)$fatal(1,"FAIL stop wait timeout F5 T");
        adc=1403;f2();capture();
        fall();f2();repeat(20)@(negedge clk);off();
        if(dut.control_state!=0 || dut.recovery_attempts!=0)$fatal(1,"FAIL F2 cancel immediate restart");
        adc=1403;f2();capture();fall();force dut.f1_press=1;@(negedge clk);force dut.f1_press=0;repeat(8)@(negedge clk);off();
        if(dut.control_state!=0 || dut.actual_position_deci!=0 || dut.raw_position_q16!=raw)
            $fatal(1,"FAIL F1 stop/zero while immediate pending");
        frame();frame();f2();capture();good=0;repeat(100010)@(negedge clk);off();
        if(dut.control_state!=7 || dut.fault_code!=4 || dut.fault_detail!=2)$fatal(1,"FAIL bad ADC must not automatically pump");
        good=1;f2();capture();adc=1754;otr=1;low=1;repeat(8)@(negedge clk);off();
        if(dut.control_state!=7 || dut.fault_code!=1)$fatal(1,"FAIL OTR must outrank immediate recovery");
        adc=1403;otr=0;low=0;f2();capture();fall();adc=4095;repeat(8)@(negedge clk);off();
        if(dut.control_state!=7 || dut.fault_code!=1)$fatal(1,"FAIL hard ADC fault in restart gap");
        adc=1403;f2();capture();fall();raw=raw+48'sd472514560;repeat(16)@(negedge clk);off();
        if(dut.control_state!=7 || dut.fault_code!=2)$fatal(1,"FAIL position limit in immediate task");
        $display("PASS immediate task,10ms minimum off,2dps/100ms stopped arm,no down wait,stop timeout,3-retry cap,auto-timeout,1s reset,continuous POS hold,F2/F1 cancellation,hard faults");$finish;
    end
    initial begin #20000000;$fatal(1,"FAIL immediate recovery timeout");end
endmodule
