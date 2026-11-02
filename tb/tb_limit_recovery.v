`timescale 1ns/1ps
// 复现旧版累计±720°后F1清故障但F2仍被锁住的路径，并验证双向恢复。
module tb_limit_recovery;
    reg clk=0;always #10 clk=~clk;
    reg signed [47:0] raw=0;
    reg [11:0] adc=3;
    reg good=0,otr=1,low=1;
    wire pwm,a1,a2,stby;
    inverted_pendulum_autoswing_top dut(.clk(clk),.button(1'b0),.button_dec(1'b0),.button_inc(1'b0),
        .button_page(1'b0),.button_speed_page(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),
        .adc_data(12'd3),.adc_otr(1'b0),.adc_clk(),.tft_cs_n(),.tft_sclk(),.tft_mosi(),
        .tft_dc(),.tft_rst_n(),.tft_bl(),.tb_pwma(pwm),.tb_ain1(a1),.tb_ain2(a2),.tb_stby(stby));
    task f2;begin
        @(negedge clk);force dut.f2_press=1;@(negedge clk);force dut.f2_press=0;
        repeat(8)@(negedge clk);
    end endtask
    task frame;begin
        @(negedge clk);force dut.frame_tick=1;@(negedge clk);force dut.frame_tick=0;
        repeat(8)@(negedge clk);
    end endtask
    task ack_zero;reg signed [47:0] saved;begin
        saved=raw;@(negedge clk);force dut.f1_press=1;#1;
        if(stby || pwm || a1 || a2)$fatal(1,"FAIL F1 must disable before changing origin");
        @(negedge clk);force dut.f1_press=0;repeat(8)@(negedge clk);
        if(dut.control_state!=0 || dut.actual_position_deci!=0 || dut.raw_position_q16!=saved)
            $fatal(1,"FAIL F1 must clear fault and zero only relative position");
        // 归零尚未结束时不能抢先启动；两帧结束后也不能自动启动。
        f2();if(dut.control_state!=0 || stby)$fatal(1,"FAIL F2 started during zero settle");
        frame();frame();repeat(8)@(negedge clk);
        if(dut.zero_busy || dut.control_state!=0 || stby || pwm)$fatal(1,"FAIL no automatic rearm after zero");
    end endtask
    task fault_check;input [3:0] code;begin
        repeat(8)@(negedge clk);
        if(dut.control_state!=7 || dut.fault_code!=code || stby || pwm || a1 || a2)
            $fatal(1,"FAIL fault stop code=%d state=%d actual=%d",code,dut.control_state,dut.fault_code);
    end endtask
    task display_frame;begin
        force dut.u_tft.startup_count=50000;force dut.u_tft.state=3;force dut.u_tft.init_index=18;force dut.u_tft.tx_done=1;
        repeat(2)@(negedge clk);force dut.u_tft.state=6;force dut.u_tft.tx_done=0;
    end endtask
    initial begin
        force dut.startup=0;force dut.frame_tick=0;force dut.f1_press=0;force dut.f2_press=0;
        force dut.u_encoder.pos_q16=raw;force dut.adc_average=adc;force dut.adc_valid=1;
        force dut.feedback_valid=good;force dut.pid_adc_valid=good;
        force dut.adc_overrange=otr;force dut.low_otr_only=low;
        #100;force dut.startup=50000;repeat(8)@(negedge clk);
        f2();if(dut.control_state!=1 || !stby)$fatal(1,"FAIL initial startup");
        raw=48'sd472514560; // +721°，超过软件范围；编码器累计量不重置。
        fault_check(2);f2();fault_check(2); // 未确认前再次启动仍必须阻断。
        ack_zero();f2();
        if(dut.control_state!=1 || !stby || dut.raw_position_q16!=48'sd472514560)
            $fatal(1,"FAIL F1 must release persistent position limit without raw reset");
        // 本次起摆±120°仍独立生效；单次重试只重取起摆起点，不改POS。
        raw=48'sd551813120; // 原始842°，相对新零点121°。
        fault_check(2);display_frame();
        if(dut.u_tft.line_char(10,13,0)!="T")$fatal(1,"FAIL TFT F2 T travel reason");
        f2();if(dut.control_state!=1 || dut.actual_position_deci!=1210 || !stby)
            $fatal(1,"FAIL travel retry must use new start without changing POS");
        ack_zero();raw=-48'sd472514560;repeat(8)@(negedge clk);f2();fault_check(2);
        display_frame();if(dut.u_tft.line_char(10,13,0)!="L")$fatal(1,"FAIL TFT F2 L soft-limit reason");
        ack_zero();f2();if(dut.control_state!=1 || !stby || dut.raw_position_q16!=-48'sd472514560)
            $fatal(1,"FAIL negative-limit recovery");
        // 再触发一次F2，确认不是只可恢复一次；故障恢复后可直接手扶入PID。
        raw=48'sd655360;fault_check(2);ack_zero();
        adc=1403;good=1;otr=0;low=0;repeat(8)@(negedge clk);f2();frame();frame();frame();
        if(dut.control_state!=6 || !dut.pid_active)$fatal(1,"FAIL manual PID reentry after repeated limit faults");
        // 速度仍基于原始累计值：F1归零不引入任何位置跳变到速度估计器。
        force dut.u_speed_measure.last_position_q16=raw;
        force dut.u_speed_measure.filtered_speed_q16=0;
        force dut.u_speed_measure.raw_speed_q16=0;
        ack_zero();force dut.u_speed_measure.period_count=249999;@(negedge clk);
        force dut.u_speed_measure.period_count=0;repeat(8)@(negedge clk);
        if(dut.u_speed_measure.delta_sampled_q16!=0 || dut.actual_speed_deci_dps!=0)
            $fatal(1,"FAIL reset origin introduced false speed");
        $display("PASS F2 recovery positive/negative/repeated,limits retained,F1 stop/zero,no auto restart,manual PID and raw speed preserved");$finish;
    end
    initial begin #2000000;$fatal(1,"FAIL recovery timeout");end
endmodule
