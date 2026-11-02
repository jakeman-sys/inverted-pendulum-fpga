`timescale 1ns/1ps
module tb_autoswing_top;
    reg clk=0;always #10 clk=~clk;
    wire pwm,a1,a2,stby;
    // 用1ms往返参数加速系统集成；真实默认时长在综合网表再验证。
    inverted_pendulum_autoswing_top #(.SWING_PULSE_MS(1),.SWING_REVERSE_GAP_MS(1),.AUTO_RECOVER_ENABLE(0)) dut(
        .clk(clk),.button(1'b0),.button_dec(1'b0),.button_inc(1'b0),.button_page(1'b0),.button_speed_page(1'b0),
        .encoder_a(1'b0),.encoder_b(1'b0),.adc_data(12'd1403),.adc_otr(1'b0),
        .adc_clk(),.tft_cs_n(),.tft_sclk(),.tft_mosi(),.tft_dc(),.tft_rst_n(),.tft_bl(),
        .tb_pwma(pwm),.tb_ain1(a1),.tb_ain2(a2),.tb_stby(stby));
    reg [11:0] adc=3;reg good=0,pid_good=0,overrange=1,low_only=1;
    integer i,highs;
    task frame;input [11:0] code;input ok;begin
        @(negedge clk);adc=code;good=ok;pid_good=ok;force dut.frame_tick=1;
        @(negedge clk);force dut.frame_tick=0;repeat(5)@(negedge clk);end endtask
    task f2;begin @(negedge clk);force dut.f2_press=1;@(negedge clk);force dut.f2_press=0;repeat(4)@(negedge clk);end endtask
    task pid_update;begin
        repeat(12)@(negedge clk);force dut.u_angle_pid.pid_count=249999;
        @(negedge clk);force dut.u_angle_pid.pid_count=0;@(negedge clk);release dut.u_angle_pid.pid_count;
        repeat(12)@(negedge clk);end endtask
    task capture;begin frame(1403,1);frame(1403,1);
        if(dut.pid_request)$fatal(1,"FAIL early PID capture");frame(1403,1);
        if(!dut.pid_request || !dut.pid_active || dut.swing_request)$fatal(1,"FAIL three-frame PID handoff");end endtask
    task display_frame;begin
        force dut.u_tft.startup_count=50000;force dut.u_tft.state=3;force dut.u_tft.init_index=18;force dut.u_tft.tx_done=1;
        repeat(2)@(negedge clk);force dut.u_tft.state=6;force dut.u_tft.tx_done=0;
        force dut.u_tft.bcd_start=1;@(negedge clk);force dut.u_tft.bcd_start=0;repeat(30)@(negedge clk);end endtask
    initial begin
        force dut.startup=0;force dut.frame_tick=0;force dut.adc_valid=1;force dut.adc_average=adc;
        force dut.feedback_valid=good;force dut.adc_overrange=overrange;force dut.low_otr_only=low_only;
        force dut.pid_adc_valid=pid_good;force dut.adc_batch_spread=2704;
        force dut.f1_press=0;force dut.f2_press=0;
        force dut.u_encoder.pos_q16=48'sd0;
        #100;force dut.startup=50000;repeat(8)@(negedge clk);
        if(stby || pwm || dut.running)$fatal(1,"FAIL powerup drive enabled");f2();
        if(dut.control_state!=1 || !stby || !a1 || a2)$fatal(1,"FAIL low-end OTR initial escape");
        highs=0;for(i=0;i<5000;i=i+1)begin if(pwm)highs=highs+1;@(negedge clk);end
        if(highs!=50*dut.SWING_DUTY_PERCENT)$fatal(1,"FAIL configured top PWM count %d",highs);
        wait(dut.control_state==2);#1;if(stby || pwm || a1 || a2)$fatal(1,"FAIL top reverse coast");
        wait(dut.control_state==3);#1;
        if(!stby || a1 || !a2)$fatal(1,"FAIL top reverse after coast");
        wait(dut.control_state==4);@(negedge clk);if(stby || pwm)$fatal(1,"FAIL peak wait output");
        overrange=0;low_only=0;frame(2700,0);frame(3,0);frame(2700,0);
        if(dut.control_state!=4)$fatal(1,"FAIL wrap triggered pump/capture");
        frame(500,1);frame(600,1);frame(550,1);
        if(dut.control_state!=1 || !a1 || a2)$fatal(1,"FAIL left peak drive");
        wait(dut.control_state==4);frame(3,0);frame(2300,1);frame(2100,1);frame(2150,1);
        if(dut.control_state!=1 || a1 || !a2)$fatal(1,"FAIL right peak drive");
        frame(1403,0);frame(1403,0);frame(1403,0);if(dut.pid_request)$fatal(1,"FAIL bad averages captured");
        capture();if(dut.angle_target_code!=1403)$fatal(1,"FAIL baseline center changed");
        adc=1393;good=1;pid_update();pid_update();
        if(!stby || a1 || !a2)$fatal(1,"FAIL original PID below-target direction");
        adc=1413;pid_update();pid_update();
        if(!stby || !a1 || a2)$fatal(1,"FAIL original PID above-target direction");
        good=0;pid_good=0;adc=1403;repeat(6400)@(negedge clk); // 两个真正不合格批
        if(!dut.pid_active || dut.control_state!=6 || dut.pid_adc!=1413 || !stby)
            $fatal(1,"FAIL transient quality rejection must not reset PID");
        good=1;pid_good=1;repeat(8)@(negedge clk);
        // 起摆严格资格丢失，但PID平均值依然有效时，持续超过2ms也不能退出。
        good=0;repeat(100010)@(negedge clk);
        if(!dut.pid_active || dut.control_state!=6)$fatal(1,"FAIL swing quality criterion leaked into running PID");
        good=1;
        display_frame();
        if(dut.u_tft.frame_target_adc!=1403 || dut.u_tft.ikp_bcd!=16'h2000 || dut.u_tft.iki_bcd!=16'h0045 ||
           dut.u_tft.ikd_bcd!=16'h3500 || dut.u_tft.line_char(10,0,0)!="P")$fatal(1,"FAIL baseline TFT PID parameters");
        adc=1754;#1;if(stby || pwm)$fatal(1,"FAIL fallen pendulum immediate disable");
        repeat(4)@(negedge clk);
        if(dut.control_state!=7 || dut.fault_code!=4 || dut.fault_detail!=1 || dut.fault_adc!=1754)
            $fatal(1,"FAIL fall fault snapshot");
        adc=1403;display_frame();
        if(dut.u_tft.frame_adc!=1754 || dut.u_tft.line_char(6,0,0)!="E" || dut.u_tft.line_char(10,13,0)!="W")
            $fatal(1,"FAIL TFT must display frozen ERR ADC and F4 W");
        frame(1403,1);frame(1403,1);frame(1403,1);if(dut.running)$fatal(1,"FAIL fallen auto restarted");
        f2();capture();good=0;pid_good=0;repeat(100010)@(negedge clk);
        if(dut.control_state!=7 || dut.fault_code!=4 || dut.fault_detail!=2 || stby || pwm)
            $fatal(1,"FAIL persistent sample loss F4 A");
        display_frame();
        if(dut.u_tft.line_char(10,13,0)!="A" || dut.u_tft.line_char(0,0,0)!="S" || dut.u_tft.spread_bcd!=16'h2704)
            $fatal(1,"FAIL TFT F4 A reason and frozen spread");
        good=1;frame(1403,1);if(dut.running)$fatal(1,"FAIL sample recovery automatically restarted");
        f2();capture();overrange=1;low_only=1;good=0;repeat(4)@(negedge clk);
        if(stby || pwm || dut.fault_code!=1)$fatal(1,"FAIL PID cannot use low OTR exception");
        // 主页面F1停止并归零，原始坐标保留；不会在正在控制时跳变坐标。
        force dut.u_encoder.pos_q16=48'sd58982400;repeat(4)@(negedge clk);force dut.f1_press=1;#1;
        if(stby || pwm)$fatal(1,"FAIL F1 stop before zero");
        @(negedge clk);force dut.f1_press=0;repeat(6)@(negedge clk);
        if(dut.actual_position_deci!=0 || dut.raw_position_q16!=48'sd58982400 || dut.control_state!=0)
            $fatal(1,"FAIL F1 origin-only zero");
        $display("PASS integrated configured PWM,1ms timing override,dead OTR escape,wrap/peak pumping,three-frame original PID,signs,TFT and safe F1");$finish;
    end
    initial begin #20000000;$fatal(1,"FAIL autoswing top timeout");end
endmodule
