`timescale 1ns/1ps
// 真实ADC+原手动版ADC参考：持续普通纹波不退出PID，跨死区混批仍停机。
module tb_adc_pid_handoff;
    reg clk=0;always #10 clk=~clk;
    reg [11:0] data=1393;
    wire pwm,a1,a2,stby,adc_clk;
    inverted_pendulum_autoswing_top dut(.clk(clk),.button(1'b0),.button_dec(1'b0),.button_inc(1'b0),
        .button_page(1'b0),.button_speed_page(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),
        .adc_data(data),.adc_otr(1'b0),.adc_clk(adc_clk),.tft_cs_n(),.tft_sclk(),.tft_mosi(),
        .tft_dc(),.tft_rst_n(),.tft_bl(),.tb_pwma(pwm),.tb_ain1(a1),.tb_ain2(a2),.tb_stby(stby));
    integer n;
    wire [11:0] ref_average;wire ref_valid,ref_otr;
    reference_ad9220_capture ref_adc(.clk_50m(clk),.adc_data(data),.adc_otr(1'b0),
        .adc_clk(),.average(ref_average),.valid(ref_valid),.overrange(ref_otr));
    always @(negedge clk)
        if(ref_valid && (dut.adc_average!==ref_average || dut.adc_valid!==ref_valid || dut.adc_overrange!==ref_otr))
            $fatal(1,"FAIL ADC average/protocol differs from original working manual project");
    task frame;begin @(negedge clk);force dut.frame_tick=1;@(negedge clk);force dut.frame_tick=0;
        repeat(5)@(negedge clk);end endtask
    initial begin
        force dut.startup=0;force dut.frame_tick=0;force dut.f2_press=0;
        repeat(7000)@(negedge clk);force dut.startup=50000;repeat(10)@(negedge clk);
        if(!dut.feedback_valid)$fatal(1,"FAIL real ADC did not qualify");
        force dut.f2_press=1;@(negedge clk);force dut.f2_press=0;
        frame();frame();frame();
        if(dut.control_state!=6 || !dut.pid_active)$fatal(1,"FAIL real ADC static handoff");
        // 5ms原PID首次更新后有非零反馈转矩。
        repeat(250020)@(negedge clk);
        if(!stby || a1 || !a2)$fatal(1,"FAIL PID not driving after normal first update");
        wait(dut.u_adc.sample_count==10);@(negedge clk);data=4095;
        @(negedge adc_clk);@(negedge clk);data=1393;
        repeat(5)@(negedge clk);
        if(dut.feedback_valid || !dut.pid_feedback_ok || !dut.pid_active || dut.control_state!=6 || !stby)
            $fatal(1,"FAIL real single-point corruption terminated PID");
        if(dut.pid_adc!=1393)$fatal(1,"FAIL bad point fed PID");
        wait(dut.u_adc.average_good==0);repeat(5)@(negedge clk);
        if(!dut.pid_feedback_ok || dut.control_state!=6 || !dut.pid_active || !stby)
            $fatal(1,"FAIL one contaminated 64-point batch terminated PID");
        repeat(7000)@(negedge clk);
        if(!dut.feedback_valid || dut.control_state!=6 || !dut.pid_active)$fatal(1,"FAIL good batch recovery");
        // 166码跨度的持续纹波：旧PID6超过2ms必报A，新PID7使用平均值继续控制。
        for(n=0;n<110;n=n+1)begin
            @(negedge clk);data=1320;repeat(16)@(negedge adc_clk);
            @(negedge clk);data=1486;repeat(16)@(negedge adc_clk);
        end
        if(dut.feedback_valid || !dut.pid_adc_valid || dut.control_state!=6 || !dut.pid_active)
            $fatal(1,"FAIL ordinary sustained ripple must not produce F4 A");
        if(dut.adc_batch_spread!=166 || dut.adc_average<1380 || dut.adc_average>1420)
            $fatal(1,"FAIL continuous ripple test not exercised");
        // 真跨周：每批32个2708和32个4，均值1356伪装成直立但必须拒绝。
        wait(dut.u_adc.sample_count==0);
        for(n=0;n<42;n=n+1)begin
            @(negedge clk);data=2708;repeat(32)@(negedge adc_clk);
            @(negedge clk);data=4;repeat(32)@(negedge adc_clk);
        end
        if(dut.control_state!=7 || dut.fault_code!=4 || dut.fault_detail!=2 || stby || pwm || dut.fault_spread!=2704)
            $fatal(1,"FAIL true wrap-mixed ADC must produce F4 A with frozen spread");
        if(dut.fault_adc<1053 || dut.fault_adc>1753)$fatal(1,"FAIL sustained noise fault snapshot");
        $display("PASS manual ADC average equivalence, static handoff, transient corruption, sustained166-code ripple, true wrap F4 A");$finish;
    end
    initial begin #20000000;$fatal(1,"FAIL real ADC handoff timeout");end
endmodule
