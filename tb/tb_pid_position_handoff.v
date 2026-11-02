`timescale 1ns/1ps
// 起摆接管不回POS=0；内环先工作，外环渐进接入；停机不重置实际坐标。
module tb_pid_position_handoff;
    reg clk=0;always #10 clk=~clk;
    reg signed [47:0] raw=48'sd39321600; // +60°
    wire pwm,a1,a2,stby;
    inverted_pendulum_autoswing_top dut(.clk(clk),.button(1'b0),.button_dec(1'b0),.button_inc(1'b0),
        .button_page(1'b0),.button_speed_page(1'b0),.encoder_a(1'b0),.encoder_b(1'b0),
        .adc_data(12'd1403),.adc_otr(1'b0),.adc_clk(),.tft_cs_n(),.tft_sclk(),.tft_mosi(),
        .tft_dc(),.tft_rst_n(),.tft_bl(),.tb_pwma(pwm),.tb_ain1(a1),.tb_ain2(a2),.tb_stby(stby));
    reg small_active=0;
    reg signed [15:0] small_command=25;
    wire small_ready;wire signed [15:0] small_output;
    outer_pid_soft_start #(.DELAY_CYCLES(10),.RAMP_PERIOD_CYCLES(8),.RAMP_STEP_ADC(1),.MAX_OFFSET_ADC(3)) u_short_start(
        .clk(clk),.rst_n(1'b1),.active(small_active),.requested_offset_adc(small_command),
        .outer_ready(small_ready),.offset_adc(small_output));
    wire immediate_ready;wire signed [15:0] immediate_output;
    outer_pid_soft_start #(.DELAY_CYCLES(0),.RAMP_STEP_ADC(0),.MAX_OFFSET_ADC(3)) immediate(
        .clk(clk),.rst_n(1'b1),.active(small_active),.requested_offset_adc(small_command),
        .outer_ready(immediate_ready),.offset_adc(immediate_output));
    task f2;begin
        @(negedge clk);force dut.f2_press=1;@(negedge clk);force dut.f2_press=0;
        repeat(8)@(negedge clk);
    end endtask
    task frame;begin
        @(negedge clk);force dut.frame_tick=1;@(negedge clk);force dut.frame_tick=0;
        repeat(8)@(negedge clk);
    end endtask
    task capture;input signed [31:0] expected_position;begin
        f2();frame();frame();frame();repeat(8)@(negedge clk);
        if(dut.control_state!=6 || !dut.pid_active || dut.outer_ready || dut.angle_target_code!=1403 ||
           dut.position_target_deci!=expected_position || dut.position_edit_deci!=expected_position ||
           dut.actual_position_deci!=expected_position || dut.raw_position_q16!=raw)
            $fatal(1,"FAIL handoff must hold actual signed POS without resetting encoder or enabling outer PID");
    end endtask
    initial begin : small_test
        repeat(4)@(negedge clk);small_active=1;
        repeat(9)begin @(negedge clk);if(small_ready || small_output!=0)$fatal(1,"FAIL short angle-only delay");end
        @(negedge clk);if(!small_ready || small_output!=0)$fatal(1,"FAIL exact angle-only delay");
        repeat(8)@(negedge clk);@(negedge clk);if(small_output!=1)$fatal(1,"FAIL first1-code authority");
        small_command=-25;@(negedge clk);if(small_output!=-1)$fatal(1,"FAIL negative authority clamp");
        repeat(16)@(negedge clk);if(small_output!=-3)$fatal(1,"FAIL authority must reach full configured limit");
        if(!immediate_ready || immediate_output!=-3)$fatal(1,"FAIL zero delay/ramp bypass interface");
        small_active=0;@(negedge clk);if(small_ready || small_output!=0 || immediate_output!=0)
            $fatal(1,"FAIL stop must clear both handoff implementations");
    end
    initial begin : top_test
        force dut.startup=0;force dut.frame_tick=0;force dut.f1_press=0;force dut.f2_press=0;
        force dut.u_encoder.pos_q16=raw;force dut.adc_valid=1;force dut.adc_average=1403;
        force dut.feedback_valid=1;force dut.pid_adc_valid=1;force dut.adc_overrange=0;force dut.low_otr_only=0;
        #100;force dut.startup=50000;repeat(8)@(negedge clk);capture(600);
        // 摆杆刚接住时横杆可为内环纠偏移动；300ms期间不能产生外环目标偏移。
        raw=48'sd42598400;repeat(12)@(negedge clk);
        if(dut.position_target_deci!=600 || dut.actual_position_deci!=650 || dut.outer_offset_adc!=0 ||
           dut.angle_target_code!=1403 || dut.u_position_pid.period_count!=0)
            $fatal(1,"FAIL angle-only interval must ignore position error while preserving capture target");
        raw=48'sd39321600;repeat(12)@(negedge clk);
        force dut.u_outer_start.delay_count=14999998;@(negedge clk);
        if(dut.outer_ready)$fatal(1,"FAIL short default300ms delay");
        force dut.u_outer_start.delay_count=14999999;@(negedge clk);
        release dut.u_outer_start.delay_count;repeat(20)@(negedge clk);
        if(!dut.outer_ready || dut.outer_offset_adc!=0)$fatal(1,"FAIL default300ms delay boundary");
        // +60°处进入位置环：目标同为+60°，原位置PID不能因回零误差产生输出。
        force dut.u_position_pid.period_count=2499999;@(negedge clk);
        force dut.u_position_pid.period_count=0;@(negedge clk);release dut.u_position_pid.period_count;
        repeat(12)@(negedge clk);
        if(dut.position_offset_adc!=0 || dut.u_position_pid.integral!=0 || dut.angle_target_code!=1403)
            $fatal(1,"FAIL outer PID pulled toward old POS zero");
        force dut.position_offset_adc=25;
        force dut.u_outer_start.ramp_count=2499998;@(negedge clk);
        if(dut.u_outer_start.authority_adc!=0)$fatal(1,"FAIL short default50ms ramp period");
        force dut.u_outer_start.ramp_count=2499999;@(negedge clk);
        force dut.u_outer_start.ramp_count=0;repeat(3)@(negedge clk);
        if(dut.outer_offset_adc!=1 || dut.angle_target_code!=1404)$fatal(1,"FAIL default ramp must initially allow only1 ADC");
        force dut.position_offset_adc=-25;repeat(3)@(negedge clk);
        if(dut.outer_offset_adc!=-1 || dut.angle_target_code!=1402)$fatal(1,"FAIL signed negative ramp");
        force dut.position_offset_adc=0;repeat(3)@(negedge clk);
        f2();if(dut.running || dut.outer_ready || dut.outer_offset_adc!=0 || dut.actual_position_deci!=600)
            $fatal(1,"FAIL stop changed actual POS or retained handoff history");
        release dut.position_offset_adc;release dut.u_outer_start.ramp_count;
        raw=-48'sd29491200;repeat(12)@(negedge clk);capture(-450);
        if(dut.position_target_deci!=-450 || dut.angle_target_code!=1403)
            $fatal(1,"FAIL repeated negative capture target");
        // S1进入编辑页时，NEW从锁定的负位置起步，按E3/F2仍可设置新的明确目标。
        force dut.s1_press=1;@(negedge clk);force dut.s1_press=0;repeat(3)@(negedge clk);
        if(dut.page!=1 || dut.position_edit_deci!=-450)$fatal(1,"FAIL edit starts at held capture target");
        force dut.e3_press=1;@(negedge clk);force dut.e3_press=0;repeat(3)@(negedge clk);f2();
        if(dut.position_target_deci!=-400 || dut.page!=1 || !dut.pid_active)
            $fatal(1,"FAIL explicit position edit after handoff");
        $display("PASS capture locks signed current POS,no encoder zero,300ms angle-only,50ms/1ADC authority ramp,stop/reentry and page editing");$finish;
    end
    initial begin #2000000;$fatal(1,"FAIL position handoff timeout");end
endmodule
