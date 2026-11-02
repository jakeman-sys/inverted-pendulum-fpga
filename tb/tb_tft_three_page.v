`timescale 1ns/1ps
module tb_tft_three_page;
    reg clk=0; always #10 clk=~clk;
    wire cs_n,sclk,mosi,dc,rst_n,bl;
    tft_cascade_display dut (
        .clk(clk),.actual_adc(12'd646),.target_adc(12'd650),
        .pid_active(1'b1),.ain1(1'b1),.ain2(1'b0),
        .page(2'd2),.speed_mode(1'b1),
        .control_state(4'd6),.fault_code(4'd0),.fault_detail(3'd0),.fault_spread(12'd0),.recovery_attempts(4'd0),
        .position_target_deci(32'sd123),
        .position_edit_deci(32'sd133),
        .position_actual_deci(32'sd120),
        .speed_target_dps(16'sd12),.speed_edit_dps(16'sd15),
        .speed_actual_deci(32'sd97),
        .tft_cs_n(cs_n),.tft_sclk(sclk),.tft_mosi(mosi),
        .tft_dc(dc),.tft_rst_n(rst_n),.tft_bl(bl)
    );
    reg reached_pixel=0;
    always @(posedge clk)
        if (dut.state == 4'd7 || dut.state == 4'd8)
            reached_pixel <= 1'b1;
    initial begin
        repeat (2) @(negedge clk);
        force dut.startup_count=16'd50000;
        // 快速进入 BCD 帧准备阶段，给快照寄存器设定固定测试值。
        force dut.frame_adc=12'd646;
        force dut.frame_target_adc=12'd650;
        force dut.frame_set_deci=32'sd123;
        force dut.frame_new_deci=32'sd133;
        force dut.frame_pos_deci=32'sd120;
        force dut.frame_speed_set_dps=16'sd12;
        force dut.frame_speed_new_dps=16'sd15;
        force dut.frame_speed_actual_deci=32'sd97;
        force dut.frame_page=2'd2;
        force dut.frame_speed_mode=1'b1;
        force dut.frame_state=4'd6;
        force dut.state=4'd5; #1; release dut.state;
        repeat (45) @(negedge clk);
        if (!reached_pixel) $fatal(1,"TFT frame did not advance");
        if (dut.speed_set_bcd !== 20'h00120 ||
            dut.speed_new_bcd !== 20'h00150 ||
            dut.speed_actual_bcd !== 20'h00097)
            $fatal(1,"TFT speed numbers incorrect");
        if (dut.line_char(4'd0,4'd0,2'd2) !== "S" ||
            dut.line_char(4'd0,4'd1,2'd2) !== "P" ||
            dut.line_char(4'd0,4'd2,2'd2) !== "D" ||
            dut.line_char(4'd10,4'd1,2'd2) !== "2" ||
            dut.line_char(4'd10,4'd3,2'd0) !== "9" ||
            dut.line_char(4'd10,4'd4,2'd0) !== "I" ||
            dut.line_char(4'd10,4'd12,2'd0) !== "V")
            $fatal(1,"TFT page label/mode/version incorrect");
        force dut.frame_state=4'd0;
        if(dut.line_char(10,13,0)!="9" || dut.line_char(10,14,0)!="I")
            $fatal(1,"TFT waiting version marker incorrect");
        force dut.frame_recovery_attempts=3;force dut.frame_state=8;
        if(dut.line_char(10,0,0)!="F" || dut.line_char(10,3,0)!="L" || dut.line_char(10,8,0)!="3")
            $fatal(1,"TFT fall/retry status incorrect");
        force dut.frame_state=9;
        if(dut.line_char(10,0,0)!="D" || dut.line_char(10,3,0)!="N")$fatal(1,"TFT down wait status incorrect");
        force dut.frame_state=10;
        if(dut.line_char(10,0,0)!="S" || dut.line_char(10,4,0)!="T" || dut.line_char(10,8,0)!="3")
            $fatal(1,"TFT immediate restart status incorrect");
        force dut.frame_state=7;force dut.frame_fault=5;force dut.frame_fault_detail=1;
        if(dut.line_char(10,13,0)!="R")$fatal(1,"TFT recovery exhaustion reason incorrect");
        force dut.frame_fault_detail=2;
        if(dut.line_char(10,13,0)!="T")$fatal(1,"TFT recovery timeout reason incorrect");
        $display("PASS TFT three-page data,PID9I,START/FALL/DOWN counts and F5 reasons");
        $finish;
    end
endmodule
