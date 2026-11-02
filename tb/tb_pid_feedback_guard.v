`timescale 1ns/1ps
module tb_pid_feedback_guard;
    reg clk=0;always #10 clk=~clk;
    reg rst_n=0,valid=1,good=0,otr=0;
    reg [11:0] adc=1403;
    wire ok;wire [11:0] value;wire [2:0] reason;
    pid_feedback_guard #(.HOLD_CYCLES(12)) dut(.clk(clk),.rst_n(rst_n),.adc_code(adc),
        .source_valid(valid),.sample_good(good),.overrange(otr),.pid_adc(value),
        .feedback_ok(ok),.reject_reason(reason));
    task ticks;input integer n;begin repeat(n)@(negedge clk);end endtask
    initial begin
        ticks(2);rst_n=1;ticks(2);
        if(ok || reason!=2)$fatal(1,"FAIL must not invent first trustworthy sample");
        good=1;adc=1393;ticks(2);
        if(!ok || value!=1393)$fatal(1,"FAIL live valid sample");
        good=0;adc=1420;#1;
        if(!ok || value!=1393)$fatal(1,"FAIL invalid batch must hold old code");
        // 原版直接使用good时，这一时刻已退出PID；新版短暂毛刺仍有资格。
        ticks(11);if(!ok)$fatal(1,"FAIL hold duration too short");
        ticks(1);if(ok || reason!=2)$fatal(1,"FAIL expired ADC must disable PID");
        good=1;adc=1413;ticks(2);good=0;adc=1400;ticks(4);good=1;ticks(1);
        if(!ok || value!=1400)$fatal(1,"FAIL recovered sample not used");
        adc=1754;#1;if(ok || reason!=1)$fatal(1,"FAIL outside-window cannot be held");
        ticks(2);adc=1403;good=0;#1;
        if(ok)$fatal(1,"FAIL window reentry cannot reuse pre-fall sample");
        good=1;ticks(2);otr=1;#1;
        if(ok || reason!=4)$fatal(1,"FAIL OTR must disable immediately");
        ticks(2);otr=0;good=0;#1;if(ok)$fatal(1,"FAIL OTR clears history");
        good=1;ticks(2);valid=0;#1;
        if(ok || reason!=3)$fatal(1,"FAIL source not ready");
        $display("PASS PID feedback guard: bounded hold, recovery, strict window/OTR, no invented or stale feedback");$finish;
    end
endmodule
