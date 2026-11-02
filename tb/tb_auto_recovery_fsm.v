`timescale 1ns/1ps
// 加速计数器，覆盖首次倒下、连续失败、稳定清次数和人工/硬故障优先。
module tb_auto_recovery_fsm;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,start=0,stop=0,tick=0,good=1,electrical=1,pid_ok=1,fall=0,ready=0,limit=0;
    reg [11:0] adc=1403;reg signed [31:0] pos=0;
    wire [3:0] state,fault,tries;wire [1:0] why;
    wire running,pid,swing,pending;
    adc_peak_swingup #(.AUTO_RECOVER_ENABLE(1),.RECOVER_STOP_CYCLES(5),.RECOVER_READY_CYCLES(4),
        .RECOVER_WAIT_TIMEOUT_CYCLES(80),.RECOVER_MAX_ATTEMPTS(3),.RECOVER_STABLE_CYCLES(30),
        .PULSE_CYCLES(20),.REVERSE_GAP_CYCLES(2),.TOTAL_TIMEOUT_CYCLES(1000),.DEAD_RETRY_CYCLES(0)) dut(
        .clk(clk),.rst_n(rst),.start(start),.stop(stop),.frame_tick(tick),.adc_code(adc),.sample_good(good),
        .electrical_ok(electrical),.pid_feedback_ok(pid_ok),.pulse_ready(swing),.position_limit(limit),
        .fall_window_exit(fall),.recovery_down_ready(ready),.position_deci(pos),.state(state),.fault(fault),
        .running(running),.pid_enable(pid),.swing_enable(swing),.recovery_pending(pending),
        .recovery_attempts(tries),.recovery_fault_reason(why),.pulse_forward(),.left_peak(),.right_peak(),.limit_reason());
    task take;begin @(negedge clk);adc=1403;good=1;pid_ok=1;fall=0;tick=1;@(negedge clk);tick=0;end endtask
    task capture;begin take();take();take();if(!pid)$fatal(1,"FAIL capture during recovery test");end endtask
    task arm;begin
        @(negedge clk);stop=1;electrical=1;limit=0;pid_ok=1;fall=0;ready=0;adc=1403;good=1;
        @(negedge clk);stop=0;start=1;@(negedge clk);start=0;capture();
    end endtask
    task drop;begin
        @(negedge clk);adc=1754;pid_ok=0;fall=1;ready=0;
        @(negedge clk);if(state!=8 || pid || swing || !pending)$fatal(1,"FAIL fall must stop before waiting");
    end endtask
    task restart_auto;input integer expected;begin
        adc=3;good=0;ready=1;wait(state==1);@(negedge clk);
        if(tries!=expected || !swing || dut.start_position!=pos)$fatal(1,"FAIL auto attempts/start origin");
    end endtask
    integer n;
    initial begin
        repeat(3)@(negedge clk);rst=1;ready=1;repeat(10)@(negedge clk);
        if(running)$fatal(1,"FAIL powerup auto armed without button");
        arm();drop();pos=10000; // 停驱等待时不使用旧的±120起摆参考。
        repeat(4)begin @(negedge clk);if(state!=8 || swing)$fatal(1,"FAIL stop interval too short");end
        @(negedge clk);if(state!=9)$fatal(1,"FAIL stop interval boundary");
        repeat(10)@(negedge clk);if(state!=9 || tries!=0)$fatal(1,"FAIL no ready condition yet");
        restart_auto(1);capture();drop();restart_auto(2);capture();drop();restart_auto(3);capture();
        @(negedge clk);adc=1052;pid_ok=0;fall=1;@(negedge clk);
        if(state!=7 || fault!=5 || why!=1 || running)$fatal(1,"FAIL max3 retries must latch F5 R");
        ready=1;repeat(100)@(negedge clk);if(running)$fatal(1,"FAIL retry exhaustion restarted itself");
        arm();drop();wait(state==7);@(negedge clk);
        if(fault!=5 || why!=2)$fatal(1,"FAIL waiting down timeout F5 T");
        // 自动起摆超时可进入下一次有界重试，初次手动起摆超时仍用原F3。
        arm();drop();restart_auto(1);ready=0;adc=500;good=1;wait(state==8);@(negedge clk);
        if(fault!=3 || tries!=1)$fatal(1,"FAIL failed auto swing must wait before retry");
        restart_auto(2);ready=0;adc=500;good=1;wait(state==8);restart_auto(3);
        ready=0;adc=500;good=1;wait(state==7);if(fault!=5 || why!=1)$fatal(1,"FAIL auto swing timeouts exhausted count");
        arm();drop();restart_auto(1);capture();repeat(30)@(negedge clk);
        if(tries!=0)$fatal(1,"FAIL sustained PID did not reset failed count");
        drop();restart_auto(1);capture();
        pid_ok=0;fall=0;@(negedge clk);if(state!=7 || fault!=4)$fatal(1,"FAIL ADC validity fault auto restarted");
        arm();drop();electrical=0;@(negedge clk);if(state!=7 || fault!=1)$fatal(1,"FAIL electrical fault in recovery");
        arm();drop();limit=1;@(negedge clk);if(state!=7 || fault!=2)$fatal(1,"FAIL position fault in recovery");
        arm();drop();stop=1;@(negedge clk);stop=0;ready=1;repeat(100)@(negedge clk);
        if(state!=0 || tries!=0 || swing || pid)$fatal(1,"FAIL manual stop must cancel recovery");
        $display("PASS fall-stop/down/auto-PID,3-retry cap,wait timeout,auto swing timeout,retry reset after stable,no unarmed/ADC/electrical/limit/manual restart");$finish;
    end
    initial begin #500000;$fatal(1,"FAIL auto recovery FSM timeout");end
endmodule
