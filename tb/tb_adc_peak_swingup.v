`timescale 1ns/1ps
module tb_adc_peak_swingup;
    reg clk=0;always #10 clk=~clk;
    reg rst=0,start=0,stop=0,frame=0,good=0,electrical=1,pid_ok=1,raw_limit=0;
    reg [11:0] adc=3;reg signed [31:0] pos=0;
    wire [3:0] state,fault;wire run,pid,swing,forward,left,right;
    // 该单元另用窄窗口检查边界；顶层默认已恢复350。
    adc_peak_swingup #(.CAPTURE_WINDOW_CODE(150),.PULSE_CYCLES(20),.REVERSE_GAP_CYCLES(3),.TOTAL_TIMEOUT_CYCLES(5000),.DEAD_RETRY_CYCLES(1000)) dut(
        .clk(clk),.rst_n(rst),.start(start),.stop(stop),.frame_tick(frame),.adc_code(adc),.sample_good(good),
        .electrical_ok(electrical),.pid_feedback_ok(pid_ok),.pulse_ready(swing),.position_limit(raw_limit),.position_deci(pos),
        .state(state),.fault(fault),.running(run),.pid_enable(pid),.swing_enable(swing),.pulse_forward(forward),.left_peak(left),.right_peak(right));
    task take;input [11:0] code;input ok;begin
        @(negedge clk);adc=code;good=ok;frame=1;@(negedge clk);frame=0;end endtask
    task begin_run;begin @(negedge clk);start=1;@(negedge clk);start=0;end endtask
    task clear;begin @(negedge clk);stop=1;@(negedge clk);stop=0;end endtask
    task finish_pair;begin wait(state==4);@(negedge clk);end endtask
    integer i;
    initial begin
        repeat(3)@(negedge clk);rst=1;begin_run();
        if(state!=1 || !forward)$fatal(1,"FAIL initial escape direction");
        // 完整20周期正段，3周期断驱滑行，再进入完整20周期反段。
        for(i=0;i<19;i=i+1)begin @(negedge clk);if(state!=1 || !swing)$fatal(1,"FAIL short initial pulse");end
        @(negedge clk);if(state!=2 || swing)$fatal(1,"FAIL reverse gap entry");
        repeat(2)begin @(negedge clk);if(swing)$fatal(1,"FAIL short reverse gap");end
        @(negedge clk);if(state!=3 || forward || !swing)$fatal(1,"FAIL reverse after configured gap");
        finish_pair();
        take(2500,1);take(4,0);take(2700,0);take(2500,1);
        if(state!=4 || left || right || pid)$fatal(1,"FAIL endpoint jump created peak/capture");
        take(500,1);take(600,1);take(550,1);
        if(state!=1 || !forward)$fatal(1,"FAIL left-side maximum trigger");finish_pair();
        take(598,1);take(600,1);take(599,1);
        if(state!=4)$fatal(1,"FAIL repeated noise peak not suppressed");
        take(3,0);take(2300,1);take(2100,1);take(2150,1);
        if(state!=1 || forward)$fatal(1,"FAIL right-side minimum trigger");finish_pair();
        take(1403,0);take(1403,0);take(1403,0);
        if(pid)$fatal(1,"FAIL invalid average captured");
        take(1200,1);take(1200,1);take(1200,1);
        if(pid)$fatal(1,"FAIL lower old350 window must not capture new150 window");
        take(1600,1);take(1600,1);take(1600,1);
        if(pid)$fatal(1,"FAIL upper old350 window must not capture new150 window");
        take(1253,1);take(1553,1);if(pid)$fatal(1,"FAIL captured before three samples");take(1403,1);
        if(state!=6 || swing || !pid)$fatal(1,"FAIL three inclusive-window frames must capture PID");
        pid_ok=0;@(negedge clk);if(state!=7 || fault!=4 || run)$fatal(1,"FAIL PID fall did not stop");
        pid_ok=1;take(1403,1);take(1403,1);take(1403,1);if(run)$fatal(1,"FAIL fall auto restarted");
        clear();adc=1403;good=1;begin_run();if(state!=5 || swing)$fatal(1,"FAIL lifted startup must check before pulse");
        take(1403,1);take(1403,1);take(1403,1);if(!pid)$fatal(1,"FAIL lifted direct capture");
        clear();adc=3;good=0;begin_run();pos=1201;@(negedge clk);
        if(state!=7 || fault!=2)$fatal(1,"FAIL travel limit");
        clear();pos=0;electrical=0;begin_run();if(fault!=1 || run)$fatal(1,"FAIL electrical fault entry");
        clear();electrical=1;begin_run();finish_pair();repeat(1002)@(negedge clk);
        if(state!=1 && state!=3)$fatal(1,"FAIL deadzone retry missing");
        clear();begin_run();finish_pair();adc=500;good=1;repeat(5002)@(negedge clk);
        if(state!=7 || fault!=3 || run)$fatal(1,"FAIL finite total timeout");
        $display("PASS full pulse pair/coast gap,deadzone/wrap rejection,left/right extrema,rearm,three-frame PID capture,limits and no restart");$finish;
    end
    initial begin #500000;$fatal(1,"FAIL swing FSM timeout");end
endmodule
