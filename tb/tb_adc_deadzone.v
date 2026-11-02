`timescale 1ns/1ps
module tb_adc_deadzone;
    reg clk=0;always #10 clk=~clk;
    reg [11:0] data=1404;reg otr=0;
    wire adc_clk,valid,good,pid_good,overrange,low_only;wire [11:0] average,spread;
    ad9220_capture dut(.clk_50m(clk),.adc_data(data),.adc_otr(otr),.adc_clk(adc_clk),
        .average(average),.valid(valid),.overrange(overrange),.feedback_valid(good),
        .pid_feedback_valid(pid_good),.batch_spread(spread),.overrange_low_only(low_only));
    initial begin
        repeat(5000)@(negedge clk);
        if(!valid || !good || !pid_good || average!=1404)$fatal(1,"FAIL valid calibrated ADC");
        data=3;repeat(100)@(negedge clk);
        if(good)$fatal(1,"FAIL deadzone must immediately invalidate previous average");
        repeat(6500)@(negedge clk);
        if(average!=3 || good || pid_good)$fatal(1,"FAIL 0~4 deadzone must remain invalid");
        data=1404;repeat(7000)@(negedge clk);
        if(!good)$fatal(1,"FAIL recovery after full good batches");
        wait(dut.sample_count==0);@(negedge clk);data=2708;
        repeat(32)@(negedge adc_clk);
        @(negedge clk);data=4;
        repeat(32)@(negedge adc_clk);#1;
        $display("ADC boundary mixed average=%d valid=%b feedback_valid=%b",average,valid,good);
        if(average<1000 || average>1700 || good || pid_good || spread!=2704)
            $fatal(1,"FAIL false upright average accepted");
        data=1404;repeat(7000)@(negedge clk);
        if(!good)$fatal(1,"FAIL recovered ADC after mixed batch");
        otr=1;repeat(100)@(negedge clk);
        if(good)$fatal(1,"FAIL raw OTR immediate invalidate");
        data=0;repeat(7000)@(negedge clk);
        if(!overrange || !low_only || good)$fatal(1,"FAIL low-end underrange classification");
        // 低端码与4095越量程混在一批内，不能被低端例外放行。
        wait(dut.sample_count==0);@(negedge clk);data=4095;
        repeat(2)@(negedge adc_clk);@(negedge clk);data=0;
        repeat(62)@(negedge adc_clk);#1;
        if(!overrange || low_only || good)$fatal(1,"FAIL mixed OTR classified as low-only");
        data=1404;repeat(7000)@(negedge clk);
        if(!overrange || low_only || good)$fatal(1,"FAIL non-endpoint OTR classification");
        otr=0;repeat(7000)@(negedge clk);
        if(overrange || low_only || !good)$fatal(1,"FAIL OTR recovery");
        $display("PASS ADC deadzone, mixed-window rejection, recovery, low/high/mixed OTR classification");$finish;
    end
    initial begin #2000000;$fatal(1,"FAIL ADC timeout");end
endmodule
