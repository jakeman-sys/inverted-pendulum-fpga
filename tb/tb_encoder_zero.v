`timescale 1ns/1ps
module tb_encoder_zero;
    reg clk=0; always #10 clk=~clk;
    reg a=0,b=0,zero=0;
    wire signed [31:0] count,relative_deci,speed_deci;
    wire signed [47:0] relative_q16,raw_q16;
    quadrature_position #(.FILTER_CYCLES(3)) encoder (
        .clk(clk),.a(a),.b(b),.zero_position(zero),.count(count),
        .position_deci_deg(relative_deci),
        .position_q16(relative_q16),.raw_position_q16(raw_q16)
    );
    encoder_speed_estimator #(.PERIOD_CYCLES(12),.FILTER_SHIFT(1)) measure (
        .clk(clk),.zero_position(1'b0),.position_q16(raw_q16),
        .speed_deci_dps(speed_deci)
    );
    task ab_step; input aa,bb; begin
        a=aa; b=bb; repeat (8) @(negedge clk);
    end endtask
    initial begin
        repeat (4) @(negedge clk);
        // 单拍短毛刺应被滤除。
        b=1; @(negedge clk); b=0; repeat (8) @(negedge clk);
        if (count !== 0) $fatal(1,"encoder counted a short glitch");
        ab_step(0,1); ab_step(1,1); ab_step(1,0); ab_step(0,0);
        if (count !== 4 || relative_deci !== 14)
            $fatal(1,"encoder forward count failed");
        zero=1; @(negedge clk); zero=0; @(negedge clk);
        if (relative_deci !== 0 || raw_q16 !== 48'sd887452)
            $fatal(1,"F1 zero changed raw encoder accumulation");
        ab_step(0,1);
        if (relative_deci !== 3 || raw_q16 !== 48'sd1109315)
            $fatal(1,"encoder stopped accumulating after zero");
        $display("PASS encoder glitch rejection and origin-only zero");
        $finish;
    end
endmodule
