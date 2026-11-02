`timescale 1ns/1ps
module tb_button_release;
    reg clk=0;always #10 clk=~clk;
    reg button=1;wire press,repeating,stable;
    integer presses=0,repeats=0;
    button_repeat #(.DEBOUNCE_CYCLES(5),.HOLD_CYCLES(12),.REPEAT_CYCLES(4)) dut(
        .clk(clk),.button(button),.press(press),.repeat_press(repeating),.stable(stable));
    always @(negedge clk)begin if(press)presses=presses+1;if(repeating)repeats=repeats+1;end
    initial begin
        repeat(40)@(posedge clk);#1;
        if(presses!=0 || repeats!=0)$fatal(1,"FAIL held power-up button activated");
        button=0;repeat(12)@(posedge clk);#1;button=1;repeat(12)@(posedge clk);#1;
        if(presses!=1)$fatal(1,"FAIL released then pressed %d",presses);
        repeat(25)@(posedge clk);#1;
        if(repeats<2)$fatal(1,"FAIL long press repeat");
        button=0;repeat(12)@(posedge clk);#1;
        $display("PASS held-on-powerup release guard, short press, long press");$finish;
    end
endmodule
