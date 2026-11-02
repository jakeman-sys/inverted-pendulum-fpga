// 起摆脉冲的PWM入口。方向跟随状态机；换向滑行间隔由状态机关闭enable实现。
// 45%可从顶层修改，但始终钳位到60%；不改变原PID模块的换向逻辑。
module swing_pulse_pwm #(
    parameter integer PWM_PERIOD=2500,DUTY_PERCENT=45
)(input wire clk,rst_n,enable,forward,
  output wire pwma,ain1,ain2,stby);
    localparam integer PERCENT=DUTY_PERCENT<0 ? 0 : DUTY_PERCENT>60 ? 60 : DUTY_PERCENT;
    localparam integer DUTY=PWM_PERIOD*PERCENT/100;
    reg [11:0] counter=0;
    always @(posedge clk)begin
        if(!rst_n || !enable)counter<=0;
        else if(counter==PWM_PERIOD-1)counter<=0;
        else counter<=counter+1'b1;
    end
    assign stby=rst_n && enable && DUTY!=0;
    assign pwma=stby && counter<DUTY;
    assign ain1=stby && forward;
    assign ain2=stby && !forward;
endmodule
