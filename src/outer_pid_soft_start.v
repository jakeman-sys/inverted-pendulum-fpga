module outer_pid_soft_start #(
    parameter integer DELAY_CYCLES=15000000,       // 50MHz下300ms
    parameter integer RAMP_PERIOD_CYCLES=2500000, // 50ms放开一步
    parameter integer RAMP_STEP_ADC=1,
    parameter integer MAX_OFFSET_ADC=25
)(
    input wire clk,rst_n,active,
    input wire signed [15:0] requested_offset_adc,
    output reg outer_ready=0,
    output reg signed [15:0] offset_adc=0
);
    localparam integer LIMIT=MAX_OFFSET_ADC<0 ? 0 :
        (MAX_OFFSET_ADC>32767 ? 32767 : MAX_OFFSET_ADC);
    localparam integer RAMP_PERIOD=RAMP_PERIOD_CYCLES<1 ? 1 : RAMP_PERIOD_CYCLES;
    reg [31:0] delay_count=0,ramp_count=0;
    reg [15:0] authority_adc=0;
    // 加一位做有符号比较，避免把负偏移当无符号大数。
    wire signed [16:0] command=$signed({requested_offset_adc[15],requested_offset_adc});
    wire signed [16:0] bound=$signed({1'b0,authority_adc});
    always @(posedge clk)begin
        if(!rst_n || !active)begin
            outer_ready<=0;offset_adc<=0;
            delay_count<=0;ramp_count<=0;authority_adc<=0;
        end else if(!outer_ready)begin
            offset_adc<=0;
            if(DELAY_CYCLES<=0 || delay_count>=DELAY_CYCLES-1)begin
                outer_ready<=1;delay_count<=0;ramp_count<=0;
                authority_adc<=RAMP_STEP_ADC<=0 ? LIMIT : 0;
            end else delay_count<=delay_count+1'b1;
        end else begin
            if(RAMP_STEP_ADC<=0)begin authority_adc<=LIMIT;ramp_count<=0;end
            else if(authority_adc<LIMIT)begin
                if(ramp_count>=RAMP_PERIOD-1)begin
                    ramp_count<=0;
                    authority_adc<=authority_adc+RAMP_STEP_ADC>=LIMIT ?
                        LIMIT : authority_adc+RAMP_STEP_ADC;
                end else ramp_count<=ramp_count+1'b1;
            end
            // 两级原PID输出先受逐步增长的限幅约束；放开到最大后沿用原输出。
            // 输出再寄存一拍，避免把比较器接到内环乘加的关键路径上。
            if(command>bound)offset_adc<=$signed(authority_adc);
            else if(command< -bound)offset_adc<=-$signed(authority_adc);
            else offset_adc<=requested_offset_adc;
        end
    end
endmodule
