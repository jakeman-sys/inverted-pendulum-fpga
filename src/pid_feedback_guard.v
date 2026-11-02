// PID反馈资格：短暂坏批不重置PID；有界保留最后一次可信ADC。
// sample_good来自PID专用平均值资格，不再复用起摆用的64码跨度限制。
module pid_feedback_guard #(
    parameter integer CENTER_CODE=1403,WINDOW_CODE=350,
    parameter integer HOLD_CYCLES=100000 // 50MHz时2ms；设0禁用保留
)(
    input wire clk,rst_n,
    input wire [11:0] adc_code,
    input wire source_valid,sample_good,overrange,
    output wire [11:0] pid_adc,
    output wire feedback_ok,
    // 0正常，1平均码越过角度窗口，2持续坏批/无可信样本，3ADC未就绪，4OTR。
    output wire [2:0] reject_reason
);
    localparam integer LOW=CENTER_CODE-WINDOW_CODE,HIGH=CENTER_CODE+WINDOW_CODE;
    localparam integer AGE_WIDTH=HOLD_CYCLES<2 ? 1 : $clog2(HOLD_CYCLES+1);
    reg [AGE_WIDTH-1:0] invalid_age=0;
    reg [11:0] last_good_adc=0;
    reg have_good=0;
    wire in_window=adc_code>=LOW && adc_code<=HIGH;
    wire accept=source_valid && !overrange && sample_good && in_window;
    wire held=have_good && HOLD_CYCLES>0 && invalid_age<HOLD_CYCLES;
    always @(posedge clk)begin
        if(!rst_n)begin invalid_age<=0;last_good_adc<=0;have_good<=0;end
        else if(accept)begin invalid_age<=0;last_good_adc<=adc_code;have_good<=1;end
        else begin
            if(invalid_age<HOLD_CYCLES)invalid_age<=invalid_age+1'b1;
            // 越界/OTR不是可容忍噪声；不得继续用越界前的历史授权PID。
            if(!source_valid || overrange || !in_window)have_good<=0;
        end
    end
    assign pid_adc=accept ? adc_code : last_good_adc;
    assign reject_reason=!source_valid ? 3 : overrange ? 4 : !in_window ? 1 :
        !(accept || held) ? 2 : 0;
    assign feedback_ok=rst_n && reject_reason==0;
endmodule
