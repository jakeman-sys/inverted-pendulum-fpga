// AD9220 并行接口：模块标成 D12 的引脚实际是 OTR 越量程指示。
// ADC 在 CLK 上升沿采集模拟输入；FPGA 延后半个时钟周期读数据，
// 避开 ADC 输出寄存器的传播延迟。
module ad9220_capture #(
    parameter integer DEAD_ADC_MAX=8, SENSOR_MAX_CODE=2708,
    parameter integer END_MARGIN_CODE=32, MAX_BATCH_SPREAD=64
) (
    input  wire        clk_50m,
    input  wire [11:0] adc_data,
    input  wire        adc_otr,
    output reg         adc_clk = 1'b0,
    output reg  [11:0] average = 12'd0,
    output reg         overrange = 1'b0,
    output reg         valid = 1'b0,
    output wire        feedback_valid,
    // 起摆极值/三帧判断用严格反馈；PID沿用手动版64点平均，单独排除跨周混批。
    output wire        pid_feedback_valid,
    output reg [11:0] batch_spread=0,
    // 本批OTR是否全部发生在低端死区码；混入任一非低端OTR便为0。
    // 供起摆状态区别“下垂端点欠量程”和其它越量程，不能恢复角度有效性。
    output reg         overrange_low_only=1'b0
);
    // 死区0~4：预留少量噪声余量。平均窗口跨越死区时，即便平均值
    // 恰好接近1404，也不能当成直立。检查所有原始点及整批最大/最小差。
    reg batch_bad=0, average_good=0, raw_bad=1;
    reg pid_average_good=0;
    reg [11:0] batch_min=4095,batch_max=0;
    wire current_bad=adc_data<=DEAD_ADC_MAX || adc_data>SENSOR_MAX_CODE+8 || adc_otr;
    wire [11:0] next_min=adc_data<batch_min ? adc_data : batch_min;
    wire [11:0] next_max=adc_data>batch_max ? adc_data : batch_max;
    // 0/2708两端混入同一批，均值会伪装成1403附近；这种批不能用于PID。
    // 不能把普通噪声的批内跨度>64等同于传感器跨周。
    wire wrap_mixed=next_min<=END_MARGIN_CODE && next_max>=SENSOR_MAX_CODE-END_MARGIN_CODE;
    assign feedback_valid=valid && average_good && !raw_bad && !overrange &&
        average>END_MARGIN_CODE && average<SENSOR_MAX_CODE-END_MARGIN_CODE;
    assign pid_feedback_valid=valid && pid_average_good && !overrange &&
        average>END_MARGIN_CODE && average<SENSOR_MAX_CODE-END_MARGIN_CODE;
    // 50 MHz / (2 × 25) = 1 MHz，时钟高、低电平各保持 500 ns。
    reg [4:0] half_count = 5'd0;
    reg [3:0] warmup = 4'd0;   // 丢弃 ADC 上电后流水线尚未稳定的数据
    reg [5:0] sample_count = 6'd0;
    reg [17:0] sum = 18'd0;   // 64 × 4095 < 2^18
    reg otr_any = 1'b0;
    reg otr_not_low_any=1'b0;
    wire otr_not_low_now=adc_otr && adc_data>DEAD_ADC_MAX;
    wire [17:0] next_sum = sum + {6'd0, adc_data};

    always @(posedge clk_50m) begin
        if (half_count == 5'd24) begin
            half_count <= 5'd0;
            adc_clk <= ~adc_clk;
            // adc_clk 的旧值为 1 时，本拍会把引脚拉低。
            // 此时距离上一个 ADC 上升沿约 500 ns，12 位数据已稳定。
            if (adc_clk) begin
                raw_bad<=current_bad;
                if (warmup != 4'd8) begin
                    warmup <= warmup + 1'b1;
                end else if (sample_count == 6'd63) begin
                    average <= (next_sum + 18'd32) >> 6; // 64 点四舍五入平均
                    overrange <= otr_any | adc_otr;
                    overrange_low_only<=(otr_any | adc_otr) && !(otr_not_low_any | otr_not_low_now);
                    valid <= 1'b1;
                    average_good<=!batch_bad && !current_bad &&
                        next_max-next_min<=MAX_BATCH_SPREAD;
                    // 按已验证手动工程使用64点平均，不用单个raw_bad或64码跨度退出PID。
                    // OTR由overrange挡住；端点平均码、角度窗口由后级继续检查。
                    pid_average_good<=!wrap_mixed;
                    batch_spread<=next_max-next_min;
                    batch_bad<=0;batch_min<=4095;batch_max<=0;
                    sample_count <= 6'd0;
                    sum <= 18'd0;
                    otr_any <= 1'b0;
                    otr_not_low_any<=1'b0;
                end else begin
                    batch_bad<=batch_bad || current_bad;
                    batch_min<=next_min;batch_max<=next_max;
                    sample_count <= sample_count + 1'b1;
                    sum <= next_sum;
                    otr_any <= otr_any | adc_otr;
                    otr_not_low_any<=otr_not_low_any | otr_not_low_now;
                end
            end
        end else begin
            half_count <= half_count + 1'b1;
        end
    end
endmodule
