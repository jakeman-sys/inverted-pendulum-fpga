// AD9220 并行接口：模块标成 D12 的引脚实际是 OTR 越量程指示。
// ADC 在 CLK 上升沿采集模拟输入；FPGA 延后半个时钟周期读数据，
// 避开 ADC 输出寄存器的传播延迟。
module reference_ad9220_capture (
    input  wire        clk_50m,
    input  wire [11:0] adc_data,
    input  wire        adc_otr,
    output reg         adc_clk = 1'b0,
    output reg  [11:0] average = 12'd0,
    output reg         overrange = 1'b0,
    output reg         valid = 1'b0
);
    // 50 MHz / (2 × 25) = 1 MHz，时钟高、低电平各保持 500 ns。
    reg [4:0] half_count = 5'd0;
    reg [3:0] warmup = 4'd0;   // 丢弃 ADC 上电后流水线尚未稳定的数据
    reg [5:0] sample_count = 6'd0;
    reg [17:0] sum = 18'd0;   // 64 × 4095 < 2^18
    reg otr_any = 1'b0;
    wire [17:0] next_sum = sum + {6'd0, adc_data};

    always @(posedge clk_50m) begin
        if (half_count == 5'd24) begin
            half_count <= 5'd0;
            adc_clk <= ~adc_clk;
            // adc_clk 的旧值为 1 时，本拍会把引脚拉低。
            // 此时距离上一个 ADC 上升沿约 500 ns，12 位数据已稳定。
            if (adc_clk) begin
                if (warmup != 4'd8) begin
                    warmup <= warmup + 1'b1;
                end else if (sample_count == 6'd63) begin
                    average <= (next_sum + 18'd32) >> 6; // 64 点四舍五入平均
                    overrange <= otr_any | adc_otr;
                    valid <= 1'b1;
                    sample_count <= 6'd0;
                    sum <= 18'd0;
                    otr_any <= 1'b0;
                end else begin
                    sample_count <= sample_count + 1'b1;
                    sum <= next_sum;
                    otr_any <= otr_any | adc_otr;
                end
            end
        end else begin
            half_count <= half_count + 1'b1;
        end
    end
endmodule
