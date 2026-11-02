// 用编码器位置差测输出轴速度，单位是 0.1 度/秒。
// 默认每 50 ms 采样一次，再用一阶低通减少单个编码器计数造成的跳动。
// 若单独修改采样周期，须同时把 SAMPLE_RATE_HZ 改为对应的每秒采样次数。
module encoder_speed_estimator #(
    parameter integer PERIOD_CYCLES = 2_500_000,
    parameter integer SAMPLE_RATE_HZ = 20,
    parameter integer FILTER_SHIFT = 2 // 1/4 新值 + 3/4 旧值
) (
    input wire clk,
    input wire zero_position,
    input wire signed [47:0] position_q16, // 0.1 度 × 65536
    output wire signed [31:0] speed_deci_dps
);
    reg [31:0] period_count = 32'd0;
    reg signed [47:0] last_position_q16 = 48'sd0;
    reg signed [47:0] filtered_speed_q16 = 48'sd0;
    reg signed [47:0] delta_sampled_q16 = 48'sd0;
    reg signed [47:0] raw_speed_q16 = 48'sd0;
    reg scale_pending = 1'b0, filter_pending = 1'b0;

    // 先寄存位置差，下一拍再乘采样频率；避免综合器把有符号
    // (当前位置-上次位置) 和乘法合并成曾导致符号错误的 DSP 预加器。
    wire signed [47:0] speed_scaled_q16 =
        delta_sampled_q16*SAMPLE_RATE_HZ;
    wire signed [47:0] filter_delta = raw_speed_q16-filtered_speed_q16;

    always @(posedge clk) begin
        if (zero_position) begin
            period_count <= 32'd0;
            last_position_q16 <= 48'sd0;
            filtered_speed_q16 <= 48'sd0;
            delta_sampled_q16 <= 48'sd0;
            raw_speed_q16 <= 48'sd0;
            scale_pending <= 1'b0;
            filter_pending <= 1'b0;
        end else if (period_count == PERIOD_CYCLES-1) begin
            period_count <= 32'd0;
            delta_sampled_q16 <= position_q16-last_position_q16;
            last_position_q16 <= position_q16;
            scale_pending <= 1'b1;
        end else
            period_count <= period_count+1'b1;

        if (!zero_position) begin
            if (scale_pending) begin
                raw_speed_q16 <= speed_scaled_q16;
                scale_pending <= 1'b0;
                filter_pending <= 1'b1;
            end
            if (filter_pending) begin
            filtered_speed_q16 <= filtered_speed_q16 +
                    (filter_delta >>> FILTER_SHIFT);
                filter_pending <= 1'b0;
            end
        end
    end

    wire [47:0] speed_abs_q16 = filtered_speed_q16[47] ?
        -filtered_speed_q16 : filtered_speed_q16;
    wire [31:0] speed_abs_deci = (speed_abs_q16+48'd32768) >> 16;
    assign speed_deci_dps = filtered_speed_q16[47] ?
        -$signed(speed_abs_deci) : $signed(speed_abs_deci);
endmodule
