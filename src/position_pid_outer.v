// 位置外环：位置误差单位 0.1°，输出为有符号 ADC 码偏移。
// 增益以百分之一为单位，OUT_KP_X100=100 表示位置差 1° 产生 1 ADC 码。
// Ki/Kd 是每次采样的离散增益，变更 PERIOD_CYCLES 后需要重调。
module position_pid_outer #(
    parameter integer OUT_KP_X100 = 100,
    parameter integer OUT_KI_X100 = 0,
    parameter integer OUT_KD_X100 = 0,
    parameter integer MAX_OFFSET_ADC = 20,
    parameter integer PERIOD_CYCLES = 2_500_000, // 50 MHz 时 50 ms
    parameter integer INTEGRAL_SEP_DEG = 10,
    parameter integer INTEGRAL_LIMIT_DECI = 20_000,
    parameter integer OUT_INVERT = 0
) (
    input wire clk,
    input wire active,
    input wire signed [31:0] target_deci_deg,
    input wire signed [31:0] actual_deci_deg,
    output reg signed [15:0] offset_adc = 16'sd0
);
    reg [21:0] period_count = 22'd0;
    reg signed [31:0] integral = 32'sd0;
    reg signed [31:0] previous_error = 32'sd0;
    // 编码器换算、位置误差和 PID 乘加分拍完成；50 ms 更新时取已稳定值。
    reg signed [31:0] actual_sampled = 32'sd0;
    reg signed [31:0] error = 32'sd0;
    wire signed [31:0] error_raw = target_deci_deg - actual_sampled;
    // 限制极端计数或接线异常时的中间数值，避免乘法溢出。
    wire signed [31:0] error_clamped =
        error_raw > 32'sd100000 ? 32'sd100000 :
        error_raw < -32'sd100000 ? -32'sd100000 : error_raw;
    always @(posedge clk) begin
        actual_sampled <= actual_deci_deg;
        error <= error_clamped;
    end
    wire [31:0] error_abs = error[31] ? -error : error;
    wire integral_allowed = error_abs <= INTEGRAL_SEP_DEG*10;
    wire signed [31:0] integral_raw = integral + error;
    wire signed [31:0] integral_clamped =
        integral_raw > INTEGRAL_LIMIT_DECI ? INTEGRAL_LIMIT_DECI :
        integral_raw < -INTEGRAL_LIMIT_DECI ? -INTEGRAL_LIMIT_DECI :
        integral_raw;
    wire signed [31:0] integral_for_calc = integral_allowed ?
        integral_clamped : 32'sd0;
    wire signed [31:0] delta = error - previous_error;
    wire [31:0] integral_abs = integral_for_calc[31] ?
        -integral_for_calc : integral_for_calc;
    wire [31:0] delta_abs = delta[31] ? -delta : delta;

    // Q16 增益：error 是 0.1°，X100 是 0.01 ADC码/°，故分母为 1000。
    // 三项均先乘正幅值再恢复符号，规避内环曾遇到的 DSP 符号映射问题。
    localparam integer KP_Q16 = (OUT_KP_X100*65536+500)/1000;
    localparam integer KI_Q16 = (OUT_KI_X100*65536+500)/1000;
    localparam integer KD_Q16 = (OUT_KD_X100*65536+500)/1000;
    wire [47:0] p_abs = error_abs * KP_Q16;
    wire [47:0] i_abs = integral_abs * KI_Q16;
    wire [47:0] d_abs = delta_abs * KD_Q16;
    wire signed [47:0] p_term = error[31] ? -$signed(p_abs) : $signed(p_abs);
    wire signed [47:0] i_term = integral_for_calc[31] ?
        -$signed(i_abs) : $signed(i_abs);
    wire signed [47:0] d_term = delta[31] ? -$signed(d_abs) : $signed(d_abs);
    reg signed [47:0] p_pipe = 48'sd0, i_pipe = 48'sd0, d_pipe = 48'sd0;
    reg signed [47:0] command_q16 = 48'sd0;
    always @(posedge clk) begin
        p_pipe <= p_term;
        i_pipe <= i_term;
        d_pipe <= d_term;
        command_q16 <= p_pipe + i_pipe + d_pipe;
    end
    wire command_negative = command_q16[47];
    wire [47:0] command_abs = command_negative ? -command_q16 : command_q16;
    wire [47:0] command_rounded = (command_abs + 48'd32768) >> 16;
    wire [15:0] offset_abs = command_rounded > MAX_OFFSET_ADC ?
        MAX_OFFSET_ADC : command_rounded[15:0];
    wire offset_negative = command_negative ^ (OUT_INVERT != 0);

    always @(posedge clk) begin
        if (!active) begin
            period_count <= 22'd0;
            integral <= 32'sd0;
            previous_error <= 32'sd0;
            offset_adc <= 16'sd0;
        end else if (period_count == PERIOD_CYCLES-1) begin
            period_count <= 22'd0;
            previous_error <= error;
            // 积分分离：误差大于界限时清零；输出已饱和且误差继续推大
            // 积分时冻结，防止恢复时持续反向超调。
            if (!integral_allowed)
                integral <= 32'sd0;
            else if (!(command_rounded > MAX_OFFSET_ADC &&
                       (command_negative == error[31])))
                integral <= integral_clamped;
            offset_adc <= offset_negative ?
                -$signed(offset_abs) : $signed(offset_abs);
        end else
            period_count <= period_count + 1'b1;
    end
endmodule
