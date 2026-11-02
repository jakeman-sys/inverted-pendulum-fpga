// 速度外环：速度误差以 0.1 度/秒为单位，输出有符号 ADC 目标偏移码。
// Kp/Ki/Kd 是离散增益；微分先行用 C 系数实现：目标速度变化后的
// 下一次 PID 计算 C=0，随后恢复 C=1，避免设定值阶跃造成 D 项突变。
module speed_pid_outer #(
    parameter integer KP_X100 = 20,
    parameter integer KI_X100 = 0,
    parameter integer KD_X100 = 0,
    parameter integer PERIOD_CYCLES = 2_500_000,
    parameter integer MAX_OFFSET_ADC = 20,
    parameter integer INTEGRAL_LIMIT_DECI = 20_000,
    parameter integer OUT_INVERT = 0
) (
    input wire clk,
    input wire active,
    input wire signed [15:0] target_speed_dps,
    input wire signed [31:0] actual_speed_deci_dps,
    output reg signed [15:0] offset_adc = 16'sd0
);
    localparam [1:0] S_WAIT=2'd0, S_TERMS=2'd1,
                     S_SUM=2'd2, S_APPLY=2'd3;
    reg [1:0] state = S_WAIT;
    reg [31:0] period_count = 32'd0;
    reg signed [31:0] integral = 32'sd0;
    reg signed [31:0] previous_error = 32'sd0;
    reg signed [15:0] last_target_seen = 16'sd0;
    reg target_changed_pending = 1'b0;
    reg first_sample = 1'b1;

    wire signed [31:0] target_deci = $signed(target_speed_dps)*32'sd10;
    wire signed [31:0] error_raw = target_deci-actual_speed_deci_dps;
    wire signed [31:0] error_now =
        error_raw > 32'sd100_000 ? 32'sd100_000 :
        error_raw < -32'sd100_000 ? -32'sd100_000 : error_raw;
    wire signed [31:0] integral_raw = integral+error_now;
    wire signed [31:0] integral_limited =
        integral_raw > INTEGRAL_LIMIT_DECI ? INTEGRAL_LIMIT_DECI :
        integral_raw < -INTEGRAL_LIMIT_DECI ? -INTEGRAL_LIMIT_DECI : integral_raw;

    reg signed [31:0] sampled_error = 32'sd0;
    reg signed [31:0] sampled_integral = 32'sd0;
    reg signed [31:0] sampled_delta = 32'sd0;
    reg skip_derivative = 1'b1;
    reg signed [47:0] p_pipe=48'sd0, i_pipe=48'sd0, d_pipe=48'sd0;
    reg signed [47:0] command_q16=48'sd0;

    // X100 增益是每 1 度/秒误差对应 0.01 ADC 码；误差单位为 0.1 度/秒。
    localparam integer KP_Q16 = (KP_X100*65536+500)/1000;
    localparam integer KI_Q16 = (KI_X100*65536+500)/1000;
    localparam integer KD_Q16 = (KD_X100*65536+500)/1000;
    wire [31:0] p_abs_in = sampled_error[31] ? -sampled_error : sampled_error;
    wire [31:0] i_abs_in = sampled_integral[31] ? -sampled_integral : sampled_integral;
    wire [31:0] d_abs_in = sampled_delta[31] ? -sampled_delta : sampled_delta;
    wire [47:0] p_abs = p_abs_in*KP_Q16;
    wire [47:0] i_abs = i_abs_in*KI_Q16;
    wire [47:0] d_abs = d_abs_in*KD_Q16;
    wire signed [47:0] p_term = sampled_error[31] ? -$signed(p_abs) : $signed(p_abs);
    wire signed [47:0] i_term = sampled_integral[31] ? -$signed(i_abs) : $signed(i_abs);
    wire signed [47:0] d_term = skip_derivative ? 48'sd0 :
        sampled_delta[31] ? -$signed(d_abs) : $signed(d_abs);
    wire command_negative = command_q16[47];
    wire [47:0] command_abs = command_negative ? -command_q16 : command_q16;
    wire [47:0] command_rounded = (command_abs+48'd32768) >> 16;
    wire [15:0] output_abs = command_rounded > MAX_OFFSET_ADC ?
        MAX_OFFSET_ADC : command_rounded[15:0];
    wire output_negative = command_negative ^ (OUT_INVERT != 0);

    always @(posedge clk) begin
        if (!active) begin
            state <= S_WAIT;
            period_count <= 32'd0;
            integral <= 32'sd0;
            previous_error <= 32'sd0;
            last_target_seen <= target_speed_dps;
            target_changed_pending <= 1'b0;
            first_sample <= 1'b1;
            offset_adc <= 16'sd0;
        end else begin
            // 目标可能在两次 50 ms 计算之间变化，必须记住这个事件。
            if (target_speed_dps != last_target_seen) begin
                last_target_seen <= target_speed_dps;
                target_changed_pending <= 1'b1;
            end
            if (period_count == PERIOD_CYCLES-1)
                period_count <= 32'd0;
            else
                period_count <= period_count+1'b1;
            case (state)
                S_WAIT: if (period_count == PERIOD_CYCLES-1) begin
                    sampled_error <= error_now;
                    sampled_integral <= integral_limited;
                    sampled_delta <= error_now-previous_error;
                    skip_derivative <= first_sample || target_changed_pending ||
                        (target_speed_dps != last_target_seen);
                    target_changed_pending <= 1'b0;
                    state <= S_TERMS;
                end
                S_TERMS: begin
                    // 分拍乘法，避免一次穿过误差、PID 乘加和输出限幅。
                    p_pipe <= p_term;
                    i_pipe <= i_term;
                    d_pipe <= d_term;
                    state <= S_SUM;
                end
                S_SUM: begin
                    command_q16 <= p_pipe+i_pipe+d_pipe;
                    state <= S_APPLY;
                end
                S_APPLY: begin
                    offset_adc <= output_negative ?
                        -$signed(output_abs) : $signed(output_abs);
                    previous_error <= sampled_error;
                    first_sample <= 1'b0;
                    // 输出饱和且误差还在推动饱和时，不再积累积分。
                    if (!(command_rounded > MAX_OFFSET_ADC &&
                          command_negative == sampled_error[31]))
                        integral <= sampled_integral;
                    state <= S_WAIT;
                end
                default: state <= S_WAIT;
            endcase
        end
    end
endmodule
