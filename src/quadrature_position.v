// 霍尔编码器 A/B 两相四倍频。13 PPR × 20.45 减速比 × 4 =
// 输出轴每转 1063.4 计数；每计数约 0.3385368°。
// pos_q16 的单位为 0.1° × 2^16，步长 221863 来自上述比例。
module quadrature_position #(
    parameter integer ENCODER_INVERT = 0,
    // 50 MHz 时 100 拍 = 2 us。A/B 必须持续稳定才采纳，抑制电机 PWM 毛刺。
    parameter integer FILTER_CYCLES = 100
) (
    input  wire clk,
    input  wire a,
    input  wire b,
    input  wire zero_position,
    output reg signed [31:0] count = 32'sd0,
    output wire signed [31:0] position_deci_deg,
    output wire signed [47:0] position_q16,
    output wire signed [47:0] raw_position_q16
);
    reg a_meta = 1'b0, b_meta = 1'b0;
    reg a_sync = 1'b0, b_sync = 1'b0;
    reg a_filtered = 1'b0, b_filtered = 1'b0;
    reg [15:0] a_stable_count = 16'd0, b_stable_count = 16'd0;
    reg [1:0] previous = 2'b00;
    reg signed [47:0] pos_q16 = 48'sd0;
    reg signed [47:0] zero_origin_q16 = 48'sd0;
    localparam signed [47:0] STEP_Q16 = 48'sd221863;

    always @(posedge clk) begin
        a_meta <= a;
        b_meta <= b;
        a_sync <= a_meta;
        b_sync <= b_meta;
        // 两相分别滤波；任一相反跳回原电平时，稳定计时立即重新开始。
        if (a_sync == a_filtered)
            a_stable_count <= 16'd0;
        else if (a_stable_count == FILTER_CYCLES-1) begin
            a_filtered <= a_sync;
            a_stable_count <= 16'd0;
        end else
            a_stable_count <= a_stable_count + 1'b1;

        if (b_sync == b_filtered)
            b_stable_count <= 16'd0;
        else if (b_stable_count == FILTER_CYCLES-1) begin
            b_filtered <= b_sync;
            b_stable_count <= 16'd0;
        end else
            b_stable_count <= b_stable_count + 1'b1;
        previous <= {a_filtered,b_filtered};
        // F1 只改变位置坐标原点；原始累计量继续用于速度估计，
        // 因而运行中置零不会在速度环中造成虚假的高速脉冲。
        if (zero_position)
            zero_origin_q16 <= pos_q16;

        // 上电时以当前位置为 0°；运行中持续累计，停机和再次入环均不清零。
        // 仅接受一次只变化一相的合法格雷码跳变，丢弃同时变化的毛刺。
        case ({previous,a_filtered,b_filtered})
            4'b0001,4'b0111,4'b1110,4'b1000: begin
                count <= ENCODER_INVERT ? count-1'b1 : count+1'b1;
                pos_q16 <= ENCODER_INVERT ?
                    pos_q16-STEP_Q16 : pos_q16+STEP_Q16;
            end
            4'b0010,4'b1011,4'b1101,4'b0100: begin
                count <= ENCODER_INVERT ? count+1'b1 : count-1'b1;
                pos_q16 <= ENCODER_INVERT ?
                    pos_q16+STEP_Q16 : pos_q16-STEP_Q16;
            end
            default: begin end
        endcase
    end

    assign raw_position_q16 = pos_q16;
    assign position_q16 = pos_q16-zero_origin_q16;
    wire [47:0] abs_q16 = position_q16[47] ? -position_q16 : position_q16;
    wire [31:0] abs_deci = (abs_q16 + 48'd32768) >> 16;
    assign position_deci_deg = position_q16[47] ?
        -$signed(abs_deci) : $signed(abs_deci);
endmodule
