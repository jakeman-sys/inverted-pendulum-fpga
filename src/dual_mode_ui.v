// 三页操作：主页、位置目标页、速度目标页。页面切换不改变正在运行的外环。
// 只有在相应页面按 F2 应用目标后，才切换到位置或速度控制模式。
module dual_mode_ui #(
    parameter integer SPEED_LIMIT_DPS = 360,
    // 自动起摆接管时锁住当前位置；默认0兼容原手动工程的零位置目标。
    parameter integer HOLD_CURRENT_ON_START = 0
) (
    input wire clk,
    input wire active,
    input wire started,
    input wire s1_press,
    input wire s2_press,
    input wire f2_press,
    input wire f1_press,
    input wire f1_repeat,
    input wire e3_press,
    input wire e3_repeat,
    input wire signed [31:0] actual_position_deci,
    output reg [1:0] page = 2'd0, // 0:主页 1:位置页 2:速度页
    output reg speed_mode = 1'b0,
    output reg zero_position = 1'b0,
    output reg signed [31:0] target_position_deci = 32'sd0,
    output reg signed [31:0] edit_position_deci = 32'sd0,
    output reg signed [15:0] target_speed_dps = 16'sd0,
    output reg signed [15:0] edit_speed_dps = 16'sd0
);
    wire minus_step = f1_press || f1_repeat;
    wire plus_step = e3_press || e3_repeat;
    // 每次按键的步进：位置页 5.0°（50×0.1°），速度页 5 dps。
    localparam signed [31:0] POSITION_STEP_DECI = 32'sd50;
    localparam signed [15:0] SPEED_STEP_DPS = 16'sd5;

    always @(posedge clk) begin
        zero_position <= 1'b0;
        if (!active || started) begin
            page <= 2'd0;
            speed_mode <= 1'b0;
            target_position_deci <= started && HOLD_CURRENT_ON_START!=0 ?
                actual_position_deci : 32'sd0;
            edit_position_deci <= started && HOLD_CURRENT_ON_START!=0 ?
                actual_position_deci : 32'sd0;
            target_speed_dps <= 16'sd0;
            edit_speed_dps <= 16'sd0;
            // 停机时也允许在主页按 F1 建立新的输出轴零点。
            if (!active && f1_press)
                zero_position <= 1'b1;
        end else if (s1_press) begin
            if (page == 2'd1)
                page <= 2'd0;
            else begin
                page <= 2'd1;
                // 从速度模式返回位置设置时，以当前位置作编辑起点。
                edit_position_deci <= speed_mode ?
                    actual_position_deci : target_position_deci;
            end
        end else if (s2_press) begin
            if (page == 2'd2)
                page <= 2'd0;
            else begin
                page <= 2'd2;
                edit_speed_dps <= target_speed_dps;
            end
        end else begin
            case (page)
                2'd0: if (f1_press) begin
                    // 只重设编码器位置坐标；速度来自原始累计位置，不受影响。
                    zero_position <= 1'b1;
                    target_position_deci <= 32'sd0;
                    edit_position_deci <= 32'sd0;
                end
                2'd1: if (f2_press) begin
                    target_position_deci <= edit_position_deci;
                    target_speed_dps <= 16'sd0;
                    speed_mode <= 1'b0;
                end else if (minus_step && !plus_step &&
                             edit_position_deci > -32'sd99_990)
                    edit_position_deci <= edit_position_deci-POSITION_STEP_DECI;
                else if (plus_step && !minus_step &&
                         edit_position_deci < 32'sd99_990)
                    edit_position_deci <= edit_position_deci+POSITION_STEP_DECI;
                2'd2: if (f2_press) begin
                    target_speed_dps <= edit_speed_dps;
                    if (edit_speed_dps == 16'sd0) begin
                        // 速度设为 0：退出速度环，位置环锁住切换这一刻的位置。
                        target_position_deci <= actual_position_deci;
                        speed_mode <= 1'b0;
                    end else
                        speed_mode <= 1'b1;
                end else if (minus_step && !plus_step &&
                             edit_speed_dps > -SPEED_LIMIT_DPS)
                    edit_speed_dps <= edit_speed_dps-SPEED_STEP_DPS;
                else if (plus_step && !minus_step &&
                         edit_speed_dps < SPEED_LIMIT_DPS)
                    edit_speed_dps <= edit_speed_dps+SPEED_STEP_DPS;
                default: page <= 2'd0;
            endcase
        end
    end
endmodule
