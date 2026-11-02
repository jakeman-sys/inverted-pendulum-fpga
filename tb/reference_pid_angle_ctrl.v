// 角度内环：ADC 原码为反馈，目标码由位置外环在直立中心码上叠加偏移。
// 增益使用百分之一为单位：KP_X100=100 表示 Kp=1.00。
// 离散公式：PWM计数 = Kp*误差 + Ki*误差累加 + Kd*(本次误差-上次误差)。
// Ki/Kd 是以 PID_PERIOD_CYCLES 为采样周期的离散增益，改采样周期后须重调。
module reference_pid_angle_ctrl #(
    parameter integer CENTER_CODE = 2048,
    parameter integer WINDOW_CODE = 200,
    parameter integer KP_X100 = 500,
    parameter integer KI_X100 = 0,
    parameter integer KD_X100 = 0,
    parameter integer MOTOR_INVERT = 0,
    parameter integer PID_PERIOD_CYCLES = 250_000, // 50 MHz 时 5 ms / 200 Hz
    parameter integer PWM_PERIOD = 2500,          // 20 kHz
    parameter integer MAX_PWM_TICKS = 500,        // 初始限幅 20%
    parameter integer DEBOUNCE_CYCLES = 1_000_000,// F2 消抖 20 ms
    parameter integer INTEGRAL_LIMIT = 20_000
) (
    input  wire        clk,
    input  wire        button,
    input  wire [11:0] target_code,
    input  wire [11:0] adc_code,
    input  wire        adc_valid,
    input  wire        adc_otr,
    output reg         pid_active = 1'b0,
    output wire        tb_pwma,
    output wire        tb_ain1,
    output wire        tb_ain2,
    output wire        tb_stby
);
    wire [11:0] TARGET = target_code;
    localparam integer LOW_CODE =
        CENTER_CODE > WINDOW_CODE ? CENTER_CODE-WINDOW_CODE : 0;
    localparam integer HIGH_CODE =
        CENTER_CODE+WINDOW_CODE < 4095 ? CENTER_CODE+WINDOW_CODE : 4095;
    wire in_window = (adc_code >= LOW_CODE) && (adc_code <= HIGH_CODE);
    wire feedback_ok = adc_valid && !adc_otr && in_window;

    // F2 为高电平按下。只有看见一次稳定松开，新的按下沿才可申请入环。
    // 这样按住 F2 上电不会使电机意外启动；一次按下只检查一次入环条件。
    reg button_meta = 1'b0, button_sync = 1'b0;
    reg button_stable = 1'b0, button_press = 1'b0;
    reg release_seen = 1'b0;
    reg [19:0] debounce_count = 20'd0;
    reg [19:0] release_count = 20'd0;
    always @(posedge clk) begin
        button_meta <= button;
        button_sync <= button_meta;
        button_press <= 1'b0;

        if (!release_seen) begin
            if (button_sync)
                release_count <= 20'd0;
            else if (release_count == DEBOUNCE_CYCLES-1)
                release_seen <= 1'b1;
            else
                release_count <= release_count + 1'b1;
        end

        if (button_sync == button_stable)
            debounce_count <= 20'd0;
        else if (debounce_count == DEBOUNCE_CYCLES-1) begin
            button_stable <= button_sync;
            debounce_count <= 20'd0;
            if (button_sync)
                button_press <= 1'b1;
        end else
            debounce_count <= debounce_count + 1'b1;
    end

    // 常数在综合时预先换成 Q8；运行时仅使用乘法和移位。
    localparam integer KP_Q8 = (KP_X100*256+50)/100;
    localparam integer KI_Q8 = (KI_X100*256+50)/100;
    localparam integer KD_Q8 = (KD_X100*256+50)/100;
    // Gowin 1.9.12 曾把“有符号 (TARGET-adc_code) × Kp”合并到 DSP 预加器，
    // 综合后变成 (TARGET+4096-adc_code) × Kp，导致误差始终为正。
    // 因此先用无符号比较求误差幅值，再分别处理符号与乘法。
    wire error_negative = adc_code > TARGET;
    wire [11:0] error_magnitude = error_negative ?
        (adc_code - TARGET) : (TARGET - adc_code);
    wire signed [13:0] error_now = error_negative ?
        -$signed({2'b00,error_magnitude}) :
         $signed({2'b00,error_magnitude});
    // 外环目标码动态变化；先寄存误差，把目标加法/比较与 PID 乘加
    // 分在相邻时钟周期。延迟 20 ns，远小于 5 ms 内环周期。
    reg signed [13:0] error_sampled = 14'sd0;
    reg [11:0] magnitude_sampled = 12'd0;
    reg negative_sampled = 1'b0;
    always @(posedge clk) begin
        error_sampled <= error_now;
        magnitude_sampled <= error_magnitude;
        negative_sampled <= error_negative;
    end
    reg signed [13:0] error_last = 14'sd0;
    wire signed [31:0] error_ext =
        {{18{error_sampled[13]}},error_sampled};
    wire signed [14:0] error_delta =
        $signed({error_sampled[13],error_sampled}) -
        $signed({error_last[13],error_last});
    wire signed [31:0] delta_ext = {{17{error_delta[14]}},error_delta};

    reg signed [31:0] integral = 32'sd0;
    wire signed [31:0] integral_raw = integral + error_ext;
    wire signed [31:0] integral_next =
        integral_raw > INTEGRAL_LIMIT ? INTEGRAL_LIMIT :
        integral_raw < -INTEGRAL_LIMIT ? -INTEGRAL_LIMIT : integral_raw;
    // 各项都先乘非负幅值，最后恢复符号，避免综合器将有符号减法
    // 错误地折叠进 DSP 乘法器。Kp/Ki/Kd 参数应为非负数。
    wire [31:0] p_magnitude_q8 = magnitude_sampled * KP_Q8;
    wire [31:0] integral_magnitude = integral_next[31] ?
        -integral_next : integral_next;
    wire [31:0] i_magnitude_q8 = integral_magnitude * KI_Q8;
    wire [31:0] delta_magnitude = delta_ext[31] ?
        -delta_ext : delta_ext;
    wire [31:0] d_magnitude_q8 = delta_magnitude * KD_Q8;
    wire signed [31:0] p_term = negative_sampled ?
        -$signed(p_magnitude_q8) : $signed(p_magnitude_q8);
    wire signed [31:0] i_term = integral_next[31] ?
        -$signed(i_magnitude_q8) : $signed(i_magnitude_q8);
    wire signed [31:0] d_term = delta_ext[31] ?
        -$signed(d_magnitude_q8) : $signed(d_magnitude_q8);
    // 分级寄存三项和最终控制量，避免一拍内串过减法、乘法、累加、
    // 绝对值和 PWM 限幅。更新周期 5 ms，流水延迟仅 40 ns。
    reg signed [31:0] p_pipe = 32'sd0;
    reg signed [31:0] i_pipe = 32'sd0;
    reg signed [31:0] d_pipe = 32'sd0;
    reg signed [31:0] command_q8 = 32'sd0;
    always @(posedge clk) begin
        p_pipe <= p_term;
        i_pipe <= i_term;
        d_pipe <= d_term;
        command_q8 <= p_pipe + i_pipe + d_pipe;
    end
    // 先对完整 PID 和判符号，再取绝对值计算 PWM。方向只由 PID 和的
    // 正负决定；MOTOR_INVERT 仅交换 AIN1/AIN2 的物理极性。
    wire command_negative = command_q8 < 0;
    wire [31:0] command_magnitude_q8 = command_negative ?
        -command_q8 : command_q8;
    wire [31:0] command_magnitude = command_magnitude_q8 >> 8;
    wire [11:0] duty_next = command_magnitude > MAX_PWM_TICKS ?
        MAX_PWM_TICKS : command_magnitude[11:0];
    wire forward_next = (!command_negative) ^ (MOTOR_INVERT != 0);

    reg [19:0] pid_count = 20'd0;
    reg [11:0] duty = 12'd0;
    reg forward = 1'b1;
    always @(posedge clk) begin
        if (!pid_active) begin
            pid_count <= 20'd0;
            duty <= 12'd0;
            integral <= 32'sd0;
            if (button_press && release_seen && feedback_ok) begin
                pid_active <= 1'b1;
                error_last <= error_sampled;
            end
        end else if (!feedback_ok) begin
            // 越过窗口、ADC 无效或 OTR=1 时立即退出，不能自动重新入环。
            pid_active <= 1'b0;
            pid_count <= 20'd0;
            duty <= 12'd0;
            integral <= 32'sd0;
        end else if (pid_count == PID_PERIOD_CYCLES-1) begin
            pid_count <= 20'd0;
            error_last <= error_sampled;
            // 输出饱和且误差还在推动积分增大时，冻结积分以减少 windup。
            if (!((command_magnitude > MAX_PWM_TICKS) &&
                  ((command_negative && error_sampled < 0) ||
                   (!command_negative && error_sampled > 0))))
                integral <= integral_next;
            // 换向先关闭一次 PWM；下一次 5 ms 更新才施加反向转矩。
            if (duty != 0 && forward_next != forward) begin
                duty <= 12'd0;
                forward <= forward_next;
            end else begin
                duty <= duty_next;
                forward <= forward_next;
            end
        end else
            pid_count <= pid_count + 1'b1;
    end

    reg [11:0] pwm_count = 12'd0;
    always @(posedge clk)
        if (pwm_count == PWM_PERIOD-1)
            pwm_count <= 12'd0;
        else
            pwm_count <= pwm_count + 1'b1;

    // 安全门直接由最新 ADC 状态控制：越界后的下一个组合传播即关 STBY。
    wire drive_enable = pid_active && feedback_ok && (duty != 0);
    assign tb_pwma = drive_enable && (pwm_count < duty);
    assign tb_ain1 = drive_enable && forward;
    assign tb_ain2 = drive_enable && !forward;
    assign tb_stby = drive_enable;
endmodule

