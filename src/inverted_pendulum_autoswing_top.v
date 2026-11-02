module inverted_pendulum_autoswing_top #(
    parameter integer CENTER_CODE=1404,WINDOW_CODE=350,
    parameter integer KP_X100=2000,KI_X100=45,KD_X100=3500,MOTOR_INVERT=1,MAX_PWM_TICKS=1500,
    parameter integer OUT_KP_X100=50,OUT_KI_X100=1,OUT_KD_X100=400,
    parameter integer OUT_MAX_OFFSET_ADC=25,OUT_PERIOD_CYCLES=2500000,
    parameter integer OUT_INTEGRAL_SEP_DEG=10,OUT_INTEGRAL_LIMIT_DECI=20000,OUT_INVERT=0,
    parameter integer SPEED_KP_X100=8,SPEED_KI_X100=0,SPEED_KD_X100=0,
    parameter integer SPEED_PERIOD_CYCLES=2500000,SPEED_MAX_OFFSET_ADC=20,
    parameter integer SPEED_INTEGRAL_LIMIT_DECI=20000,SPEED_OUT_INVERT=0,SPEED_FILTER_SHIFT=2,
    parameter integer SPEED_TARGET_LIMIT_DPS=360,ENCODER_INVERT=0,ENCODER_FILTER_CYCLES=100,
    parameter integer BUTTON_DEBOUNCE_CYCLES=1000000,BUTTON_HOLD_CYCLES=25000000,BUTTON_REPEAT_CYCLES=5000000,
    // 自动起摆修改接口：百分比、每段时长、三帧采样间隔和三个独立方向。
    // FORWARD=1代表AIN1=1/AIN2=0，0代表相反；不是屏幕POS正负方向。
    parameter integer SWING_DUTY_PERCENT=43,SWING_PULSE_MS=88,SWING_REVERSE_GAP_MS=10,
    // 接管窗口与PID运行中的安全窗口分别设置；三帧都在接管窗口内才入环。
    parameter integer SWING_FRAME_MS=20,SWING_CAPTURE_WINDOW_CODE=110,
    // 接管先稳定角度，再缓慢放开外环作用；位置目标锁定接管时POS，不回上电零点。
    parameter integer OUTER_START_DELAY_MS=300,OUTER_RAMP_PERIOD_MS=50,OUTER_RAMP_STEP_ADC=1,
    parameter integer SWING_INITIAL_FORWARD=1,SWING_LEFT_PEAK_FORWARD=1,SWING_RIGHT_PEAK_FORWARD=0,
    parameter integer SENSOR_MAX_CODE=2708,DEAD_ADC_MAX=10,SENSOR_END_MARGIN=32,
    parameter integer SWING_WRAP_JUMP_ADC=1000,SWING_PEAK_MIN_DELTA_ADC=1,SWING_PEAK_REARM_ADC=60,
    parameter integer SWING_DEAD_RETRY_MS=800,SWING_TIMEOUT_MS=15000,SWING_TRAVEL_LIMIT_DEG=120,
    parameter integer ALLOW_SWING_LOW_OTR=1,SOFT_LIMIT_DEG=720,
    // 定速度允许累计多圈：0关闭速度模式累计行程限制；正数为相对入速度模式的限位度数。
    parameter integer SPEED_SOFT_LIMIT_DEG=0,
    // PID坏批保留上一次有效值的最大时间；越角度窗口和OTR不延时。
    parameter integer PID_FEEDBACK_HOLD_MS=2,
    // 倒下立即切入恢复任务；停驱至少10ms并用编码器确认横杆停稳，不等摆杆下垂。
    // 停稳=横杆速度绝对值<=2度/秒连续100ms；IMMEDIATE=0使用旧下垂等待模式。
    parameter integer RECOVER_IMMEDIATE=1,RECOVER_IMMEDIATE_GAP_MS=10,
    parameter integer RECOVER_IMMEDIATE_READY_MS=100,RECOVER_IMMEDIATE_ARM_MAX_SPEED_DPS=2,
    parameter integer AUTO_RECOVER_ENABLE=1,RECOVER_STOP_MS=300,RECOVER_READY_MS=200,
    parameter integer RECOVER_WAIT_TIMEOUT_MS=5000,RECOVER_MAX_ATTEMPTS=3,RECOVER_STABLE_MS=1000,
    parameter integer RECOVER_DOWN_WINDOW_ADC=32,RECOVER_ARM_MAX_SPEED_DPS=20
)(
    input wire clk,button,button_dec,button_inc,button_page,button_speed_page,encoder_a,encoder_b,
    input wire [11:0] adc_data,input wire adc_otr,
    output wire adc_clk,tft_cs_n,tft_sclk,tft_mosi,tft_dc,tft_rst_n,tft_bl,
    output wire tb_pwma,tb_ain1,tb_ain2,tb_stby
);
    reg [15:0] startup=0;
    always @(posedge clk)if(startup!=50000)startup<=startup+1'b1;
    wire rst_n=startup==50000;
    localparam integer FRAME_CYCLES=SWING_FRAME_MS<1 ? 50000 : SWING_FRAME_MS*50000;
    reg [31:0] frame_count=0;
    reg frame_tick=0;
    always @(posedge clk)begin
        frame_tick<=0;
        if(!rst_n)frame_count<=0;
        else if(frame_count==FRAME_CYCLES-1)begin frame_count<=0;frame_tick<=1;end
        else frame_count<=frame_count+1'b1;
    end
    wire [11:0] adc_average;
    wire adc_valid,adc_overrange,feedback_valid,pid_adc_valid,low_otr_only;
    wire [11:0] adc_batch_spread;
    ad9220_capture #(.DEAD_ADC_MAX(DEAD_ADC_MAX),.SENSOR_MAX_CODE(SENSOR_MAX_CODE),.END_MARGIN_CODE(SENSOR_END_MARGIN))
        u_adc(.clk_50m(clk),.adc_data(adc_data),.adc_otr(adc_otr),.adc_clk(adc_clk),
        .average(adc_average),.valid(adc_valid),.overrange(adc_overrange),
        .feedback_valid(feedback_valid),.pid_feedback_valid(pid_adc_valid),.batch_spread(adc_batch_spread),
        .overrange_low_only(low_otr_only));
    // 低端欠量程只允许有限起摆，绝不允许PID使用该ADC。其它OTR一律停机。
    wire electrical_ok=adc_valid && adc_average<=SENSOR_MAX_CODE+8 &&
        (!adc_overrange || (ALLOW_SWING_LOW_OTR!=0 && low_otr_only));
    wire pid_feedback_ok;
    wire [11:0] pid_adc;
    wire [2:0] feedback_reject_reason;
    pid_feedback_guard #(.CENTER_CODE(CENTER_CODE),.WINDOW_CODE(WINDOW_CODE),
        .HOLD_CYCLES(PID_FEEDBACK_HOLD_MS*50000)) u_feedback(
        .clk(clk),.rst_n(rst_n),.adc_code(adc_average),.source_valid(adc_valid),
        .sample_good(pid_adc_valid),.overrange(adc_overrange),
        .pid_adc(pid_adc),.feedback_ok(pid_feedback_ok),.reject_reason(feedback_reject_reason));
    wire signed [31:0] encoder_count,encoder_position_deci,actual_speed_deci_dps;
    reg signed [31:0] actual_position_deci=0;
    always @(posedge clk)actual_position_deci<=encoder_position_deci;
    wire signed [47:0] relative_position_q16,raw_position_q16;
    wire zero_position;
    quadrature_position #(.ENCODER_INVERT(ENCODER_INVERT),.FILTER_CYCLES(ENCODER_FILTER_CYCLES))
        u_encoder(.clk(clk),.a(encoder_a),.b(encoder_b),.zero_position(zero_position),.count(encoder_count),
        .position_deci_deg(encoder_position_deci),.position_q16(relative_position_q16),.raw_position_q16(raw_position_q16));
    encoder_speed_estimator #(.PERIOD_CYCLES(2500000),.SAMPLE_RATE_HZ(20),.FILTER_SHIFT(SPEED_FILTER_SHIFT))
        u_speed_measure(.clk(clk),.zero_position(1'b0),.position_q16(raw_position_q16),.speed_deci_dps(actual_speed_deci_dps));
    wire f2_press,f1_press,f1_repeat,e3_press,e3_repeat,s1_press,s2_press;
    button_repeat #(.DEBOUNCE_CYCLES(BUTTON_DEBOUNCE_CYCLES),.HOLD_CYCLES(BUTTON_HOLD_CYCLES),.REPEAT_CYCLES(BUTTON_REPEAT_CYCLES))
        u_f2(.clk(clk),.button(button),.press(f2_press),.repeat_press(),.stable());
    button_repeat #(.DEBOUNCE_CYCLES(BUTTON_DEBOUNCE_CYCLES),.HOLD_CYCLES(BUTTON_HOLD_CYCLES),.REPEAT_CYCLES(BUTTON_REPEAT_CYCLES))
        u_f1(.clk(clk),.button(button_dec),.press(f1_press),.repeat_press(f1_repeat),.stable());
    button_repeat #(.DEBOUNCE_CYCLES(BUTTON_DEBOUNCE_CYCLES),.HOLD_CYCLES(BUTTON_HOLD_CYCLES),.REPEAT_CYCLES(BUTTON_REPEAT_CYCLES))
        u_e3(.clk(clk),.button(button_inc),.press(e3_press),.repeat_press(e3_repeat),.stable());
    button_repeat #(.DEBOUNCE_CYCLES(BUTTON_DEBOUNCE_CYCLES),.HOLD_CYCLES(BUTTON_HOLD_CYCLES),.REPEAT_CYCLES(BUTTON_REPEAT_CYCLES))
        u_s1(.clk(clk),.button(button_page),.press(s1_press),.repeat_press(),.stable());
    button_repeat #(.DEBOUNCE_CYCLES(BUTTON_DEBOUNCE_CYCLES),.HOLD_CYCLES(BUTTON_HOLD_CYCLES),.REPEAT_CYCLES(BUTTON_REPEAT_CYCLES))
        u_s2(.clk(clk),.button(button_speed_page),.press(s2_press),.repeat_press(),.stable());
    wire [1:0] page;
    wire speed_mode,pid_active,running,pid_request,swing_request,pulse_forward,left_peak,right_peak;
    wire [3:0] control_state,fault_code;
    wire recovery_pending;
    wire [3:0] recovery_attempts;
    wire [1:0] recovery_fault_reason;
    wire stop_for_zero=f1_press && page==0;
    reg [1:0] zero_pending=0;
    always @(posedge clk)begin
        if(!rst_n)zero_pending<=0;
        else if(stop_for_zero || zero_position)zero_pending<=2;
        else if(frame_tick && zero_pending!=0)zero_pending<=zero_pending-1'b1;
    end
    wire zero_busy=stop_for_zero || zero_position || zero_pending!=0;
    wire stop_request=stop_for_zero || (f2_press && page==0 && running);
    wire start_request=f2_press && page==0 && !running && !zero_busy;
    // F4锁存发生瞬间的ADC及原因；不能用几百ms后刷新到的ADC追查故障。
    reg [11:0] fault_adc=0;
    reg [11:0] fault_spread=0;
    reg [2:0] fault_detail=0;
    always @(posedge clk)begin
        if(!rst_n || stop_request || start_request)begin fault_adc<=0;fault_detail<=0;fault_spread<=0;end
        else if(pid_request && !pid_feedback_ok)begin
            fault_adc<=adc_average;fault_detail<=feedback_reject_reason;
            fault_spread<=adc_batch_spread;
        end
    end
    // 位置与速度模式分别判断行程；保护参考独立于POS，切模式不清编码器和测速历史。
    // 速度设0或在S1应用定位目标时，以切换位置重建保护参考，避免大POS立即触发F2 L。
    localparam signed [47:0] SOFT_LIMIT_Q16=48'sd1*SOFT_LIMIT_DEG*10*65536;
    localparam signed [47:0] SPEED_LIMIT_Q16=48'sd1*SPEED_SOFT_LIMIT_DEG*10*65536;
    reg signed [47:0] limit_position_q16=0;
    reg signed [47:0] limit_origin_q16=0;
    reg signed [48:0] limit_travel_q16=0;
    reg speed_mode_d=0;
    reg recovery_pending_d=0;
    always @(posedge clk)speed_mode_d<=rst_n && speed_mode;
    always @(posedge clk)recovery_pending_d<=rst_n && recovery_pending;
    wire mode_changed=speed_mode_d!=speed_mode;
    wire recovery_enter=recovery_pending && !recovery_pending_d;
    always @(posedge clk)begin
        if(!rst_n || zero_busy)begin
            limit_position_q16<=0;limit_origin_q16<=0;limit_travel_q16<=0;
        end else begin
            limit_position_q16<=relative_position_q16;
            if(mode_changed || recovery_enter)begin limit_origin_q16<=limit_position_q16;limit_travel_q16<=0;end
            else limit_travel_q16<=$signed({limit_position_q16[47],limit_position_q16})-
                $signed({limit_origin_q16[47],limit_origin_q16});
        end
    end
    wire speed_control=pid_request && speed_mode;
    wire position_range_exceeded=SOFT_LIMIT_DEG>0 &&
        (limit_travel_q16>SOFT_LIMIT_Q16 || limit_travel_q16< -SOFT_LIMIT_Q16);
    wire speed_range_exceeded=SPEED_SOFT_LIMIT_DEG>0 &&
        (limit_travel_q16>SPEED_LIMIT_Q16 || limit_travel_q16< -SPEED_LIMIT_Q16);
    // 更新保护参考的一拍屏蔽旧参考比较，下一拍继续保护。
    wire position_limit=!mode_changed && !recovery_enter &&
        (speed_control ? speed_range_exceeded : position_range_exceeded);
    wire [1:0] limit_reason; // F2细分：1=本次起摆行程T，2=POS软件限位L。
    wire swing_pwm,swing_a1,swing_a2,swing_stby;
    // 下垂包括盲区0~10及高码另一侧；原始源未就绪/异常高码不能授权。
    wire recovery_down_near=adc_average<=RECOVER_DOWN_WINDOW_ADC ||
        adc_average>=SENSOR_MAX_CODE-RECOVER_DOWN_WINDOW_ADC;
    wire recovery_arm_slow=actual_speed_deci_dps<=RECOVER_ARM_MAX_SPEED_DPS*10 &&
        actual_speed_deci_dps>=-RECOVER_ARM_MAX_SPEED_DPS*10;
    wire recovery_down_ready=electrical_ok && recovery_down_near && recovery_arm_slow;
    wire recovery_arm_stopped=actual_speed_deci_dps<=RECOVER_IMMEDIATE_ARM_MAX_SPEED_DPS*10 &&
        actual_speed_deci_dps>=-RECOVER_IMMEDIATE_ARM_MAX_SPEED_DPS*10;
    adc_peak_swingup #(.CENTER_CODE(CENTER_CODE),.WINDOW_CODE(WINDOW_CODE),
        .CAPTURE_WINDOW_CODE(SWING_CAPTURE_WINDOW_CODE),.SENSOR_MAX_CODE(SENSOR_MAX_CODE),
        .DEAD_ADC_MAX(DEAD_ADC_MAX),.END_MARGIN_CODE(SENSOR_END_MARGIN),.WRAP_JUMP_ADC(SWING_WRAP_JUMP_ADC),
        .PEAK_MIN_DELTA_ADC(SWING_PEAK_MIN_DELTA_ADC),.PEAK_REARM_ADC(SWING_PEAK_REARM_ADC),
        .INITIAL_FORWARD(SWING_INITIAL_FORWARD),.LEFT_PEAK_FORWARD(SWING_LEFT_PEAK_FORWARD),.RIGHT_PEAK_FORWARD(SWING_RIGHT_PEAK_FORWARD),
        .PULSE_CYCLES(SWING_PULSE_MS*50000),.REVERSE_GAP_CYCLES(SWING_REVERSE_GAP_MS*50000),
        .DEAD_RETRY_CYCLES(SWING_DEAD_RETRY_MS*50000),.TOTAL_TIMEOUT_CYCLES(SWING_TIMEOUT_MS*50000),
        .TRAVEL_LIMIT_DECI(SWING_TRAVEL_LIMIT_DEG*10),.AUTO_RECOVER_ENABLE(AUTO_RECOVER_ENABLE),
        .RECOVER_IMMEDIATE(RECOVER_IMMEDIATE),.RECOVER_IMMEDIATE_GAP_CYCLES(RECOVER_IMMEDIATE_GAP_MS*50000),
        .RECOVER_IMMEDIATE_READY_CYCLES(RECOVER_IMMEDIATE_READY_MS*50000),
        .RECOVER_STOP_CYCLES(RECOVER_STOP_MS*50000),.RECOVER_READY_CYCLES(RECOVER_READY_MS*50000),
        .RECOVER_WAIT_TIMEOUT_CYCLES(RECOVER_WAIT_TIMEOUT_MS*50000),.RECOVER_MAX_ATTEMPTS(RECOVER_MAX_ATTEMPTS),
        .RECOVER_STABLE_CYCLES(RECOVER_STABLE_MS*50000)) u_swing(
        .clk(clk),.rst_n(rst_n),.start(start_request),.stop(stop_request),.frame_tick(frame_tick),
        .adc_code(adc_average),.sample_good(feedback_valid),
        .electrical_ok(electrical_ok && (!pid_request || !adc_overrange)),.pid_feedback_ok(pid_feedback_ok),
        .fall_window_exit(feedback_reject_reason==1),.recovery_down_ready(recovery_down_ready),.recovery_arm_stopped(recovery_arm_stopped),
        .pulse_ready(swing_stby),.position_limit(position_limit),.position_deci(actual_position_deci),
        .state(control_state),.fault(fault_code),.limit_reason(limit_reason),.running(running),.pid_enable(pid_request),
        .swing_enable(swing_request),.pulse_forward(pulse_forward),.left_peak(left_peak),.right_peak(right_peak),
        .recovery_pending(recovery_pending),.recovery_attempts(recovery_attempts),.recovery_fault_reason(recovery_fault_reason));
    wire drive_allowed=rst_n && !stop_request && !zero_busy && !position_limit;
    swing_pulse_pwm #(.DUTY_PERCENT(SWING_DUTY_PERCENT)) u_swing_motor(.clk(clk),.rst_n(rst_n),
        .enable(swing_request && electrical_ok && drive_allowed),.forward(pulse_forward),
        .pwma(swing_pwm),.ain1(swing_a1),.ain2(swing_a2),.stby(swing_stby));
    reg pid_active_d=0;
    always @(posedge clk)pid_active_d<=pid_active;
    wire pid_started=pid_active && !pid_active_d;
    wire signed [31:0] position_target_deci,position_edit_deci;
    wire signed [15:0] speed_target_dps,speed_edit_dps;
    dual_mode_ui #(.SPEED_LIMIT_DPS(SPEED_TARGET_LIMIT_DPS),.HOLD_CURRENT_ON_START(1)) u_ui(
        .clk(clk),.active(pid_active),.started(pid_started),.s1_press(s1_press),.s2_press(s2_press),
        .f2_press(f2_press),.f1_press(f1_press),.f1_repeat(f1_repeat),.e3_press(e3_press),.e3_repeat(e3_repeat),
        .actual_position_deci(actual_position_deci),.page(page),.speed_mode(speed_mode),.zero_position(zero_position),
        .target_position_deci(position_target_deci),.edit_position_deci(position_edit_deci),
        .target_speed_dps(speed_target_dps),.edit_speed_dps(speed_edit_dps));
    wire signed [15:0] position_offset_adc,speed_offset_adc;
    wire outer_ready;
    position_pid_outer #(.OUT_KP_X100(OUT_KP_X100),.OUT_KI_X100(OUT_KI_X100),.OUT_KD_X100(OUT_KD_X100),
        .MAX_OFFSET_ADC(OUT_MAX_OFFSET_ADC),.PERIOD_CYCLES(OUT_PERIOD_CYCLES),.INTEGRAL_SEP_DEG(OUT_INTEGRAL_SEP_DEG),
        .INTEGRAL_LIMIT_DECI(OUT_INTEGRAL_LIMIT_DECI),.OUT_INVERT(OUT_INVERT)) u_position_pid(
        .clk(clk),.active(pid_active && outer_ready && !speed_mode && !zero_position),.target_deci_deg(position_target_deci),
        .actual_deci_deg(actual_position_deci),.offset_adc(position_offset_adc));
    speed_pid_outer #(.KP_X100(SPEED_KP_X100),.KI_X100(SPEED_KI_X100),.KD_X100(SPEED_KD_X100),
        .PERIOD_CYCLES(SPEED_PERIOD_CYCLES),.MAX_OFFSET_ADC(SPEED_MAX_OFFSET_ADC),
        .INTEGRAL_LIMIT_DECI(SPEED_INTEGRAL_LIMIT_DECI),.OUT_INVERT(SPEED_OUT_INVERT)) u_speed_pid(
        .clk(clk),.active(pid_active && outer_ready && speed_mode),.target_speed_dps(speed_target_dps),
        .actual_speed_deci_dps(actual_speed_deci_dps),.offset_adc(speed_offset_adc));
    wire signed [15:0] requested_outer_offset_adc=speed_mode ? speed_offset_adc : position_offset_adc;
    wire signed [15:0] outer_offset_adc;
    localparam integer OUTER_OFFSET_LIMIT=OUT_MAX_OFFSET_ADC>SPEED_MAX_OFFSET_ADC ?
        OUT_MAX_OFFSET_ADC : SPEED_MAX_OFFSET_ADC;
    outer_pid_soft_start #(.DELAY_CYCLES(OUTER_START_DELAY_MS*50000),
        .RAMP_PERIOD_CYCLES(OUTER_RAMP_PERIOD_MS*50000),.RAMP_STEP_ADC(OUTER_RAMP_STEP_ADC),
        .MAX_OFFSET_ADC(OUTER_OFFSET_LIMIT)) u_outer_start(
        .clk(clk),.rst_n(rst_n),.active(pid_active && drive_allowed && pid_feedback_ok),
        .requested_offset_adc(requested_outer_offset_adc),.outer_ready(outer_ready),.offset_adc(outer_offset_adc));
    wire signed [31:0] target_sum=CENTER_CODE+$signed({{16{outer_offset_adc[15]}},outer_offset_adc});
    wire [11:0] angle_target_code=target_sum<0 ? 0 : target_sum>4095 ? 4095 : target_sum[11:0];
    wire pid_pwm,pid_a1,pid_a2,pid_stby;
    // 仅新增外部启停授权；误差符号、乘加、离散周期、抗饱和和原换向逻辑保留。
    pid_angle_ctrl #(.CENTER_CODE(CENTER_CODE),.WINDOW_CODE(WINDOW_CODE),.KP_X100(KP_X100),.KI_X100(KI_X100),.KD_X100(KD_X100),
        .MOTOR_INVERT(MOTOR_INVERT),.MAX_PWM_TICKS(MAX_PWM_TICKS),.EXTERNAL_CONTROL(1)) u_angle_pid(
        .clk(clk),.button(1'b0),.external_enable(pid_request && drive_allowed && pid_feedback_ok),
        .target_code(angle_target_code),.adc_code(pid_adc),.adc_valid(pid_feedback_ok),.adc_otr(adc_overrange),
        .pid_active(pid_active),.tb_pwma(pid_pwm),.tb_ain1(pid_a1),.tb_ain2(pid_a2),.tb_stby(pid_stby));
    // 两个驱动入口由互斥状态选择；PID接管时起摆PWM立即退出，原PID约5ms后首次输出。
    wire output_allowed=drive_allowed && running && (pid_request ? pid_feedback_ok : electrical_ok);
    assign tb_pwma=output_allowed && (pid_request ? pid_pwm : swing_pwm);
    assign tb_ain1=output_allowed && (pid_request ? pid_a1 : swing_a1);
    assign tb_ain2=output_allowed && (pid_request ? pid_a2 : swing_a2);
    assign tb_stby=output_allowed && (pid_request ? pid_stby : swing_stby);
    tft_cascade_display #(.KP_X100(KP_X100),.KI_X100(KI_X100),.KD_X100(KD_X100),
        .OUT_KP_X100(OUT_KP_X100),.OUT_KI_X100(OUT_KI_X100),.OUT_KD_X100(OUT_KD_X100),
        .SPEED_KP_X100(SPEED_KP_X100),.SPEED_KI_X100(SPEED_KI_X100),.SPEED_KD_X100(SPEED_KD_X100)) u_tft(
        .clk(clk),.actual_adc(control_state==7 && fault_code==4 ? fault_adc : (adc_valid ? adc_average : 12'd0)),
        .target_adc(angle_target_code),.pid_active(pid_active),
        .ain1(tb_ain1),.ain2(tb_ain2),.page(page),.speed_mode(speed_mode),.control_state(control_state),.fault_code(fault_code),
        .fault_detail(fault_code==2 ? {1'b0,limit_reason} : fault_code==5 ? {1'b0,recovery_fault_reason} : fault_detail),
        .fault_spread(fault_spread),.recovery_attempts(recovery_attempts),
        .position_target_deci(position_target_deci),.position_edit_deci(position_edit_deci),.position_actual_deci(actual_position_deci),
        .speed_target_dps(speed_target_dps),.speed_edit_dps(speed_edit_dps),.speed_actual_deci(actual_speed_deci_dps),
        .tft_cs_n(tft_cs_n),.tft_sclk(tft_sclk),.tft_mosi(tft_mosi),.tft_dc(tft_dc),.tft_rst_n(tft_rst_n),.tft_bl(tft_bl));
endmodule
