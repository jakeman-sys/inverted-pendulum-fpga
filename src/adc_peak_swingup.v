// 三帧起摆状态机：一次启动→脱离下垂死区→往返脉冲→最高点再泵能→原PID。
// 原始ADC跨0/2708跳变时清历史，不能对盲区做线性插值或极值判断。
module adc_peak_swingup #(
    parameter integer CENTER_CODE=1403,WINDOW_CODE=350,CAPTURE_WINDOW_CODE=350,SENSOR_MAX_CODE=2708,
    parameter integer DEAD_ADC_MAX=10,END_MARGIN_CODE=32,WRAP_JUMP_ADC=1000,
    parameter integer PEAK_MIN_DELTA_ADC=1,PEAK_REARM_ADC=60,
    parameter integer INITIAL_FORWARD=1,LEFT_PEAK_FORWARD=1,RIGHT_PEAK_FORWARD=0,
    parameter integer PULSE_CYCLES=5000000,REVERSE_GAP_CYCLES=500000,
    parameter integer DEAD_RETRY_CYCLES=40000000,TOTAL_TIMEOUT_CYCLES=750000000,
    parameter integer TRAVEL_LIMIT_DECI=1200,
    // 独立单元默认关闭以兼容旧接口；顶层启用倒下自动恢复。
    parameter integer AUTO_RECOVER_ENABLE=0,RECOVER_IMMEDIATE=0,RECOVER_IMMEDIATE_GAP_CYCLES=500000,
    parameter integer RECOVER_IMMEDIATE_READY_CYCLES=5000000,
    parameter integer RECOVER_STOP_CYCLES=15000000,
    parameter integer RECOVER_READY_CYCLES=10000000,RECOVER_WAIT_TIMEOUT_CYCLES=250000000,
    parameter integer RECOVER_MAX_ATTEMPTS=3,RECOVER_STABLE_CYCLES=50000000
)(
    input wire clk,rst_n,start,stop,frame_tick,
    input wire [11:0] adc_code,
    input wire sample_good,electrical_ok,pid_feedback_ok,pulse_ready,position_limit,
    input wire fall_window_exit,recovery_down_ready,recovery_arm_stopped,
    input wire signed [31:0] position_deci,
    output reg [3:0] state=0,fault=0,
    output reg [1:0] limit_reason=0,
    output wire running,pid_enable,swing_enable,
    output reg pulse_forward=1,
    output reg [3:0] recovery_attempts=0,
    output reg [1:0] recovery_fault_reason=0, // F5:1=连续次数耗尽R，2=等待下垂/减速超时T。
    output wire recovery_pending,
    output wire left_peak,right_peak
);
    localparam [3:0] IDLE=0,OUT=1,GAP=2,BACK=3,WAIT_PEAK=4,CHECK_UPRIGHT=5,PID=6,FAULT=7,FALL_STOP=8,WAIT_DOWN=9,RESTART_GAP=10;
    localparam integer MAX_RECOVER=RECOVER_MAX_ATTEMPTS<0 ? 0 : RECOVER_MAX_ATTEMPTS>15 ? 15 : RECOVER_MAX_ATTEMPTS;
    reg [31:0] recover_stop_count=0,recover_wait_count=0,recover_ready_count=0,stable_count=0;
    reg recovery_session=0;
    // 接管默认±350；PID运行安全窗口由顶层WINDOW_CODE控制。
    // 防止接管窗口配置得比安全窗口大，入环后立即越界。
    localparam integer CAPTURE_WINDOW=CAPTURE_WINDOW_CODE<1 ? 1 :
        (CAPTURE_WINDOW_CODE>WINDOW_CODE ? WINDOW_CODE : CAPTURE_WINDOW_CODE);
    localparam integer LOW=CENTER_CODE-CAPTURE_WINDOW,HIGH=CENTER_CODE+CAPTURE_WINDOW;
    localparam integer PULSE=PULSE_CYCLES<1 ? 1 : PULSE_CYCLES;
    assign running=state!=IDLE && state!=FAULT;
    assign pid_enable=state==PID;
    assign swing_enable=state==OUT || state==BACK;
    // 立即进入恢复任务；先停驱并确认横杆停稳，不等待摆杆下垂位置。
    assign recovery_pending=state==FALL_STOP || state==WAIT_DOWN || state==RESTART_GAP;
    wire swing_task=state==OUT || state==GAP || state==BACK || state==WAIT_PEAK || state==CHECK_UPRIGHT;
    reg [31:0] pulse_count=0,gap_count=0,total_count=0,retry_count=0;
    reg signed [31:0] start_position=0;
    reg [11:0] oldest=0,middle=0,last_peak_adc=0;
    reg [1:0] history_count=0,center_count=0;
    reg peak_armed=1;
    wire [11:0] frame_step=adc_code>middle ? adc_code-middle : middle-adc_code;
    wire jump=history_count!=0 && frame_step>WRAP_JUMP_ADC;
    wire usable=sample_good && !jump;
    wire near_upright=sample_good && adc_code>=LOW && adc_code<=HIGH;
    wire endpoint=adc_code<=DEAD_ADC_MAX || adc_code>=SENSOR_MAX_CODE-END_MARGIN_CODE;
    wire [11:0] rearm_distance=adc_code>last_peak_adc ? adc_code-last_peak_adc : last_peak_adc-adc_code;
    // 位于低码半周的极大值是左侧最高点；高码半周的极小值是右侧最高点。
    // 三点必须同在接管窗口外的一侧，避免普通经过中心时误判。
    assign left_peak=frame_tick && usable && history_count==2 && peak_armed &&
        oldest<LOW && middle<LOW && adc_code<LOW &&
        middle>oldest && middle>adc_code && middle-oldest>=PEAK_MIN_DELTA_ADC && middle-adc_code>=PEAK_MIN_DELTA_ADC;
    assign right_peak=frame_tick && usable && history_count==2 && peak_armed &&
        oldest>HIGH && middle>HIGH && adc_code>HIGH &&
        middle<oldest && middle<adc_code && oldest-middle>=PEAK_MIN_DELTA_ADC && adc_code-middle>=PEAK_MIN_DELTA_ADC;
    // 位置差先用较宽有符号值计算，零点变化前顶层先停机。
    wire signed [32:0] travel=$signed({position_deci[31],position_deci})-$signed({start_position[31],start_position});
    wire travel_limit=TRAVEL_LIMIT_DECI>0 && (travel>TRAVEL_LIMIT_DECI || travel< -TRAVEL_LIMIT_DECI);
    always @(posedge clk)begin
        if(!rst_n || stop)begin
            state<=IDLE;fault<=0;limit_reason<=0;pulse_count<=0;gap_count<=0;total_count<=0;retry_count<=0;
            history_count<=0;center_count<=0;peak_armed<=1;
            recovery_attempts<=0;recovery_fault_reason<=0;recovery_session<=0;
            recover_stop_count<=0;recover_wait_count<=0;recover_ready_count<=0;stable_count<=0;
        end else if(!running)begin
            pulse_count<=0;gap_count<=0;total_count<=0;retry_count<=0;history_count<=0;center_count<=0;peak_armed<=1;
            stable_count<=0;
            if(start)begin
                recovery_attempts<=0;recovery_fault_reason<=0;recovery_session<=0;
                recover_stop_count<=0;recover_wait_count<=0;recover_ready_count<=0;
                start_position<=position_deci;pulse_forward<=INITIAL_FORWARD!=0;fault<=0;limit_reason<=0;
                if(!electrical_ok)begin state<=FAULT;fault<=1;end
                else if(position_limit)begin state<=FAULT;fault<=2;limit_reason<=2;end
                else state<=sample_good && adc_code>=LOW && adc_code<=HIGH ? CHECK_UPRIGHT : OUT;
            end
        end else if(!electrical_ok || position_limit || (swing_task && travel_limit))begin
            state<=FAULT;fault<=!electrical_ok ? 1 : 2;
            limit_reason<=!electrical_ok ? 0 : (position_limit ? 2 : 1);
            recovery_session<=0;stable_count<=0;
        end else if(state==PID)begin
            if(!pid_feedback_ok)begin
                stable_count<=0;
                if(AUTO_RECOVER_ENABLE!=0 && fall_window_exit)begin
                    if(recovery_attempts>=MAX_RECOVER)begin
                        state<=FAULT;fault<=5;recovery_fault_reason<=1;recovery_session<=0;
                    end else begin
                        // 越界这一拍就退出PID并进入起摆任务；停驱并确认横杆停稳后才输出PWM。
                        state<=RECOVER_IMMEDIATE!=0 ? RESTART_GAP : FALL_STOP;
                        fault<=4;recovery_session<=1;recovery_fault_reason<=0;
                        recover_stop_count<=0;recover_wait_count<=0;recover_ready_count<=0;
                        pulse_count<=0;gap_count<=0;total_count<=0;retry_count<=0;
                        history_count<=0;center_count<=0;peak_armed<=1;
                    end
                end else begin state<=FAULT;fault<=4;recovery_session<=0;end
            end else if(RECOVER_STABLE_CYCLES<=0 || stable_count>=RECOVER_STABLE_CYCLES-1)begin
                // 持续平衡后才清零次数；短暂擦入PID又倒下不能无限重试。
                recovery_attempts<=0;recovery_session<=0;
            end else stable_count<=stable_count+1'b1;
        end else if(recovery_pending)begin
            // 立即模式只等横杆停稳；旧模式还等摆杆下垂。两种等待均受超时保护。
            if(RECOVER_WAIT_TIMEOUT_CYCLES>0 && recover_wait_count>=RECOVER_WAIT_TIMEOUT_CYCLES-1)begin
                state<=FAULT;fault<=5;recovery_fault_reason<=2;recovery_session<=0;
            end else begin
                if(RECOVER_WAIT_TIMEOUT_CYCLES>0)recover_wait_count<=recover_wait_count+1'b1;
                if(state==FALL_STOP)begin
                    recover_ready_count<=0;
                    if(RECOVER_STOP_CYCLES<=0 || recover_stop_count>=RECOVER_STOP_CYCLES-1)begin
                        state<=WAIT_DOWN;recover_stop_count<=0;
                    end else recover_stop_count<=recover_stop_count+1'b1;
                end else if(state==RESTART_GAP && RECOVER_IMMEDIATE_GAP_CYCLES>0 && recover_stop_count<RECOVER_IMMEDIATE_GAP_CYCLES-1)
                    recover_stop_count<=recover_stop_count+1'b1;
                else if(state==RESTART_GAP && !recovery_arm_stopped)recover_ready_count<=0;
                else if(state==WAIT_DOWN && !recovery_down_ready)recover_ready_count<=0;
                else if((state==RESTART_GAP && (RECOVER_IMMEDIATE_READY_CYCLES<=0 || recover_ready_count>=RECOVER_IMMEDIATE_READY_CYCLES-1)) ||
                        (state==WAIT_DOWN && (RECOVER_READY_CYCLES<=0 || recover_ready_count>=RECOVER_READY_CYCLES-1)))begin
                    if(recovery_attempts>=MAX_RECOVER)begin
                        state<=FAULT;fault<=5;recovery_fault_reason<=1;recovery_session<=0;
                    end else begin
                        recovery_attempts<=recovery_attempts+1'b1;
                        // 重取本次起摆起点，仅供±120°行程检查，不改变编码器POS。
                        start_position<=position_deci;pulse_forward<=INITIAL_FORWARD!=0;
                        state<=OUT;fault<=0;limit_reason<=0;recovery_fault_reason<=0;
                        pulse_count<=0;gap_count<=0;total_count<=0;retry_count<=0;
                        history_count<=0;center_count<=0;peak_armed<=1;
                        recover_stop_count<=0;recover_ready_count<=0;recover_wait_count<=0;stable_count<=0;
                    end
                end else recover_ready_count<=recover_ready_count+1'b1;
            end
        end else if(TOTAL_TIMEOUT_CYCLES>0 && total_count>=TOTAL_TIMEOUT_CYCLES-1)begin
            if(AUTO_RECOVER_ENABLE!=0 && recovery_session)begin
                if(recovery_attempts>=MAX_RECOVER)begin
                    state<=FAULT;fault<=5;recovery_fault_reason<=1;recovery_session<=0;
                end else begin
                    state<=RECOVER_IMMEDIATE!=0 ? RESTART_GAP : FALL_STOP;fault<=3;
                    recover_stop_count<=0;recover_wait_count<=0;recover_ready_count<=0;
                    history_count<=0;center_count<=0;peak_armed<=1;
                end
            end else begin state<=FAULT;fault<=3;end
        end else begin
            total_count<=total_count+1'b1;
            if(frame_tick)begin
                // 死区、混批或跨周跳变均中断“连续三帧”，下一有效帧重新起算。
                if(!sample_good)begin history_count<=0;center_count<=0;end
                else if(jump)begin
                    // 当前是有效新码，则作为新序列第一帧；跨周前的两帧丢弃。
                    oldest<=adc_code;middle<=adc_code;history_count<=1;center_count<=near_upright ? 1 : 0;
                end
                else begin
                    oldest<=middle;middle<=adc_code;
                    if(history_count<2)history_count<=history_count+1'b1;
                    center_count<=near_upright ? (center_count<2 ? center_count+1'b1 : center_count) : 0;
                end
                if(endpoint || (sample_good && rearm_distance>=PEAK_REARM_ADC))peak_armed<=1;
            end
            // 三帧接管优先于往返计时，可在任何起摆子状态终止起摆脉冲。
            if(frame_tick && near_upright && !jump && center_count==2)begin
                state<=PID;pulse_count<=0;retry_count<=0;stable_count<=0;
            end else case(state)
                CHECK_UPRIGHT:if(frame_tick && !near_upright)begin
                    state<=OUT;pulse_count<=0;pulse_forward<=INITIAL_FORWARD!=0;
                end
                OUT:if(pulse_ready)begin
                    if(pulse_count==PULSE-1)begin
                        pulse_count<=0;pulse_forward<=!pulse_forward;
                        if(REVERSE_GAP_CYCLES>0)begin state<=GAP;gap_count<=0;end
                        else state<=BACK; // 接口仍允许设0；正常默认停驱10ms。
                    end else pulse_count<=pulse_count+1'b1;
                end
                GAP:if(gap_count>=REVERSE_GAP_CYCLES-1)begin state<=BACK;gap_count<=0;end
                    else gap_count<=gap_count+1'b1;
                BACK:if(pulse_ready)begin
                    if(pulse_count==PULSE-1)begin
                        state<=WAIT_PEAK;pulse_count<=0;retry_count<=0;history_count<=0;
                    end else pulse_count<=pulse_count+1'b1;
                end
                WAIT_PEAK:begin
                    if(left_peak || right_peak)begin
                        state<=OUT;pulse_count<=0;retry_count<=0;
                        pulse_forward<=left_peak ? LEFT_PEAK_FORWARD!=0 : RIGHT_PEAK_FORWARD!=0;
                        last_peak_adc<=middle;peak_armed<=0;history_count<=0;
                    end else if(endpoint && DEAD_RETRY_CYCLES>0)begin
                        if(retry_count>=DEAD_RETRY_CYCLES-1)begin
                            state<=OUT;pulse_forward<=INITIAL_FORWARD!=0;pulse_count<=0;retry_count<=0;
                            history_count<=0;center_count<=0;
                        end else retry_count<=retry_count+1'b1;
                    end else retry_count<=0;
                end
                default:begin state<=FAULT;fault<=6;end
            endcase
        end
    end
endmodule
