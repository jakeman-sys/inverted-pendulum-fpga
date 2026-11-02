// GMT020-02-8P / ST7789VM 三页面。屏幕显示只读取状态，不参与控制时序。
module tft_cascade_display #(
    parameter integer KP_X100 = 2000,
    parameter integer KI_X100 = 35,
    parameter integer KD_X100 = 3500,
    parameter integer OUT_KP_X100 = 100,
    parameter integer OUT_KI_X100 = 0,
    parameter integer OUT_KD_X100 = 0,
    parameter integer SPEED_KP_X100 = 20,
    parameter integer SPEED_KI_X100 = 0,
    parameter integer SPEED_KD_X100 = 0
) (
    input wire clk,
    input wire [11:0] actual_adc,
    input wire [11:0] target_adc,
    input wire pid_active,
    input wire ain1,
    input wire ain2,
    input wire [1:0] page,
    input wire speed_mode,
    input wire [3:0] control_state,fault_code,
    input wire [2:0] fault_detail,
    input wire [11:0] fault_spread,
    input wire [3:0] recovery_attempts,
    input wire signed [31:0] position_target_deci,
    input wire signed [31:0] position_edit_deci,
    input wire signed [31:0] position_actual_deci,
    input wire signed [15:0] speed_target_dps,
    input wire signed [15:0] speed_edit_dps,
    input wire signed [31:0] speed_actual_deci,
    output wire        tft_cs_n,
    output wire        tft_sclk,
    output wire        tft_mosi,
    output wire        tft_dc,
    output wire        tft_rst_n,
    output wire        tft_bl
);
    // 同一帧使用同一次状态快照，避免角度、位置和目标跨帧错位。
    reg [11:0] frame_adc = 12'd0, frame_target_adc = 12'd0;
    reg frame_pid_on = 1'b0;
    reg frame_ain1 = 1'b0;
    reg frame_ain2 = 1'b0;
    reg [1:0] frame_page = 2'd0;
    reg frame_speed_mode = 1'b0;
    reg [3:0] frame_state=0,frame_fault=0;
    reg [2:0] frame_fault_detail=0;
    reg [11:0] frame_fault_spread=0;
    reg [3:0] frame_recovery_attempts=0;
    reg signed [31:0] frame_set_deci = 32'sd0, frame_new_deci = 32'sd0;
    reg signed [31:0] frame_pos_deci = 32'sd0;
    reg signed [15:0] frame_speed_set_dps = 16'sd0;
    reg signed [15:0] frame_speed_new_dps = 16'sd0;
    reg signed [31:0] frame_speed_actual_deci = 32'sd0;

    wire [31:0] set_abs = frame_set_deci[31] ? -frame_set_deci : frame_set_deci;
    wire [31:0] new_abs = frame_new_deci[31] ? -frame_new_deci : frame_new_deci;
    wire [31:0] pos_abs = frame_pos_deci[31] ?
        -frame_pos_deci : frame_pos_deci;
    wire [16:0] set_value = set_abs > 99999 ? 17'd99999 : set_abs[16:0];
    wire [16:0] new_value = new_abs > 99999 ? 17'd99999 : new_abs[16:0];
    wire [16:0] pos_value = pos_abs > 99999 ? 17'd99999 : pos_abs[16:0];
    wire signed [31:0] speed_set_deci = $signed(frame_speed_set_dps)*32'sd10;
    wire signed [31:0] speed_new_deci = $signed(frame_speed_new_dps)*32'sd10;
    wire [31:0] speed_set_abs = speed_set_deci[31] ? -speed_set_deci : speed_set_deci;
    wire [31:0] speed_new_abs = speed_new_deci[31] ? -speed_new_deci : speed_new_deci;
    wire [31:0] speed_actual_abs = frame_speed_actual_deci[31] ?
        -frame_speed_actual_deci : frame_speed_actual_deci;
    wire [16:0] speed_set_value = speed_set_abs > 99999 ? 17'd99999 : speed_set_abs[16:0];
    wire [16:0] speed_new_value = speed_new_abs > 99999 ? 17'd99999 : speed_new_abs[16:0];
    wire [16:0] speed_actual_value = speed_actual_abs > 99999 ?
        17'd99999 : speed_actual_abs[16:0];

    reg bcd_start = 1'b0;
    localparam [12:0] IKP_DISPLAY = KP_X100;
    localparam [12:0] IKI_DISPLAY = KI_X100;
    localparam [12:0] IKD_DISPLAY = KD_X100;
    localparam [12:0] OKP_DISPLAY = OUT_KP_X100;
    localparam [12:0] OKI_DISPLAY = OUT_KI_X100;
    localparam [12:0] OKD_DISPLAY = OUT_KD_X100;
    localparam [12:0] SKP_DISPLAY = SPEED_KP_X100;
    localparam [12:0] SKI_DISPLAY = SPEED_KI_X100;
    localparam [12:0] SKD_DISPLAY = SPEED_KD_X100;
    wire [15:0] ikp_bcd, iki_bcd, ikd_bcd, okp_bcd, oki_bcd, okd_bcd;
    wire [15:0] skp_bcd, ski_bcd, skd_bcd;
    wire [15:0] adc_bcd, target_bcd;
    wire [15:0] spread_bcd;
    wire [19:0] set_bcd, new_bcd, pos_bcd;
    wire [19:0] speed_set_bcd, speed_new_bcd, speed_actual_bcd;
    wire ikp_done, iki_done, ikd_done, okp_done, oki_done, okd_done;
    wire adc_done, target_done, set_done, new_done, pos_done;
    wire speed_set_done, speed_new_done, speed_actual_done;
    bin13_to_bcd4 u_ikp (.clk(clk),.start(bcd_start),.value(IKP_DISPLAY),
        .bcd(ikp_bcd),.busy(),.done(ikp_done));
    bin13_to_bcd4 u_iki (.clk(clk),.start(bcd_start),.value(IKI_DISPLAY),
        .bcd(iki_bcd),.busy(),.done(iki_done));
    bin13_to_bcd4 u_ikd (.clk(clk),.start(bcd_start),.value(IKD_DISPLAY),
        .bcd(ikd_bcd),.busy(),.done(ikd_done));
    bin13_to_bcd4 u_okp (.clk(clk),.start(bcd_start),.value(OKP_DISPLAY),
        .bcd(okp_bcd),.busy(),.done(okp_done));
    bin13_to_bcd4 u_oki (.clk(clk),.start(bcd_start),.value(OKI_DISPLAY),
        .bcd(oki_bcd),.busy(),.done(oki_done));
    bin13_to_bcd4 u_okd (.clk(clk),.start(bcd_start),.value(OKD_DISPLAY),
        .bcd(okd_bcd),.busy(),.done(okd_done));
    bin13_to_bcd4 u_skp (.clk(clk),.start(bcd_start),.value(SKP_DISPLAY),
        .bcd(skp_bcd),.busy(),.done());
    bin13_to_bcd4 u_ski (.clk(clk),.start(bcd_start),.value(SKI_DISPLAY),
        .bcd(ski_bcd),.busy(),.done());
    bin13_to_bcd4 u_skd (.clk(clk),.start(bcd_start),.value(SKD_DISPLAY),
        .bcd(skd_bcd),.busy(),.done());
    bin13_to_bcd4 u_adc (.clk(clk),.start(bcd_start),.value({1'b0,frame_adc}),
        .bcd(adc_bcd),.busy(),.done(adc_done));
    bin13_to_bcd4 u_spread (.clk(clk),.start(bcd_start),.value({1'b0,frame_fault_spread}),
        .bcd(spread_bcd),.busy(),.done());
    bin13_to_bcd4 u_target (.clk(clk),.start(bcd_start),
        .value({1'b0,frame_target_adc}),
        .bcd(target_bcd),.busy(),.done(target_done));
    bin17_to_bcd5 u_set (.clk(clk),.start(bcd_start),.value(set_value),
        .bcd(set_bcd),.done(set_done));
    bin17_to_bcd5 u_new (.clk(clk),.start(bcd_start),.value(new_value),
        .bcd(new_bcd),.done(new_done));
    bin17_to_bcd5 u_pos (.clk(clk),.start(bcd_start),.value(pos_value),
        .bcd(pos_bcd),.done(pos_done));
    bin17_to_bcd5 u_speed_set (.clk(clk),.start(bcd_start),.value(speed_set_value),
        .bcd(speed_set_bcd),.done(speed_set_done));
    bin17_to_bcd5 u_speed_new (.clk(clk),.start(bcd_start),.value(speed_new_value),
        .bcd(speed_new_bcd),.done(speed_new_done));
    bin17_to_bcd5 u_speed_actual (.clk(clk),.start(bcd_start),.value(speed_actual_value),
        .bcd(speed_actual_bcd),.done(speed_actual_done));

    // FPGA 上电后先等待内部初始化完成，再开始复位屏幕。
    reg [15:0] startup_count = 16'd0;
    always @(posedge clk)
        if (startup_count != 16'd50_000)
            startup_count <= startup_count + 1'b1;
    wire rst_n = (startup_count == 16'd50_000);

    reg tx_start = 1'b0;
    reg tx_dc = 1'b0;
    reg [7:0] tx_byte = 8'd0;
    wire tx_busy, tx_done;
    spi_byte_tx u_spi (
        .clk(clk), .rst_n(rst_n), .start(tx_start),
        .dc_in(tx_dc), .data_in(tx_byte),
        .busy(tx_busy), .done(tx_done),
        .cs_n(tft_cs_n), .sclk(tft_sclk),
        .mosi(tft_mosi), .dc(tft_dc)
    );

    localparam [3:0] S_RESET_LOW = 4'd0;
    localparam [3:0] S_RESET_WAIT = 4'd1;
    localparam [3:0] S_INIT_SEND = 4'd2;
    localparam [3:0] S_INIT_WAIT = 4'd3;
    localparam [3:0] S_INIT_DELAY = 4'd4;
    localparam [3:0] S_PREP_BCD = 4'd5;
    localparam [3:0] S_WAIT_BCD = 4'd6;
    localparam [3:0] S_PIXEL_SEND = 4'd7;
    localparam [3:0] S_PIXEL_WAIT = 4'd8;
    localparam [3:0] S_FRAME_DONE = 4'd9;

    reg [3:0] state = S_RESET_LOW;
    reg [23:0] delay_count = 24'd0;
    reg [4:0] init_index = 5'd0;
    reg [7:0] x = 8'd0;
    reg [8:0] y = 9'd0;
    reg pixel_low_byte = 1'b0;
    reg reset_out = 1'b0;
    reg backlight_out = 1'b0;
    assign tft_rst_n = reset_out;
    assign tft_bl = backlight_out; // 首帧写完后输出高电平点亮背光

    function [7:0] init_data;
        input [4:0] index;
        begin
            case (index)
                 0: init_data = 8'h01; // SWRESET
                 1: init_data = 8'h36; // MADCTL
                 2: init_data = 8'h08; // BGR, portrait
                 3: init_data = 8'h3A; // COLMOD
                 4: init_data = 8'h55; // RGB565
                 5: init_data = 8'h21; // INVON
                 6: init_data = 8'h11; // SLPOUT
                 7: init_data = 8'h29; // DISPON
                 8: init_data = 8'h2A; // CASET
                 9: init_data = 8'h00;
                10: init_data = 8'h00;
                11: init_data = 8'h00;
                12: init_data = 8'hEF; // column 239
                13: init_data = 8'h2B; // RASET
                14: init_data = 8'h00;
                15: init_data = 8'h00;
                16: init_data = 8'h01;
                17: init_data = 8'h3F; // row 319
                18: init_data = 8'h2C; // RAMWR
                default: init_data = 8'h00;
            endcase
        end
    endfunction

    function init_is_data;
        input [4:0] index;
        begin
            case (index)
                2,4,9,10,11,12,14,15,16,17: init_is_data = 1'b1;
                default: init_is_data = 1'b0;
            endcase
        end
    endfunction

    // 位置字符串格式：+0000.0°，屏上省去度符号。
    function [7:0] position_character;
        input [3:0] index;
        input [19:0] digits;
        input negative;
        begin
            position_character = 8'h20;
            case (index)
                4: position_character = negative ? "-" : "+";
                5: position_character = 8'h30 + {4'd0,digits[19:16]};
                6: position_character = 8'h30 + {4'd0,digits[15:12]};
                7: position_character = 8'h30 + {4'd0,digits[11:8]};
                8: position_character = 8'h30 + {4'd0,digits[7:4]};
                9: position_character = ".";
                10: position_character = 8'h30 + {4'd0,digits[3:0]};
            endcase
        end
    endfunction

    function [7:0] line_char;
        input [3:0] line;
        input [3:0] position;
        input [1:0] display_page;
        reg [15:0] gain_digits;
        reg [19:0] pos_digits;
        reg pos_negative;
        begin
            line_char = 8'h20;
            gain_digits = 16'd0;
            pos_digits = 20'd0;
            pos_negative = 1'b0;
            if (display_page == 2'd0) begin
                if (line <= 4'd5) begin
                    case (line)
                        0: gain_digits = ikp_bcd;
                        1: gain_digits = iki_bcd;
                        2: gain_digits = ikd_bcd;
                        3: gain_digits = okp_bcd;
                        4: gain_digits = oki_bcd;
                        5: gain_digits = okd_bcd;
                    endcase
                    case (position)
                        0: line_char = line <= 4'd2 ? "I" : "O";
                        1: line_char = "K";
                        2: case (line)
                            0,3: line_char = "P";
                            1,4: line_char = "I";
                            default: line_char = "D";
                        endcase
                        4: line_char = 8'h30 + {4'd0,gain_digits[15:12]};
                        5: line_char = 8'h30 + {4'd0,gain_digits[11:8]};
                        6: line_char = ".";
                        7: line_char = 8'h30 + {4'd0,gain_digits[7:4]};
                        8: line_char = 8'h30 + {4'd0,gain_digits[3:0]};
                    endcase
                end else if (line == 4'd6 || line == 4'd7) begin
                    case (position)
                        0: line_char = line == 4'd6 ? (frame_state==7 && frame_fault==4 ? "E" : "A") : "T";
                        1: line_char = line == 4'd6 ? (frame_state==7 && frame_fault==4 ? "R" : "N") : "G";
                        2: line_char = line == 4'd6 ? (frame_state==7 && frame_fault==4 ? "R" : "G") : "T";
                        4: line_char = 8'h30 + {4'd0,
                            (line == 4'd6 ? adc_bcd[15:12] : target_bcd[15:12])};
                        5: line_char = 8'h30 + {4'd0,
                            (line == 4'd6 ? adc_bcd[11:8] : target_bcd[11:8])};
                        6: line_char = 8'h30 + {4'd0,
                            (line == 4'd6 ? adc_bcd[7:4] : target_bcd[7:4])};
                        7: line_char = 8'h30 + {4'd0,
                            (line == 4'd6 ? adc_bcd[3:0] : target_bcd[3:0])};
                    endcase
                end else if (line == 4'd8 || line == 4'd9) begin
                    pos_digits = line == 4'd8 ? set_bcd : pos_bcd;
                    pos_negative = line == 4'd8 ? frame_set_deci[31] : frame_pos_deci[31];
                    if (position >= 4'd4 && position <= 4'd10)
                        line_char = position_character(position,pos_digits,pos_negative);
                    else case (position)
                        0: line_char = line == 4'd8 ? "S" : "P";
                        1: line_char = line == 4'd8 ? "E" : "O";
                        2: line_char = line == 4'd8 ? "T" : "S";
                    endcase
                end else if (line == 4'd10) begin
                    case (position) // PID9I：倒下立即切入起摆，保留连续速度模式。
                        0: line_char = "P";
                        1: line_char = "I";
                        2: line_char = "D";
                        3: line_char = "9";
                        4: line_char = "I";
                        6: line_char = "O";
                        7: line_char = frame_pid_on ? "N" : "F";
                        8: if (!frame_pid_on) line_char = "F";
                        9: line_char = frame_ain1 ? "1" : "0";
                        10: line_char = frame_ain2 ? "1" : "0";
                        12: line_char = frame_speed_mode ? "V" : "P";
                    endcase
                    // START为倒下后的停驱/横杆停稳检查；FALL/DOWN仅用于可选旧等待模式。
                    if(frame_state==0 || (frame_state>=1 && frame_state<=5) || frame_state==7 || frame_state==8 || frame_state==9 || frame_state==10)begin
                        if(position<=4)case(frame_state)
                            0:case(position)0:line_char="W";1:line_char="A";2:line_char="I";3:line_char="T";4:line_char=" ";endcase
                            7:case(position)0:line_char="F";1:line_char="A";2:line_char="U";3:line_char="L";4:line_char="T";endcase
                            8:case(position)0:line_char="F";1:line_char="A";2,3:line_char="L";4:line_char=" ";endcase
                            9:case(position)0:line_char="D";1:line_char="O";2:line_char="W";3:line_char="N";4:line_char=" ";endcase
                            10:case(position)0:line_char="S";1:line_char="T";2:line_char="A";3:line_char="R";4:line_char="T";endcase
                            default:case(position)0:line_char="S";1:line_char="W";2:line_char="N";3:line_char="G";4:line_char=" ";endcase
                        endcase
                        if(position==5)line_char=" ";
                        if(position==6)line_char="F";
                        if(position==7)line_char="0"+frame_fault;
                        if(frame_state!=7 && position==13)line_char="9";
                        if(frame_state!=7 && position==14)line_char="I";
                        if(frame_state==8 || frame_state==9 || frame_state==10)begin
                            if(position==6)line_char="R";
                            if(position==7)line_char=frame_recovery_attempts>=10 ? "1" : "0";
                            if(position==8)line_char="0"+(frame_recovery_attempts>=10 ? frame_recovery_attempts-10 : frame_recovery_attempts);
                            if(position==9 || position==10 || position==12)line_char=" ";
                        end
                        // F4后缀：W角度越界，A持续采样异常，N未就绪，O越量程。
                        if(position==13 && frame_state==7 && frame_fault==4)
                            case(frame_fault_detail)
                                1:line_char="W";2:line_char="A";3:line_char="N";4:line_char="O";
                                default:line_char=" ";
                            endcase
                        // F2后缀：T=本次起摆±120°，L=当前模式保护参考下的行程越界。
                        if(position==13 && frame_state==7 && frame_fault==2)
                            case(frame_fault_detail)
                                1:line_char="T";2:line_char="L";
                                default:line_char=" ";
                            endcase
                        if(position==13 && frame_state==7 && frame_fault==5)
                            case(frame_fault_detail)
                                1:line_char="R";2:line_char="T";
                                default:line_char=" ";
                            endcase
                    end
                end
            end else if (display_page == 2'd1) begin
                if (line <= 4'd2) begin
                    case (line)
                        0: begin pos_digits=set_bcd; pos_negative=frame_set_deci[31]; end
                        1: begin pos_digits=new_bcd; pos_negative=frame_new_deci[31]; end
                        2: begin pos_digits=pos_bcd; pos_negative=frame_pos_deci[31]; end
                    endcase
                    if (position >= 4'd4 && position <= 4'd10)
                        line_char = position_character(position,pos_digits,pos_negative);
                    else case (position)
                        0: case (line)
                            0: line_char="S"; 1: line_char="N"; 2: line_char="P";
                        endcase
                        1: case (line)
                            0: line_char="E"; 1: line_char="E"; 2: line_char="O";
                        endcase
                        2: case (line)
                            0: line_char="T"; 1: line_char="W"; 2: line_char="S";
                        endcase
                    endcase
                end else if (line == 4'd3) begin // F1 - E3 +
                    case (position)
                        0: line_char="F"; 1: line_char="1";
                        3: line_char="-"; 5: line_char="E";
                        6: line_char="3"; 8: line_char="+";
                    endcase
                end else if (line == 4'd4) begin // F2 APPLY
                    case (position)
                        0: line_char="F"; 1: line_char="2";
                        3: line_char="A"; 4,5: line_char="P";
                        6: line_char="L"; 7: line_char="Y";
                    endcase
                end else if (line == 4'd5) begin // S1 BACK
                    case (position)
                        0: line_char="S"; 1: line_char="1";
                        3: line_char="B"; 4: line_char="A";
                        5: line_char="C"; 6: line_char="K";
                    endcase
                end else if (line == 4'd6) begin
                    case (position)
                        0: line_char="P"; 1: line_char="I";
                        2: line_char="D"; 3: line_char="9";
                        4: line_char="I";
                        6: line_char="O";
                        7: line_char=frame_pid_on ? "N" : "F";
                        8: if (!frame_pid_on) line_char="F";
                    endcase
                end
            end else begin // 第三页：速度设置及角度反馈
                if (line <= 4'd2) begin
                    case (line)
                        0: begin
                            pos_digits=speed_actual_bcd;
                            pos_negative=frame_speed_actual_deci[31];
                        end
                        1: begin
                            pos_digits=speed_set_bcd;
                            pos_negative=speed_set_deci[31];
                        end
                        2: begin
                            pos_digits=speed_new_bcd;
                            pos_negative=speed_new_deci[31];
                        end
                    endcase
                    if (position >= 4'd4 && position <= 4'd10)
                        line_char=position_character(position,pos_digits,pos_negative);
                    else case (position)
                        0: case (line)
                            0: line_char="S"; 1: line_char="S"; 2: line_char="N";
                        endcase
                        1: case (line)
                            0: line_char="P"; 1: line_char="E"; 2: line_char="E";
                        endcase
                        2: case (line)
                            0: line_char="D"; 1: line_char="T"; 2: line_char="W";
                        endcase
                    endcase
                end else if (line == 4'd3 || line == 4'd4) begin
                    case (position)
                        0: line_char=line==4'd3 ? "A" : "T";
                        1: line_char=line==4'd3 ? "N" : "G";
                        2: line_char=line==4'd3 ? "G" : "T";
                        4: line_char=8'h30+{4'd0,
                            (line==4'd3 ? adc_bcd[15:12] : target_bcd[15:12])};
                        5: line_char=8'h30+{4'd0,
                            (line==4'd3 ? adc_bcd[11:8] : target_bcd[11:8])};
                        6: line_char=8'h30+{4'd0,
                            (line==4'd3 ? adc_bcd[7:4] : target_bcd[7:4])};
                        7: line_char=8'h30+{4'd0,
                            (line==4'd3 ? adc_bcd[3:0] : target_bcd[3:0])};
                    endcase
                end else if (line >= 4'd5 && line <= 4'd7) begin
                    case (line)
                        5: gain_digits=skp_bcd;
                        6: gain_digits=ski_bcd;
                        7: gain_digits=skd_bcd;
                    endcase
                    case (position)
                        0: line_char="S";
                        1: line_char="K";
                        2: case (line)
                            5: line_char="P";
                            6: line_char="I";
                            default: line_char="D";
                        endcase
                        4: line_char=8'h30+{4'd0,gain_digits[15:12]};
                        5: line_char=8'h30+{4'd0,gain_digits[11:8]};
                        6: line_char=".";
                        7: line_char=8'h30+{4'd0,gain_digits[7:4]};
                        8: line_char=8'h30+{4'd0,gain_digits[3:0]};
                    endcase
                end else if (line == 4'd8) begin // F1 - E3 +
                    case (position)
                        0: line_char="F"; 1: line_char="1";
                        3: line_char="-"; 5: line_char="E";
                        6: line_char="3"; 8: line_char="+";
                    endcase
                end else if (line == 4'd9) begin // F2 APPLY
                    case (position)
                        0: line_char="F"; 1: line_char="2";
                        3: line_char="A"; 4,5: line_char="P";
                        6: line_char="L"; 7: line_char="Y";
                    endcase
                end else if (line == 4'd10) begin // S2 BACK
                    case (position)
                        0: line_char="S"; 1: line_char="2";
                        3: line_char="B"; 4: line_char="A";
                        5: line_char="C"; 6: line_char="K";
                    endcase
                end
            end
            // F4采样异常时首行显示锁存的批内跨度，方便分辨0/2708混批。
            if(display_page==0 && frame_state==7 && frame_fault==4 && frame_fault_detail==2 && line==0)begin
                line_char=" ";
                case(position)
                    0:line_char="S";1:line_char="P";2:line_char="N";
                    4:line_char="0"+{4'd0,spread_bcd[15:12]};
                    5:line_char="0"+{4'd0,spread_bcd[11:8]};
                    6:line_char="0"+{4'd0,spread_bcd[7:4]};
                    7:line_char="0"+{4'd0,spread_bcd[3:0]};
                endcase
            end
        end
    endfunction

    function [7:0] font_row;
        input [7:0] ch;
        input [2:0] row;
        begin
            font_row = 8'h00;
            case (ch)
                "0": case (row)
                    0: font_row=8'h3C; 1: font_row=8'h66; 2: font_row=8'h6E;
                    3: font_row=8'h76; 4,5: font_row=8'h66; 6: font_row=8'h3C;
                endcase
                "1": case (row)
                    0: font_row=8'h18; 1: font_row=8'h38; 2,3,4,5: font_row=8'h18;
                    6: font_row=8'h3C;
                endcase
                "2": case (row)
                    0: font_row=8'h3C; 1: font_row=8'h66; 2: font_row=8'h06;
                    3: font_row=8'h0C; 4: font_row=8'h30; 5: font_row=8'h60;
                    6: font_row=8'h7E;
                endcase
                "3": case (row)
                    0: font_row=8'h3C; 1: font_row=8'h66; 2: font_row=8'h06;
                    3: font_row=8'h1C; 4: font_row=8'h06; 5: font_row=8'h66;
                    6: font_row=8'h3C;
                endcase
                "4": case (row)
                    0: font_row=8'h0C; 1: font_row=8'h1C; 2: font_row=8'h3C;
                    3: font_row=8'h6C; 4: font_row=8'h7E; 5,6: font_row=8'h0C;
                endcase
                "5": case (row)
                    0: font_row=8'h7E; 1: font_row=8'h60; 2: font_row=8'h7C;
                    3,4: font_row=8'h06; 5: font_row=8'h66; 6: font_row=8'h3C;
                endcase
                "6": case (row)
                    0: font_row=8'h1C; 1: font_row=8'h30; 2: font_row=8'h60;
                    3: font_row=8'h7C; 4,5: font_row=8'h66; 6: font_row=8'h3C;
                endcase
                "7": case (row)
                    0: font_row=8'h7E; 1: font_row=8'h06; 2: font_row=8'h0C;
                    3: font_row=8'h18; 4,5,6: font_row=8'h30;
                endcase
                "8": case (row)
                    0: font_row=8'h3C; 1,2: font_row=8'h66; 3: font_row=8'h3C;
                    4,5: font_row=8'h66; 6: font_row=8'h3C;
                endcase
                "9": case (row)
                    0: font_row=8'h3C; 1,2: font_row=8'h66; 3: font_row=8'h3E;
                    4: font_row=8'h06; 5: font_row=8'h0C; 6: font_row=8'h38;
                endcase
                "A": case (row)
                    0: font_row=8'h18; 1: font_row=8'h3C; 2,3: font_row=8'h66;
                    4: font_row=8'h7E; 5,6: font_row=8'h66;
                endcase
                "N": case (row)
                    0: font_row=8'h66; 1: font_row=8'h76; 2,3: font_row=8'h7E;
                    4: font_row=8'h6E; 5,6: font_row=8'h66;
                endcase
                "G": case (row)
                    0: font_row=8'h3C; 1: font_row=8'h66; 2: font_row=8'h60;
                    3: font_row=8'h6E; 4,5: font_row=8'h66; 6: font_row=8'h3C;
                endcase
                "L": case (row)
                    0,1,2,3,4,5: font_row=8'h60; 6: font_row=8'h7E;
                endcase
                "E": case (row)
                    0: font_row=8'h7E; 1,2: font_row=8'h60; 3: font_row=8'h7C;
                    4,5: font_row=8'h60; 6: font_row=8'h7E;
                endcase
                "D": case (row)
                    0: font_row=8'h7C; 1,2,3,4,5: font_row=8'h66;
                    6: font_row=8'h7C;
                endcase
                "C": case (row)
                    0: font_row=8'h3C; 1: font_row=8'h66; 2,3,4: font_row=8'h60;
                    5: font_row=8'h66; 6: font_row=8'h3C;
                endcase
                "V": case (row)
                    0,1,2,3,4: font_row=8'h66; 5: font_row=8'h3C;
                    6: font_row=8'h18;
                endcase
                "I": case (row)
                    0: font_row=8'h3C; 1,2,3,4,5: font_row=8'h18;
                    6: font_row=8'h3C;
                endcase
                "M": case (row)
                    0: font_row=8'h63; 1: font_row=8'h77; 2: font_row=8'h7F;
                    3: font_row=8'h6B; 4,5,6: font_row=8'h63;
                endcase
                "O": case (row)
                    0: font_row=8'h3C; 1,2,3,4,5: font_row=8'h66;
                    6: font_row=8'h3C;
                endcase
                "T": case (row)
                    0: font_row=8'h7E; 1,2,3,4,5,6: font_row=8'h18;
                endcase
                "R": case (row)
                    0: font_row=8'h7C; 1,2: font_row=8'h66; 3: font_row=8'h7C;
                    4: font_row=8'h78; 5: font_row=8'h6C; 6: font_row=8'h66;
                endcase
                "P": case (row)
                    0: font_row=8'h7C; 1,2: font_row=8'h66;
                    3: font_row=8'h7C; 4,5,6: font_row=8'h60;
                endcase
                "K": case (row)
                    0,1: font_row=8'h66; 2: font_row=8'h6C;
                    3: font_row=8'h78; 4: font_row=8'h6C;
                    5,6: font_row=8'h66;
                endcase
                "W": case (row)
                    0,1,2: font_row=8'h63; 3,4: font_row=8'h6B;
                    5: font_row=8'h7F; 6: font_row=8'h36;
                endcase
                "F": case (row)
                    0: font_row=8'h7E; 1,2: font_row=8'h60;
                    3: font_row=8'h7C; 4,5,6: font_row=8'h60;
                endcase
                "S": case (row)
                    0: font_row=8'h3E; 1,2: font_row=8'h60;
                    3: font_row=8'h3C; 4,5: font_row=8'h06;
                    6: font_row=8'h7C;
                endcase
                "B": case (row)
                    0: font_row=8'h7C; 1,2: font_row=8'h66;
                    3: font_row=8'h7C; 4,5: font_row=8'h66;
                    6: font_row=8'h7C;
                endcase
                "Y": case (row)
                    0,1: font_row=8'h66; 2: font_row=8'h3C;
                    3,4,5: font_row=8'h18; 6: font_row=8'h3C;
                endcase
                "+": case (row)
                    1,2,4,5: font_row=8'h18; 3: font_row=8'h7E;
                endcase
                "-": case (row)
                    3: font_row=8'h7E;
                endcase
                ".": case (row)
                    5,6: font_row=8'h18;
                endcase
                default: font_row=8'h00;
            endcase
        end
    endfunction

    // 2 倍 8×8 字模，每行高 16 像素；第一页共 11 行。
    reg [3:0] text_line;
    reg [8:0] text_y;
    reg text_area;
    always @* begin
        text_line = 4'd0;
        text_y = 9'd0;
        text_area = 1'b1;
        if      (y >= 9'd8   && y < 9'd24)  begin text_line=0;  text_y=y-9'd8;   end
        else if (y >= 9'd36  && y < 9'd52)  begin text_line=1;  text_y=y-9'd36;  end
        else if (y >= 9'd64  && y < 9'd80)  begin text_line=2;  text_y=y-9'd64;  end
        else if (y >= 9'd92  && y < 9'd108) begin text_line=3;  text_y=y-9'd92;  end
        else if (y >= 9'd120 && y < 9'd136) begin text_line=4;  text_y=y-9'd120; end
        else if (y >= 9'd148 && y < 9'd164) begin text_line=5;  text_y=y-9'd148; end
        else if (y >= 9'd176 && y < 9'd192) begin text_line=6;  text_y=y-9'd176; end
        else if (y >= 9'd204 && y < 9'd220) begin text_line=7;  text_y=y-9'd204; end
        else if (y >= 9'd232 && y < 9'd248) begin text_line=8;  text_y=y-9'd232; end
        else if (y >= 9'd260 && y < 9'd276) begin text_line=9;  text_y=y-9'd260; end
        else if (y >= 9'd288 && y < 9'd304) begin text_line=10; text_y=y-9'd288; end
        else text_area = 1'b0;
    end
    wire [7:0] text_char = line_char(text_line, x[7:4], frame_page);
    wire [7:0] glyph_row = font_row(text_char, text_y[3:1]);
    wire text_pixel = text_area && glyph_row[3'd7-x[3:1]];
    wire [15:0] pixel_color = !text_pixel ? 16'h0000 :
        (((frame_page==2'd0) && (text_line==4'd6 || text_line==4'd9)) ||
          ((frame_page==2'd1) && text_line==4'd2) ||
          ((frame_page==2'd2) && (text_line==4'd0 || text_line==4'd3))) ? 16'h07FF :
        (((frame_page==2'd0) && (text_line==4'd7 || text_line==4'd8)) ||
          ((frame_page==2'd1) && text_line==4'd1) ||
          ((frame_page==2'd2) && (text_line==4'd2 || text_line==4'd4))) ? 16'hFFE0 :
        (((frame_page==2'd0) && text_line==4'd10) ||
          ((frame_page==2'd1) && text_line==4'd6)) ?
              (frame_pid_on ? 16'h07E0 : 16'hF800) :
        16'hFFFF;

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_RESET_LOW;
            delay_count <= 24'd0;
            init_index <= 5'd0;
            x <= 8'd0;
            y <= 9'd0;
            pixel_low_byte <= 1'b0;
            reset_out <= 1'b0;
            backlight_out <= 1'b0;
            tx_start <= 1'b0;
            tx_dc <= 1'b0;
            tx_byte <= 8'd0;
            bcd_start <= 1'b0;
            frame_adc <= 12'd0;
            frame_target_adc <= 12'd0;
            frame_pid_on <= 1'b0;
            frame_ain1 <= 1'b0;
            frame_ain2 <= 1'b0;
            frame_page <= 2'd0;
            frame_speed_mode <= 1'b0;
            frame_state<=0;frame_fault<=0;frame_fault_detail<=0;frame_fault_spread<=0;
            frame_recovery_attempts<=0;
            frame_set_deci <= 32'sd0;
            frame_new_deci <= 32'sd0;
            frame_pos_deci <= 32'sd0;
            frame_speed_set_dps <= 16'sd0;
            frame_speed_new_dps <= 16'sd0;
            frame_speed_actual_deci <= 32'sd0;
        end else begin
            tx_start <= 1'b0;
            bcd_start <= 1'b0;
            case (state)
                S_RESET_LOW: begin
                    reset_out <= 1'b0;
                    if (delay_count == 24'd2_500_000-1) begin
                        delay_count <= 24'd0;
                        reset_out <= 1'b1;
                        state <= S_RESET_WAIT;
                    end else delay_count <= delay_count + 1'b1;
                end
                S_RESET_WAIT: begin
                    if (delay_count == 24'd7_500_000-1) begin
                        delay_count <= 24'd0;
                        state <= S_INIT_SEND;
                    end else delay_count <= delay_count + 1'b1;
                end
                S_INIT_SEND: if (!tx_busy) begin
                    tx_byte <= init_data(init_index);
                    tx_dc <= init_is_data(init_index);
                    tx_start <= 1'b1;
                    state <= S_INIT_WAIT;
                end
                S_INIT_WAIT: if (tx_done) begin
                    if (init_index == 5'd18) begin
                        // 发送 RAMWR 后，只锁存画面快照；PID 独立运行。
                        frame_adc <= actual_adc;
                        frame_target_adc <= target_adc;
                        frame_pid_on <= pid_active;
                        frame_ain1 <= ain1;
                        frame_ain2 <= ain2;
                        frame_page <= page;
                        frame_speed_mode <= speed_mode;
                        frame_state<=control_state;frame_fault<=fault_code;frame_fault_detail<=fault_detail;
                        frame_fault_spread<=fault_spread;
                        frame_recovery_attempts<=recovery_attempts;
                        frame_set_deci <= position_target_deci;
                        frame_new_deci <= position_edit_deci;
                        frame_pos_deci <= position_actual_deci;
                        frame_speed_set_dps <= speed_target_dps;
                        frame_speed_new_dps <= speed_edit_dps;
                        frame_speed_actual_deci <= speed_actual_deci;
                        x <= 8'd0;
                        y <= 9'd0;
                        pixel_low_byte <= 1'b0;
                        state <= S_PREP_BCD;
                    end else if (init_index == 5'd0) begin
                        init_index <= init_index + 1'b1;
                        delay_count <= 24'd7_500_000;
                        state <= S_INIT_DELAY;
                    end else if (init_index == 5'd6) begin
                        init_index <= init_index + 1'b1;
                        delay_count <= 24'd6_000_000;
                        state <= S_INIT_DELAY;
                    end else if (init_index == 5'd7) begin
                        init_index <= init_index + 1'b1;
                        delay_count <= 24'd2_500_000;
                        state <= S_INIT_DELAY;
                    end else begin
                        init_index <= init_index + 1'b1;
                        state <= S_INIT_SEND;
                    end
                end
                S_INIT_DELAY: begin
                    if (delay_count == 24'd0)
                        state <= S_INIT_SEND;
                    else delay_count <= delay_count - 1'b1;
                end
                S_PREP_BCD: begin
                    bcd_start <= 1'b1;
                    state <= S_WAIT_BCD;
                end
                // 四位转换 13 拍，五位转换 17 拍；前者结果已稳定，
                // 只等三个五位转换完成，不能要求不同周期的 done 同拍。
                S_WAIT_BCD: if (set_done && new_done && pos_done &&
                               speed_set_done && speed_new_done &&
                               speed_actual_done)
                    state <= S_PIXEL_SEND;
                S_PIXEL_SEND: if (!tx_busy) begin
                    tx_byte <= pixel_low_byte ? pixel_color[7:0] :
                                                 pixel_color[15:8];
                    tx_dc <= 1'b1;
                    tx_start <= 1'b1;
                    state <= S_PIXEL_WAIT;
                end
                S_PIXEL_WAIT: if (tx_done) begin
                    if (!pixel_low_byte) begin
                        pixel_low_byte <= 1'b1;
                    end else begin
                        pixel_low_byte <= 1'b0;
                        if (x == 8'd239) begin
                            x <= 8'd0;
                            if (y == 9'd319)
                                state <= S_FRAME_DONE;
                            else
                                y <= y + 1'b1;
                        end else x <= x + 1'b1;
                    end
                    if (!(pixel_low_byte && x == 8'd239 && y == 9'd319))
                        state <= S_PIXEL_SEND;
                end
                S_FRAME_DONE: begin
                    backlight_out <= 1'b1;
                    init_index <= 5'd18; // 窗口地址保持不变，重新发 RAMWR 刷新
                    state <= S_INIT_SEND;
                end
                default: state <= S_RESET_LOW;
            endcase
        end
    end
endmodule
