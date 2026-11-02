// 逐位“加 3 移位”转换：把二进制整数变成四位十进制，供 TFT 字模显示。
// 输入范围 0..8191；start 后经过 13 个时钟，done 脉冲表示 bcd 已更新。
module bin13_to_bcd4 (
    input  wire        clk,
    input  wire        start,
    input  wire [12:0] value,
    output reg  [15:0] bcd = 16'd0,
    output reg         busy = 1'b0,
    output reg         done = 1'b0
);
    reg [12:0] binary_shift = 13'd0;
    reg [15:0] bcd_shift = 16'd0;
    reg [3:0] bit_count = 4'd0;
    function [3:0] add_three_if_five;
        input [3:0] digit;
        begin
            add_three_if_five = digit >= 4'd5 ? digit + 4'd3 : digit;
        end
    endfunction
    wire [15:0] adjusted = {
        add_three_if_five(bcd_shift[15:12]),
        add_three_if_five(bcd_shift[11:8]),
        add_three_if_five(bcd_shift[7:4]),
        add_three_if_five(bcd_shift[3:0])
    };
    wire [15:0] shifted = {adjusted[14:0], binary_shift[12]};

    always @(posedge clk) begin
        done <= 1'b0;
        if (start && !busy) begin
            binary_shift <= value;
            bcd_shift <= 16'd0;
            bit_count <= 4'd0;
            busy <= 1'b1;
        end else if (busy) begin
            binary_shift <= {binary_shift[11:0], 1'b0};
            bcd_shift <= shifted;
            if (bit_count == 4'd12) begin
                bcd <= shifted;
                busy <= 1'b0;
                done <= 1'b1;
            end else begin
                bit_count <= bit_count + 1'b1;
            end
        end
    end
endmodule
