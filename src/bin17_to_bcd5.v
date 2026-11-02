// 0..99999 十分之一度显示，17 个周期完成五位十进制转换。
module bin17_to_bcd5 (
    input wire clk,
    input wire start,
    input wire [16:0] value,
    output reg [19:0] bcd = 20'd0,
    output reg done = 1'b0
);
    reg [16:0] binary_shift = 17'd0;
    reg [19:0] bcd_shift = 20'd0;
    reg [4:0] bit_count = 5'd0;
    reg busy = 1'b0;
    function [3:0] add3;
        input [3:0] digit;
        begin add3 = digit >= 4'd5 ? digit + 4'd3 : digit; end
    endfunction
    wire [19:0] adjusted = {
        add3(bcd_shift[19:16]), add3(bcd_shift[15:12]),
        add3(bcd_shift[11:8]), add3(bcd_shift[7:4]),
        add3(bcd_shift[3:0])
    };
    wire [19:0] shifted = {adjusted[18:0],binary_shift[16]};
    always @(posedge clk) begin
        done <= 1'b0;
        if (start && !busy) begin
            binary_shift <= value;
            bcd_shift <= 20'd0;
            bit_count <= 5'd0;
            busy <= 1'b1;
        end else if (busy) begin
            binary_shift <= {binary_shift[15:0],1'b0};
            bcd_shift <= shifted;
            if (bit_count == 5'd16) begin
                bcd <= shifted;
                busy <= 1'b0;
                done <= 1'b1;
            end else
                bit_count <= bit_count + 1'b1;
        end
    end
endmodule
