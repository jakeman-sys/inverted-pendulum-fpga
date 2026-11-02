// ST7789VM 单字节 SPI 发送器：Mode 0、最高位先发，只写不读。
// 输入时钟 50 MHz，SPI 时钟 5 MHz；每个字节发送完会释放 CS。
module spi_byte_tx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire       dc_in,
    input  wire [7:0] data_in,
    output reg        busy = 1'b0,
    output reg        done = 1'b0,
    output reg        cs_n = 1'b1,
    output reg        sclk = 1'b0,
    output reg        mosi = 1'b0,
    output reg        dc = 1'b0
);
    reg [2:0] divider = 3'd0;
    reg [2:0] bit_index = 3'd0;
    reg [7:0] shift = 8'd0;

    always @(posedge clk) begin
        if (!rst_n) begin
            busy <= 1'b0;
            done <= 1'b0;
            cs_n <= 1'b1;
            sclk <= 1'b0;
            mosi <= 1'b0;
            dc <= 1'b0;
            divider <= 3'd0;
            bit_index <= 3'd0;
            shift <= 8'd0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                busy <= 1'b1;
                cs_n <= 1'b0;
                sclk <= 1'b0;
                dc <= dc_in;
                shift <= data_in;
                mosi <= data_in[7];
                bit_index <= 3'd0;
                divider <= 3'd0;
            end else if (busy) begin
                if (divider == 3'd4) begin
                    divider <= 3'd0;
                    if (!sclk) begin
                        sclk <= 1'b1;
                    end else begin
                        sclk <= 1'b0;
                        if (bit_index == 3'd7) begin
                            busy <= 1'b0;
                            cs_n <= 1'b1;
                            done <= 1'b1;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            shift <= {shift[6:0], 1'b0};
                            mosi <= shift[6];
                        end
                    end
                end else begin
                    divider <= divider + 1'b1;
                end
            end
        end
    end
endmodule
