// 高电平按下。短按在稳定按下时立即产生一次 press；持续按住超过
// HOLD_CYCLES 后按 REPEAT_CYCLES 周期产生 repeat。
module button_repeat #(
    parameter integer DEBOUNCE_CYCLES = 1_000_000,
    parameter integer HOLD_CYCLES = 25_000_000,
    parameter integer REPEAT_CYCLES = 5_000_000
) (
    input wire clk,
    input wire button,
    output reg press = 1'b0,
    output reg repeat_press = 1'b0,
    output reg stable = 1'b0
);
    reg meta = 1'b0, sync = 1'b0;
    reg [19:0] db_count = 20'd0;
    reg [24:0] hold_count = 25'd0;
    reg [22:0] repeat_count = 23'd0;
    reg repeating = 1'b0;
    reg released=0;
    reg [31:0] release_count=0;
    always @(posedge clk) begin
        meta <= button;
        sync <= meta;
        press <= 1'b0;
        repeat_press <= 1'b0;
        // 上电一直按住不会启动；先稳定松开一次，再允许产生按下事件。
        if(!released)begin
            if(sync)release_count<=0;
            else if(release_count==DEBOUNCE_CYCLES-1)released<=1;
            else release_count<=release_count+1'b1;
        end
        if (sync == stable)
            db_count <= 20'd0;
        else if (db_count == DEBOUNCE_CYCLES-1) begin
            stable <= sync;
            db_count <= 20'd0;
            if (sync && released) press <= 1'b1;
        end else
            db_count <= db_count + 1'b1;

        if (!stable || !released) begin
            hold_count <= 25'd0;
            repeat_count <= 23'd0;
            repeating <= 1'b0;
        end else if (!repeating) begin
            if (hold_count == HOLD_CYCLES-1) begin
                repeating <= 1'b1;
                repeat_press <= 1'b1;
                repeat_count <= 23'd0;
            end else
                hold_count <= hold_count + 1'b1;
        end else if (repeat_count == REPEAT_CYCLES-1) begin
            repeat_press <= 1'b1;
            repeat_count <= 23'd0;
        end else
            repeat_count <= repeat_count + 1'b1;
    end
endmodule
