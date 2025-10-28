`timescale 1ns / 1ps
// cam_to_udp_serializer.v
// 在 udp_clk 域读取异步 FIFO 的 32-bit 数据并序列化为 8-bit 字节。
// 输出：app_data/app_valid；同时输出 app_length 和 app_port（常量，若需改可在模块中调整）
module cam_to_udp_serializer (
    input           clk,
    input           rst_n,
    input   [8:0]   fifo_rd_usedw,  // FIFO 已用字数
    input   [31:0]  fifo_dout,      // FIFO 输出
    output  reg     fifo_re,        // 读请求到 FIFO（pulse）
    output  reg [31:0] cam_data,      // 输出32位数据
    output  reg        cam_data_valid // 数据有效信号
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_re        <= 1'b0;
        cam_data       <= 32'd0;
        cam_data_valid <= 1'b0;
    end
    else begin
        fifo_re        <= 1'b0;
        cam_data_valid <= 1'b0;
        if (fifo_rd_usedw != 9'd0) begin
            fifo_re <= 1'b1;
            cam_data <= fifo_dout;
            cam_data_valid <= 1'b1;
        end
    end
end

endmodule