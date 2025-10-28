module cam_word_to_bytes (
    input  wire        clk,
    input  wire        rst_n,

    // 输入：来自 cam_to_udp_serializer
    input  wire [31:0] in_word,
    input  wire        in_word_valid, // 单词有效

    // 输出：字节流 (udp 域)
    output reg  [7:0]  out_byte,
    output reg         out_byte_valid,
    // 每个 32-bit 单词最后一个字节脉冲（可用于打帧或计数）
    output reg         out_word_last
);

reg [1:0] byte_state;
reg [31:0] latched_word;
reg        active;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_byte <= 8'd0;
        out_byte_valid <= 1'b0;
        out_word_last <= 1'b0;
        byte_state <= 2'd0;
        latched_word <= 32'd0;
        active <= 1'b0;
    end else begin
        out_byte_valid <= 1'b0;
        out_word_last <= 1'b0;

        if (!active) begin
            if (in_word_valid) begin
                latched_word <= in_word;
                active <= 1'b1;
                byte_state <= 2'd0;
                // emit first byte immediately
                out_byte <= in_word[23:16]; // R
                out_byte_valid <= 1'b1;
            end
        end else begin
            // after emitting first byte, produce remaining bytes in subsequent cycles
            if (!out_byte_valid) begin
                case (byte_state)
                    2'd0: begin
                        byte_state <= 2'd1;
                        out_byte <= latched_word[15:8]; // G
                        out_byte_valid <= 1'b1;
                    end
                    2'd1: begin
                        byte_state <= 2'd2;
                        out_byte <= latched_word[7:0]; // B
                        out_byte_valid <= 1'b1;
                        out_word_last <= 1'b1;
                    end
                    2'd2: begin
                        active <= 1'b0; // done with this word
                    end
                endcase
            end
        end

        // allow immediate latching of next word if present and we're free
        if (!active && in_word_valid && !out_byte_valid) begin
            latched_word <= in_word;
            active <= 1'b1;
            byte_state <= 2'd0;
            out_byte <= in_word[23:16];
            out_byte_valid <= 1'b1;
        end
    end
end
endmodule
