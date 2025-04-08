`timescale 1ns / 1ps

module diff_detector_pixel (
    input logic [3:0] prev_pixel,
    input logic [3:0] curr_pixel,
    output logic diff_detected
);

    assign diff_detected = (prev_pixel != curr_pixel);

endmodule


module diff_pixel_counter (
    input logic clk,
    input logic reset,
    input logic frame_done,
    input logic diff_detected,

    output logic [$clog2(320 * 240) - 1 : 0] diff_pixel_cnt
);
    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            diff_pixel_cnt <= 0;
        end else if (frame_done) begin
            diff_pixel_cnt <= 0;
        end else if (diff_detected) begin
            diff_pixel_cnt <= diff_pixel_cnt + 1;
        end
    end
endmodule

