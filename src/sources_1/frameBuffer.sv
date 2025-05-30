`timescale 1ns / 1ps



module frameBuffer_4bit (
    // write side
    input  logic        wclk,
    input  logic        we,
    input  logic [31:0] wAddr,
    input  logic [3:0] wData,
    // read side
    input  logic        rclk,
    input  logic        oe,
    input  logic [31:0] rAddr,
    output logic [3:0] rData
);
    logic [3:0] mem[0 : (320 * 240 - 1)];

    // write side 
    always_ff @(posedge wclk) begin
        if (we) begin
            mem[wAddr] <= wData;
        end
    end

    // read side
    always_ff @( posedge rclk ) begin
        if (oe) begin
            rData <= mem[rAddr];
        end
    end
endmodule

