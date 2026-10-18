`timescale 1ns / 1ps

module dma_reset (
    input  wire clk,
    input  wire rst_n_in,
    output reg  rst_n_out
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] sr = 2'b00;

    initial rst_n_out = 1'b0;

    always @(posedge clk) begin
        sr        <= {sr[0], rst_n_in};
        rst_n_out <= sr[1];
    end
endmodule
