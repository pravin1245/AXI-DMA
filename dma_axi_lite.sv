`timescale 1ns / 1ps

module dma_axi_lite #(
    parameter integer ADDR_WIDTH = 7
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire [ADDR_WIDTH-1:0] awaddr,
    input  wire                  awvalid,
    output wire                  awready,
    input  wire [31:0]           wdata,
    input  wire [3:0]            wstrb,
    input  wire                  wvalid,
    output wire                  wready,
    output wire [1:0]            bresp,
    output reg                   bvalid,
    input  wire                  bready,
    input  wire [ADDR_WIDTH-1:0] araddr,
    input  wire                  arvalid,
    output wire                  arready,
    output reg  [31:0]           rdata,
    output wire [1:0]            rresp,
    output reg                   rvalid,
    input  wire                  rready,

    output reg                   reg_wr_en,
    output reg  [ADDR_WIDTH-1:0] reg_wr_addr,
    output reg  [31:0]           reg_wr_data,
    output reg  [3:0]            reg_wr_strb,
    output reg  [ADDR_WIDTH-1:0] reg_rd_addr,
    input  wire [31:0]           reg_rd_data
);
    reg aw_got, w_got, ar_got;

    assign bresp   = 2'b00;
    assign rresp   = 2'b00;
    assign awready = ~aw_got & ~bvalid;
    assign wready  = ~w_got  & ~bvalid;
    assign arready = ~ar_got & ~rvalid;

    always @(posedge clk) begin
        if (!rst_n) begin
            aw_got      <= 1'b0;
            w_got       <= 1'b0;
            bvalid      <= 1'b0;
            reg_wr_en   <= 1'b0;
            reg_wr_addr <= {ADDR_WIDTH{1'b0}};
            reg_wr_data <= 32'd0;
            reg_wr_strb <= 4'd0;
        end else begin
            reg_wr_en <= 1'b0;

            if (awvalid && awready) begin
                aw_got      <= 1'b1;
                reg_wr_addr <= awaddr;
            end
            if (wvalid && wready) begin
                w_got       <= 1'b1;
                reg_wr_data <= wdata;
                reg_wr_strb <= wstrb;
            end

            if (aw_got && w_got && !bvalid) begin
                reg_wr_en <= 1'b1;
                aw_got    <= 1'b0;
                w_got     <= 1'b0;
                bvalid    <= 1'b1;
            end else if (bvalid && bready) begin
                bvalid <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            ar_got      <= 1'b0;
            rvalid      <= 1'b0;
            rdata       <= 32'd0;
            reg_rd_addr <= {ADDR_WIDTH{1'b0}};
        end else begin
            if (arvalid && arready) begin
                ar_got      <= 1'b1;
                reg_rd_addr <= araddr;
            end

            if (ar_got && !rvalid) begin
                rdata  <= reg_rd_data;
                rvalid <= 1'b1;
                ar_got <= 1'b0;
            end else if (rvalid && rready) begin
                rvalid <= 1'b0;
            end
        end
    end
endmodule
