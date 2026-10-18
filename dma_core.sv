`timescale 1ns / 1ps

module dma_core #(
    parameter integer LENGTH_WIDTH           = 23,
    parameter integer ADDR_WIDTH             = 32,
    parameter integer LITE_ADDR_WIDTH        = 7,
    parameter integer MM2S_MM_DATA_WIDTH     = 32,
    parameter integer MM2S_STREAM_DATA_WIDTH = 8,
    parameter integer MM2S_MAX_BURST         = 16,
    parameter integer S2MM_MM_DATA_WIDTH     = 32,
    parameter integer S2MM_STREAM_DATA_WIDTH = 8,
    parameter integer S2MM_MAX_BURST         = 16,
    parameter integer FIFO_DEPTH             = 64
)(
    input  wire                                aclk,
    input  wire                                aresetn,

    input  wire [LITE_ADDR_WIDTH-1:0]          s_axi_lite_awaddr,
    input  wire                                s_axi_lite_awvalid,
    output wire                                s_axi_lite_awready,
    input  wire [31:0]                         s_axi_lite_wdata,
    input  wire [3:0]                          s_axi_lite_wstrb,
    input  wire                                s_axi_lite_wvalid,
    output wire                                s_axi_lite_wready,
    output wire [1:0]                          s_axi_lite_bresp,
    output wire                                s_axi_lite_bvalid,
    input  wire                                s_axi_lite_bready,
    input  wire [LITE_ADDR_WIDTH-1:0]          s_axi_lite_araddr,
    input  wire                                s_axi_lite_arvalid,
    output wire                                s_axi_lite_arready,
    output wire [31:0]                         s_axi_lite_rdata,
    output wire [1:0]                          s_axi_lite_rresp,
    output wire                                s_axi_lite_rvalid,
    input  wire                                s_axi_lite_rready,

    output wire [ADDR_WIDTH-1:0]               m_axi_mm2s_araddr,
    output wire [7:0]                          m_axi_mm2s_arlen,
    output wire [2:0]                          m_axi_mm2s_arsize,
    output wire [1:0]                          m_axi_mm2s_arburst,
    output wire                                m_axi_mm2s_arvalid,
    input  wire                                m_axi_mm2s_arready,
    input  wire [MM2S_MM_DATA_WIDTH-1:0]       m_axi_mm2s_rdata,
    input  wire [1:0]                          m_axi_mm2s_rresp,
    input  wire                                m_axi_mm2s_rlast,
    input  wire                                m_axi_mm2s_rvalid,
    output wire                                m_axi_mm2s_rready,

    output wire [ADDR_WIDTH-1:0]               m_axi_s2mm_awaddr,
    output wire [7:0]                          m_axi_s2mm_awlen,
    output wire [2:0]                          m_axi_s2mm_awsize,
    output wire [1:0]                          m_axi_s2mm_awburst,
    output wire                                m_axi_s2mm_awvalid,
    input  wire                                m_axi_s2mm_awready,
    output wire [S2MM_MM_DATA_WIDTH-1:0]       m_axi_s2mm_wdata,
    output wire [S2MM_MM_DATA_WIDTH/8-1:0]     m_axi_s2mm_wstrb,
    output wire                                m_axi_s2mm_wlast,
    output wire                                m_axi_s2mm_wvalid,
    input  wire                                m_axi_s2mm_wready,
    input  wire [1:0]                          m_axi_s2mm_bresp,
    input  wire                                m_axi_s2mm_bvalid,
    output wire                                m_axi_s2mm_bready,

    output wire [MM2S_STREAM_DATA_WIDTH-1:0]   m_axis_mm2s_tdata,
    output wire                                m_axis_mm2s_tlast,
    output wire                                m_axis_mm2s_tvalid,
    input  wire                                m_axis_mm2s_tready,

    input  wire [S2MM_STREAM_DATA_WIDTH-1:0]   s_axis_s2mm_tdata,
    input  wire                                s_axis_s2mm_tlast,
    input  wire                                s_axis_s2mm_tvalid,
    output wire                                s_axis_s2mm_tready
);
    wire rst_n;

    dma_reset u_rst (.clk(aclk), .rst_n_in(aresetn), .rst_n_out(rst_n));

    wire                        reg_wr_en;
    wire [LITE_ADDR_WIDTH-1:0]  reg_wr_addr;
    wire [31:0]                 reg_wr_data;
    wire [3:0]                  reg_wr_strb;
    wire [LITE_ADDR_WIDTH-1:0]  reg_rd_addr;
    wire [31:0]                 reg_rd_data;

    dma_axi_lite #(.ADDR_WIDTH(LITE_ADDR_WIDTH)) u_lite (
        .clk(aclk), .rst_n(rst_n),
        .awaddr(s_axi_lite_awaddr), .awvalid(s_axi_lite_awvalid), .awready(s_axi_lite_awready),
        .wdata(s_axi_lite_wdata), .wstrb(s_axi_lite_wstrb),
        .wvalid(s_axi_lite_wvalid), .wready(s_axi_lite_wready),
        .bresp(s_axi_lite_bresp), .bvalid(s_axi_lite_bvalid), .bready(s_axi_lite_bready),
        .araddr(s_axi_lite_araddr), .arvalid(s_axi_lite_arvalid), .arready(s_axi_lite_arready),
        .rdata(s_axi_lite_rdata), .rresp(s_axi_lite_rresp),
        .rvalid(s_axi_lite_rvalid), .rready(s_axi_lite_rready),
        .reg_wr_en(reg_wr_en), .reg_wr_addr(reg_wr_addr),
        .reg_wr_data(reg_wr_data), .reg_wr_strb(reg_wr_strb),
        .reg_rd_addr(reg_rd_addr), .reg_rd_data(reg_rd_data)
    );

    wire                    mm_cmd_valid, mm_abort, mm_done, mm_err_slv, mm_err_dec;
    wire [ADDR_WIDTH-1:0]   mm_cmd_addr;
    wire [LENGTH_WIDTH-1:0] mm_cmd_len;
    wire                    sm_cmd_valid, sm_abort, sm_done, sm_err_slv, sm_err_dec;
    wire [ADDR_WIDTH-1:0]   sm_cmd_addr;
    wire [LENGTH_WIDTH-1:0] sm_cmd_len, sm_rx_bytes;

    dma_registers #(
        .ADDR_WIDTH(ADDR_WIDTH), .LITE_ADDR_WIDTH(LITE_ADDR_WIDTH), .LENGTH_WIDTH(LENGTH_WIDTH)
    ) u_regs (
        .clk(aclk), .rst_n(rst_n),
        .wr_en(reg_wr_en), .wr_addr(reg_wr_addr), .wr_data(reg_wr_data), .wr_strb(reg_wr_strb),
        .rd_addr(reg_rd_addr), .rd_data(reg_rd_data),
        .mm_cmd_valid(mm_cmd_valid), .mm_cmd_addr(mm_cmd_addr), .mm_cmd_len(mm_cmd_len),
        .mm_abort(mm_abort), .mm_done(mm_done), .mm_err_slv(mm_err_slv), .mm_err_dec(mm_err_dec),
        .sm_cmd_valid(sm_cmd_valid), .sm_cmd_addr(sm_cmd_addr), .sm_cmd_len(sm_cmd_len),
        .sm_abort(sm_abort), .sm_done(sm_done), .sm_err_slv(sm_err_slv), .sm_err_dec(sm_err_dec),
        .sm_rx_bytes(sm_rx_bytes)
    );

    dma_mm2s #(
        .ADDR_WIDTH(ADDR_WIDTH), .MM_WIDTH(MM2S_MM_DATA_WIDTH), .ST_WIDTH(MM2S_STREAM_DATA_WIDTH),
        .LENGTH_WIDTH(LENGTH_WIDTH), .MAX_BURST(MM2S_MAX_BURST), .FIFO_DEPTH(FIFO_DEPTH)
    ) u_mm2s (
        .clk(aclk), .rst_n(rst_n),
        .cmd_valid(mm_cmd_valid), .cmd_addr(mm_cmd_addr), .cmd_len(mm_cmd_len),
        .abort(mm_abort), .busy(), .done(mm_done), .err_slv(mm_err_slv), .err_dec(mm_err_dec),
        .araddr(m_axi_mm2s_araddr), .arlen(m_axi_mm2s_arlen), .arsize(m_axi_mm2s_arsize),
        .arburst(m_axi_mm2s_arburst), .arvalid(m_axi_mm2s_arvalid), .arready(m_axi_mm2s_arready),
        .rdata(m_axi_mm2s_rdata), .rresp(m_axi_mm2s_rresp), .rlast(m_axi_mm2s_rlast),
        .rvalid(m_axi_mm2s_rvalid), .rready(m_axi_mm2s_rready),
        .tdata(m_axis_mm2s_tdata), .tlast(m_axis_mm2s_tlast),
        .tvalid(m_axis_mm2s_tvalid), .tready(m_axis_mm2s_tready)
    );

    dma_s2mm #(
        .ADDR_WIDTH(ADDR_WIDTH), .MM_WIDTH(S2MM_MM_DATA_WIDTH), .ST_WIDTH(S2MM_STREAM_DATA_WIDTH),
        .LENGTH_WIDTH(LENGTH_WIDTH), .MAX_BURST(S2MM_MAX_BURST), .FIFO_DEPTH(FIFO_DEPTH)
    ) u_s2mm (
        .clk(aclk), .rst_n(rst_n),
        .cmd_valid(sm_cmd_valid), .cmd_addr(sm_cmd_addr), .cmd_len(sm_cmd_len),
        .abort(sm_abort), .busy(), .done(sm_done), .err_slv(sm_err_slv), .err_dec(sm_err_dec),
        .rx_bytes(sm_rx_bytes),
        .awaddr(m_axi_s2mm_awaddr), .awlen(m_axi_s2mm_awlen), .awsize(m_axi_s2mm_awsize),
        .awburst(m_axi_s2mm_awburst), .awvalid(m_axi_s2mm_awvalid), .awready(m_axi_s2mm_awready),
        .wdata(m_axi_s2mm_wdata), .wstrb(m_axi_s2mm_wstrb), .wlast(m_axi_s2mm_wlast),
        .wvalid(m_axi_s2mm_wvalid), .wready(m_axi_s2mm_wready),
        .bresp(m_axi_s2mm_bresp), .bvalid(m_axi_s2mm_bvalid), .bready(m_axi_s2mm_bready),
        .tdata(s_axis_s2mm_tdata), .tlast(s_axis_s2mm_tlast),
        .tvalid(s_axis_s2mm_tvalid), .tready(s_axis_s2mm_tready)
    );
endmodule
