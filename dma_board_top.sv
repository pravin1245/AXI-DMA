`timescale 1ns / 1ps

module dma_board_top #(
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
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI_LITE:S_AXI_S2MM:M_AXI_MM2S:M_AXI_S2MM:M_AXIS_MM2S,FREQ_HZ 100000000,PHASE 0.0,CLK_DOMAIN dma_board_top_aclk" *)
    input wire aclk,

    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input wire aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE AWADDR" *)
    input wire [LITE_ADDR_WIDTH-1:0] S_AXI_LITE_AWADDR,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE AWVALID" *)
    input wire S_AXI_LITE_AWVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE AWREADY" *)
    output wire S_AXI_LITE_AWREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WDATA" *)
    input wire [31:0] S_AXI_LITE_WDATA,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WSTRB" *)
    input wire [3:0] S_AXI_LITE_WSTRB,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WVALID" *)
    input wire S_AXI_LITE_WVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE WREADY" *)
    output wire S_AXI_LITE_WREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE BRESP" *)
    output wire [1:0] S_AXI_LITE_BRESP,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE BVALID" *)
    output wire S_AXI_LITE_BVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE BREADY" *)
    input wire S_AXI_LITE_BREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE ARADDR" *)
    input wire [LITE_ADDR_WIDTH-1:0] S_AXI_LITE_ARADDR,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE ARVALID" *)
    input wire S_AXI_LITE_ARVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE ARREADY" *)
    output wire S_AXI_LITE_ARREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RDATA" *)
    output wire [31:0] S_AXI_LITE_RDATA,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RRESP" *)
    output wire [1:0] S_AXI_LITE_RRESP,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RVALID" *)
    output wire S_AXI_LITE_RVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI_LITE RREADY" *)
    input wire S_AXI_LITE_RREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARADDR" *)
    output wire [ADDR_WIDTH-1:0] M_AXI_MM2S_ARADDR,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARLEN" *)
    output wire [7:0] M_AXI_MM2S_ARLEN,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARSIZE" *)
    output wire [2:0] M_AXI_MM2S_ARSIZE,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARBURST" *)
    output wire [1:0] M_AXI_MM2S_ARBURST,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARVALID" *)
    output wire M_AXI_MM2S_ARVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S ARREADY" *)
    input wire M_AXI_MM2S_ARREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RDATA" *)
    input wire [MM2S_MM_DATA_WIDTH-1:0] M_AXI_MM2S_RDATA,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RRESP" *)
    input wire [1:0] M_AXI_MM2S_RRESP,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RLAST" *)
    input wire M_AXI_MM2S_RLAST,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RVALID" *)
    input wire M_AXI_MM2S_RVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_MM2S RREADY" *)
    output wire M_AXI_MM2S_RREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWADDR" *)
    output wire [ADDR_WIDTH-1:0] M_AXI_S2MM_AWADDR,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWLEN" *)
    output wire [7:0] M_AXI_S2MM_AWLEN,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWSIZE" *)
    output wire [2:0] M_AXI_S2MM_AWSIZE,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWBURST" *)
    output wire [1:0] M_AXI_S2MM_AWBURST,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWVALID" *)
    output wire M_AXI_S2MM_AWVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM AWREADY" *)
    input wire M_AXI_S2MM_AWREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WDATA" *)
    output wire [S2MM_MM_DATA_WIDTH-1:0] M_AXI_S2MM_WDATA,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WSTRB" *)
    output wire [S2MM_MM_DATA_WIDTH/8-1:0] M_AXI_S2MM_WSTRB,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WLAST" *)
    output wire M_AXI_S2MM_WLAST,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WVALID" *)
    output wire M_AXI_S2MM_WVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM WREADY" *)
    input wire M_AXI_S2MM_WREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM BRESP" *)
    input wire [1:0] M_AXI_S2MM_BRESP,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM BVALID" *)
    input wire M_AXI_S2MM_BVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI_S2MM BREADY" *)
    output wire M_AXI_S2MM_BREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS_MM2S TDATA" *)
    output wire [MM2S_STREAM_DATA_WIDTH-1:0] M_AXIS_MM2S_TDATA,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS_MM2S TLAST" *)
    output wire M_AXIS_MM2S_TLAST,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS_MM2S TVALID" *)
    output wire M_AXIS_MM2S_TVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS_MM2S TREADY" *)
    input wire M_AXIS_MM2S_TREADY,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXI_S2MM TDATA" *)
    input wire [S2MM_STREAM_DATA_WIDTH-1:0] S_AXI_S2MM_TDATA,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXI_S2MM TLAST" *)
    input wire S_AXI_S2MM_TLAST,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXI_S2MM TVALID" *)
    input wire S_AXI_S2MM_TVALID,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXI_S2MM TREADY" *)
    output wire S_AXI_S2MM_TREADY
);

    dma_core #(
        .LENGTH_WIDTH(LENGTH_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .LITE_ADDR_WIDTH(LITE_ADDR_WIDTH),
        .MM2S_MM_DATA_WIDTH(MM2S_MM_DATA_WIDTH),
        .MM2S_STREAM_DATA_WIDTH(MM2S_STREAM_DATA_WIDTH),
        .MM2S_MAX_BURST(MM2S_MAX_BURST),
        .S2MM_MM_DATA_WIDTH(S2MM_MM_DATA_WIDTH),
        .S2MM_STREAM_DATA_WIDTH(S2MM_STREAM_DATA_WIDTH),
        .S2MM_MAX_BURST(S2MM_MAX_BURST),
        .FIFO_DEPTH(FIFO_DEPTH)
    ) u_core (
        .aclk(aclk),
        .aresetn(aresetn),

        .s_axi_lite_awaddr(S_AXI_LITE_AWADDR),
        .s_axi_lite_awvalid(S_AXI_LITE_AWVALID),
        .s_axi_lite_awready(S_AXI_LITE_AWREADY),
        .s_axi_lite_wdata(S_AXI_LITE_WDATA),
        .s_axi_lite_wstrb(S_AXI_LITE_WSTRB),
        .s_axi_lite_wvalid(S_AXI_LITE_WVALID),
        .s_axi_lite_wready(S_AXI_LITE_WREADY),
        .s_axi_lite_bresp(S_AXI_LITE_BRESP),
        .s_axi_lite_bvalid(S_AXI_LITE_BVALID),
        .s_axi_lite_bready(S_AXI_LITE_BREADY),
        .s_axi_lite_araddr(S_AXI_LITE_ARADDR),
        .s_axi_lite_arvalid(S_AXI_LITE_ARVALID),
        .s_axi_lite_arready(S_AXI_LITE_ARREADY),
        .s_axi_lite_rdata(S_AXI_LITE_RDATA),
        .s_axi_lite_rresp(S_AXI_LITE_RRESP),
        .s_axi_lite_rvalid(S_AXI_LITE_RVALID),
        .s_axi_lite_rready(S_AXI_LITE_RREADY),

        .m_axi_mm2s_araddr(M_AXI_MM2S_ARADDR),
        .m_axi_mm2s_arlen(M_AXI_MM2S_ARLEN),
        .m_axi_mm2s_arsize(M_AXI_MM2S_ARSIZE),
        .m_axi_mm2s_arburst(M_AXI_MM2S_ARBURST),
        .m_axi_mm2s_arvalid(M_AXI_MM2S_ARVALID),
        .m_axi_mm2s_arready(M_AXI_MM2S_ARREADY),
        .m_axi_mm2s_rdata(M_AXI_MM2S_RDATA),
        .m_axi_mm2s_rresp(M_AXI_MM2S_RRESP),
        .m_axi_mm2s_rlast(M_AXI_MM2S_RLAST),
        .m_axi_mm2s_rvalid(M_AXI_MM2S_RVALID),
        .m_axi_mm2s_rready(M_AXI_MM2S_RREADY),

        .m_axi_s2mm_awaddr(M_AXI_S2MM_AWADDR),
        .m_axi_s2mm_awlen(M_AXI_S2MM_AWLEN),
        .m_axi_s2mm_awsize(M_AXI_S2MM_AWSIZE),
        .m_axi_s2mm_awburst(M_AXI_S2MM_AWBURST),
        .m_axi_s2mm_awvalid(M_AXI_S2MM_AWVALID),
        .m_axi_s2mm_awready(M_AXI_S2MM_AWREADY),
        .m_axi_s2mm_wdata(M_AXI_S2MM_WDATA),
        .m_axi_s2mm_wstrb(M_AXI_S2MM_WSTRB),
        .m_axi_s2mm_wlast(M_AXI_S2MM_WLAST),
        .m_axi_s2mm_wvalid(M_AXI_S2MM_WVALID),
        .m_axi_s2mm_wready(M_AXI_S2MM_WREADY),
        .m_axi_s2mm_bresp(M_AXI_S2MM_BRESP),
        .m_axi_s2mm_bvalid(M_AXI_S2MM_BVALID),
        .m_axi_s2mm_bready(M_AXI_S2MM_BREADY),

        .m_axis_mm2s_tdata(M_AXIS_MM2S_TDATA),
        .m_axis_mm2s_tlast(M_AXIS_MM2S_TLAST),
        .m_axis_mm2s_tvalid(M_AXIS_MM2S_TVALID),
        .m_axis_mm2s_tready(M_AXIS_MM2S_TREADY),

        .s_axis_s2mm_tdata(S_AXI_S2MM_TDATA),
        .s_axis_s2mm_tlast(S_AXI_S2MM_TLAST),
        .s_axis_s2mm_tvalid(S_AXI_S2MM_TVALID),
        .s_axis_s2mm_tready(S_AXI_S2MM_TREADY)
    );

endmodule
