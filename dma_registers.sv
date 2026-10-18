`timescale 1ns / 1ps

module dma_chan_regs #(
    parameter integer ADDR_WIDTH   = 32,
    parameter integer LENGTH_WIDTH = 23,
    parameter integer IS_S2MM      = 0
)(
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire                    we_cr,
    input  wire                    we_sr,
    input  wire                    we_addr,
    input  wire                    we_len,
    input  wire [31:0]             wdata,
    input  wire [3:0]              wstrb,

    output wire [31:0]             cr_q,
    output wire [31:0]             sr_q,
    output wire [31:0]             addr_q,
    output wire [31:0]             len_q,

    output reg                     cmd_valid,
    output reg  [ADDR_WIDTH-1:0]   cmd_addr,
    output reg  [LENGTH_WIDTH-1:0] cmd_len,
    output wire                    abort,

    input  wire                    eng_done,
    input  wire                    eng_err_slv,
    input  wire                    eng_err_dec,
    input  wire [LENGTH_WIDTH-1:0] eng_len
);
    reg                    rs, busy, ioc, err, slv, dec, rst_req;
    reg [31:0]             addr_r;
    reg [LENGTH_WIDTH-1:0] len_r;

    wire halted = ~rs & ~busy;
    wire idle   =  rs & ~busy;

    assign abort  = rst_req;
    assign cr_q   = {29'd0, rst_req, 1'b0, rs};
    assign sr_q   = {17'd0, err, 1'b0, ioc, 5'd0, dec, slv, 3'd0, idle, halted};
    assign addr_q = addr_r;
    assign len_q  = len_r;

    function [31:0] merge;
        input [31:0] old;
        input [31:0] nw;
        input [3:0]  strb;
        begin
            merge[ 7: 0] = strb[0] ? nw[ 7: 0] : old[ 7: 0];
            merge[15: 8] = strb[1] ? nw[15: 8] : old[15: 8];
            merge[23:16] = strb[2] ? nw[23:16] : old[23:16];
            merge[31:24] = strb[3] ? nw[31:24] : old[31:24];
        end
    endfunction

    always @(posedge clk) begin
        if (!rst_n) begin
            rs        <= 1'b0;
            busy      <= 1'b0;
            ioc       <= 1'b0;
            err       <= 1'b0;
            slv       <= 1'b0;
            dec       <= 1'b0;
            rst_req   <= 1'b0;
            addr_r    <= 32'd0;
            len_r     <= {LENGTH_WIDTH{1'b0}};
            cmd_valid <= 1'b0;
            cmd_addr  <= {ADDR_WIDTH{1'b0}};
            cmd_len   <= {LENGTH_WIDTH{1'b0}};
        end else begin
            cmd_valid <= 1'b0;

            if (eng_done) begin
                busy <= 1'b0;
                if (IS_S2MM != 0) len_r <= eng_len;
                if (!rst_req) begin
                    ioc <= 1'b1;
                    if (eng_err_slv || eng_err_dec) begin
                        err <= 1'b1;
                        slv <= eng_err_slv;
                        dec <= eng_err_dec;
                    end
                end
            end

            if (rst_req && !busy && !cmd_valid && !eng_done) begin
                rs      <= 1'b0;
                ioc     <= 1'b0;
                err     <= 1'b0;
                slv     <= 1'b0;
                dec     <= 1'b0;
                rst_req <= 1'b0;
            end

            if (we_cr && wstrb[0] && !rst_req) begin
                rs <= wdata[0];
                if (wdata[2]) rst_req <= 1'b1;
            end

            if (we_sr && wstrb[1]) begin
                if (wdata[12]) ioc <= 1'b0;
                if (wdata[14]) err <= 1'b0;
            end

            if (we_addr) addr_r <= merge(addr_r, wdata, wstrb);

            if (we_len && wstrb[0] && !rst_req && rs && !busy) begin
                if (wdata[LENGTH_WIDTH-1:0] == {LENGTH_WIDTH{1'b0}}) begin
                    ioc <= 1'b1;
                end else begin
                    cmd_valid <= 1'b1;
                    cmd_addr  <= addr_r[ADDR_WIDTH-1:0];
                    cmd_len   <= wdata[LENGTH_WIDTH-1:0];
                    len_r     <= wdata[LENGTH_WIDTH-1:0];
                    busy      <= 1'b1;
                    slv       <= 1'b0;
                    dec       <= 1'b0;
                end
            end
        end
    end
endmodule

module dma_registers #(
    parameter integer ADDR_WIDTH      = 32,
    parameter integer LITE_ADDR_WIDTH = 7,
    parameter integer LENGTH_WIDTH    = 23
)(
    input  wire                       clk,
    input  wire                       rst_n,

    input  wire                       wr_en,
    input  wire [LITE_ADDR_WIDTH-1:0] wr_addr,
    input  wire [31:0]                wr_data,
    input  wire [3:0]                 wr_strb,
    input  wire [LITE_ADDR_WIDTH-1:0] rd_addr,
    output reg  [31:0]                rd_data,

    output wire                       mm_cmd_valid,
    output wire [ADDR_WIDTH-1:0]      mm_cmd_addr,
    output wire [LENGTH_WIDTH-1:0]    mm_cmd_len,
    output wire                       mm_abort,
    input  wire                       mm_done,
    input  wire                       mm_err_slv,
    input  wire                       mm_err_dec,

    output wire                       sm_cmd_valid,
    output wire [ADDR_WIDTH-1:0]      sm_cmd_addr,
    output wire [LENGTH_WIDTH-1:0]    sm_cmd_len,
    output wire                       sm_abort,
    input  wire                       sm_done,
    input  wire                       sm_err_slv,
    input  wire                       sm_err_dec,
    input  wire [LENGTH_WIDTH-1:0]    sm_rx_bytes
);
    wire [4:0] wa = wr_addr[6:2];
    wire [4:0] ra = rd_addr[6:2];

    wire [31:0] mm_cr, mm_sr, mm_ad, mm_ln;
    wire [31:0] sm_cr, sm_sr, sm_ad, sm_ln;

    dma_chan_regs #(.ADDR_WIDTH(ADDR_WIDTH), .LENGTH_WIDTH(LENGTH_WIDTH), .IS_S2MM(0)) u_mm (
        .clk(clk), .rst_n(rst_n),
        .we_cr  (wr_en && wa == 5'd0),
        .we_sr  (wr_en && wa == 5'd1),
        .we_addr(wr_en && wa == 5'd6),
        .we_len (wr_en && wa == 5'd10),
        .wdata(wr_data), .wstrb(wr_strb),
        .cr_q(mm_cr), .sr_q(mm_sr), .addr_q(mm_ad), .len_q(mm_ln),
        .cmd_valid(mm_cmd_valid), .cmd_addr(mm_cmd_addr), .cmd_len(mm_cmd_len),
        .abort(mm_abort),
        .eng_done(mm_done), .eng_err_slv(mm_err_slv), .eng_err_dec(mm_err_dec),
        .eng_len({LENGTH_WIDTH{1'b0}})
    );

    dma_chan_regs #(.ADDR_WIDTH(ADDR_WIDTH), .LENGTH_WIDTH(LENGTH_WIDTH), .IS_S2MM(1)) u_sm (
        .clk(clk), .rst_n(rst_n),
        .we_cr  (wr_en && wa == 5'd12),
        .we_sr  (wr_en && wa == 5'd13),
        .we_addr(wr_en && wa == 5'd18),
        .we_len (wr_en && wa == 5'd22),
        .wdata(wr_data), .wstrb(wr_strb),
        .cr_q(sm_cr), .sr_q(sm_sr), .addr_q(sm_ad), .len_q(sm_ln),
        .cmd_valid(sm_cmd_valid), .cmd_addr(sm_cmd_addr), .cmd_len(sm_cmd_len),
        .abort(sm_abort),
        .eng_done(sm_done), .eng_err_slv(sm_err_slv), .eng_err_dec(sm_err_dec),
        .eng_len(sm_rx_bytes)
    );

    always @* begin
        case (ra)
            5'd0:    rd_data = mm_cr;
            5'd1:    rd_data = mm_sr;
            5'd6:    rd_data = mm_ad;
            5'd10:   rd_data = mm_ln;
            5'd12:   rd_data = sm_cr;
            5'd13:   rd_data = sm_sr;
            5'd18:   rd_data = sm_ad;
            5'd22:   rd_data = sm_ln;
            default: rd_data = 32'd0;
        endcase
    end
endmodule
