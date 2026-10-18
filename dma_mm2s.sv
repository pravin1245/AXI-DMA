`timescale 1ns / 1ps

module dma_mm2s #(
    parameter integer ADDR_WIDTH   = 32,
    parameter integer MM_WIDTH     = 32,
    parameter integer ST_WIDTH     = 8,
    parameter integer LENGTH_WIDTH = 23,
    parameter integer MAX_BURST    = 16,
    parameter integer FIFO_DEPTH   = 64
)(
    input  wire                    clk,
    input  wire                    rst_n,
 
    input  wire                    cmd_valid,
    input  wire [ADDR_WIDTH-1:0]   cmd_addr,
    input  wire [LENGTH_WIDTH-1:0] cmd_len,
    input  wire                    abort,
    output wire                    busy,
    output reg                     done,
    output reg                     err_slv,
    output reg                     err_dec,
 
    output wire [ADDR_WIDTH-1:0]   araddr,
    output wire [7:0]              arlen,
    output wire [2:0]              arsize,
    output wire [1:0]              arburst,
    output wire                    arvalid,
    input  wire                    arready,
    input  wire [MM_WIDTH-1:0]     rdata,
    input  wire [1:0]              rresp,
    input  wire                    rlast,
    input  wire                    rvalid,
    output wire                    rready,
 
    output wire [ST_WIDTH-1:0]     tdata,
    output wire                    tlast,
    output wire                    tvalid,
    input  wire                    tready
);
    localparam integer MB    = MM_WIDTH / 8;
    localparam integer SB    = ST_WIDTH / 8;
    localparam integer RATIO = MM_WIDTH / ST_WIDTH;
    localparam integer MBSH  = $clog2(MB);
    localparam integer SBSH  = $clog2(SB);
    localparam integer RSH   = $clog2(RATIO);
    localparam integer CW    = LENGTH_WIDTH + 1;
    localparam integer LANE_W = (RATIO > 1) ? $clog2(RATIO) : 1;
    // Narrow datapath widths (timing): burst size <= MAX_BURST (<=256) and the
    // outstanding/reserved word counters never exceed FIFO_DEPTH + MAX_BURST.
    localparam integer NB_W  = $clog2(MAX_BURST + 1);
    localparam integer RW    = $clog2(FIFO_DEPTH + MAX_BURST + 1) + 1;
    localparam [RW-1:0] FIFO_DEPTH_W = FIFO_DEPTH;
 
    initial begin
        if (FIFO_DEPTH < MAX_BURST || (FIFO_DEPTH & (FIFO_DEPTH - 1)) != 0)
            $error("dma_mm2s: FIFO_DEPTH must be a power of two >= MAX_BURST");
        if (MAX_BURST < 1 || MAX_BURST > 256)
            $error("dma_mm2s: MAX_BURST must be 1..256");
        if (ST_WIDTH < 8 || MM_WIDTH < ST_WIDTH || (MM_WIDTH % ST_WIDTH) != 0)
            $error("dma_mm2s: bad MM_WIDTH / ST_WIDTH");
    end
 
    reg                 active;
    reg                 aborting;
    reg                 flush;
 
    reg [ADDR_WIDTH-1:0] ar_addr;
    reg [CW-1:0]         ar_words_left;
    reg                  arvalid_r;
    reg [ADDR_WIDTH-1:0] araddr_r;
    reg [7:0]            arlen_r;
 
    reg [RW-1:0]         reserved;
    reg [RW-1:0]         ar_out;
 
    reg [CW-1:0]         st_left;
    reg [LANE_W-1:0]     lane;
 
    wire [31:0] cmd_addr32 = cmd_addr;
    wire [31:0] cmd_len32  = cmd_len;
    wire [31:0] off_bytes  = cmd_addr32 & (MB - 1);
    wire [31:0] off_beats  = off_bytes >> SBSH;
    wire [31:0] beats32    = (cmd_len32 + (SB - 1)) >> SBSH;
    wire [31:0] nw32       = (beats32 + off_beats + (RATIO - 1)) >> RSH;
 
    wire [12:0] to4k_bytes = 13'd4096 - {1'b0, ar_addr[11:0]};
    wire [12:0] to4k_words = to4k_bytes >> MBSH;
 
    // Burst size = min(MAX_BURST, words left, words to 4KB boundary).
    // Done on narrow slices: any upper bit set => operand >= MAX_BURST.
    wire              w_hi = |ar_words_left[CW-1:NB_W];
    wire [NB_W-1:0]   w_lo = ar_words_left[NB_W-1:0];
    wire              t_hi = |to4k_words[12:NB_W];
    wire [NB_W-1:0]   t_lo = to4k_words[NB_W-1:0];
 
    reg [NB_W-1:0] n_burst;
    always @* begin
        n_burst = MAX_BURST;
        if (!w_hi && (w_lo < n_burst)) n_burst = w_lo;
        if (!t_hi && (t_lo < n_burst)) n_burst = t_lo;
    end
 
    wire [RW-1:0] n_burst_x = {{(RW-NB_W){1'b0}}, n_burst};
    wire          words_nz  = (ar_words_left != {CW{1'b0}});
    // reserved + n_burst <= FIFO_DEPTH  <=>  reserved <= FIFO_DEPTH - n_burst
    wire          room_ok   = (reserved <= (FIFO_DEPTH_W - n_burst_x));
 
    wire issue = active && !aborting && !abort && !arvalid_r &&
                 words_nz && room_ok;
 
    wire                fifo_empty;
    wire                fifo_full;
    wire [MM_WIDTH-1:0] fifo_rd;
 
    wire tvalid_i  = active && !aborting && (st_left != 0) && !fifo_empty;
    wire st_fire   = tvalid_i && tready;
    wire last_beat = (st_left == 1);
    wire word_end  = last_beat || (lane == RATIO - 1);
    wire pop       = st_fire && word_end;
 
    wire [MM_WIDTH-1:0] shifted = fifo_rd >> (lane * ST_WIDTH);
 
    assign tdata  = shifted[ST_WIDTH-1:0];
    assign tlast  = last_beat;
    assign tvalid = tvalid_i;
 
    assign araddr  = araddr_r;
    assign arlen   = arlen_r;
    assign arsize  = MBSH;
    assign arburst = 2'b01;
    assign arvalid = arvalid_r;
    assign rready  = 1'b1;
    assign busy    = active | aborting;
 
    dma_axis_fifo #(.WIDTH(MM_WIDTH), .DEPTH(FIFO_DEPTH)) u_fifo (
        .clk(clk), .rst_n(rst_n & ~flush),
        .wr_en(rvalid), .wr_data(rdata), .full(fifo_full),
        .rd_en(pop), .rd_data(fifo_rd), .empty(fifo_empty)
    );
 
    wire [RW-1:0] ar_out_next = ar_out + (issue ? n_burst_x : {RW{1'b0}}) - {{(RW-1){1'b0}}, rvalid};
 
    always @(posedge clk) begin
        if (!rst_n) begin
            active        <= 1'b0;
            aborting      <= 1'b0;
            flush         <= 1'b0;
            done          <= 1'b0;
            err_slv       <= 1'b0;
            err_dec       <= 1'b0;
            ar_addr       <= {ADDR_WIDTH{1'b0}};
            ar_words_left <= {CW{1'b0}};
            arvalid_r     <= 1'b0;
            araddr_r      <= {ADDR_WIDTH{1'b0}};
            arlen_r       <= 8'd0;
            reserved      <= {RW{1'b0}};
            ar_out        <= {RW{1'b0}};
            st_left       <= {CW{1'b0}};
            lane          <= {LANE_W{1'b0}};
        end else begin
            done  <= 1'b0;
            flush <= 1'b0;
 
            if (arvalid_r && arready) arvalid_r <= 1'b0;
 
            if (rvalid && rresp[1]) begin
                if (rresp[0]) err_dec <= 1'b1;
                else          err_slv <= 1'b1;
            end
 
            reserved <= reserved + (issue ? n_burst_x : {RW{1'b0}}) - {{(RW-1){1'b0}}, pop};
            ar_out   <= ar_out_next;
 
            if (issue) begin
                arvalid_r     <= 1'b1;
                araddr_r      <= ar_addr;
                arlen_r       <= n_burst_x[7:0] - 8'd1;
                ar_addr       <= ar_addr + ({{(ADDR_WIDTH-NB_W){1'b0}}, n_burst} << MBSH);
                ar_words_left <= ar_words_left - {{(CW-NB_W){1'b0}}, n_burst};
            end
 
            if (st_fire) begin
                st_left <= st_left - 1'b1;
                if (word_end) lane <= {LANE_W{1'b0}};
                else          lane <= lane + 1'b1;
                if (last_beat) begin
                    active <= 1'b0;
                    done   <= 1'b1;
                end
            end
 
            if (active && !aborting && abort) aborting <= 1'b1;
 
            if (aborting && !arvalid_r && ar_out == {RW{1'b0}}) begin
                aborting      <= 1'b0;
                active        <= 1'b0;
                done          <= 1'b1;
                flush         <= 1'b1;
                reserved      <= {RW{1'b0}};
                ar_words_left <= {CW{1'b0}};
            end
 
            if (cmd_valid && !active && !aborting) begin
                if (abort || cmd_len == {LENGTH_WIDTH{1'b0}}) begin
                    done <= 1'b1;
                end else begin
                    active        <= 1'b1;
                    err_slv       <= 1'b0;
                    err_dec       <= 1'b0;
                    ar_addr       <= cmd_addr & ~(MB - 1);
                    ar_words_left <= nw32[CW-1:0];
                    st_left       <= beats32[CW-1:0];
                    lane          <= off_beats[LANE_W-1:0];
                end
            end
        end
    end
endmodule
 