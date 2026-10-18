`timescale 1ns / 1ps

module dma_s2mm #(
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
    output wire [LENGTH_WIDTH-1:0] rx_bytes,
 
    output wire [ADDR_WIDTH-1:0]   awaddr,
    output wire [7:0]              awlen,
    output wire [2:0]              awsize,
    output wire [1:0]              awburst,
    output wire                    awvalid,
    input  wire                    awready,
    output wire [MM_WIDTH-1:0]     wdata,
    output wire [MM_WIDTH/8-1:0]   wstrb,
    output wire                    wlast,
    output wire                    wvalid,
    input  wire                    wready,
    input  wire [1:0]              bresp,
    input  wire                    bvalid,
    output wire                    bready,
 
    input  wire [ST_WIDTH-1:0]     tdata,
    input  wire                    tlast,
    input  wire                    tvalid,
    output wire                    tready
);
    localparam integer MB    = MM_WIDTH / 8;
    localparam integer SB    = ST_WIDTH / 8;
    localparam integer RATIO = MM_WIDTH / ST_WIDTH;
    localparam integer MBSH  = $clog2(MB);
    localparam integer SBSH  = $clog2(SB);
    localparam integer CW    = LENGTH_WIDTH + 1;
    localparam integer LANE_W = (RATIO > 1) ? $clog2(RATIO) : 1;
    localparam integer FWD   = MM_WIDTH + MB;
    localparam [MB-1:0] SBONES = (1 << SB) - 1;
    // Narrow widths (timing): burst <= MAX_BURST; avail <= FIFO_DEPTH.
    localparam integer NB_W = $clog2(MAX_BURST + 1);
    localparam integer AV_W = $clog2(FIFO_DEPTH + MAX_BURST + 1) + 1;
    localparam integer BO_W = 16;
 
    initial begin
        if (FIFO_DEPTH < MAX_BURST || (FIFO_DEPTH & (FIFO_DEPTH - 1)) != 0)
            $error("dma_s2mm: FIFO_DEPTH must be a power of two >= MAX_BURST");
        if (MAX_BURST < 1 || MAX_BURST > 256)
            $error("dma_s2mm: MAX_BURST must be 1..256");
        if (ST_WIDTH < 8 || MM_WIDTH < ST_WIDTH || (MM_WIDTH % ST_WIDTH) != 0)
            $error("dma_s2mm: bad MM_WIDTH / ST_WIDTH");
    end
 
    reg                  active;
    reg                  aborting;
    reg                  flush;
    reg                  ended;
 
    reg [MM_WIDTH-1:0]   asm;
    reg [MB-1:0]         asm_strb;
    reg [LANE_W-1:0]     lane;
    reg [CW-1:0]         beats_left;
    reg [CW-1:0]         rx_beats;
 
    reg [ADDR_WIDTH-1:0] wr_addr;
    reg [AV_W-1:0]       avail;
 
    reg                  awvalid_r;
    reg [ADDR_WIDTH-1:0] awaddr_r;
    reg [7:0]            awlen_r;
    reg [8:0]            aw_n;
 
    reg [8:0]            bq0, bq1;
    reg                  bq_wp, bq_rp;
    reg [1:0]            bq_cnt;
 
    reg                  w_active;
    reg [8:0]            w_left;
    reg [BO_W-1:0]       b_out;
 
    wire [31:0] cmd_addr32 = cmd_addr;
    wire [31:0] cmd_len32  = cmd_len;
    wire [31:0] off_bytes  = cmd_addr32 & (MB - 1);
    wire [31:0] off_beats  = off_bytes >> SBSH;
    wire [31:0] beats32    = (cmd_len32 + (SB - 1)) >> SBSH;
 
    wire                 fifo_full;
    wire                 fifo_empty;
    wire [FWD-1:0]       fifo_rd;
 
    assign tready = active && !aborting && !ended && !fifo_full;
 
    wire accept = tvalid && tready;
    wire last_beat = tlast || (beats_left == 1);
    wire word_full = last_beat || (lane == RATIO - 1);
    wire push = accept && word_full;
 
    wire [MM_WIDTH-1:0] beat_ext = tdata;
    wire [MM_WIDTH-1:0] asm_n    = asm | (beat_ext << (lane * ST_WIDTH));
    wire [MB-1:0]       strb_n   = asm_strb | (SBONES << (lane * SB));
 
    wire [12:0] to4k_bytes = 13'd4096 - {1'b0, wr_addr[11:0]};
    wire [12:0] to4k_words = to4k_bytes >> MBSH;
    wire             t_hi   = |to4k_words[12:NB_W];
    wire [NB_W-1:0]  t_lo   = to4k_words[NB_W-1:0];
    wire [NB_W-1:0]  target = (!t_hi && (t_lo < MAX_BURST)) ? t_lo : MAX_BURST;
 
    wire [AV_W-1:0]  target_x   = {{(AV_W-NB_W){1'b0}}, target};
    wire             full_burst = (avail >= target_x);
    wire             avail_nz   = (avail != {AV_W{1'b0}});
    // when !full_burst, avail < target <= MAX_BURST, so the low bits suffice
    wire [NB_W-1:0]  n_burst    = full_burst ? target : avail[NB_W-1:0];
    wire [8:0]            n_burst9 = n_burst;
    wire [AV_W-1:0]       n_burst_x = n_burst;
    wire [ADDR_WIDTH-1:0] n_burst_a = n_burst;
 
    wire bq_full = (bq_cnt == 2'd2);
 
    wire issue = active && !aborting && !abort && !awvalid_r && !bq_full &&
                 (full_burst || (ended && avail_nz));
 
    wire aw_hs = awvalid_r && awready;
 
    wire w_load = !w_active && (bq_cnt != 2'd0);
    wire w_fire = wvalid && wready;
    wire [8:0] bq_head = bq_rp ? bq1 : bq0;
 
    assign wvalid = w_active && !fifo_empty;
    assign wlast  = (w_left == 9'd1);
    assign wdata  = fifo_rd[MM_WIDTH-1:0];
    assign wstrb  = fifo_rd[FWD-1:MM_WIDTH];
 
    assign awaddr  = awaddr_r;
    assign awlen   = awlen_r;
    assign awsize  = MBSH;
    assign awburst = 2'b01;
    assign awvalid = awvalid_r;
    assign bready  = 1'b1;
    assign busy    = active | aborting;
    assign rx_bytes = rx_beats << SBSH;
 
    dma_axis_fifo #(.WIDTH(FWD), .DEPTH(FIFO_DEPTH)) u_fifo (
        .clk(clk), .rst_n(rst_n & ~flush),
        .wr_en(push), .wr_data({strb_n, asm_n}), .full(fifo_full),
        .rd_en(w_fire), .rd_data(fifo_rd), .empty(fifo_empty)
    );
 
    wire idle_bus = !awvalid_r && (bq_cnt == 2'd0) && !w_active && (b_out == {BO_W{1'b0}});
 
    always @(posedge clk) begin
        if (!rst_n) begin
            active     <= 1'b0;
            aborting   <= 1'b0;
            flush      <= 1'b0;
            ended      <= 1'b0;
            done       <= 1'b0;
            err_slv    <= 1'b0;
            err_dec    <= 1'b0;
            asm        <= {MM_WIDTH{1'b0}};
            asm_strb   <= {MB{1'b0}};
            lane       <= {LANE_W{1'b0}};
            beats_left <= {CW{1'b0}};
            rx_beats   <= {CW{1'b0}};
            wr_addr    <= {ADDR_WIDTH{1'b0}};
            avail      <= {AV_W{1'b0}};
            awvalid_r  <= 1'b0;
            awaddr_r   <= {ADDR_WIDTH{1'b0}};
            awlen_r    <= 8'd0;
            aw_n       <= 9'd0;
            bq0        <= 9'd0;
            bq1        <= 9'd0;
            bq_wp      <= 1'b0;
            bq_rp      <= 1'b0;
            bq_cnt     <= 2'd0;
            w_active   <= 1'b0;
            w_left     <= 9'd0;
            b_out      <= {BO_W{1'b0}};
        end else begin
            done  <= 1'b0;
            flush <= 1'b0;
 
            if (accept) begin
                beats_left <= beats_left - 1'b1;
                rx_beats   <= rx_beats + 1'b1;
                if (word_full) begin
                    asm      <= {MM_WIDTH{1'b0}};
                    asm_strb <= {MB{1'b0}};
                    lane     <= {LANE_W{1'b0}};
                end else begin
                    asm      <= asm_n;
                    asm_strb <= strb_n;
                    lane     <= lane + 1'b1;
                end
                if (last_beat) ended <= 1'b1;
            end
 
            avail <= avail + {{(AV_W-1){1'b0}}, push} - (issue ? n_burst_x : {AV_W{1'b0}});
 
            if (issue) begin
                awvalid_r <= 1'b1;
                awaddr_r  <= wr_addr;
                awlen_r   <= n_burst9[7:0] - 8'd1;
                aw_n      <= n_burst9;
                wr_addr   <= wr_addr + (n_burst_a << MBSH);
            end
 
            if (aw_hs) begin
                awvalid_r <= 1'b0;
                if (bq_wp) bq1 <= aw_n; else bq0 <= aw_n;
                bq_wp <= ~bq_wp;
            end
 
            if (w_load) begin
                w_active <= 1'b1;
                w_left   <= bq_head;
                bq_rp    <= ~bq_rp;
            end
            bq_cnt <= bq_cnt + (aw_hs ? 2'd1 : 2'd0) - (w_load ? 2'd1 : 2'd0);
 
            if (w_fire) begin
                w_left <= w_left - 1'b1;
                if (w_left == 9'd1) w_active <= 1'b0;
            end
 
            b_out <= b_out + {{(BO_W-1){1'b0}}, aw_hs} - {{(BO_W-1){1'b0}}, bvalid};
 
            if (bvalid && bresp[1]) begin
                if (bresp[0]) err_dec <= 1'b1;
                else          err_slv <= 1'b1;
            end
 
            if (active && !aborting && abort) begin
                aborting <= 1'b1;
                asm      <= {MM_WIDTH{1'b0}};
                asm_strb <= {MB{1'b0}};
            end
 
            if (aborting && idle_bus) begin
                aborting <= 1'b0;
                active   <= 1'b0;
                done     <= 1'b1;
                flush    <= 1'b1;
                avail    <= {AV_W{1'b0}};
            end else if (active && !aborting && ended && !avail_nz && idle_bus) begin
                active <= 1'b0;
                done   <= 1'b1;
            end
 
            if (cmd_valid && !active && !aborting) begin
                if (abort || cmd_len == {LENGTH_WIDTH{1'b0}}) begin
                    done <= 1'b1;
                end else begin
                    active     <= 1'b1;
                    ended      <= 1'b0;
                    err_slv    <= 1'b0;
                    err_dec    <= 1'b0;
                    asm        <= {MM_WIDTH{1'b0}};
                    asm_strb   <= {MB{1'b0}};
                    lane       <= off_beats[LANE_W-1:0];
                    beats_left <= beats32[CW-1:0];
                    rx_beats   <= {CW{1'b0}};
                    wr_addr    <= cmd_addr & ~(MB - 1);
                    avail      <= {AV_W{1'b0}};
                end
            end
        end
    end
endmodule
 