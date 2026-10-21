`timescale 1ns / 1ps
// ============================================================================
//  dma_tb_pkg : layered verification environment for dma_core
//
//    Test layer        : tests in tb_dma_layered.sv
//    Functional layer  : dma_txn, dma_gen, scoreboard, tb_cov
//    Command layer     : dma_driver (AXI-Lite programming + stream drive)
//    Signal layer      : BFMs (axi_rd_bfm / axi_wr_bfm / axis_sink), monitors,
//                        dma_if
// ============================================================================
package dma_tb_pkg;

    // ------------------------------------------------------------ constants
    localparam int unsigned MEM_BYTES = 32'h0040_0000;   // 4 MB modelled memory
    localparam int unsigned SRC_BASE  = 32'h0001_0000;   // input image lives here
    localparam int unsigned DST_BASE  = 32'h0020_0000;   // output image is written here
    localparam int unsigned SLV_BASE  = 32'h0030_0000;   // 4 KB window that answers SLVERR
    localparam int unsigned SLV_SIZE  = 32'h0000_1000;
    localparam int unsigned DEC_BASE  = 32'h0050_0000;   // outside memory -> DECERR

    localparam bit [6:0] MM_CR = 7'h00, MM_SR = 7'h04, MM_SA = 7'h18, MM_LEN = 7'h28;
    localparam bit [6:0] SM_CR = 7'h30, SM_SR = 7'h34, SM_DA = 7'h48, SM_LEN = 7'h58;

    localparam bit [31:0] SR_HALTED = 32'h1, SR_IDLE = 32'h2, SR_SLV = 32'h20, SR_DEC = 32'h40,
                          SR_IOC = 32'h1000, SR_ERR = 32'h4000;

    // image shared between top (loads it) and env (puts it in memory)
    byte unsigned g_img[$];
    int           g_img_fmt = 0;      // 0 = byte per line, 1 = 32-bit word per line

    // ------------------------------------------------------------ logging
    class tb_log;
        static int errors = 0;
        static int checks = 0;
        static function void chk(bit cond, string msg);
            checks++;
            if (!cond) begin
                errors++;
                if (errors <= 60) $display("[%0t] ** ERROR ** %s", $time, msg);
            end
        endfunction
        static function void info(string m);
            $display("[%0t] INFO  %s", $time, m);
        endfunction
    endclass

    // ------------------------------------------------------------ knobs
    class tb_cfg;
        int ar_rdy_pct = 100, r_gap_pct = 0, rd_lat_min = 0, rd_lat_max = 0;
        int aw_rdy_pct = 100, w_rdy_pct = 100, b_lat_min = 0, b_lat_max = 0;
        int snk_rdy_pct = 100, src_gap_pct = 0, filt_stall = 0, lite_gap_max = 0;
        bit chk_stable = 1;
        int unsigned timeout = 400000;   // clock cycles per transfer
        string name = "fast";

        function void prof_fast();
            ar_rdy_pct = 100; r_gap_pct = 0; rd_lat_min = 0; rd_lat_max = 0;
            aw_rdy_pct = 100; w_rdy_pct = 100; b_lat_min = 0; b_lat_max = 0;
            snk_rdy_pct = 100; src_gap_pct = 0; filt_stall = 0; lite_gap_max = 0; name = "fast";
        endfunction
        function void prof_slow();
            ar_rdy_pct = 30; r_gap_pct = 40; rd_lat_min = 2; rd_lat_max = 14;
            aw_rdy_pct = 30; w_rdy_pct = 50; b_lat_min = 1; b_lat_max = 10;
            snk_rdy_pct = 35; src_gap_pct = 40; filt_stall = 35; lite_gap_max = 3; name = "slow";
        endfunction
        function void prof_random();
            int a [4] = '{100, 70, 30, 10};
            ar_rdy_pct = a[$urandom_range(3)]; r_gap_pct = $urandom_range(60);
            rd_lat_min = $urandom_range(4); rd_lat_max = rd_lat_min + $urandom_range(12);
            aw_rdy_pct = a[$urandom_range(3)]; w_rdy_pct = 20 + $urandom_range(80);
            b_lat_min = $urandom_range(3); b_lat_max = b_lat_min + $urandom_range(10);
            snk_rdy_pct = 15 + $urandom_range(85); src_gap_pct = $urandom_range(60);
            filt_stall = $urandom_range(60); lite_gap_max = $urandom_range(4); name = "random";
        endfunction
    endclass

    // ------------------------------------------------------------ memory model
    class mem_model;
        byte unsigned m[];
        function new();
            m = new[MEM_BYTES];
            for (int unsigned i = 0; i < MEM_BYTES; i++) m[i] = pat(i);
        endfunction
        static function byte unsigned pat(int unsigned a);
            logic [31:0] t;
            t = a * 37 + (a >> 8) * 11 + 32'h5A;
            return t[7:0];
        endfunction
        // 0 = OKAY, 2 = SLVERR, 3 = DECERR  (byte address of any byte in the word)
        function int resp(int unsigned a);
            if (a >= MEM_BYTES) return 3;
            if (a >= SLV_BASE && a < SLV_BASE + SLV_SIZE) return 2;
            return 0;
        endfunction
        function byte unsigned rd_byte(int unsigned a);
            if (resp(a) != 0) return 8'h00;
            return m[a];
        endfunction
        function void wr_byte(int unsigned a, byte unsigned d);
            if (resp(a) == 0) m[a] = d;
        endfunction
        function logic [31:0] rd_word(int unsigned wa);
            return {rd_byte(wa+3), rd_byte(wa+2), rd_byte(wa+1), rd_byte(wa)};
        endfunction
        function void wr_word(int unsigned wa, logic [31:0] d, logic [3:0] s);
            for (int i = 0; i < 4; i++) if (s[i]) wr_byte(wa + i, d[8*i +: 8]);
        endfunction
        function void load_image(int unsigned base);
            for (int i = 0; i < g_img.size(); i++) m[base + i] = g_img[i];
        endfunction
        function void copy_from(mem_model o);
            m = o.m;
        endfunction
        // bit0 = touches SLVERR window, bit1 = touches non-existent memory
        static function int region_flags(int unsigned a, int n);
            int unsigned s, e;
            int f = 0;
            if (n <= 0) return 0;
            s = a & ~32'd3;
            e = (a + n + 3) & ~32'd3;
            if (s < SLV_BASE + SLV_SIZE && e > SLV_BASE) f |= 1;
            if (e > MEM_BYTES) f |= 2;
            return f;
        endfunction
    endclass

    // ------------------------------------------------------------ transaction
    typedef enum {K_XFER, K_REGS, K_ZERO, K_HALTED, K_HWRESET} kind_e;
    typedef enum {M_LOOP, M_MM, M_SM} mode_e;

    class dma_txn;
        kind_e kind = K_XFER;
        mode_e mode = M_LOOP;
        string tag  = "";
        int unsigned src = SRC_BASE, dst = DST_BASE;
        int len = 0;                  // MM2S length (bytes)
        int sm_len = 0;               // S2MM programmed length
        byte unsigned stream[$];      // M_SM : bytes the TB streams in
        bit  abort_en = 0;  bit abort_first_sm = 0;  int abort_delay = 50;
        bit  poke_busy = 0;           // write LEN again while busy (must be ignored)
        int  hw_delay = 100;
        bit  dump_out = 0;            // write output file after this txn
        // ---- results gathered by the driver
        bit use_mm, use_sm, timeout;
        bit [31:0] sr_mm, sr_sm, sr_mm_after, sr_sm_after, cr_mm, cr_sm, len_sm_rd, len_mm_rd, len_mm_busy_rd;
        function void finish_setup();
            use_mm = (mode != M_SM);
            use_sm = (mode != M_MM);
            if (mode == M_LOOP) sm_len = len;
        endfunction
    endclass

    // ------------------------------------------------------------ coverage
    class tb_cov;
        int cbin[string];
        function void hit(string b);
            if (cbin.exists(b)) cbin[b]++; else cbin[b] = 1;
        endfunction
        function string lenbin(int n);
            if (n == 0) return "0"; if (n == 1) return "1"; if (n < 4) return "2-3"; if (n == 4) return "4";
            if (n < 16) return "5-15"; if (n == 16) return "16"; if (n < 64) return "17-63"; if (n == 64) return "64";
            if (n < 256) return "65-255"; if (n == 256) return "256"; if (n < 4096) return "257-4095";
            if (n == 4096) return "4096"; if (n < 65536) return "4097-65535"; return ">=65536";
        endfunction
        function void sample(dma_txn t);
            hit($sformatf("kind:%s", t.kind.name()));
            if (t.kind != K_XFER) return;
            hit($sformatf("mode:%s", t.mode.name()));
            if (t.use_mm) begin
                hit($sformatf("mm_len:%s", lenbin(t.len)));
                hit($sformatf("mm_src_off:%0d", t.src & 3));
                if (((t.src & 4095) + t.len) > 4096) hit("mm_cross_4k");
            end
            if (t.use_sm) begin
                hit($sformatf("sm_len:%s", lenbin(t.sm_len)));
                hit($sformatf("sm_dst_off:%0d", t.dst & 3));
                if (((t.dst & 4095) + t.sm_len) > 4096) hit("sm_cross_4k");
            end
            if (t.mode == M_SM) begin
                if (t.stream.size() < t.sm_len) hit("sm_early_tlast");
                else if (t.stream.size() > t.sm_len) hit("sm_stream_longer_than_len");
                else hit("sm_exact_len");
            end
            if (t.abort_en) hit($sformatf("abort:%s_%s", t.mode.name(), t.abort_first_sm ? "sm_first" : "mm_first"));
            if (t.poke_busy) hit("len_write_while_busy");
            if (t.use_mm) begin int f = mem_model::region_flags(t.src, t.len); if (f&1) hit("mm_slverr"); if (f&2) hit("mm_decerr"); end
            if (t.use_sm) begin int f = mem_model::region_flags(t.dst, t.sm_len); if (f&1) hit("sm_slverr"); if (f&2) hit("sm_decerr"); end
        endfunction
        function void report(string must[$]);
            int holes = 0;
            string keys[$];
            $display("\n----------------------- FUNCTIONAL COVERAGE -----------------------");
            foreach (cbin[k]) keys.push_back(k);
            keys.sort();
            foreach (keys[i]) $display("  %-34s %0d", keys[i], cbin[keys[i]]);
            foreach (must[i]) if (!cbin.exists(must[i])) begin $display("  HOLE: %s never hit", must[i]); holes++; end
            $display("  required bins hit: %0d / %0d", must.size() - holes, must.size());
        endfunction
    endclass

    // ============================================================ SCOREBOARD
    //  Monitors only observe the pins.  They forward what they see to the
    //  scoreboard through "analysis port" style method calls.
    // ============================================================
    class scoreboard;
        mem_model ref_mem;
        // captured streams / bus activity (cleared at the beginning of each txn)
        byte unsigned mm_data[$];  bit mm_last[$];
        byte unsigned sm_data[$];  bit sm_last[$];
        int unsigned  wr_addr[$];  byte unsigned wr_data[$];
        int ar_cnt, ar_words, r_beats, aw_cnt, aw_words, w_beats, b_cnt, lite_wr_cnt, lite_rd_cnt;
        int total_txn = 0, passed_txn = 0;

        function new(mem_model rm); ref_mem = rm; endfunction

        function void clear();
            mm_data.delete(); mm_last.delete(); sm_data.delete(); sm_last.delete();
            wr_addr.delete(); wr_data.delete();
            ar_cnt = 0; ar_words = 0; r_beats = 0; aw_cnt = 0; aw_words = 0; w_beats = 0; b_cnt = 0;
        endfunction

        function bit bus_quiet();
            return (ar_cnt == 0 && aw_cnt == 0 && mm_data.size() == 0 && sm_data.size() == 0);
        endfunction

        // The place where a real image filter would be modelled
        function byte unsigned filter_model(byte unsigned b);
            return b;   // pass-through stage in the TB
        endfunction

        function bit [31:0] exp_sr(bit [31:0] base, int flags);
            bit [31:0] v = base | SR_IDLE | SR_IOC;
            if (flags != 0) v |= SR_ERR;
            if (flags & 1) v |= SR_SLV;
            if (flags & 2) v |= SR_DEC;
            return v;
        endfunction

        // ---------------- scoreboard main check for one transfer ----------------
        function void check(dma_txn t);
            string p = $sformatf("[%s] ", t.tag);
            int n_mm = 0, n_sm = 0, f_mm = 0, f_sm = 0;
            total_txn++;
            n_mm = t.len;

            // ===================== MM2S : memory -> stream
            if (t.use_mm) begin
                f_mm = mem_model::region_flags(t.src, t.len);
                if (t.abort_en) begin
                    tb_log::chk(mm_data.size() <= t.len, {p, "MM2S sent more bytes than programmed"});
                end else begin
                    tb_log::chk(mm_data.size() == t.len, $sformatf("%sMM2S stream bytes %0d != len %0d", p, mm_data.size(), t.len));
                end
                for (int k = 0; k < mm_data.size() && k < t.len; k++) begin
                    byte unsigned e = ref_mem.rd_byte(t.src + k);
                    if (mm_data[k] !== e) begin
                        tb_log::chk(0, $sformatf("%sMM2S data[%0d] got %02h exp %02h (addr %08h)", p, k, mm_data[k], e, t.src + k));
                        break;
                    end
                    tb_log::checks++;
                end
                if (!t.abort_en && mm_last.size() > 0) begin
                    int nl = 0;
                    foreach (mm_last[k]) if (mm_last[k]) nl++;
                    tb_log::chk(nl == 1 && mm_last[mm_last.size()-1], $sformatf("%sMM2S TLAST wrong (count=%0d, on last=%0d)", p, nl, mm_last[mm_last.size()-1]));
                end
                if (!t.abort_en) begin
                    int words = ((t.src & 3) + t.len + 3) >> 2;
                    tb_log::chk(ar_words == words, $sformatf("%sMM2S AR words %0d != expected %0d", p, ar_words, words));
                    tb_log::chk(r_beats == ar_words, $sformatf("%sR beats %0d != AR words %0d", p, r_beats, ar_words));
                    tb_log::chk(t.sr_mm === exp_sr(0, f_mm), $sformatf("%sMM2S SR %08h exp %08h", p, t.sr_mm, exp_sr(0, f_mm)));
                    tb_log::chk(t.len_mm_rd == t.len, $sformatf("%sMM2S LEN reg %0d != %0d", p, t.len_mm_rd, t.len));
                end
            end

            // ===================== S2MM : stream -> memory
            if (t.use_sm) begin
                bit [31:0] exp_len;
                // expected accepted stream
                if (t.mode == M_LOOP) begin
                    for (int k = 0; k < sm_data.size() && k < mm_data.size(); k++) begin
                        byte unsigned e = filter_model(mm_data[k]);
                        if (sm_data[k] !== e) begin tb_log::chk(0, $sformatf("%sfilter-stage data[%0d] got %02h exp %02h", p, k, sm_data[k], e)); break; end
                        tb_log::checks++;
                    end
                    if (!t.abort_en) tb_log::chk(sm_data.size() == mm_data.size(), $sformatf("%sS2MM accepted %0d bytes, MM2S produced %0d", p, sm_data.size(), mm_data.size()));
                end else begin
                    int n = (t.stream.size() < t.sm_len) ? t.stream.size() : t.sm_len;
                    if (t.abort_en) tb_log::chk(sm_data.size() <= n, $sformatf("%sS2MM accepted too many bytes", p));
                    else            tb_log::chk(sm_data.size() == n, $sformatf("%sS2MM accepted %0d bytes, expected %0d", p, sm_data.size(), n));
                    for (int k = 0; k < sm_data.size() && k < t.stream.size(); k++)
                        if (sm_data[k] !== t.stream[k]) begin tb_log::chk(0, $sformatf("%sS2MM accepted data[%0d] mismatch", p, k)); break; end
                end
                n_sm = sm_data.size();
                f_sm = mem_model::region_flags(t.dst, n_sm);
                // bytes seen on the write bus
                if (t.abort_en) tb_log::chk(wr_data.size() <= sm_data.size(), $sformatf("%sS2MM wrote %0d bytes > accepted %0d", p, wr_data.size(), sm_data.size()));
                else            tb_log::chk(wr_data.size() == sm_data.size(), $sformatf("%sS2MM wrote %0d bytes, accepted %0d", p, wr_data.size(), sm_data.size()));
                for (int k = 0; k < wr_data.size() && k < sm_data.size(); k++) begin
                    if (wr_addr[k] !== t.dst + k || wr_data[k] !== sm_data[k]) begin
                        tb_log::chk(0, $sformatf("%sS2MM write byte %0d addr %08h data %02h, exp addr %08h data %02h", p, k, wr_addr[k], wr_data[k], t.dst + k, sm_data[k]));
                        break;
                    end
                    tb_log::checks++;
                end
                // update reference memory with what must be stored
                for (int k = 0; k < wr_data.size() && k < sm_data.size(); k++) ref_mem.wr_byte(t.dst + k, sm_data[k]);
                // AW/W/B bookkeeping
                if (!t.abort_en) begin
                    int words = ((t.dst & 3) + n_sm + 3) >> 2;
                    if (n_sm == 0) words = 0;
                    tb_log::chk(aw_words == words, $sformatf("%sS2MM AW words %0d != expected %0d", p, aw_words, words));
                    exp_len = n_sm;
                    tb_log::chk(t.len_sm_rd == exp_len, $sformatf("%sS2MM LEN reg (rx bytes) %0d != %0d", p, t.len_sm_rd, n_sm));
                    tb_log::chk(t.sr_sm === exp_sr(0, f_sm), $sformatf("%sS2MM SR %08h exp %08h", p, t.sr_sm, exp_sr(0, f_sm)));
                end
                tb_log::chk(w_beats == aw_words, $sformatf("%sW beats %0d != AW words %0d", p, w_beats, aw_words));
                tb_log::chk(b_cnt == aw_cnt, $sformatf("%sB count %0d != AW count %0d", p, b_cnt, aw_cnt));
            end

            // ===================== registers after clear
            if (!t.abort_en) begin
                if (t.use_mm) tb_log::chk(t.sr_mm_after === (SR_IDLE | ((f_mm & 1) ? SR_SLV : 0) | ((f_mm & 2) ? SR_DEC : 0)),
                                  $sformatf("%sMM2S SR after W1C = %08h", p, t.sr_mm_after));
                if (t.use_sm) tb_log::chk(t.sr_sm_after === (SR_IDLE | ((f_sm & 1) ? SR_SLV : 0) | ((f_sm & 2) ? SR_DEC : 0)),
                                  $sformatf("%sS2MM SR after W1C = %08h", p, t.sr_sm_after));
            end else begin
                if (t.use_mm) tb_log::chk(t.sr_mm_after === SR_HALTED && t.cr_mm === 0, $sformatf("%sMM2S after abort SR=%08h CR=%08h (exp 1/0)", p, t.sr_mm_after, t.cr_mm));
                if (t.use_sm) tb_log::chk(t.sr_sm_after === SR_HALTED && t.cr_sm === 0, $sformatf("%sS2MM after abort SR=%08h CR=%08h (exp 1/0)", p, t.sr_sm_after, t.cr_sm));
            end

            // ===================== memory image compare (touched region + guard)
            if (t.use_sm) begin
                int unsigned lo = (t.dst > 64) ? t.dst - 64 : 0;
                int unsigned hi = t.dst + n_sm + 64;
                if (hi > MEM_BYTES) hi = MEM_BYTES;
                begin : cmpblk
                    int bad = 0;
                    for (int unsigned a = lo; a < hi; a++) begin
                        if (dut_mem.m[a] !== ref_mem.m[a]) begin
                            if (bad < 3) tb_log::chk(0, $sformatf("%smemory mismatch @%08h dut=%02h ref=%02h", p, a, dut_mem.m[a], ref_mem.m[a]));
                            bad++;
                        end
                    end
                    tb_log::chk(bad == 0, $sformatf("%s%0d memory bytes differ", p, bad));
                end
            end
            if (tb_log::errors == errs_at_start) passed_txn++;
            errs_at_start = tb_log::errors;
        endfunction

        mem_model dut_mem;
        int errs_at_start = 0;

        function void full_mem_compare(string where_);
            int bad = 0;
            for (int unsigned a = 0; a < MEM_BYTES; a++) if (dut_mem.m[a] !== ref_mem.m[a]) bad++;
            tb_log::chk(bad == 0, $sformatf("full memory compare %s : %0d bytes differ", where_, bad));
        endfunction
    endclass

    // ---------------------------------------------------------------- axis monitor
    class axis_mon;
        virtual dma_if vif; tb_cfg cfg; scoreboard sb;
        int which;   // 0 = DUT MM2S output, 1 = DUT S2MM input
        task run();
            bit pv = 0; logic [7:0] pd = 0; bit pl = 0;
            forever begin
                logic v, r, l; logic [7:0] d;
                @(posedge vif.aclk);
                if (which == 0) begin v = vif.mm_tvalid; r = vif.mm_tready; d = vif.mm_tdata; l = vif.mm_tlast; end
                else            begin v = vif.sm_tvalid; r = vif.sm_tready; d = vif.sm_tdata; l = vif.sm_tlast; end
                if (!vif.aresetn) begin pv = 0; continue; end
                if (cfg.chk_stable && pv) begin
                    tb_log::chk(v === 1'b1, $sformatf("AXIS[%0d] TVALID dropped before TREADY", which));
                    tb_log::chk(d === pd && l === pl, $sformatf("AXIS[%0d] payload changed while stalled", which));
                end
                pv = (v === 1'b1) && (r !== 1'b1); pd = d; pl = l;
                if (v === 1'b1 && r === 1'b1) begin
                    if (which == 0) begin sb.mm_data.push_back(d); sb.mm_last.push_back(l); end
                    else            begin sb.sm_data.push_back(d); sb.sm_last.push_back(l); end
                end
            end
        endtask
    endclass

    // ---------------------------------------------------------------- AXI read monitor
    class axi_rd_mon;
        virtual dma_if vif; tb_cfg cfg; scoreboard sb;
        task run();
            bit pv = 0; logic [31:0] pa = 0; logic [7:0] pl = 0;
            int rem = 0;      // outstanding R beats
            forever begin
                @(posedge vif.aclk);
                if (!vif.aresetn) begin pv = 0; rem = 0; continue; end
                if (cfg.chk_stable && pv) begin
                    tb_log::chk(vif.rd_arvalid === 1'b1, "AR: ARVALID dropped before ARREADY");
                    tb_log::chk(vif.rd_araddr === pa && vif.rd_arlen === pl, "AR: payload changed while stalled");
                end
                pv = vif.rd_arvalid && !vif.rd_arready; pa = vif.rd_araddr; pl = vif.rd_arlen;
                if (vif.rd_arvalid && vif.rd_arready) begin
                    int n = vif.rd_arlen + 1;
                    tb_log::chk(vif.rd_arsize === 3'd2,  "AR: ARSIZE != 2 (4 bytes)");
                    tb_log::chk(vif.rd_arburst === 2'b01, "AR: ARBURST != INCR");
                    tb_log::chk(n <= 16, $sformatf("AR: burst length %0d > 16", n));
                    tb_log::chk((vif.rd_araddr[1:0]) === 2'b00, $sformatf("AR: unaligned ARADDR %08h", vif.rd_araddr));
                    tb_log::chk((vif.rd_araddr[11:0] + n * 4) <= 4096, $sformatf("AR: burst crosses 4KB (addr %08h len %0d)", vif.rd_araddr, n));
                    sb.ar_cnt++; sb.ar_words += n; rem += n;
                end
                if (vif.rd_rvalid && vif.rd_rready) begin
                    sb.r_beats++; rem--;
                end
            end
        endtask
    endclass

    // ---------------------------------------------------------------- AXI write monitor
    class axi_wr_mon;
        virtual dma_if vif; tb_cfg cfg; scoreboard sb;
        int unsigned aq_addr[$]; int aq_len[$];
        task run();
            bit pv = 0; logic [31:0] pa = 0; logic [7:0] pl = 0;
            int unsigned cur_addr = 0; int cur_n = 0, beat = 0; bit in_burst = 0;
            forever begin
                @(posedge vif.aclk);
                if (!vif.aresetn) begin pv = 0; aq_addr.delete(); aq_len.delete(); in_burst = 0; continue; end
                if (cfg.chk_stable && pv) begin
                    tb_log::chk(vif.wr_awvalid === 1'b1, "AW: AWVALID dropped before AWREADY");
                    tb_log::chk(vif.wr_awaddr === pa && vif.wr_awlen === pl, "AW: payload changed while stalled");
                end
                pv = vif.wr_awvalid && !vif.wr_awready; pa = vif.wr_awaddr; pl = vif.wr_awlen;
                // W channel is processed first so a same-cycle AW is not consumed early
                if (vif.wr_wvalid && vif.wr_wready) begin
                    if (!in_burst) begin
                        if (aq_addr.size() == 0) begin
                            tb_log::chk(0, "W beat observed with no AW outstanding");
                        end else begin
                            cur_addr = aq_addr.pop_front(); cur_n = aq_len.pop_front(); beat = 0; in_burst = 1;
                        end
                    end
                    if (in_burst) begin
                        int unsigned wa = (cur_addr & ~32'd3) + beat * 4;
                        tb_log::chk(vif.wr_wlast === (beat == cur_n - 1), $sformatf("W: WLAST wrong at beat %0d of %0d", beat, cur_n));
                        tb_log::chk(vif.wr_wstrb !== 4'h0, "W: WSTRB all zero");
                        for (int i = 0; i < 4; i++)
                            if (vif.wr_wstrb[i]) begin sb.wr_addr.push_back(wa + i); sb.wr_data.push_back(vif.wr_wdata[8*i +: 8]); end
                        sb.w_beats++;
                        beat++;
                        if (beat == cur_n) in_burst = 0;
                    end
                end
                if (vif.wr_awvalid && vif.wr_awready) begin
                    int n = vif.wr_awlen + 1;
                    tb_log::chk(vif.wr_awsize === 3'd2,  "AW: AWSIZE != 2 (4 bytes)");
                    tb_log::chk(vif.wr_awburst === 2'b01, "AW: AWBURST != INCR");
                    tb_log::chk(n <= 16, $sformatf("AW: burst length %0d > 16", n));
                    tb_log::chk(vif.wr_awaddr[1:0] === 2'b00, $sformatf("AW: unaligned AWADDR %08h", vif.wr_awaddr));
                    tb_log::chk((vif.wr_awaddr[11:0] + n * 4) <= 4096, $sformatf("AW: burst crosses 4KB (addr %08h len %0d)", vif.wr_awaddr, n));
                    aq_addr.push_back(vif.wr_awaddr); aq_len.push_back(n);
                    sb.aw_cnt++; sb.aw_words += n;
                end
                if (vif.wr_bvalid && vif.wr_bready) sb.b_cnt++;
            end
        endtask
    endclass

    // ---------------------------------------------------------------- AXI-Lite monitor
    class lite_mon;
        virtual dma_if vif; tb_cfg cfg; scoreboard sb;
        task run();
            bit pb = 0, pr = 0;
            forever begin
                @(posedge vif.aclk);
                if (!vif.aresetn) begin pb = 0; pr = 0; continue; end
                if (pb) tb_log::chk(vif.l_bvalid === 1'b1, "LITE: BVALID dropped before BREADY");
                if (pr) tb_log::chk(vif.l_rvalid === 1'b1, "LITE: RVALID dropped before RREADY");
                pb = vif.l_bvalid && !vif.l_bready;
                pr = vif.l_rvalid && !vif.l_rready;
                if (vif.l_bvalid && vif.l_bready) begin sb.lite_wr_cnt++; tb_log::chk(vif.l_bresp === 2'b00, "LITE: BRESP != OKAY"); end
                if (vif.l_rvalid && vif.l_rready) begin sb.lite_rd_cnt++; tb_log::chk(vif.l_rresp === 2'b00, "LITE: RRESP != OKAY"); end
            end
        endtask
    endclass

    // ============================================================ BFMs
    class axi_rd_bfm;   // memory slave for the MM2S read master
        virtual dma_if vif; tb_cfg cfg; mem_model mem;
        int unsigned q_addr[$]; int q_len[$];
        task run();
            int st = 0, lat = 0, beat = 0, cur_len = 0; int unsigned cur_addr = 0;
            forever begin
                bit hs, hold;
                @(posedge vif.aclk);
                if (!vif.aresetn) begin
                    q_addr.delete(); q_len.delete(); st = 0;
                    vif.rd_arready <= 1'b0; vif.rd_rvalid <= 1'b0; vif.rd_rlast <= 1'b0;
                    continue;
                end
                // ---- AR channel
                if (vif.rd_arvalid && vif.rd_arready) begin q_addr.push_back(vif.rd_araddr); q_len.push_back(vif.rd_arlen + 1); end
                vif.rd_arready <= ($urandom_range(99) < cfg.ar_rdy_pct);
                // ---- R channel
                hs   = vif.rd_rvalid &&  vif.rd_rready;
                hold = vif.rd_rvalid && !vif.rd_rready;
                if (hs) begin
                    beat++; vif.rd_rvalid <= 1'b0; vif.rd_rlast <= 1'b0;
                    if (beat == cur_len) st = 0;
                end
                if (!hold) begin
                    if (st == 0 && q_addr.size() > 0) begin
                        cur_addr = q_addr.pop_front(); cur_len = q_len.pop_front(); beat = 0; st = 1;
                        lat = cfg.rd_lat_min + $urandom_range(cfg.rd_lat_max - cfg.rd_lat_min);
                    end
                    if (st == 1) begin if (lat > 0) lat--; else st = 2; end
                    if (st == 2 && $urandom_range(99) >= cfg.r_gap_pct) begin
                        int unsigned wa = (cur_addr & ~32'd3) + beat * 4;
                        int rsp = mem.resp(wa);
                        vif.rd_rdata  <= mem.rd_word(wa);
                        vif.rd_rresp  <= rsp[1:0];
                        vif.rd_rlast  <= (beat == cur_len - 1);
                        vif.rd_rvalid <= 1'b1;
                    end
                end
            end
        endtask
    endclass

    class axi_wr_bfm;   // memory slave for the S2MM write master
        virtual dma_if vif; tb_cfg cfg; mem_model mem;
        int unsigned aq_addr[$]; int aq_len[$];
        int bq_resp[$];
        task run();
            int unsigned cur_addr = 0; int cur_n = 0, beat = 0, cur_resp = 0; bit in_burst = 0;
            int b_wait = 0;
            forever begin
                bit more;
                @(posedge vif.aclk);
                if (!vif.aresetn) begin
                    aq_addr.delete(); aq_len.delete(); bq_resp.delete(); in_burst = 0;
                    vif.wr_awready <= 1'b0; vif.wr_wready <= 1'b0; vif.wr_bvalid <= 1'b0;
                    continue;
                end
                // ---- AW
                if (vif.wr_awvalid && vif.wr_awready) begin aq_addr.push_back(vif.wr_awaddr); aq_len.push_back(vif.wr_awlen + 1); end
                vif.wr_awready <= ($urandom_range(99) < cfg.aw_rdy_pct);
                // ---- W
                if (vif.wr_wvalid && vif.wr_wready) begin
                    if (!in_burst) begin
                        cur_addr = aq_addr.pop_front(); cur_n = aq_len.pop_front(); beat = 0; cur_resp = 0; in_burst = 1;
                    end
                    begin
                        int unsigned wa = (cur_addr & ~32'd3) + beat * 4;
                        int r = mem.resp(wa);
                        if (r > cur_resp) cur_resp = r;
                        mem.wr_word(wa, vif.wr_wdata, vif.wr_wstrb);
                        beat++;
                        if (beat == cur_n) begin
                            in_burst = 0; bq_resp.push_back(cur_resp);
                            if (bq_resp.size() == 1) b_wait = cfg.b_lat_min + $urandom_range(cfg.b_lat_max - cfg.b_lat_min);
                        end
                    end
                end
                more = in_burst || (aq_addr.size() > 0);
                vif.wr_wready <= more && ($urandom_range(99) < cfg.w_rdy_pct);
                // ---- B
                if (vif.wr_bvalid && vif.wr_bready) vif.wr_bvalid <= 1'b0;
                if (!(vif.wr_bvalid && !vif.wr_bready) && bq_resp.size() > 0) begin
                    if (b_wait > 0) b_wait--;
                    else begin
                        int r = bq_resp.pop_front();
                        vif.wr_bresp <= r[1:0]; vif.wr_bvalid <= 1'b1;
                        b_wait = cfg.b_lat_min + $urandom_range(cfg.b_lat_max - cfg.b_lat_min);
                    end
                end
            end
        endtask
    endclass

    class axis_sink;    // ready generator for DUT MM2S output (non-loopback mode)
        virtual dma_if vif; tb_cfg cfg;
        task run();
            forever begin
                @(posedge vif.aclk);
                vif.snk_tready <= ($urandom_range(99) < cfg.snk_rdy_pct);
            end
        endtask
    endclass

    // ============================================================ DRIVER
    class dma_driver;
        virtual dma_if vif; tb_cfg cfg; mem_model mem; scoreboard sb; tb_cov cov;
        mailbox #(dma_txn) mbx;
        int done_cnt = 0;
        dma_txn cur;
        bit src_kill = 0, src_active = 0;
        string out_file = "output_image.mem";

        // ----------------------------------------------- AXI-Lite master
        task lite_gap();
            if (cfg.lite_gap_max > 0) repeat ($urandom_range(cfg.lite_gap_max)) @(posedge vif.aclk);
        endtask

        // order: 0 = AW & W together, 1 = AW first, 2 = W first
        task lite_wr(input bit [6:0] a, input bit [31:0] d, input bit [3:0] s = 4'hF, input int order = 0, input int bdly = 0);
            int aw_wait = (order == 2) ? 4 : 0;
            int w_wait  = (order == 1) ? 4 : 0;
            lite_gap();
            fork
                begin
                    repeat (aw_wait) @(posedge vif.aclk);
                    vif.l_awaddr <= a; vif.l_awvalid <= 1'b1;
                    do @(posedge vif.aclk); while (!vif.l_awready);
                    vif.l_awvalid <= 1'b0;
                end
                begin
                    repeat (w_wait) @(posedge vif.aclk);
                    vif.l_wdata <= d; vif.l_wstrb <= s; vif.l_wvalid <= 1'b1;
                    do @(posedge vif.aclk); while (!vif.l_wready);
                    vif.l_wvalid <= 1'b0;
                end
            join
            repeat (bdly) @(posedge vif.aclk);
            vif.l_bready <= 1'b1;
            do @(posedge vif.aclk); while (!vif.l_bvalid);
            vif.l_bready <= 1'b0;
        endtask

        task lite_rd(input bit [6:0] a, output bit [31:0] d, input int rdly = 0);
            lite_gap();
            vif.l_araddr <= a; vif.l_arvalid <= 1'b1;
            do @(posedge vif.aclk); while (!vif.l_arready);
            vif.l_arvalid <= 1'b0;
            repeat (rdly) @(posedge vif.aclk);
            vif.l_rready <= 1'b1;
            do @(posedge vif.aclk); while (!vif.l_rvalid);
            d = vif.l_rdata;
            vif.l_rready <= 1'b0;
        endtask

        task expect_reg(input bit [6:0] a, input bit [31:0] e, input string what);
            bit [31:0] g;
            lite_rd(a, g);
            tb_log::chk(g === e, $sformatf("REG %s @%02h = %08h, expected %08h", what, a, g, e));
        endtask

        // ----------------------------------------------- reset helpers
        task hw_reset(input int cycles = 12);
            vif.aresetn <= 1'b0;
            repeat (cycles) @(posedge vif.aclk);
            vif.aresetn <= 1'b1;
            repeat (10) @(posedge vif.aclk);
        endtask

        // ----------------------------------------------- stream source (M_SM)
        task src_proc();
            dma_txn t = cur;
            int n = t.stream.size();
            src_active = 1;
            for (int i = 0; i < n && !src_kill; i++) begin
                while (!src_kill && $urandom_range(99) < cfg.src_gap_pct) begin
                    vif.src_tvalid <= 1'b0; @(posedge vif.aclk);
                end
                if (src_kill) break;
                vif.src_tdata <= t.stream[i]; vif.src_tlast <= (i == n - 1); vif.src_tvalid <= 1'b1;
                do @(posedge vif.aclk); while (!src_kill && !vif.sm_tready);
            end
            vif.src_tvalid <= 1'b0; vif.src_tlast <= 1'b0;
            src_active = 0;
        endtask

        task wait_ioc(input bit [6:0] sr_addr, output bit [31:0] sr, output bit to);
            int unsigned t0 = vif.cyc;
            to = 0;
            forever begin
                lite_rd(sr_addr, sr);
                if (sr[12]) break;
                if (vif.cyc - t0 > cfg.timeout) begin to = 1; break; end
                repeat ($urandom_range(4)) @(posedge vif.aclk);
            end
        endtask

        // ----------------------------------------------- abort (soft reset via CR[2])
        task abort_chan(input bit [6:0] cr, input bit [6:0] sr, output bit [31:0] crv, output bit [31:0] srv, output bit to);
            int unsigned t0 = vif.cyc;
            to = 0;
            lite_wr(cr, 32'h5);
            forever begin
                lite_rd(cr, crv);
                lite_rd(sr, srv);
                if (!crv[2] && srv[0]) break;
                if (vif.cyc - t0 > cfg.timeout) begin to = 1; break; end
            end
        endtask

        // ----------------------------------------------- one DMA transfer
        task run_xfer(dma_txn t);
            bit to1 = 0, to2 = 0, to3 = 0, saved_stable = cfg.chk_stable;
            int saved_snk = cfg.snk_rdy_pct;
            t.finish_setup();
            cur = t;
            sb.clear();
            vif.loop_en    <= (t.mode == M_LOOP);
            vif.filt_stall <= cfg.filt_stall;
            if (t.abort_en) cfg.chk_stable = 0;
            if (t.mode == M_SM && t.stream.size() > t.sm_len) cfg.chk_stable = 0;
            if (t.poke_busy) cfg.snk_rdy_pct = 0;
            src_kill = 0;

            if (t.use_sm) begin lite_wr(SM_CR, 32'h1); lite_wr(SM_DA, t.dst); lite_wr(SM_LEN, t.sm_len); end
            if (t.use_mm) begin lite_wr(MM_CR, 32'h1); lite_wr(MM_SA, t.src); lite_wr(MM_LEN, t.len);   end
            if (t.mode == M_SM) fork src_proc(); join_none

            if (t.poke_busy) begin
                repeat (40) @(posedge vif.aclk);
                lite_wr(MM_LEN, t.len + 7);                 // must be ignored: transfer in flight
                lite_rd(MM_LEN, t.len_mm_busy_rd);
                cfg.snk_rdy_pct = saved_snk;
            end

            if (t.abort_en) begin
                repeat (t.abort_delay) @(posedge vif.aclk);
                if (t.mode == M_MM) abort_chan(MM_CR, MM_SR, t.cr_mm, t.sr_mm_after, to1);
                else if (t.mode == M_SM) abort_chan(SM_CR, SM_SR, t.cr_sm, t.sr_sm_after, to1);
                else if (t.abort_first_sm) begin
                    abort_chan(SM_CR, SM_SR, t.cr_sm, t.sr_sm_after, to1);
                    repeat (20) @(posedge vif.aclk);
                    abort_chan(MM_CR, MM_SR, t.cr_mm, t.sr_mm_after, to2);
                end else begin
                    abort_chan(MM_CR, MM_SR, t.cr_mm, t.sr_mm_after, to1);
                    repeat (20) @(posedge vif.aclk);
                    abort_chan(SM_CR, SM_SR, t.cr_sm, t.sr_sm_after, to2);
                end
                src_kill = 1;
            end else begin
                if (t.use_sm) wait_ioc(SM_SR, t.sr_sm, to1);
                if (t.use_mm) wait_ioc(MM_SR, t.sr_mm, to2);
                src_kill = 1;
                if (t.use_sm) lite_rd(SM_LEN, t.len_sm_rd);
                if (t.use_mm) lite_rd(MM_LEN, t.len_mm_rd);
                // clear IOC / ERR (write-1-to-clear, byte lane 1)
                if (t.use_sm) begin lite_wr(SM_SR, (SR_IOC | SR_ERR), 4'b0010); lite_rd(SM_SR, t.sr_sm_after); end
                if (t.use_mm) begin lite_wr(MM_SR, (SR_IOC | SR_ERR), 4'b0010); lite_rd(MM_SR, t.sr_mm_after); end
            end
            t.timeout = to1 | to2 | to3;
            tb_log::chk(!t.timeout, $sformatf("[%s] TIMEOUT waiting for the DMA", t.tag));
            while (src_active) @(posedge vif.aclk);
            vif.src_tvalid <= 1'b0;
            repeat (40) @(posedge vif.aclk);        // let monitors / write responses settle
            cfg.chk_stable = saved_stable;
            cfg.snk_rdy_pct = saved_snk;
            if (t.timeout) begin hw_reset(); sb.ref_mem.copy_from(mem); end
            else           sb.check(t);
            if (t.dump_out && !t.timeout) dump_output(t);
        endtask

        // ----------------------------------------------- output file
        task dump_output(dma_txn t);
            int fd = $fopen(out_file, "w");
            int n = sb.sm_data.size();
            if (fd == 0) begin tb_log::chk(0, {"cannot open ", out_file}); return; end
            if (g_img_fmt == 0) begin
                for (int i = 0; i < n; i++) $fdisplay(fd, "%02h", mem.m[t.dst + i]);
            end else begin
                for (int i = 0; i < n; i += 4) begin
                    logic [31:0] w = 0;
                    for (int j = 0; j < 4; j++) if (i + j < n) w[8*j +: 8] = mem.m[t.dst + i + j];
                    $fdisplay(fd, "%08h", w);
                end
            end
            $fclose(fd);
            tb_log::info($sformatf("wrote %0d bytes to %s (from DDR model @%08h)", n, out_file, t.dst));
        endtask

        // ----------------------------------------------- register test
        task run_regs(dma_txn t);
            bit [31:0] g;
            string tg = t.tag;
            sb.clear();
            // reset defaults
            foreach (RLIST[i]) expect_reg(RLIST[i], (RLIST[i] == MM_SR || RLIST[i] == SM_SR) ? 32'h1 : 32'h0, {tg, " reset default"});
            // unmapped addresses read zero and ignore writes
            foreach (UNMAP[i]) begin lite_wr(UNMAP[i], 32'hFFFF_FFFF); expect_reg(UNMAP[i], 32'h0, {tg, " unmapped"}); end
            // address registers: full word, byte strobes
            lite_wr(MM_SA, 32'hDEAD_BEEF);            expect_reg(MM_SA, 32'hDEAD_BEEF, "MM_SA");
            lite_wr(MM_SA, 32'h1122_3344, 4'b0101);   expect_reg(MM_SA, 32'hDE22_BE44, "MM_SA strb 0101");
            lite_wr(MM_SA, 32'hAABB_CCDD, 4'b1000);   expect_reg(MM_SA, 32'hAA22_BE44, "MM_SA strb 1000");
            lite_wr(MM_SA, 32'h0, 4'b0000);           expect_reg(MM_SA, 32'hAA22_BE44, "MM_SA strb 0000");
            lite_wr(SM_DA, 32'h1234_5678);            expect_reg(SM_DA, 32'h1234_5678, "SM_DA");
            lite_wr(SM_DA, 32'hFFFF_FFFF, 4'b0010);   expect_reg(SM_DA, 32'h1234_FF78, "SM_DA strb 0010");
            expect_reg(MM_SA, 32'hAA22_BE44, "MM_SA unaffected by SM_DA");
            // LEN ignored while halted
            lite_wr(MM_LEN, 32'd100); expect_reg(MM_LEN, 32'h0, "MM_LEN write while halted");
            lite_wr(SM_LEN, 32'd100); expect_reg(SM_LEN, 32'h0, "SM_LEN write while halted");
            // CR run/stop
            lite_wr(MM_CR, 32'h1);        expect_reg(MM_CR, 32'h1, "MM_CR rs=1"); expect_reg(MM_SR, SR_IDLE, "MM_SR idle");
            lite_wr(MM_CR, 32'h0, 4'b0000); expect_reg(MM_CR, 32'h1, "MM_CR strb=0 ignored");
            lite_wr(MM_CR, 32'h0);        expect_reg(MM_CR, 32'h0, "MM_CR rs=0"); expect_reg(MM_SR, SR_HALTED, "MM_SR halted");
            lite_wr(SM_CR, 32'h1);        expect_reg(SM_CR, 32'h1, "SM_CR rs=1"); expect_reg(SM_SR, SR_IDLE, "SM_SR idle");
            lite_wr(SM_CR, 32'h0);        expect_reg(SM_CR, 32'h0, "SM_CR rs=0");
            // soft reset with nothing running self-clears
            lite_wr(MM_CR, 32'h5);        repeat (8) @(posedge vif.aclk);
            expect_reg(MM_CR, 32'h0, "MM_CR after soft reset"); expect_reg(MM_SR, SR_HALTED, "MM_SR after soft reset");
            // zero-length LEN sets IOC, W1C clears it
            lite_wr(MM_CR, 32'h1); lite_wr(MM_LEN, 32'h0);
            expect_reg(MM_SR, SR_IDLE | SR_IOC, "MM_SR IOC after len=0");
            lite_wr(MM_SR, SR_IOC, 4'b0001);      expect_reg(MM_SR, SR_IDLE | SR_IOC, "MM_SR W1C wrong lane ignored");
            lite_wr(MM_SR, SR_IOC, 4'b0010);      expect_reg(MM_SR, SR_IDLE, "MM_SR W1C IOC");
            lite_wr(MM_CR, 32'h0);
            // AXI-Lite channel ordering / delayed responses
            lite_wr(MM_SA, 32'h0000_1000, 4'hF, 1, 0); expect_reg(MM_SA, 32'h1000, "AW-before-W");
            lite_wr(MM_SA, 32'h0000_2000, 4'hF, 2, 0); expect_reg(MM_SA, 32'h2000, "W-before-AW");
            lite_wr(MM_SA, 32'h0000_3000, 4'hF, 0, 5); expect_reg(MM_SA, 32'h3000, "late BREADY");
            lite_rd(MM_SA, g, 6); tb_log::chk(g === 32'h3000, "late RREADY");
            // leave everything clean
            lite_wr(MM_SA, 32'h0); lite_wr(SM_DA, 32'h0);
            lite_wr(MM_CR, 32'h0); lite_wr(SM_CR, 32'h0);
            expect_reg(MM_SR, SR_HALTED, "MM_SR end"); expect_reg(SM_SR, SR_HALTED, "SM_SR end");
        endtask

        bit [6:0] RLIST[8] = '{MM_CR, MM_SR, MM_SA, MM_LEN, SM_CR, SM_SR, SM_DA, SM_LEN};
        bit [6:0] UNMAP[6] = '{7'h08, 7'h0C, 7'h14, 7'h20, 7'h3C, 7'h7C};

        // ----------------------------------------------- zero length / halted
        task run_zero(dma_txn t);
            sb.clear();
            foreach (CH[i]) begin
                bit [6:0] cr = CH[i][0], sr = CH[i][1], ln = CH[i][2];
                lite_wr(cr, 32'h1);
                lite_wr(ln, 32'h0);
                expect_reg(sr, SR_IDLE | SR_IOC, "SR after LEN=0");
                lite_wr(sr, SR_IOC | SR_ERR, 4'b0010);
                expect_reg(sr, SR_IDLE, "SR after clear");
                lite_wr(cr, 32'h0);
            end
            repeat (20) @(posedge vif.aclk);
            tb_log::chk(sb.bus_quiet(), $sformatf("[%s] LEN=0 generated bus/stream activity", t.tag));
        endtask
        bit [6:0] CH[2][3] = '{'{MM_CR, MM_SR, MM_LEN}, '{SM_CR, SM_SR, SM_LEN}};

        task run_halted(dma_txn t);
            sb.clear();
            lite_wr(MM_SA, SRC_BASE); lite_wr(MM_LEN, 32'd64);
            lite_wr(SM_DA, DST_BASE); lite_wr(SM_LEN, 32'd64);
            repeat (60) @(posedge vif.aclk);
            expect_reg(MM_SR, SR_HALTED, "MM_SR halted"); expect_reg(SM_SR, SR_HALTED, "SM_SR halted");
            tb_log::chk(sb.bus_quiet(), $sformatf("[%s] LEN write while halted started a transfer", t.tag));
            lite_wr(MM_SA, 32'h0); lite_wr(SM_DA, 32'h0);
        endtask

        // ----------------------------------------------- hardware reset in the middle of a transfer
        task run_hwreset(dma_txn t);
            t.mode = M_LOOP; t.finish_setup();
            cur = t; sb.clear();
            cfg.chk_stable = 0;
            vif.loop_en <= 1'b1;
            lite_wr(SM_CR, 1); lite_wr(SM_DA, t.dst); lite_wr(SM_LEN, t.len);
            lite_wr(MM_CR, 1); lite_wr(MM_SA, t.src); lite_wr(MM_LEN, t.len);
            repeat (t.hw_delay) @(posedge vif.aclk);
            hw_reset(12);
            cfg.chk_stable = 1;
            foreach (RLIST[i]) expect_reg(RLIST[i], (RLIST[i] == MM_SR || RLIST[i] == SM_SR) ? 32'h1 : 32'h0, "after hw reset");
            sb.ref_mem.copy_from(mem);      // partial writes before reset are legitimate
            sb.clear();
        endtask

        // ----------------------------------------------- main loop
        task run();
            forever begin
                dma_txn t;
                mbx.get(t);
                tb_log::info($sformatf("txn %0d [%s] %s %s", done_cnt, cfg.name, t.kind.name(), t.tag));
                case (t.kind)
                    K_XFER:    run_xfer(t);
                    K_REGS:    run_regs(t);
                    K_ZERO:    run_zero(t);
                    K_HALTED:  run_halted(t);
                    K_HWRESET: run_hwreset(t);
                endcase
                cov.sample(t);
                done_cnt++;
            end
        endtask
    endclass

    // ============================================================ GENERATOR
    class dma_gen;
        mailbox #(dma_txn) mbx;
        int sent = 0;
        function dma_txn xfer(mode_e m, int unsigned src, int unsigned dst, int len, string tag);
            dma_txn t = new();
            t.mode = m; t.src = src; t.dst = dst; t.len = len; t.sm_len = len; t.tag = tag;
            if (m == M_SM) begin
                for (int i = 0; i < len; i++) t.stream.push_back($urandom_range(255));
            end
            return t;
        endfunction
        task put(dma_txn t);
            mbx.put(t); sent++;
        endtask
        task put_kind(kind_e k, string tag);
            dma_txn t = new(); t.kind = k; t.tag = tag; put(t);
        endtask
        // random legal transfer
        function dma_txn random_xfer(int idx);
            int r = $urandom_range(99);
            int len;
            mode_e m;
            dma_txn t;
            if (r < 45) len = 1 + $urandom_range(40);
            else if (r < 85) len = 41 + $urandom_range(560);
            else len = 601 + $urandom_range(7000);
            r = $urandom_range(99);
            m = (r < 40) ? M_LOOP : (r < 70) ? M_MM : M_SM;
            t = xfer(m, SRC_BASE + $urandom_range(32'h8000), DST_BASE + $urandom_range(32'h8000), len, $sformatf("rand%0d", idx));
            if (m == M_SM) begin
                int q = $urandom_range(99);
                if (q < 25 && len > 1)  begin int k = 1 + $urandom_range(len - 1); while (t.stream.size() > k) void'(t.stream.pop_back()); end
                else if (q >= 85)        begin for (int i = 0; i < 1 + $urandom_range(20); i++) t.stream.push_back($urandom_range(255)); end
            end
            return t;
        endfunction
    endclass

    // ============================================================ ENV
    class dma_env;
        virtual dma_if vif;
        tb_cfg cfg;
        mem_model mem, ref_mem;
        scoreboard sb; tb_cov cov;
        lite_mon lm; axis_mon mmm, smm; axi_rd_mon rdm; axi_wr_mon wrm;
        axi_rd_bfm rdb; axi_wr_bfm wrb; axis_sink snk;
        dma_driver drv; dma_gen gen;
        mailbox #(dma_txn) mbx;

        function new(virtual dma_if v);
            vif = v;
            cfg = new(); mem = new(); ref_mem = new();
            mem.load_image(SRC_BASE); ref_mem.load_image(SRC_BASE);
            sb = new(ref_mem); sb.dut_mem = mem;
            cov = new(); mbx = new();
            lm = new();  lm.vif = v;  lm.cfg = cfg;  lm.sb = sb;
            mmm = new(); mmm.vif = v; mmm.cfg = cfg; mmm.sb = sb; mmm.which = 0;
            smm = new(); smm.vif = v; smm.cfg = cfg; smm.sb = sb; smm.which = 1;
            rdm = new(); rdm.vif = v; rdm.cfg = cfg; rdm.sb = sb;
            wrm = new(); wrm.vif = v; wrm.cfg = cfg; wrm.sb = sb;
            rdb = new(); rdb.vif = v; rdb.cfg = cfg; rdb.mem = mem;
            wrb = new(); wrb.vif = v; wrb.cfg = cfg; wrb.mem = mem;
            snk = new(); snk.vif = v; snk.cfg = cfg;
            drv = new(); drv.vif = v; drv.cfg = cfg; drv.mem = mem; drv.sb = sb; drv.cov = cov; drv.mbx = mbx;
            gen = new(); gen.mbx = mbx;
        endfunction

        task start();
            fork
                lm.run(); mmm.run(); smm.run(); rdm.run(); wrm.run();
                rdb.run(); wrb.run(); snk.run(); drv.run();
            join_none
        endtask

        task wait_idle();
            while (drv.done_cnt != gen.sent) @(posedge vif.aclk);
        endtask
    endclass

endpackage