module dma_axis_fifo #(
    parameter integer WIDTH = 32,
    parameter integer DEPTH = 256
)(
    input  wire             clk,
    input  wire             rst_n,
 
    input  wire             wr_en,
    input  wire [WIDTH-1:0] wr_data,
    output wire             full,
 
    input  wire             rd_en,
    output wire [WIDTH-1:0] rd_data,
    output wire             empty
);
    localparam integer AW = $clog2(DEPTH);
 
    (* ram_style = "distributed" *) reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW:0] wp;
    reg [AW:0] rp;
 
    integer init_i;
    initial begin
        for (init_i = 0; init_i < DEPTH; init_i = init_i + 1) mem[init_i] = {WIDTH{1'b0}};
    end
 
    assign empty   = (wp == rp);
    assign full    = (wp[AW] != rp[AW]) && (wp[AW-1:0] == rp[AW-1:0]);
    assign rd_data = mem[rp[AW-1:0]];
 
    wire do_wr = wr_en & ~full;
    wire do_rd = rd_en & ~empty;
 
    always @(posedge clk)
        if (do_wr) mem[wp[AW-1:0]] <= wr_data;
 
    always @(posedge clk) begin
        if (!rst_n) begin
            wp <= {(AW+1){1'b0}};
            rp <= {(AW+1){1'b0}};
        end else begin
            if (do_wr) wp <= wp + 1'b1;
            if (do_rd) rp <= rp + 1'b1;
        end
    end
endmodule
 