/*
 * Hacky baud rate generator to divide a 100MHz clock into a 115200 baud
 * rx/tx pair where the rx clcken oversamples by 16x.
 */
module baud_rate_gen(input wire clk_in,
                     output wire rxclk_en,
                     output wire txclk_en);

parameter RX_ACC_MAX = 100000000 / (115200 * 16);
parameter TX_ACC_MAX = 100000000 / 115200;
parameter RX_ACC_WIDTH = $clog2(RX_ACC_MAX);
parameter TX_ACC_WIDTH = $clog2(TX_ACC_MAX);
reg [RX_ACC_WIDTH - 1:0] rx_acc = 0;
reg [TX_ACC_WIDTH - 1:0] tx_acc = 0;

assign rxclk_en = (rx_acc == 5'd0);
assign txclk_en = (tx_acc == 9'd0);

always @(posedge clk_in) begin
    if (rx_acc == RX_ACC_MAX[RX_ACC_WIDTH - 1:0])
        rx_acc <= 0;
    else
        rx_acc <= rx_acc + 5'b1;
end

always @(posedge clk_in) begin
    if (tx_acc == TX_ACC_MAX[TX_ACC_WIDTH - 1:0])
        tx_acc <= 0;
    else
        tx_acc <= tx_acc + 9'b1;
end

endmodule

module transmitter(input wire [7:0] din,
                   input wire wr_en,
                   input wire clk_in,
                   input wire clken,
                   output reg tx,
                   output wire tx_busy);

initial begin
     tx = 1'b1;
end

parameter STATE_IDLE    = 2'b00;
parameter STATE_START    = 2'b01;
parameter STATE_DATA    = 2'b10;
parameter STATE_STOP    = 2'b11;

reg [7:0] data = 8'h00;
reg [2:0] bitpos = 3'h0;
reg [1:0] state = STATE_IDLE;

always @(posedge clk_in) begin
    case (state)
    STATE_IDLE: begin
        if (wr_en) begin
            state <= STATE_START;
            data <= din;
            bitpos <= 3'h0;
        end
    end
    STATE_START: begin
        if (clken) begin
            tx <= 1'b0;
            state <= STATE_DATA;
        end
    end
    STATE_DATA: begin
        if (clken) begin
            if (bitpos == 3'h7)
                state <= STATE_STOP;
            else
                bitpos <= bitpos + 3'h1;
            tx <= data[bitpos];
        end
    end
    STATE_STOP: begin
        if (clken) begin
            tx <= 1'b1;
            state <= STATE_IDLE;
        end
    end
    default: begin
        tx <= 1'b1;
        state <= STATE_IDLE;
    end
    endcase
end

assign tx_busy = (state != STATE_IDLE);

endmodule

module my_uart(input wire [7:0] din,
           input wire wr_en,
           input wire clk_in,
           output wire tx,
           output wire tx_busy);

wire rxclk_en, txclk_en;

baud_rate_gen uart_baud(.clk_in(clk_in),
            .rxclk_en(rxclk_en),
            .txclk_en(txclk_en));
transmitter uart_tx(.din(din),
            .wr_en(wr_en),
            .clk_in(clk_in),
            .clken(txclk_en),
            .tx(tx),
            .tx_busy(tx_busy));
endmodule
