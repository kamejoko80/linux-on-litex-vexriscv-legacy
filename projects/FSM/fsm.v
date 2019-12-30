/* Machine-generated using Migen */
module top(
	output reg [7:0] s,
	output reg [7:0] a,
	output reg [7:0] b,
	input op,
	input en,
	input sys_clk,
	input sys_rst
);

reg [1:0] state = 2'd0;
reg [1:0] next_state;
reg [7:0] a_next_value0;
reg a_next_value_ce0;
reg [7:0] b_next_value1;
reg b_next_value_ce1;

// synthesis translate_off
reg dummy_s;
initial dummy_s <= 1'd0;
// synthesis translate_on


// synthesis translate_off
reg dummy_d;
// synthesis translate_on
always @(*) begin
	s <= 8'd0;
	next_state <= 2'd0;
	a_next_value0 <= 8'd0;
	a_next_value_ce0 <= 1'd0;
	b_next_value1 <= 8'd0;
	b_next_value_ce1 <= 1'd0;
	next_state <= state;
	case (state)
		1'd1: begin
			s <= (a + b);
			next_state <= 2'd3;
		end
		2'd2: begin
			s <= (a - b);
			next_state <= 2'd3;
		end
		2'd3: begin
			next_state <= 2'd3;
		end
		default: begin
			if ((en == 1'd1)) begin
				a_next_value0 <= 3'd5;
				a_next_value_ce0 <= 1'd1;
				b_next_value1 <= 2'd2;
				b_next_value_ce1 <= 1'd1;
				if ((op == 1'd0)) begin
					next_state <= 1'd1;
				end else begin
					next_state <= 2'd2;
				end
			end
		end
	endcase
// synthesis translate_off
	dummy_d <= dummy_s;
// synthesis translate_on
end

always @(posedge sys_clk) begin
	state <= next_state;
	if (a_next_value_ce0) begin
		a <= a_next_value0;
	end
	if (b_next_value_ce1) begin
		b <= b_next_value1;
	end
	if (sys_rst) begin
		a <= 8'd0;
		b <= 8'd0;
		state <= 2'd0;
	end
end

endmodule


