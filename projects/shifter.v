module shifter(
	input  sck_i,
	input  sdi_i,
	output sdo_o,
	input  csn_i,
);

reg [7:0] reg_i = 8'd0;
reg [7:0] cnt = 8'd0;

always @(negedge csn_i) begin
	cnt <= 1'd0;
end

always @(negedge sck_i) begin
	if ((cnt < 4'd8)) begin
		reg_i <= (reg_i <<< 1'd1);
	end
end

always @(posedge sck) begin
	if ((cnt < 4'd8)) begin
		cnt <= (cnt + 1'd1);
	end
	reg_i[0] <= sdi_i;
end

endmodule
