//Fake CPU

module CPU (
	input         reset,
	input         clk_sys,
	input         ce_n,
	input         ce_p,
	output [15:0] cpu_addr,
	output  [7:0] cpu_dout,
	input   [7:0] cpu_din,
	output reg    nMREQ,
	output reg    nIORQ,
	input         nINT,
	output reg    nRD,
	output reg    nWR,
	output reg    nM1,
	output reg    nRFSH
);

reg [2:0] MCycle, MCycles;
reg [2:0] TState, TStates;

reg [15:0] A;

assign cpu_addr = A;


// Fake LDIR bus signals
always @(posedge clk_sys) begin
	if (reset) begin
		MCycle <= 1;
		MCycles <= 5;
		TState <= 1;
		TStates <= 4;
		A <= 16'h4000;
		nMREQ <= 1;
		nIORQ <= 1;
		nRD <= 1;
		nWR <= 1;
		nM1 <= 0;
		nRFSH <= 1;
	end else begin
		case (MCycle)
		1:
		begin
			case (TState)
			1: if (ce_n) {nMREQ, nRD} <= 0;
			2: if (ce_p) begin {nMREQ, nRD, nM1} <= 3'b111; nRFSH <= 0; A<=16'h8000; end
			3: if (ce_n) nMREQ <= 0;
			4: if (ce_n) nMREQ <= 1; else if (ce_p) begin nRFSH <= 1; A<=16'h4000; end
			default: ;
			endcase
		end

		2:
		begin
			case (TState)
			1: ;
			2: ;
			3: ;
			4: if (ce_p) TStates <= 3;
			default: ;
			endcase
		end

		3:
		begin
			case (TState)
			1: ;
			2: ;
			3: if (ce_p) TStates <= 5;
			default: ;
			endcase
		end
		
		4:
		begin
			case (TState)
			1: ;
			2: ;
			3: ;
			4: ;
			5: ;
			default: ;
			endcase
		end

		5:
		begin
			case (TState)
			1: ;
			2: ;
			3: ;
			4: ;
			5: if (ce_p) TStates <= 4;
			default: ;
			endcase
		end

		default: ;

		endcase
		
		if (ce_p) begin
			if (TState == TStates) begin
				TState <= 1;
				if (MCycle == MCycles)
					MCycle <= 1;
				else
					MCycle <= MCycle + 1'd1;
			end else begin
				TState <= TState + 1'd1;
			end
		end
	end
end

endmodule
