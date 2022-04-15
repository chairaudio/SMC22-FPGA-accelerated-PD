
module MIPipeline#(parameter N_MASSES = 43, parameter ADDR_WIDTH = 5)
(
	clk, 
	reset, 
	i_force, 
	out, 
	data_ready_flag,
	shared_addr,
	shared_read_data
);
	// with clk @ 50Mhz, and SR @ 48 Khz: N_MASSES <= 43, ADDR_WIDTH=5, N_CONNECTIONS <= 903
	// 1043 cycles - 43*42/2+3*43 cycles = 9 cycles left over
	input clk;
	input reset;
	input  signed[26:0]i_force;
	// shared memory access
	output reg [9:0] shared_addr;
	input  [31:0] shared_read_data;
	
	output reg signed[26:0]out;
	output reg data_ready_flag;

	reg [ADDR_WIDTH:0] idx_m1, idx_m2;
	reg [ADDR_WIDTH:0] input_node = 2;
	reg [ADDR_WIDTH:0] output_node = 3;
	wire signed[26:0] tension;
	assign tension = 27'h003F_FFFF;//shared_read_data [26:0];// 27'h003F_FFFF;
	reg signed[26:0] damping = 27'h00_8FFF; // 27'h0000_8FFF
	
	reg [9:0] flat_mat_idx = 0 /* synthesis: preserve*/;
	
	
	// define a string
	reg  [0:903-1] flat_mat = {1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1};
	reg  [N_MASSES-1:0]  wall = {1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1 };
	// 25 masses: reg  [25-1:0]  wall = {1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1};
	
	wire signed[26:0] pos_m1_old, pos_m2_old, pos_m1, pos_m2, pos_m1_in, pos_m1_update, clear_or_new_force_m1, clear_or_new_force_m2, pos_m2_in, pos_m2_old_in;
		// automaton states
	reg wait_for_shared_ram = 0;
	reg calculate_mass_forces = 0;
	reg calculate_wall_forces = 0;
	reg calculate_position_updates = 0;
	reg clear_force_accumulator = 0;
	reg mass_pos_write_en = 0;
	reg force_acc1_write_en = 0;
	reg get_input_force = 0;
	
	
	

	fast_dual_ram #(.SIZE(27), .ADDR_WIDTH(ADDR_WIDTH+1) ) mass_position(
		.q1(pos_m1), 
		.q2(pos_m2), 
		.d(pos_m1_update),
		.w_addr(idx_m1),
		.r_addr_q1(idx_m1),
		.r_addr_q2(idx_m2),
		.we(mass_pos_write_en), 
		.clk(clk)
	);

	fast_dual_ram #(.SIZE(27), .ADDR_WIDTH(ADDR_WIDTH+1)) mass_position_old(
		.q1(pos_m1_old), 
		.q2(pos_m2_old), 
		.d(pos_m1),
		.w_addr(idx_m1),
		.r_addr_q1(idx_m1),
		.r_addr_q2(idx_m2),
		.we(mass_pos_write_en),
		.clk(clk)
	);

	wire signed [26:0] new_force_m1, new_force_m2;
	wire signed [26:0] acc_force_m1, acc_force_m2;
	// maximum number of springs (calculation steps): n*(n-1)/2-n = 43*(43-1))/2 = 903

	fast_ram #(.SIZE(27), .ADDR_WIDTH(ADDR_WIDTH+1)) force_accumulator_m1(
		.q(acc_force_m1), 
		.d(clear_or_new_force_m1),
		.addr(idx_m1),
		.we(force_acc1_write_en),
		.clk(clk)
	);
	fast_ram #(.SIZE(27), .ADDR_WIDTH(ADDR_WIDTH+1)) force_accumulator_m2(
		.q(acc_force_m2), 
		.d(clear_or_new_force_m2),
		.addr(idx_m2),
		.we(force_acc1_write_en),
		.clk(clk)
	);

	wire signed [26:0] o_spring_force;
	Spring spring_mass(pos_m2_in, pos_m1, tension, o_spring_force);
	
	wire signed [26:0] o_damper_force;
	Damper damper_mass(pos_m2_in, pos_m2_old_in, pos_m1, pos_m1_old, damping, o_damper_force);
	
	wire signed [26:0] i_mass_m1 = 0; 
	MassPositionUpdater MPU_inst(pos_m1, pos_m1_old, acc_force_m1 + acc_force_m2, i_mass_m1, pos_m1_update);
	
	// force multiplexer 
	wire shared_data_flat_mat /*synthesis: preserve*/;
	assign shared_data_flat_mat = ((shared_read_data > 0) && calculate_mass_forces)? 1 : 0;
	wire local_test_flat_mat /*synthesis: preserve*/;
	assign local_test_flat_mat= (flat_mat[flat_mat_idx] && calculate_mass_forces)? 1 : 0;
	
	wire signed[27:0] force_spring_m1;
	assign force_spring_m1 = (local_test_flat_mat)? o_spring_force + o_damper_force : 0;
	
	wire signed[27:0] force_wall_m1;
	assign force_wall_m1 = (wall[idx_m1]==1)? o_spring_force + o_damper_force : 0;
	
	assign pos_m2_in = (calculate_wall_forces)? 0 : pos_m2;

	assign pos_m2_old_in = (calculate_wall_forces)? 0 : pos_m2_old;
	
	assign new_force_m1 = (calculate_wall_forces)? force_wall_m1 + acc_force_m1: force_spring_m1 + acc_force_m1;
	
	wire signed[27:0] add_input_to_new_force;
	assign add_input_to_new_force = (get_input_force)? new_force_m1 + i_force : new_force_m1;
	
	assign clear_or_new_force_m1 = (clear_force_accumulator)? 0: add_input_to_new_force;
	
	///
	assign new_force_m2 = (local_test_flat_mat)? acc_force_m2 - o_spring_force - o_damper_force : acc_force_m2;
	
	assign clear_or_new_force_m2 = (clear_force_accumulator)? 0: new_force_m2;
	
	/// state machine
	always @ (posedge clk) begin
		if(reset) begin
			data_ready_flag <= 0;
			mass_pos_write_en <=  0;	
			idx_m1 = 0;
			idx_m2 = 1;
			flat_mat_idx = 0;
			
			wait_for_shared_ram <= 1;
			shared_addr <= 0;
			calculate_mass_forces <= 0;
			force_acc1_write_en <= 1;
			//force_acc2_write_en <= 1;
			calculate_wall_forces <= 0;
			calculate_position_updates <= 0;
			clear_force_accumulator <= 0;
		end
		if (wait_for_shared_ram) begin
			calculate_mass_forces <= 1;
			shared_addr <= shared_addr+1;
			wait_for_shared_ram <= 0;
		end
		if( calculate_mass_forces ) begin
			// calculate and sum forces for each mass		
			if((idx_m2 == N_MASSES-1) && (idx_m1 == N_MASSES-2)) begin
			// finished mass force calculation
				calculate_wall_forces <= 1;
				calculate_mass_forces <= 0;
				force_acc1_write_en <= 1;
				//force_acc2_write_en <= 0;
				idx_m1 = 0;
				idx_m2 = 0;
				flat_mat_idx <= 0;
				shared_addr <= 0;
				if( 0==input_node ) begin
					get_input_force <= 1;
				end
				else get_input_force <= 0;
			end
			// increment column index
			else if(idx_m2 < N_MASSES-1) begin
				idx_m2 = idx_m2 + 1;
				flat_mat_idx <= flat_mat_idx + 1;
				shared_addr <= shared_addr + 1;
			end
			// increment row index if column at max
			else if(idx_m2 == N_MASSES-1) begin
				idx_m1 = idx_m1 + 1;
				flat_mat_idx = flat_mat_idx + 1;
				shared_addr <= shared_addr + 1;
				// start calculating after the matrix diagonal
				idx_m2 = idx_m1 + 1;
			end
		end
		if( calculate_wall_forces ) begin
			// calculate wall connections
			// add input node connections
			if( idx_m1==input_node-1 ) begin
				get_input_force <= 1;
			end
			else get_input_force <= 0;
			
			if(idx_m1 == N_MASSES-1) begin
				// forces are calculated if row and column index are at max
				calculate_wall_forces <= 0;
				calculate_position_updates <= 1;
				mass_pos_write_en <=  1;
				force_acc1_write_en <= 0;
				idx_m1 = 0;
			end
			// increment row index
			else idx_m1 = idx_m1 + 1;
		end
		if( calculate_position_updates) begin
			 // calculate position updates
			 if( idx_m1 < N_MASSES-1) begin
				//r_pos_m1_update <= pos_m1_update;
				if(idx_m1 == output_node) begin
					out <= pos_m1_update;
				end
				idx_m1 = idx_m1 + 1;
				idx_m2 = idx_m1;
			 end
			 else begin
				//force_calc_ready_flag <= 0;
				mass_pos_write_en <=  0;
				calculate_position_updates <= 0;
				clear_force_accumulator <= 1;
				force_acc1_write_en <= 1;
				idx_m1 = 0;
				idx_m2 = 0;
			end
		end
		if( clear_force_accumulator) begin
			if( idx_m1 < N_MASSES-1) begin
				idx_m1 = idx_m1 + 1;
				idx_m2 = idx_m1;
			end
			else begin
				data_ready_flag <= 1;
				force_acc1_write_en <= 0;
				clear_force_accumulator <= 0;
			end
		end
	end
	

  // the "macro" to dump signals
`ifdef COCOTB_SIM
initial begin
  $dumpfile ("MIPipeline.vcd");
  $dumpvars (0, MIPipeline);
  #1;
end
`endif

endmodule
