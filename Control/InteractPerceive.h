#pragma once

#define FT_SIZE 6

class InteractPerceive
{
public:
	InteractPerceive();

	// Main interact perceive loop
	void do_interact_perceive();
	void do_interact_perceive_orig(); // Mushtaq's original function

	// If force from FT sensor exceeds threshold, stops arm
	void check_force();

	// Start/stop the interact_perceive routine
	inline void start_interact_perceive() { set_interact_perceive_state(true); };
	inline void stop_interact_perceive() { set_interact_perceive_state(false); };
	// Returns state after toggling
	inline bool toggle_interact_perceive_state() { return set_interact_perceive_state(!_interact_perceive_state); };

	inline void end_interact_perceive_grasp() { _grasp_flag = false; };

	// Returns state
	inline bool get_interact_perceive_state() { return _interact_perceive_state; };

private:

	// Print/clear status & error messages for interact perceive
	void print_ip_status(std::string status);
	void clear_ip_status();
	void print_ip_info(std::string info);
	void print_ip_error(std::string error);
	void clear_ip_error();

	// If true, saves current compensated FT
	// If false, stops arm movement
	// Returns state after setting
	bool set_interact_perceive_state(bool state);

	// Resets all interact perceive flags
	void reset_interact_perceive_flags();

	// Print given direction: 0 = X, 1 = Y, 2 = Z.
	std::string get_dir_string(int dir);

	std::array<double, FT_SIZE> _starting_FT; // FT observed at start of interact perceive routine
	bool _interact_perceive_state; // Is interact perceive running or not
	bool _start_interact_perceive_flag; // Initializes interact perceive
	bool _initial_approach_flag; // Initializes interact perceive approach
	bool _grasp_flag; // Initializes & continues interact perceive grasp
	bool _opened_grippers_flag; // Flag to see if grippers have been opened by interact perceive yet
	bool _do_open_grippers_flag; // Flag to open the gripers at the start of interact perceive
	int _open_grippers_cntr;
};

