#pragma once
// Includes
#include "ControlLogger.h"

// Defines & Macros
#define FT_SIZE 6

// Class definition
class InteractPerceive
{
public:
	// Constructor
	InteractPerceive();

	// Destructor
	~InteractPerceive();

	// Enum for the interact perceive state machine states
	enum IntPercState
	{
		STOPPED = 0,
		STARTING,
		START_GRASP,
		MIN_TORQUE_Z,
		IP_DONE,
	};


	// Bound check forces to +/- 4. Shouldn't ever exceed this and it's a pretty big number
	void check_force();

	// Start/stop the interact_perceive routine
	inline void start_interact_perceive() { set_interact_perceive_state(STARTING); };

	// Start/stop the interact_perceive routine
	inline void stop_interact_perceive() { set_interact_perceive_state(STOPPED); };

	// Invert the interact_perceive routine between running/stopped
	IntPercState toggle_interact_perceive_state();

	// returns the current value of _interact_perceive_state
	inline IntPercState get_interact_perceive_state() { return _interact_perceive_state; };

	// Main interact perceive loop
	void do_interact_perceive();

private:

	// If STOPPED, will stop interact perceive
	// Anything else will cause the state to go to that point.
	IntPercState set_interact_perceive_state(IntPercState newstate);

	// Print given direction: 0 = X, 1 = Y, 2 = Z.
	std::string get_dir_string(int dir);

	// Function to perform the startup sequence required to do 
	// the interact perceive routine.
	int do_ip_start();

	// Function that performs the main interact perceive routine
	int do_ip_grasp();

	std::array<double, FT_SIZE> _starting_FT; // FT observed at start of interact perceive routine
	IntPercState _interact_perceive_state; // Is interact perceive running or not
	std::array<double, FT_SIZE> _threshold; // thresholds we want to trigger interact perceive reactions at from mini at sensor
	int _grasp_start_time;
	float _elapsed_grasp_time;
	int _int_perc_start_time;
	float _max_ft_time;
	float _w_dy;
	float _v_dx;
	int _oscillation_count;
	int _ip_cntr;
	int _ip_osc_loops;
	bool _touched_once;
	std::deque<std::array<double, FT_SIZE>> _prev_FT;
	std::array<double, 3> _rpy_offsets;
	std::array<double, FT_SIZE> _last_touch_pos; // position that last exceeded the FT threhsold
	std::array<double, FT_SIZE> _max_FT;
	std::array<double, FT_SIZE> _min_FT;
	std::array<double, FT_SIZE> _current_FT; // current FT acquired from mini FT sensor
};

