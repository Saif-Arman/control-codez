#pragma once

#include "ControlLogger.h"

#define FT_SIZE 6

class InteractPerceive
{
public:
	InteractPerceive();

	enum IntPercState
	{
		STOPPED = 0,
		STARTING,
		OPENING_GRIPPERS,
		GRIPPERS_OPENED,
		INITIAL_APPROACH,
		START_GRASP,
		GRASPING_OBJECT,
		GRASP_DONE,
		MIN_TORQUE_Z
	};

	// Main interact perceive loop
	void do_interact_perceive();
	void do_interact_perceive_orig(); // Mushtaq's original function

	// If force from FT sensor exceeds threshold, stops arm
	void check_force();

	// Start/stop the interact_perceive routine
	inline void start_interact_perceive() { set_interact_perceive_state(STARTING); };
	inline void stop_interact_perceive() { set_interact_perceive_state(STOPPED); };
	IntPercState toggle_interact_perceive_state();

	inline void end_interact_perceive_grasp() { if (GRASPING_OBJECT == _interact_perceive_state) _interact_perceive_state = GRASP_DONE; };
	inline IntPercState get_interact_perceive_state() { return _interact_perceive_state; };

	inline float get_move_speed() { return _move_speed; };
	inline void set_move_speed(float speed) { _move_speed = speed; };

private:

	// If STOPPED, will stop interact perceive
	// Anything else will cause the state to go to that point.
	IntPercState set_interact_perceive_state(IntPercState state);

	// Resets all interact perceive flags
	void reset_interact_perceive_flags();

	// Print given direction: 0 = X, 1 = Y, 2 = Z.
	std::string get_dir_string(int dir);

	std::array<double, FT_SIZE> _starting_FT; // FT observed at start of interact perceive routine
	IntPercState _interact_perceive_state; // Is interact perceive running or not
	float _move_speed;
	int _open_grippers_cntr;

	std::array<double, FT_SIZE> _threshold; // thresholds we want to trigger interact perceive reactions at from mini at sensor
	int _grasp_start_time;
	float _elapsed_grasp_time;
	int _int_perc_start_time;
	float _max_ft_time;
	float _v_dy;
	float _v_dx;
	int _oscillations_per_sec;
	int _ip_cntr;
	bool _grasp_done;
	bool _touched_once;
	std::deque<std::array<double, FT_SIZE>> _prev_FT;
	std::array<double, 3> _rpy_offsets;

	std::array<double, FT_SIZE> _last_touch_pos; // position that last exceeded the FT threhsold
	std::array<double, FT_SIZE> _max_FT;
	std::array<double, FT_SIZE> _min_FT;
};

