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
		GRASP_DONE
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
	ControlLogger *_logger;
};

