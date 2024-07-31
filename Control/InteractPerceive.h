#pragma once

#define FT_SIZE 6

class InteractPerceive
{
public:
	InteractPerceive();

	// Main interact perceive loop
	void do_interact_perceive();

	// If force exceeds threshold, stops arm
	void check_force();

	// If true, saves current compensated FT
	// If false, stops arm movement
	// Returns state after setting
	bool set_interact_perceive_state(bool state);

	// Returns state
	inline bool get_interact_perceive_state() { return _interact_perceive_state; };

	// Returns state after toggling
	inline bool toggle_interact_perceive_state() { return set_interact_perceive_state(!_interact_perceive_state); };

private:

	bool _interact_perceive_state;
	std::array<double, FT_SIZE> _starting_FT;
};

