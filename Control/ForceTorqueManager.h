#pragma once
#include <array>

#define FT_SIZE 6

class ForceTorqueManager
{
public:
	ForceTorqueManager();

	void ReadForceTorque();

	void updateFT(std::array<double, FT_SIZE> new_FT);

	std::array<double, FT_SIZE> get_raw_FT();

	void interact_perceive();

	inline bool get_interact_perceive_state() { return _interact_perceive_state; };

	void set_interact_perceive_state(bool state);

	void compensate_hand_FT(std::array<double, 3>& R);

	void estimate_r(std::array<double, FT_SIZE> new_ft, std::array<double, 3>& R);

	void check_force();
	
	inline void set_check_force(bool newval) { _check_force = newval; };

private:
	std::array<double, FT_SIZE> _raw_FT;
	std::array<double, FT_SIZE> _compensated_FT;
	std::array<double, FT_SIZE> _starting_FT;
	bool _interact_perceive_state;
	bool _check_force;
};