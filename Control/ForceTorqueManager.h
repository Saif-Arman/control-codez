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

	void interac_perc();

	inline bool get_move_flag_x() { return _move_flag_x; };

	inline void set_move_flag_x(bool state) { _move_flag_x = state; };

	void compensate_hand_FT(std::array<double, FT_SIZE>& compensated_FT, std::array<double, 3>& r_vec);

	void estimate_r(std::array<double, FT_SIZE> new_ft, std::array<double, 3>& r_vec);

private:
	std::array<double, FT_SIZE> _raw_FT;
	bool _move_flag_x;
};