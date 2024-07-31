#pragma once
#include <array>

#define FT_SIZE 6

class ForceTorqueManager
{
public:
	ForceTorqueManager();

	void ReadForceTorque();

	void update_FT(std::array<double, FT_SIZE> new_FT);
		
	inline std::array<double, FT_SIZE> get_compensated_FT() { return _compensated_FT; };

private:

	std::array<double, FT_SIZE> get_raw_FT();

	void compensate_hand_FT(std::array<double, 3>& R);

	void estimate_r(std::array<double, FT_SIZE> new_ft, std::array<double, 3>& R);

	std::array<double, FT_SIZE> _raw_FT;
	std::array<double, FT_SIZE> _compensated_FT;
};