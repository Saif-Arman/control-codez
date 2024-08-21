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
	inline std::array<double, FT_SIZE> get_FT_ee() { return _FT_ee; };
	inline std::array<double, FT_SIZE / 2> get_F_ee() { return _F_ee; };
	inline std::array<double, FT_SIZE / 2> get_T_ee() { return _T_ee; };

private:

	std::array<double, FT_SIZE> get_raw_FT();

	void compensate_hand_FT(std::array<double, 3>& R);
	void compensate_hand_FT_orig(std::array<double, 3>& R);

	void estimate_r(std::array<double, FT_SIZE> new_ft, std::array<double, 3>& R);

	std::array<double, FT_SIZE> _raw_FT;
	std::array<double, FT_SIZE> _compensated_FT;
	std::array<double, FT_SIZE> _FT_ee;
	std::array<double, FT_SIZE / 2> _F_ee;
	std::array<double, FT_SIZE / 2> _T_ee;
};