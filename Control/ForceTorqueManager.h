#pragma once
#include <array>

#define FT_SIZE 6

class ForceTorqueManager
{
public:
	ForceTorqueManager();
	void ReadForceTorque();
	void updateForce(std::array<double, FT_SIZE> ft_vec);
	std::array<double, 6> get_cur_FT();
	void interac_perc();
	bool get_move_flag_x() { return _move_flag_x; };
	void set_move_flag_x(bool state) { _move_flag_x = state; };
	
private:
	std::array<double, FT_SIZE> _cur_FT;
	bool _move_flag_x;
};