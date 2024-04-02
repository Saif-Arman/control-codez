#pragma once
#include <vector>

class ForceTorqueManager
{
public:
	ForceTorqueManager();

	void get_calibration();

	void ReadForce(std::vector<double> ft_vec);
	
private:
	std::vector<double>_calibration;
	//double cur_FT[6] = { 0, 0, 0, 0, 0, 0 };
	std::vector<double>_cur_FT = std::vector<double>(6);
};