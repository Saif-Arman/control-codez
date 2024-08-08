#include "Global.h"
#include "InteractPerceive.h"
#include "ForceTorqueManager.h"
#include "Functions.h"
#include <sstream>

#define X 0
#define Y 1
#define Z 2

InteractPerceive::InteractPerceive()
	: _interact_perceive_state(false)
	, _start_interact_perceive_flag(false)
	, _initial_approach_flag(false)
	, _grasp_flag(false)
	, _opened_grippers_flag(false)
	, _do_open_grippers_flag(false)
	, _open_grippers_cntr(0)
{
	std::fill(std::begin(_starting_FT), std::end(_starting_FT), 0);
}

void InteractPerceive::reset_interact_perceive_flags()
{
	_start_interact_perceive_flag = false;
	_initial_approach_flag = false;
	_grasp_flag = false;
	_do_open_grippers_flag = false;
	_opened_grippers_flag = false;
}

bool InteractPerceive::set_interact_perceive_state(bool state)
{
	_interact_perceive_state = state;
	if (state)
	{
		gotoxy(1, 45);
		std::cout << "move flag: " << _interact_perceive_state;
		clear_ip_error();
		clear_ip_status();
	}
	else
	{
		stop_arm();
		reset_interact_perceive_flags();
		//print_ip_status("Stopped interact_perceive. Press \"M\" to start.");
		gotoxy(1, 45);
		std::cout << "move flag: " << _interact_perceive_state;
	}

	return state;
}

void InteractPerceive::print_ip_status(std::string status)
{
	clear_ip_status();
	std::cout << status;
}

void InteractPerceive::clear_ip_status()
{
	gotoxy(1, 46);
	std::cout << "\r                                                     \r";
}

void InteractPerceive::print_ip_info(std::string info)
{
	print_ip_error(info);
}

void InteractPerceive::print_ip_error(std::string error)
{
	clear_ip_error();
	std::cout << error;
}

void InteractPerceive::clear_ip_error()
{
	gotoxy(1, 44);
	std::cout << "\r                                                     \r";
}

void InteractPerceive::check_force()
{
	if (!_interact_perceive_state || _grasp_flag || !_opened_grippers_flag)
		return;

	std::array<double, 3> threshold = { 2, 2, 2 };
	std::array<double, 3> F_ee = FTMgr.get_F_ee();

	for (auto i = X; i <= Z; i++)
	{
		if (abs(F_ee[i] - _starting_FT[i]) > threshold[i])
		{
			stop_interact_perceive();
			std::stringstream error;
			error << "STOPPED DUE TO check_force() IN " << get_dir_string(i) << " direction.";
			print_ip_error(error.str());
		}
	}
}

std::string InteractPerceive::get_dir_string(int dir)
{
	std::string direction;
	switch(dir)
	{
		case X:
			direction = "X";
			break;
		case Y:
			direction = "Y";
			break;
		case Z:
			direction = "Z";
			break;
		default:
			direction = "Unknown Direction";
			break;
	}
	return direction;
}

void InteractPerceive::do_interact_perceive()
{
	static bool printed_info_once = false;
	if (!printed_info_once)
	{
		printed_info_once = true;
		print_ip_status("Press \"M\" to start interact_perceive!");
	}
		
	if (!_interact_perceive_state)
		return;

	if (!_start_interact_perceive_flag)
	{
		_start_interact_perceive_flag = true;
		ini_time_inte_perc = TimeCheck();
	}

	std::array<double, 3> threshold = { 0.35, 0.35, 0.35 };
	Matrix<3, 3> Re2w = EE2w_transform3(pos);
	Matrix<3, 3> Rw2e = transpose(EE2w_transform3(pos));

	elapsed_time1 = ((float)TimeCheck() - (float)ini_time_inte_perc) / 1000; // in sec

	// Start by opening grippers if they are closed
	if (!_do_open_grippers_flag)
	{
		_do_open_grippers_flag = true;
		do_open_grippers();
	}
	else if (!_opened_grippers_flag && open_in_progress)
	{
		print_ip_status("Opening grippers ...");
	}
	else if (!_opened_grippers_flag && !open_in_progress)
	{
		_opened_grippers_flag = true;
		_initial_approach_flag = true;
		Sleep(500); // Sleep to let the arm settle
		_starting_FT = FTMgr.get_FT_ee();
		print_ip_info("Grippers have been opened!");
	}
	// Go forward until hand touches an object
	else if (_initial_approach_flag)
	{
		std::array<double, FT_SIZE> current_FT = FTMgr.get_FT_ee();
		for (int i = X; i < Z; i++)
		{
			if (abs(current_FT[i] - _starting_FT[i]) < threshold[i])
			{
				go_forward();
				print_ip_status("Approaching object in Z direction");
			}
			else
			{
				stop_arm();
				_initial_approach_flag = false;
				_grasp_flag = true;
				std::stringstream info;
				info << "Initial approach complete due to force in " << get_dir_string(i) << " direction.";
				print_ip_info(info.str());
				break;
			}
		}
	}
	// Attempt to grasp object after touching
	else if (_grasp_flag && !grab_in_progress)
	{
		do_grab_object();
		print_ip_status("Starting grasp ...");
	}
	else if (_grasp_flag)
	{
		print_ip_status("Grasping object ...");
	}
	// Finish interact perceive
	else
	{
		stop_interact_perceive();
		//print_ip_status("Grasping complete!");
	}
	
	if (elapsed_time1 > 30)
	{
		print_ip_error("Stopped interact perceive due to timeout.");
		stop_interact_perceive();
	}

	//for (int i = 1; i < 4; i++)
	//{
	//	speed[i] = static_cast<float>(Vxyz_w(i));
	//}

	new_status = true;

	counter += 1;
}

//......Mushtaq, Feb 2022 :reading from ati-ia F/T  sensor
void InteractPerceive::do_interact_perceive_orig()
{
	// to be chaneged later : //
	float fdz = 1;
	float fdx = 1;
	float vdz = 0;
	float Thrz = 0;
	//float vdx = 0;
	//float vdy = 0;
	float Thr_y = 0;
	float Thr_x = 0.01f;
	float alpha_x = 0;
	float alpha_y = 0;
	float T_xy = 2;  // for generating  osci in y , z to get feeling force
	ColumnVector<3> Vxyz_ee, Vxyz_w;
	Matrix<3, 3> Re2w = EE2w_transform3(pos);
	Matrix<3, 3> Rw2e = transpose(EE2w_transform3(pos));

	static int flag_start1 = 0;

	if (flag_start1 == 0)
	{
		flag_start1 = 1;
		ini_time_inte_perc = TimeCheck();
	}
	elapsed_time1 = ((float)TimeCheck() - (float)ini_time_inte_perc) / 1000; // in sec

	if (switch_contact == 0 && abs(abs(FTMgr.get_F_ee()[0]) - fdx) > Thr_x)
	{
		//Vxyz_ee(1) = vdx * (1 + (F_ee[0] / fdx));
		//Vxyz_ee(2) = 0;
		Vxyz_ee(3) = 0;
		elapsed_time_y = (TimeCheck() - time_y_ini) / 1000;
		//Vxyz_ee(2) = vdy * sign(float(sin(2 * M_PI * elapsed_time1 / T_xy)));
		Vxyz_ee(1) = 0;

		speed[4] = vdy * sign(float(sin(2 * M_PI * elapsed_time1 / T_xy)));

		//speed[4] = vdy;
	}
	else
	{
		Vxyz_ee(1) = 0;
		Vxyz_ee(2) = 0;
		Vxyz_ee(3) = 0;
		/*move_flag_in_x = 0;
		flag_start1 = 0;*/


		/*if (elapsed_time1 > 1)
		{
			swy = 1;
			time_y_ini = TimeCheck();
			flag_touch = 0;
			speed[4] = 0;
		}*/

		if (swy == 1)
		{
			elapsed_time_y = (TimeCheck() - time_y_ini) / 1000;
			//Vxyz_ee(2) = vdy * sign(float(sin(2 * M_PI * elapsed_time_y / T_xy)));
		}
		/*if (elapsed_time1 < 5)
		{
			Vxyz_ee(1) = vdx;
			Vxyz_ee(2) = 0;
			Vxyz_ee(3) = 0;
		}
		else
		{
			move_flag_in_x = 0;
			flag_start1 = 0;
			Vxyz_ee(1) = 0;
			Vxyz_ee(2) = 0;
			Vxyz_ee(3) = 0;
		}*/

		/*if (elapsed_time1 > 60)
		{
			move_flag_in_x = 0;
			flag_start1 = 0;
			Vxyz_ee(1) = 0;
			Vxyz_ee(2) = 0;
			Vxyz_ee(3) = 0;
		}*/
	}
	Vx_ee = static_cast<float>(Vxyz_ee(1));
	Vy_ee = static_cast<float>(Vxyz_ee(2));
	Vz_ee = static_cast<float>(Vxyz_ee(3));
	Vxyz_w = Re2w * Vxyz_ee;

	if (elapsed_time1 > 30)
	{
		speed[4] = 0;
		flag_start1 = 0;
		stop_interact_perceive();
	}

	for (int i = 1; i < 4; i++)
	{
		speed[i] = static_cast<float>(Vxyz_w(i));
	}

	new_status = true;

	gotoxy(1, 55);
	printf("Vy_ee", Vy_ee);

	counter += 1;
}