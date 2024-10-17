#include "Global.h"
#include "InteractPerceive.h"
#include "ForceTorqueManager.h"
#include "Functions.h"
#include <sstream>

#define X 0
#define Y 1
#define Z 2

InteractPerceive::InteractPerceive()
	: _interact_perceive_state(STOPPED)
	, _open_grippers_cntr(0)
	, _move_speed(5.0f)
{
	std::fill(std::begin(_starting_FT), std::end(_starting_FT), 0);
	_logger = ControlLogger::getInstance();
}

void InteractPerceive::reset_interact_perceive_flags()
{
	_interact_perceive_state = STOPPED;
}

InteractPerceive::IntPercState InteractPerceive::set_interact_perceive_state(IntPercState state)
{
	_interact_perceive_state = state;
	if (STOPPED != state)
	{
		gotoxy(1, 45);
		std::cout << "Interact Perceive State: " << _interact_perceive_state;
		_logger->clear_ip_error();
		_logger->clear_ip_status();
	}
	else
	{
		stop_arm();
		reset_interact_perceive_flags();
		_logger->print_ip_status("Stopped interact_perceive. Press \"M\" to start.");
		gotoxy(1, 45);
		std::cout << "Interact Perceive State: " << _interact_perceive_state;
	}

	return _interact_perceive_state;
}

InteractPerceive::IntPercState InteractPerceive::toggle_interact_perceive_state()
{ 
	if (STOPPED == _interact_perceive_state)
	{
		return set_interact_perceive_state(STARTING);
	}
	else
	{
		return set_interact_perceive_state(STOPPED);
	}
};

void InteractPerceive::check_force()
{
	IntPercState state = _interact_perceive_state;
	if (STOPPED == state || OPENING_GRIPPERS == state || GRASPING_OBJECT == state)
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
			_logger->print_ip_error(error.str());
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
	if (STOPPED == _interact_perceive_state)
		return;

	std::array<double, 3> threshold = { 0.6, 0.21, 0.21 };
	Matrix<3, 3> Re2w = EE2w_transform3(pos);
	Matrix<3, 3> Rw2e = transpose(EE2w_transform3(pos));
	static int grasp_start_time = 0;
	static int int_perc_start_time = 0;
	static float max_ft_time = 0;
	static float v_dy = 0;
	static float v_dx = 0;
	static float max_FT[FT_SIZE] = { -999, -999, -999, -999, -999, -999, };
	static float max_FT_pos[FT_SIZE] = { 0, 0, 0, 0, 0, 0 };
	static float max_pos[FT_SIZE] = { 0, 0, 0, 0, 0, 0 };
	static float min_pos[FT_SIZE] = { 0, 0, 0, 0, 0, 0 };

	switch (_interact_perceive_state)
	{
		case STARTING:
		{
			_logger->print_ip_status("");
			std::cout << "Please enter the desired Y speed (from -10 to 10): ";
			std::cin >> v_dy;

			if (v_dy < -10.0f || v_dy > 10.0f)
			{
				std::stringstream logstr;
				logstr << "Given speed <" << v_dy << "> is out of bounds!";
				_logger->print_ip_error(logstr.str());
				stop_interact_perceive();
				break;
			}

			_logger->print_ip_status("");
			std::cout << "Please enter the desired X speed (from -10 to 10): ";
			std::cin >> v_dx;

			if (v_dx < -10.0f || v_dx > 10.0f)
			{
				std::stringstream logstr;
				logstr << "Given speed <" << v_dx << "> is out of bounds!";
				_logger->print_ip_error(logstr.str());
				stop_interact_perceive();
				break;
			}

			int_perc_start_time = TimeCheck();
			_interact_perceive_state = INITIAL_APPROACH;
			_starting_FT = FTMgr.get_FT_ee();
			break;
		}
		case INITIAL_APPROACH:
		{
			std::array<double, FT_SIZE> current_FT = FTMgr.get_FT_ee();
			//for (int i = X; i < Z; i++)
			//{
				int i = Z;
				if (abs(current_FT[i]) < threshold[i])
				{
					go_forward(_move_speed);
					_logger->print_ip_status("Approaching object in Z direction");
				}
				else
				{
					stop_arm();
					
					std::stringstream info;
					info << "Initial approach complete due to force in " << get_dir_string(i) << " direction.";
					_logger->print_ip_info(info.str());

					// Reset needed flags for next stage
					grasp_start_time = TimeCheck();
					max_ft_time = 0;
					for (auto i = 0; i < FT_SIZE; i++)
					{
						max_FT[i] = -999;
						max_FT_pos[i] = pos[i];
						max_pos[i] = max_FT_pos[i];
						min_pos[i] = max_FT_pos[i];
					}
					
					_interact_perceive_state = START_GRASP;

					break;
				}
			//}
			break;
		}
		case START_GRASP:
		{
			float elapsed_grasp_time = ((float)TimeCheck() - (float)grasp_start_time) / 1000; // in sec
			std::array<double, FT_SIZE> current_FT = FTMgr.get_FT_ee();
			bool prev_speed_pos = 1;

			// Keep steady force as needed
			if (abs(current_FT[Z]) < threshold[Z])
				go_forward(1.1f);
			else
				go_forward(0);

			// Make gripper oscillate to find max forces & torques
			speed[4] = v_dy * sign(float(sin(2 * M_PI * elapsed_grasp_time / 2)));
				
			if (abs(current_FT[X]) > abs(max_FT[X]))
			{
				max_ft_time = elapsed_grasp_time;
				max_FT[X] = current_FT[X];
				for (int i = 0; i < 6; i++)
					max_FT_pos[i] = pos[i];
			}
			// Instead, track forces and go to position where force levels off to minimum
			// JK, use torques :)

			if (elapsed_grasp_time - max_ft_time > 3.0f)
			{
				std::stringstream info;
				info << "Going to Max F_X pos: " << pos[0] << ", " << pos[1] << ", " << pos[2] << ", "
												 << pos[3] << ", " << pos[4] << ", " << pos[5] << ", ";
				_logger->print_ip_info(info.str());
				go_to_position(max_FT_pos);
				_interact_perceive_state = GRASP_MIN_X;
			}

			//if (elapsed_grasp_time > 30)
			//{
			//	_logger->print_ip_error("Stopped interact perceive due to timeout.");
			//	stop_interact_perceive();
			//}

			new_status = true;

			std::stringstream status;
			status << "Starting grasp, elapsed time: " << elapsed_grasp_time;
			_logger->print_ip_status(status.str());
			//_interact_perceive_state = GRASPING_OBJECT;
			break;
		}
		case GRASP_MIN_X:
		{
			stop_interact_perceive();
			break;
		}
		case GRASPING_OBJECT:
		{
			if (grab_in_progress)
			{
				_logger->print_ip_status("Grasping object ...");
				break;
			}
				
			_interact_perceive_state = GRASP_DONE;
			break;
		}
		case GRASP_DONE:
		{
			_logger->print_ip_status("Interact perceive done!");
			stop_interact_perceive();
		}
	}
}


////......Mushtaq, Feb 2022 :reading from ati-ia F/T  sensor
//void InteractPerceive::do_interact_perceive_orig()
//{
//	// to be chaneged later : //
//	float fdz = 1;
//	float fdx = 1;
//	float vdz = 0;
//	float Thrz = 0;
//	//float vdx = 0;
//	//float vdy = 0;
//	float Thr_y = 0;
//	float Thr_x = 0.01f;
//	float alpha_x = 0;
//	float alpha_y = 0;
//	float T_xy = 2;  // for generating  osci in y , z to get feeling force
//	ColumnVector<3> Vxyz_ee, Vxyz_w;
//	Matrix<3, 3> Re2w = EE2w_transform3(pos);
//	Matrix<3, 3> Rw2e = transpose(EE2w_transform3(pos));
//
//	static int flag_start1 = 0;
//
//	if (flag_start1 == 0)
//	{
//		flag_start1 = 1;
//		ini_time_inte_perc = TimeCheck();
//	}
//	elapsed_time1 = ((float)TimeCheck() - (float)ini_time_inte_perc) / 1000; // in sec
//
//	if (switch_contact == 0 && abs(abs(FTMgr.get_F_ee()[0]) - fdx) > Thr_x)
//	{
//		//Vxyz_ee(1) = vdx * (1 + (F_ee[0] / fdx));
//		//Vxyz_ee(2) = 0;
//		Vxyz_ee(3) = 0;
//		elapsed_time_y = (TimeCheck() - time_y_ini) / 1000;
//		//Vxyz_ee(2) = vdy * sign(float(sin(2 * M_PI * elapsed_time1 / T_xy)));
//		Vxyz_ee(1) = 0;
//
//		speed[4] = vdy * sign(float(sin(2 * M_PI * elapsed_time1 / T_xy)));
//
//		//speed[4] = vdy;
//	}
//	else
//	{
//		Vxyz_ee(1) = 0;
//		Vxyz_ee(2) = 0;
//		Vxyz_ee(3) = 0;
//		/*move_flag_in_x = 0;
//		flag_start1 = 0;*/
//
//
//		/*if (elapsed_time1 > 1)
//		{
//			swy = 1;
//			time_y_ini = TimeCheck();
//			flag_touch = 0;
//			speed[4] = 0;
//		}*/
//
//		if (swy == 1)
//		{
//			elapsed_time_y = (TimeCheck() - time_y_ini) / 1000;
//			//Vxyz_ee(2) = vdy * sign(float(sin(2 * M_PI * elapsed_time_y / T_xy)));
//		}
//		/*if (elapsed_time1 < 5)
//		{
//			Vxyz_ee(1) = vdx;
//			Vxyz_ee(2) = 0;
//			Vxyz_ee(3) = 0;
//		}
//		else
//		{
//			move_flag_in_x = 0;
//			flag_start1 = 0;
//			Vxyz_ee(1) = 0;
//			Vxyz_ee(2) = 0;
//			Vxyz_ee(3) = 0;
//		}*/
//
//		/*if (elapsed_time1 > 60)
//		{
//			move_flag_in_x = 0;
//			flag_start1 = 0;
//			Vxyz_ee(1) = 0;
//			Vxyz_ee(2) = 0;
//			Vxyz_ee(3) = 0;
//		}*/
//	}
//	Vx_ee = static_cast<float>(Vxyz_ee(1));
//	Vy_ee = static_cast<float>(Vxyz_ee(2));
//	Vz_ee = static_cast<float>(Vxyz_ee(3));
//	Vxyz_w = Re2w * Vxyz_ee;
//
//	if (elapsed_time1 > 30)
//	{
//		speed[4] = 0;
//		flag_start1 = 0;
//		stop_interact_perceive();
//	}
//
//	for (int i = 1; i < 4; i++)
//	{
//		speed[i] = static_cast<float>(Vxyz_w(i));
//	}
//
//	new_status = true;
//
//	gotoxy(1, 55);
//	printf("Vy_ee", Vy_ee);
//
//	counter += 1;
//}