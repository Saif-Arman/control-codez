#include "Global.h"
#include "InteractPerceive.h"
#include "ForceTorqueManager.h"
#include "Functions.h"
#include <sstream>
#include <deque>
#include <array>
#include <cmath>

#define X 0
#define Y 1
#define Z 2

InteractPerceive::InteractPerceive()
	: _interact_perceive_state(STOPPED)
	, _open_grippers_cntr(0)
	, _move_speed(10.0f)
{
	std::fill(std::begin(_starting_FT), std::end(_starting_FT), 0);
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
		gLogger->clear_ip_error();
		gLogger->clear_ip_status();
	}
	else
	{
		stop_arm();
		IntPerc.end_interact_perceive_grasp();
		reset_interact_perceive_flags();
		gLogger->print_ip_info("Stopped interact_perceive. Press \"M\" to start.");
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
	if (STOPPED == state)
		return;

	std::array<double, 3> threshold = { 4, 4, 4 };
	std::array<double, 3> F_ee = FTMgr.get_F_ee();

	for (auto i = X; i <= Z; i++)
	{
		if (abs(F_ee[i] - _starting_FT[i]) > threshold[i])
		{
			//stop_interact_perceive();
			std::stringstream error;
			error << "WARNING! check_force() EXCEEDED IN " << get_dir_string(i) << " direction.";
			gLogger->print_ip_error(error.str());
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

	static std::array<double, FT_SIZE> threshold = { 0.55, 0.45, 0.45, 0.017, 0.017, 0.017 };
	Matrix<3, 3> Re2w = EE2w_transform3(pos);
	Matrix<3, 3> Rw2e = transpose(EE2w_transform3(pos));
	static int grasp_start_time = 0;
	static float elapsed_grasp_time = 0;
	static int int_perc_start_time = 0;
	static float max_ft_time = 0;
	static float v_dy = 0;
	static float v_dx = 0;
	static int oscillations_per_sec = 0;
	static bool grasp_done = false;
	static bool touched_once = false;
	static std::deque<std::array<double, FT_SIZE>> prev_FT;
	std::array<float, 3> move_speed_ee_xyz = { 0, 0, 0 };

	switch (_interact_perceive_state)
	{
		case STARTING:
		{
			// Grabbing desired speeds & forces
			float tmp = 0;

			gLogger->print_ip_status("");
			std::cout << "Please enter the desired wiggling speed (from -10 to 10): ";
			std::cin >> tmp;

			if (tmp < -10.0f || tmp > 10.0f)
			{
				std::stringstream logstr;
				logstr << "Given speed <" << tmp << "> is out of bounds!";
				gLogger->print_ip_error(logstr.str());
				stop_interact_perceive();
				break;
			}
			else
			{
				v_dy = tmp;
			}

			gLogger->print_ip_status("");
			std::cout << "Please enter the desired approach speed (from 0 to 15): ";
			std::cin >> tmp;

			if (tmp < 0 || tmp > 15.0f)
			{
				std::stringstream logstr;
				logstr << "Given speed <" << tmp << "> is out of bounds!";
				gLogger->print_ip_error(logstr.str());
				stop_interact_perceive();
				break;
			}
			else
			{
				v_dx = tmp;
			}

			gLogger->print_ip_status("");
			std::cout << "Please enter the desired force threshold! (from -5 to 0): ";
			std::cin >> tmp;

			if (tmp < -5.0f || tmp > 0)
			{
				std::stringstream logstr;
				logstr << "Given threshold <" << tmp << "> is out of bounds!";
				gLogger->print_ip_error(logstr.str());
				stop_interact_perceive();
				break;
			}
			else
			{
				threshold[0] = threshold[1] = threshold[2] = tmp;
			}

			gLogger->print_ip_status("");
			std::cout << "Please enter the desired oscilaltions per sec! (from 0 to 10): ";
			std::cin >> tmp;

			if (tmp < 0 || tmp > 10.0f)
			{
				std::stringstream logstr;
				logstr << "Given oscillations <" << tmp << "> is out of bounds!";
				gLogger->print_ip_error(logstr.str());
				stop_interact_perceive();
				break;
			}
			else
			{
				oscillations_per_sec = tmp;
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
				int i = X;
				if (current_FT[i] > threshold[i])
				{
					go_forward(_move_speed);
					gLogger->print_ip_status("Approaching object in Z direction");
				}
				else
				{
					stop_arm();
					
					std::stringstream info;
					info << "Initial approach complete due to force in " << get_dir_string(i) << " direction.";
					gLogger->print_ip_info(info.str());

					// Reset needed flags for next stage
					grasp_start_time = TimeCheck();
					max_ft_time = 0;
					grasp_done = false;
					prev_FT.clear();
					touched_once = false;
					
					_interact_perceive_state = START_GRASP;

					break;
				}
			//}
			break;
		}
		case START_GRASP:
		{
			elapsed_grasp_time = (static_cast<float>(TimeCheck()) - static_cast<float>(grasp_start_time)) / 1000; // in sec
			static std::array<double, FT_SIZE> last_touch_pos = { 0, 0, 0, 0, 0, 0 };
			std::array<double, FT_SIZE> current_FT = FTMgr.get_FT_ee();
			//std::array<double, FT_SIZE> avg_ft = { 0, 0, 0, 0, 0, 0 };
			static float thr_x_speed;
			int num_ft_samples = 3;
			speed_mode = 4;
			UpdatePos(pos);
			
			//if (prev_FT.size() >= num_ft_samples)
			//{
			//	prev_FT.pop_front();
			//	prev_FT.emplace_back(current_FT);

			//	for (auto it = prev_FT.begin(); it != prev_FT.end(); ++it )
			//	{
			//		for (auto i = 0; i < FT_SIZE; i++)
			//		{
			//			avg_ft[i] += (*it)[i];
			//		}
			//	}
			//	for (auto i = 0; i < FT_SIZE; i++)
			//	{
			//		avg_ft[i] /= num_ft_samples;
			//	}
			//}
			//else
			//{
			//	std::stringstream logstr;
			//	logstr << "Waiting for " << num_ft_samples << " FT samples ...";
			//	gLogger->print_ip_status(logstr.str());
			//	// Get num_fT_samples samples first
			//	prev_FT.emplace_back(current_FT);
			//	touched_once = false;
			//	break;
			//}

			// Keep steady force as needed
			// Threshold should be negative
			//if (avg_ft[X] > threshold[X])
			if (current_FT[X] > threshold[X])
			{
				//float forward_speed = v_dx * (1.0f - (static_cast<float>(avg_ft[X] / threshold[X])));
				float forward_speed = v_dx * (1.0f - (static_cast<float>(current_FT[X] / threshold[X])));
				if (forward_speed > v_dx)
					forward_speed = v_dx;

				go_forward(forward_speed);
				std::stringstream logstr;
				logstr << "Lost touch, re-approaching in X with speed " << forward_speed << std::endl;
				gLogger->print_ip_status(logstr.str());

				speed[X + 3] = 0;
				speed[Y + 3] = 0;
				speed[Z + 3] = 0;

				if (touched_once)
				{
					float distance = sqrt(pow((pos[0] - last_touch_pos[X]), 2) + pow((pos[1] - last_touch_pos[Y]), 2) + pow((pos[2] - last_touch_pos[Z]), 2));
					if (distance > 60)
					{
						gLogger->print_ip_status("Traveled > 60mm! Interact perceive is done.");
						_interact_perceive_state = GRASP_DONE;
						break;
					}
				}
			}
			else
			{
				touched_once = true;
				//speed[X + 1] = 0;
				//speed[Y + 1] = 0;
				//speed[Z + 1] = 0;
				go_forward(0);
				std::stringstream logstr;

				// fastest is about 0.25, making this ~ 4 oscillations per second.
				float oscillation_cntr = sin(2 * M_PI * elapsed_grasp_time * oscillations_per_sec);
				if (oscillation_cntr > 0)
				{
					logstr << "Oscillating in + Y direction, result: " << oscillation_cntr;
					//speed[4] = 1.0f + fabs(v_dy); // yaw
					//speed[4] = 5.0f; // yaw
					go_yaw(v_dy);

				}
				else
				{
					logstr << "Oscillating in - Y direction, result: " << oscillation_cntr;
					//speed[X + 4] = -1.0f - fabs(v_dy);
					//speed[X + 4] = -5.0f;
					go_yaw(-v_dy);
				}
				logstr << "ft_x: " << current_FT[X] << ", thresh_x: " << threshold[X];
				gLogger->print_ip_status(logstr.str());

				//speed[5] = 0; // pitch
				//speed[6] = 0; // roll

				last_touch_pos[X] = pos[0];
				last_touch_pos[Y] = pos[1];
				last_touch_pos[Z] = pos[2];
				last_touch_pos[3] = pos[3]; //yaw
				last_touch_pos[4] = pos[4]; //pitch
				last_touch_pos[5] = pos[5]; //roll
			}

			new_status = true;
			
			break;
		}
		case MIN_TORQUE_Z:
		{
			_interact_perceive_state = GRASP_DONE;
			break;
		}
		case GRASP_DONE:
		{
			gLogger->print_ip_status("Interact perceive done!");
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