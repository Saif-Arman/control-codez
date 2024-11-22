// Includes
#include "Global.h"
#include "InteractPerceive.h"
#include "ForceTorqueManager.h"
#include "Functions.h"
#include <sstream>
#include <deque>
#include <array>
#include <cmath>

// Defines & Macros
#define X 0
#define Y 1
#define Z 2
#define FX X
#define FY Y
#define FZ Z
#define TX X+3
#define TY Y+3
#define TZ Z+3
#define YAW X+3
#define PITCH Y+3
#define ROLL Z+3

//------------------------------------------------------------------------------------------------------------

InteractPerceive::InteractPerceive()
	: _interact_perceive_state(STOPPED)
	, _grasp_start_time(0)
	, _elapsed_grasp_time(0)
	, _int_perc_start_time(0)
	, _max_ft_time(0)
	, _v_dy(0)
	, _v_dx(0)
	, _oscillation_timer(0)
	, _ip_cntr(0)
	, _touched_once(false)
{
	std::fill(std::begin(_starting_FT), std::end(_starting_FT), 0);
	std::fill(std::begin(_max_FT), std::end(_max_FT), -999);
	std::fill(std::begin(_min_FT), std::end(_min_FT), 999);
	std::fill(std::begin(_last_touch_pos), std::end(_last_touch_pos), 0);
	std::fill(std::begin(_rpy_offsets), std::end(_rpy_offsets), 0);
	_threshold[FX] = 0.55;
	_threshold[FY] = 0.45;
	_threshold[FZ] = 0.45;
	_threshold[TX] = 0.017;
	_threshold[TY] = 0.017;
	_threshold[TZ] = 0.017;
}

//------------------------------------------------------------------------------------------------------------

InteractPerceive::IntPercState InteractPerceive::set_interact_perceive_state(IntPercState state)
{
	_interact_perceive_state = state;
	if (STOPPED == state)
	{
		gLogger->print_ip_info("Stopped interact_perceive. Press \"M\" to start.");
	}
	else
	{
		gLogger->clear_ip_error();
		gLogger->clear_ip_status();
	}

	return _interact_perceive_state;
}

//------------------------------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------------------------------

void InteractPerceive::do_interact_perceive()
{
	// only run this if we want interact perceive, otherwise just skip and return
	if (STOPPED == _interact_perceive_state)
		return;

	// Wait until robot is waiting for a command
	if (0x37F != rcvMsg.ID)
		return;

	FTMgr.ReadForceTorque(); // Read force torque from MINI FT sensor
	Matrix<3, 3> Re2w = EE2w_transform3(pos); // rotation matrix, end effector frame to world frame
	Matrix<3, 3> Rw2e = transpose(EE2w_transform3(pos)); // rotation matrix, world fram to end effector frame

	// State machine for interact perceive.
	// Organized such that each successive state is below the previous
	switch (_interact_perceive_state)
	{
		case STARTING:
		{
			// Grabbing desired parameters for interact perceive
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
				_v_dy = tmp; // 8 is ideal with oscillation value of 4
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
				_v_dx = tmp; // doesnt really matter, using 4
			}

			gLogger->print_ip_status("");
			std::cout << "Please enter the desired force threshold! (from -5 to 0): ";
			std::cin >> tmp;

			if (tmp < -3.0f || tmp > 3.0f)
			{
				std::stringstream logstr;
				logstr << "Given threshold <" << tmp << "> is out of bounds!";
				gLogger->print_ip_error(logstr.str());
				stop_interact_perceive();
				break;
			}
			else
			{
				_threshold[0] = _threshold[1] = _threshold[2] = tmp; // -0.36 works with offsetting the starting FT
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
				_oscillation_timer = tmp; // 4 is ideal with 8 v_dy
			}

			_int_perc_start_time = TimeCheck();
			_starting_FT = FTMgr.get_FT_ee();
			FTMgr.clear_plot_file();

			// Make sure arm has no speed
			for (int i = 0; i < 8; i++)
				speed[i] = 0;
			new_status = true; // Send the speed commands to the arm

			// Reset needed variables for next state
			_grasp_start_time = TimeCheck();
			_max_ft_time = 0;
			_prev_FT.clear();
			std::fill(std::begin(_max_FT), std::end(_max_FT), -999);
			std::fill(std::begin(_min_FT), std::end(_min_FT), 999);
			std::fill(std::begin(_last_touch_pos), std::end(_last_touch_pos), 0);
			_touched_once = false;
			_ip_cntr = 0;

			_interact_perceive_state = START_GRASP;

			break;
		}
		case START_GRASP:
		{
			//static std::array<double, FT_SIZE> last_touch_pos = { 0, 0, 0, 0, 0, 0 }; // position that last exceeded the FT threhsold
			//static std::array<double, FT_SIZE> max_FT = { -999, -999, -999, -999, -999, -999 };
			//static std::array<double, FT_SIZE> min_FT = { 999, 999, 999, 999, 999, 999 };

			_elapsed_grasp_time = (static_cast<float>(TimeCheck()) - static_cast<float>(_grasp_start_time)) / 1000; // in seconds, time spend in START_GRASP state
			std::array<double, FT_SIZE> avg_ft = { 0, 0, 0, 0, 0, 0 };
			std::array<double, FT_SIZE> current_FT = FTMgr.get_FT_ee(); // current FT acquired from mini FT sensor
			int num_ft_samples = 10; // How many samples to consider at a time for interact/perceive
									// this is the "sliding window" over which interact perceive behaves
			speed_mode = 4; // set robot to allow max angular speed moves
			new_status = true; // we need the program to send new commands to the robot
			
			_prev_FT.emplace_back(current_FT);
			int sz = _prev_FT.size();
			if (sz > num_ft_samples)
			{
				_prev_FT.pop_front();
				sz--;
			}

			for (auto it = _prev_FT.begin(); it != _prev_FT.end(); ++it )
			{
				for (int i = 0; i < FT_SIZE; i++)
				{
					avg_ft[i] += (*it)[i];

					// Track max & min FTs
					if ((*it)[i] > _max_FT[i])
						_max_FT[i] = (*it)[i];
					if ((*it)[i] < _min_FT[i])
						_min_FT[i] = (*it)[i];
				}
			}
			for (int i = 0; i < FT_SIZE; i++)
			{
				avg_ft[i] /= sz;
					
				// Subtract initial offset from FT
				current_FT[i] -= _starting_FT[i];
			}

			// Start each control feedback from 0 speed
			for (int i = 0; i < 8; i++)
				speed[i] = 0;

			// Keep steady force as needed , more negative = more force
			if (current_FT[X] > _threshold[X])
			{
				float forward_speed = _v_dx * (1.0f - (static_cast<float>(current_FT[X] / _threshold[X])));
				if (forward_speed > _v_dx)
					forward_speed = _v_dx;

				// Go forward in end effector frame
				go_forward(forward_speed);

				std::stringstream logstr;
				logstr << "Lost touch, +x spd: " << forward_speed << ", f_x/t_x: " << current_FT[X] << "/" << _threshold[X] << ", x_i: " << _starting_FT[X] << std::endl;
				gLogger->print_ip_status(logstr.str());

				if (_touched_once)
				{
					// Get distance traveled since last touch. If it's > length of fingers, assume grasped OR that we're past object
					float distance = sqrt(pow((pos[0] - _last_touch_pos[X]), 2) + pow((pos[1] - _last_touch_pos[Y]), 2) + pow((pos[2] - _last_touch_pos[Z]), 2));
					if (distance > 75)
					{
						gLogger->print_ip_status("Traveled > 60mm! Interact perceive is done.");
						_interact_perceive_state = IP_DONE;
						break;
					}
				}
			}
			else
			{
				_touched_once = true; // Set this flag so we know we've touched something at least once
				std::stringstream logstr;

				if (_ip_cntr < _oscillation_timer/2)
				{
					go_yaw(_v_dy);
					logstr << "+ Y (" << speed[YAW] << ", " << speed[PITCH] << ", " << speed[ROLL] << ", cnt: " << _ip_cntr;
				}
				else
				{
					logstr << "- Y (" << speed[YAW] << ", " << speed[PITCH] << ", " << speed[ROLL] << ", cnt: " << _ip_cntr;
					go_yaw(-_v_dy);
				}
				// x robot ticks in each direction
				if (++_ip_cntr >= _oscillation_timer)
					_ip_cntr = 0;

				logstr << ", ft_x: " << current_FT[X] << ", t_x: " << _threshold[X];
				gLogger->print_ip_status(logstr.str());

				_last_touch_pos[X] = pos[X];
				_last_touch_pos[Y] = pos[Y];
				_last_touch_pos[Z] = pos[Z];
				_last_touch_pos[YAW] = pos[YAW]; //yaw
				_last_touch_pos[PITCH] = pos[PITCH]; //pitch
				_last_touch_pos[ROLL] = pos[ROLL]; //roll
			}
			FTMgr.write_to_plot_file(current_FT);

			break;
		}
		case IP_DONE:
		{
			stop_interact_perceive();
			gLogger->print_ip_status("Interact perceive done!");
		}
	}
}

//------------------------------------------------------------------------------------------------------------


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