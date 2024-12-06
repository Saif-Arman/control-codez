// Includes
#include "Global.h"
#include "InteractPerceive.h"
#include "ForceTorqueManager.h"
#include "Functions.h"
#include <sstream>
#include <deque>
#include <array>
#include <cmath>

using namespace DIRECTIONS;
// Defines & Macros
//#define X 0
//#define Y 1
//#define Z 2
#define FX X
#define FY Y
#define FZ Z
#define TX X+3
#define TY Y+3
#define TZ Z+3
//#define YAW X+3
//#define PITCH Y+3
//#define ROLL Z+3
#define IP_NEXT 0
#define IP_OKAY 1
#define IP_ERROR 2

//------------------------------------------------------------------------------------------------------------

InteractPerceive::InteractPerceive()
	: _interact_perceive_state(STOPPED)
	, _grasp_start_time(0)
	, _elapsed_grasp_time(0)
	, _int_perc_start_time(0)
	, _max_ft_time(0)
	, _w_dy(0)
	, _v_dx(0)
	, _oscillation_count(0)
	, _ip_cntr(0)
	, _ip_osc_loops(0)
	, _touched_once(false)
{
	std::fill(std::begin(_starting_FT), std::end(_starting_FT), 0);
	std::fill(std::begin(_current_FT), std::end(_current_FT), 0);
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

InteractPerceive::~InteractPerceive()
{

}

//------------------------------------------------------------------------------------------------------------

InteractPerceive::IntPercState InteractPerceive::set_interact_perceive_state(IntPercState newstate)
{
	_interact_perceive_state = newstate;
	if (STOPPED == newstate)
	{
		gLogger->print_ip_status("Stopped interact_perceive. Press \"M\" to start.");
	}
	else
	{
		gLogger->clear_ip_error();
		gLogger->clear_ip_status();
		gLogger->clear_ip_info();
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

	for (int i = X; i <= Z; i++)
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
	//Matrix<3, 3> Re2w = EE2w_transform3(pos); // rotation matrix, end effector frame to world frame
	//Matrix<3, 3> Rw2e = transpose(EE2w_transform3(pos)); // rotation matrix, world fram to end effector frame

	// State machine for interact perceive.
	// Organized such that each successive state is below the previous
	switch (_interact_perceive_state)
	{
		case STARTING:
		{
			if (IP_NEXT == do_ip_start())
				_interact_perceive_state = START_GRASP;
			else
				stop_interact_perceive();

			break;
		}
		case START_GRASP:
		{
			int status = do_ip_grasp();
			if (IP_NEXT == status)
				_interact_perceive_state = IP_DONE;
			else if (IP_ERROR == status)
				stop_interact_perceive();

			break;
		}
		case IP_DONE:
		{
			stop_interact_perceive();
			gLogger->print_ip_status("Interact perceive done!");
			break;
		}
	}
}

//------------------------------------------------------------------------------------------------------------

int InteractPerceive::do_ip_start()
{
	gLogger->clear_ip_error();
	gLogger->clear_ip_info();
	gLogger->clear_ip_status();


	char defaults;
	std::stringstream logstr;

	gotoxy(1, 46);
	std::cout << "Please enter \"D\" for defaults, or anything else for custom speeds: ";
	std::cin >> defaults;

	if (std::cin.fail())
	{
		logstr << "Invalid input! Please enter a valid number.";
		gLogger->print_ip_error(logstr.str());

		std::cin.clear();
		std::cin.ignore(10000, '\n');  // Ignore the rest of the input line
		return IP_ERROR;
	}
	// Setup default values
	else if ('D' == defaults)
	{
		_w_dy = 6; // Oscillation speed
		_v_dx = 10; // Forward movement speed
		_threshold[0] = _threshold[1] = _threshold[2] = -0.36; // Threshold for forward or wiggle
		_oscillation_count = 4; // Count of controller cycles over which "wiggle" motion is created
	}
	// Grab values from user
	else
	{
		gLogger->clear_ip_status();
		gotoxy(1, 46);
		std::cout << "Please enter the desired wiggling speed (from -10 to 10): ";

		float tmp = 0;
		std::cin >> tmp;

		if (std::cin.fail())
		{
			logstr << "Invalid input! Please enter a valid number.";
			gLogger->print_ip_error(logstr.str());

			std::cin.clear();
			std::cin.ignore(10000, '\n');  // Ignore the rest of the input line
			return IP_ERROR;
		}
		else if (tmp < -10.0f || tmp > 10.0f)
		{
			logstr << "Given speed <" << tmp << "> is out of bounds!";
			gLogger->print_ip_error(logstr.str());
			return IP_ERROR;
		}
		else
		{
			_w_dy = tmp; // 8 is ideal with oscillation value of 4
		}

		gLogger->clear_ip_status();
		gotoxy(1, 46);
		std::cout << "Please enter the desired approach speed (from 0 to 15): ";
		std::cin >> tmp;

		if (std::cin.fail())
		{
			logstr << "Invalid input! Please enter a valid number.";
			gLogger->print_ip_error(logstr.str());

			std::cin.clear();
			std::cin.ignore(10000, '\n');  // Ignore the rest of the input line
			return IP_ERROR;
		}
		else if (tmp < 0 || tmp > 15.0f)
		{
			std::stringstream logstr;
			logstr << "Given speed <" << tmp << "> is out of bounds!";
			gLogger->print_ip_error(logstr.str());
			return IP_ERROR;
		}
		else
		{
			_v_dx = tmp; // doesnt really matter, using 4
		}

		gLogger->clear_ip_status();
		gotoxy(1, 46);
		std::cout << "Please enter the desired force threshold! (from -5 to 0): ";
		std::cin >> tmp;

		if (std::cin.fail())
		{
			logstr << "Invalid input! Please enter a valid number.";
			gLogger->print_ip_error(logstr.str());

			std::cin.clear();
			std::cin.ignore(10000, '\n');  // Ignore the rest of the input line
			return IP_ERROR;
		}
		else if (tmp < -3.0f || tmp > 3.0f)
		{
			std::stringstream logstr;
			logstr << "Given threshold <" << tmp << "> is out of bounds!";
			gLogger->print_ip_error(logstr.str());
			return IP_ERROR;
		}
		else
		{
			_threshold[0] = _threshold[1] = _threshold[2] = tmp; // -0.36 works with offsetting the starting FT
		}

		gLogger->clear_ip_status();
		gotoxy(1, 46);
		std::cout << "Please enter the desired oscilaltions per sec! (from 0 to 10): ";
		std::cin >> tmp;

		if (std::cin.fail())
		{
			logstr << "Invalid input! Please enter a valid number.";
			gLogger->print_ip_error(logstr.str());

			std::cin.clear();
			std::cin.ignore(10000, '\n');  // Ignore the rest of the input line
			return IP_ERROR;
		}
		else if (tmp < 0 || tmp > 10.0f)
		{
			std::stringstream logstr;
			logstr << "Given oscillations <" << tmp << "> is out of bounds!";
			gLogger->print_ip_error(logstr.str());
			return IP_ERROR;
		}
		else
		{
			_oscillation_count = static_cast<int>(tmp + 0.5); // 4 is ideal with 8 v_dy
		}
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
	_ip_osc_loops = 0;
	
	return IP_NEXT;
}

//------------------------------------------------------------------------------------------------------------

int InteractPerceive::do_ip_grasp()
{
	_elapsed_grasp_time = (static_cast<float>(TimeCheck()) - static_cast<float>(_grasp_start_time)) / 1000; // in seconds, time spend in START_GRASP state
	std::array<double, FT_SIZE> avg_ft = { 0, 0, 0, 0, 0, 0 };
	_current_FT = FTMgr.get_FT_ee();
	
	// How many samples to consider at a time for interact/perceive
	// this is the "sliding window" over which interact perceive behaves
	int oscillation_periods = 5;
	int num_ft_samples = oscillation_periods * _oscillation_count;
	
	speed_mode = 4; // set robot to allow max angular speed moves
	new_status = true; // we need the program to send new commands to the robot

	_prev_FT.emplace_back(_current_FT);
	size_t prev_ft_sz = _prev_FT.size();
	if (prev_ft_sz > num_ft_samples)
	{
		_prev_FT.pop_front();
		prev_ft_sz--;
	}

	// Calculate average FT over N oscillation periods
	for (auto it = _prev_FT.begin(); it != _prev_FT.end(); ++it)
	{
		for (int i = 0; i < FT_SIZE; i++)
			avg_ft[i] += (*it)[i];
	}
	for (int i = 0; i < FT_SIZE; i++)
	{
		avg_ft[i] /= prev_ft_sz;

		// Also offset the current FT with the initial FT obtained at the start of interact perceive
		// Need to do this because the sensor is not consistent - going to same place can range from 
		// +/- 0.7N (or more) even after re-running the calibration cloud
		_current_FT[i] -= _starting_FT[i];
	}

	// Start each control feedback from 0 speed
	for (int i = 0; i < 8; i++)
		speed[i] = 0;

	// Keep steady force as needed , more negative = more force
	// Only do force, no wiggling until we touch the object
	if (_current_FT[X] > _threshold[X])
	{
		float forward_speed = _v_dx * (1.0f - (static_cast<float>(_current_FT[X] / _threshold[X])));
		if (forward_speed > _v_dx)
			forward_speed = _v_dx;
		else if (forward_speed > 0 && forward_speed < 1)
			forward_speed = 1;

		// Go forward in end effector frame
		if (CONTROL_OKAY != apply_arm_speed(Z, forward_speed))
		{
			gLogger->print_ip_error("ERROR: Could not apply forward Z speed");
			return IP_ERROR;
		}

		// Print to log so we can track movement
		std::stringstream logstr;
		logstr.setf(std::ios::fixed);
		logstr.precision(4);
		logstr << "Lost touch, +Z speed: " << forward_speed 
			   << ", f_z/t_z: "			 << std::setfill(' ') << std::setw(8) << _current_FT[X] 
			   << "/"					 << std::setfill(' ') << std::setw(8) << _threshold[X] 
			   << ", z_i: "				 << std::setfill(' ') << std::setw(8) << _starting_FT[X];
		gLogger->print_ip_status(logstr.str());

		if (_touched_once)
		{
			// Get distance traveled since last touch. If it's > length of fingers, assume grasped OR that we're past object
			double distance = sqrt(pow((pos[0] - _last_touch_pos[X]), 2) + pow((pos[1] - _last_touch_pos[Y]), 2) + pow((pos[2] - _last_touch_pos[Z]), 2));
			if (distance > 75)
			{
				gLogger->print_ip_status("Traveled > 60mm! Interact perceive is done.");
				return IP_NEXT;
			}
		}
	}
	// If we have enough contact force, wiggle in place
	else
	{
		_touched_once = true; // Set this flag so we know we've touched something at least once
		std::stringstream logstr;
		logstr.setf(std::ios::fixed);
		logstr.precision(4);

		// Oscillate back and forth (yaw)
		// Positive v_dy for first half, negative v_dy for second half
		int wdy_sign = (_ip_cntr < _oscillation_count / 2) ? 1 : -1;

		// N robot ticks in each direction
		if (++_ip_cntr >= _oscillation_count)
		{
			_ip_cntr = 0;
			_ip_osc_loops++;
		}

		float new_speed = _w_dy;
		float speed_offset = 4.0f;
		// Get an evenly spaced number of samples so that we correctly average out sine waves
		//if (0 == _ip_cntr && _ip_osc_loops > 0)
		if (prev_ft_sz >= num_ft_samples)
		{
			if (0 == avg_ft[TX]) // change this to be some tolerance around 0
			{
				return IP_NEXT;
			}
			// if torque is NEGATIVE, rotate +YAW by adding more speed to -vdy ONLY
			else if (avg_ft[TX] > 0 && -1 == wdy_sign)
			{
					new_speed += speed_offset;
			}
			// if torque is NEGATIVE, rotate -YAW by adding more speed to vdy ONLY
			else if (avg_ft[TX] < 0 && 1 == wdy_sign) 
			{
					new_speed += speed_offset;
			}
		}

		// Set oscillation direction
		if (CONTROL_OKAY != apply_arm_speed(YAW, new_speed * wdy_sign))
		{
			gLogger->print_ip_error("ERROR: Could not apply sideways YAW speed");
			return IP_ERROR;
		}

		// Print to log so we can track movement
		logstr << wdy_sign << " Y: Wy: " << std::setfill(' ') << std::setw(7) << speed[YAW]
			<< ", Wp: " << std::setfill(' ') << std::setw(7) << speed[PITCH]
			<< ", Wr: " << std::setfill(' ') << std::setw(7) << speed[ROLL]
			<< ", cnt: " << _ip_cntr
			<< ", ft_x: " << std::setfill(' ') << std::setw(7) << _current_FT[X]
			<< ", t_x: " << std::setfill(' ') << std::setw(7) << _threshold[X];

		gLogger->print_ip_status(logstr.str());

		_last_touch_pos[X] = pos[X];
		_last_touch_pos[Y] = pos[Y];
		_last_touch_pos[Z] = pos[Z];
		_last_touch_pos[YAW] = pos[YAW]; //yaw
		_last_touch_pos[PITCH] = pos[PITCH]; //pitch
		_last_touch_pos[ROLL] = pos[ROLL]; //roll
	}
	FTMgr.write_to_plot_file(_current_FT);

	return IP_OKAY;
}

//------------------------------------------------------------------------------------------------------------