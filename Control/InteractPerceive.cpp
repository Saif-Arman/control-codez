#include "Global.h"
#include "InteractPerceive.h"
#include "ForceTorqueManager.h"
#include "Functions.h"

#define X 0
#define Y 1
#define Z 2

InteractPerceive::InteractPerceive()
	: _interact_perceive_state(false)
{
	std::fill(std::begin(_starting_FT), std::end(_starting_FT), 0);
}

bool InteractPerceive::set_interact_perceive_state(bool state)
{
	_interact_perceive_state = state;
	if (state)
	{
		gotoxy(1, 44);
		std::cout << "\r                                         \r";
		_starting_FT = FTMgr.get_compensated_FT();
	}
	else
	{
		stopArm();
		gotoxy(1, 45);
		std::cout << "move flag: " << get_interact_perceive_state();
	}

	return state;
}


void InteractPerceive::check_force()
{
	if (false == IntPerc.get_interact_perceive_state())
		return;

	double threshold = 0.4;
	std::array<double, FT_SIZE> compensated_FT = FTMgr.get_compensated_FT();

	for (auto i = X; i <= Y; i++)
	{
		if (abs(compensated_FT[i] - _starting_FT[i]) > threshold)
		{
			std::string direction;
			if (X == i)
				direction = "X";
			else if (Y == i)
				direction = "Y";
			else if (Z == i)
				direction = "Z";

			gotoxy(1, 44);
			std::cout << "STOPPED DUE TO FORCE IN " << direction << " direction.";
			set_interact_perceive_state(false);
		}
	}
}


//......Mushtaq, Feb 2022 :reading from ati-ia F/T  sensor
void InteractPerceive::do_interact_perceive()
{
	if (false == _interact_perceive_state)
		return;

	//do_approach(); //add next time

	// to be chaneged later : //
	float fdz = 1;
	//float fdx = 1;
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

	if (switch_contact == 0 && abs(abs(F_ee[0]) - fdx) > Thr_x)
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
		set_interact_perceive_state(false);
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