#include "ForceTorqueManager.h"
#include "Functions.h"
#include "Global.h"
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
#include <vector>
#include <iostream>
#include <deque>

#define PRINT_F_RAW 1, 32
#define PRINT_T_RAW PRINT_F_RAW+1
#define PRINT_F_CORR PRINT_T_RAW+1
#define PRINT_T_CORR PRINT_F_CORR+1
#define PRINT_F_EE PRINT_T_CORR+1
#define PRINT_T_EE PRINT_F_EE+1
#define PRINT_R PRINT_T_EE+1

ForceTorqueManager::ForceTorqueManager()
{
	std::fill(std::begin(_raw_FT), std::end(_raw_FT), 0);
}

void ForceTorqueManager::updateFT(std::array<double, FT_SIZE> new_FT)
{
	_raw_FT = new_FT;
}

std::array<double, FT_SIZE> ForceTorqueManager::get_raw_FT()
{
	return _raw_FT;
}

void ForceTorqueManager::ReadForceTorque()
{
	char read_pos_str[256];
	std::array<double, 3> r_vec = { 0 };

	for (auto i=0; i < FT_SIZE; i++)
	{
		FT_sensor[i]->SetReadPos(0);
		FT_sensor[i].Lock();
		FT_sensor[i]->Read((unsigned char*)read_pos_str, 10 * sizeof(BYTE), 0);
		FT_sensor[i].Unlock();
		_raw_FT[i] = static_cast<double>(atof(static_cast<const char*>(read_pos_str)));
	}

	estimate_r(_raw_FT, r_vec);

	std::array<double, FT_SIZE> compensated_FT = { 0 };
	compensate_hand_FT(compensated_FT, r_vec);

	gotoxy(PRINT_F_RAW);
	printf(" Force_raw:    Fx: %07.3f, Fy: %07.3f, Fz: %07.3f ", _raw_FT[0], _raw_FT[1], _raw_FT[2]);
	gotoxy(PRINT_T_RAW);
	printf(" Torque_raw:   Tx: %07.3f, Ty: %07.3f, Tz: %07.3f ", _raw_FT[3], _raw_FT[4], _raw_FT[5]);
	gotoxy(PRINT_F_CORR);
	printf(" Force_corrected:  Fx: %07.3f, Fy: %07.3f, Fz: %07.3f ", compensated_FT[0], compensated_FT[1], compensated_FT[2]);
	gotoxy(PRINT_T_CORR);
	printf(" Torque_corrected: Tx: %07.3f, Ty: %07.3f, Tz: %07.3f ", compensated_FT[3], compensated_FT[4], compensated_FT[5]);
	gotoxy(PRINT_F_EE);
	printf(" Force_ee:        Fx: %07.3f, Fy: %07.3f, Fz: %07.3f ", F_ee[0], F_ee[1], F_ee[2]);
	gotoxy(PRINT_T_EE);
	printf(" Torque_ee:       Tx: %07.3f, Ty: %07.3f, Tz: %07.3f ", T_ee[0], T_ee[1], T_ee[2]);
}

void ForceTorqueManager::compensate_hand_FT(std::array<double, FT_SIZE> &compensated_FT, std::array<double, 3> &r_vec)
{
	Matrix<3, 3> Rh2FT_s, Rw2FT_s;
	ColumnVector<3> Mg_w, F_offset, T_offset, F_unb, T_unb, r_vect, F_bias, F_temp, T_temp, F_ee_temp, T_ee_temp;
	Matrix<3, 3> Rw2e;
	Rw2e = transpose(EE2w_transform3(pos));
	//Rh2FT_s = -0.002, -0.998, 0.0617, 0.224, -0.060, -0.972, 0.975, 0.013, 0.224; // from calibration
	//Rh2FT_s = -0.05556, -0.9949, 0.0838, 0.2130, -0.0702, -0.9745, 0.9755, 0.0720, 0.2080; // rotatio
	Rh2FT_s = -0.0065, -0.9991, 0.0421, -0.080, -0.0414, -0.9959, 0.9968, -0.0098, -0.0796; // rotatio
	//Mg_w = 0, 0, -0.623; //  mass of wrist in N  // mushtaq, Feb 2022
	Mg_w = 0, 0, -5.832;
	//F_offset = -21.999, -13.203, 23.397;
	//T_offset = -0.3016, 0.8030, 0.2234;
	//r_vect = 0.0008, -0.0019, 0.0823;
	//F_offset = -14.5532,-13.775,18.0673;
	//T_offset = -0.2717, 0.3703, 0.2079;
	//r_vect = -0.0008, 0.0005, -0.0030;
	/*F_offset = (-15.586 - 0.1228), (-14.022 - 0.1060), (14.212+4.9509);
	T_offset = (-0.3636 + 0.057331), (0.5146 - 0.04665), (0.1977 - 0.0063);*/
	F_offset = (-15.586 - 0.22602), (-14.022 - 0.1735), (14.212 + 4.528);
	T_offset = (-0.3636 + 0.081052), (0.5146 - 0.02319), (0.1977 - 0.00585);
	//r_vect = 0.0021, -0.0038, 0.0781; // Nick 2024
	r_vect = r_vec[0], r_vec[1], r_vec[2]; // Nick 2024

	//crossProduct

	Rw2FT_s = Rh2FT_s * Rw2e;

	F_unb = Rw2FT_s * Mg_w + F_offset;
	F_bias = Rw2FT_s * Mg_w;
	T_unb = crossProduct(r_vect, F_bias) + T_offset;

	for (auto i = 0; i < 3; i++)
	{
		compensated_FT[i] = _raw_FT[i] - F_unb(i + 1);
		compensated_FT[i + 3] = _raw_FT[i + 3] - T_unb(i + 1);
	}

	F_temp = compensated_FT[0], compensated_FT[1], compensated_FT[2];
	T_temp = compensated_FT[3], compensated_FT[4], compensated_FT[5];
	F_ee_temp = transpose(Rh2FT_s) * F_temp;
	T_ee_temp = transpose(Rh2FT_s) * T_temp;

	for (auto i = 0; i < 3; i++)
	{
		F_ee[i] = F_ee_temp(i + 1);
		T_ee[i] = T_ee_temp(i + 1);
	}
}

void ForceTorqueManager::estimate_r(std::array<double, FT_SIZE> new_ft, std::array<double, 3> &r_vec)
{
	double fx = new_ft[0];
	double fy = new_ft[1];
	double fz = new_ft[2];
	double tx = new_ft[3];
	double ty = new_ft[4];
	double tz = new_ft[5];

	double A[3][3] = {
		{ -fz, fy, -1 },  // r_x
		{ -fx, -fz, -1 }, // r_y
		{ fy, fx, -1 }    // r_z
	};

	double T[3] = { tx, ty, tz };

	double detA = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
				- A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
				+ A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

	if (detA == 0) 
	{
		gotoxy(PRINT_R);
		std::cout << "\t\tError: Determinant is zero" << std::endl;
		return;
	}

	// Det for R using Cramer's rule
	double detRx =	T[0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
					- A[0][1] * (T[1] * A[2][2] - T[2] * A[1][2])
					+ A[0][2] * (T[1] * A[2][1] - T[2] * A[1][1]);

	double detRy =	 A[0][0] * (T[1] * A[2][2] - T[2] * A[1][2])
					- T[0] * (A[1][0] * A[2][2] - A[2][0] * A[1][2])
					+ A[0][2] * (A[1][0] * T[2] - A[2][0] * T[1]);

	double detRz =	A[0][0] * (A[1][1] * T[2] - A[1][2] * T[1])
					- A[0][1] * (A[1][0] * T[2] - A[2][0] * T[1])
					+ T[0] * (A[1][0] * A[2][1] - A[2][0] * A[1][1]);

	r_vec[0] = detRx / detA;
	r_vec[1] = detRy / detA;
	r_vec[2] = detRz / detA;

	gotoxy(PRINT_R);
	std::cout << "\t\tr_x: " << r_vec[0] << ", r_y: " << r_vec[1] << ", r_z: " << r_vec[2] << std::endl;
}

// pseudocode
/*
get T matrix from kinematics of MANUS

get R matrix from top left 3,3 of Tmat

get position of arm from transformation matrix

apply assumed forces onto mass of gripper (gravity vector, -9.81*mass Z)


*/

//......Mushtaq, Feb 2022 :reading from ati-ia F/T  sensor
void ForceTorqueManager::interac_perc() 
{
	int i;
	Matrix<3, 3> Rw2e;
	Rw2e = transpose(EE2w_transform3(pos));

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
	Matrix<3, 3> Re2w;
	Re2w = EE2w_transform3(pos);

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
		set_move_flag_x(0);

	}

	for (i = 1; i < 4; i++)

	{
		speed[i] = static_cast<float>(Vxyz_w(i));

	}
	new_status = true;

	gotoxy(1, 55);
	printf("Vy_ee", Vy_ee);

	counter += 1;
}