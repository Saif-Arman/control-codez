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
#define X 0
#define Y 1
#define Z 2

ForceTorqueManager::ForceTorqueManager()
{
	std::fill(std::begin(_raw_FT), std::end(_raw_FT), 0);
	std::fill(std::begin(_compensated_FT), std::end(_compensated_FT), 0);
	std::fill(std::begin(_FT_ee), std::end(_FT_ee), 0);
	std::fill(std::begin(_F_ee), std::end(_F_ee), 0);
	std::fill(std::begin(_T_ee), std::end(_T_ee), 0);
	//Mg_w = 0, 0, 0.578 * -9.807;				// 0.578kg measured with ati sensor head + hand + camera & attachments/wires
	_Mg_w[0] = 0;
	_Mg_w[1] = 0;
	//_Mg_w[2] = 0.541 * -9.807;
	_Mg_w[2] = 0.541 * 9.807; // For some reason, this needs to be positive??? need to investigate.
	_R[0] = 0.0021;
	_R[1] = -0.0026;
	_R[2] = 0.0781;

	_F_offset[0] = 5.709;  // Force offset, after recalibration & pointed left // was 6.329
	_F_offset[1] = 4.93; // was 5.564;
	_F_offset[2] = -3.8; // Was 3.350 prior to attaching hand. Adding hand causes some additional Z force due to attaching mechanism;

	_T_offset[0] = 0.426;   // Torque offset, after recalibration & pointed left
	_T_offset[1] = -0.345;
	_T_offset[2] = 0.218;

	// Mushtaq #s
	//F_offset = (-15.586 - 0.22602), (-14.022 - 0.1735), (14.212 + 4.528);
	//T_offset = (-0.3636 + 0.081052), (0.5146 - 0.02319), (0.1977 - 0.00585);
}

void ForceTorqueManager::update_FT(std::array<double, FT_SIZE> new_FT)
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
	for (int i=0; i < FT_SIZE; i++)
	{
		FT_sensor[i]->SetReadPos(0);
		FT_sensor[i].Lock();
		FT_sensor[i]->Read((unsigned char*)read_pos_str, 10 * sizeof(BYTE), 0);
		FT_sensor[i].Unlock();
		_raw_FT[i] = static_cast<double>(atof(static_cast<const char*>(read_pos_str)));
	}
	
	//std::array<double, 3> R = { 0 };
	//estimate_r(_raw_FT, R);

	compensate_hand_FT();

	gotoxy(PRINT_F_RAW);
	printf(" F_raw:       Fx: %7.3f, Fy: %7.3f, Fz: %7.3f ", _raw_FT[0], _raw_FT[1], _raw_FT[2]);
	gotoxy(PRINT_T_RAW);
	printf(" T_raw:       Tx: %7.3f, Ty: %7.3f, Tz: %7.3f ", _raw_FT[3], _raw_FT[4], _raw_FT[5]);
	gotoxy(PRINT_F_CORR);
	printf(" F - Offset:  Fx: %7.3f, Fy: %7.3f, Fz: %7.3f ", _compensated_FT[0], _compensated_FT[1], _compensated_FT[2]);
	gotoxy(PRINT_T_CORR);
	printf(" T - Offset:  Tx: %7.3f, Ty: %7.3f, Tz: %7.3f ", _compensated_FT[3], _compensated_FT[4], _compensated_FT[5]);
	gotoxy(PRINT_F_EE);
	printf(" F EE Frame:  Fx: %7.3f, Fy: %7.3f, Fz: %7.3f ", _F_ee[0], _F_ee[1], _F_ee[2]);
	gotoxy(PRINT_T_EE);
	printf(" T EE Frame:  Tx: %7.3f, Ty: %7.3f, Tz: %7.3f ", _T_ee[0], _T_ee[1], _T_ee[2]);
}

void ForceTorqueManager::compensate_hand_FT()
{
	Matrix<3, 3> Rh2FT_s;		// Rotation end effector (hand) to FT sensor
	Matrix<3, 3> Rw2FT_s;		// Rotation world to FT sensor
	Matrix<3, 3> Rw2e;			// Rotation world to end effector (hand)
	ColumnVector<3> Mg_w;		// Weight of hand due to gravity = mass * g = Newtons
	ColumnVector<3> F_grav;		// Force in the FT sensor frame due to weight of hand & attachments
	ColumnVector<3> T_grav;		// Torque in the FT sensor frame due to weight of hand & attachments
	ColumnVector<3> r_vect;		// Center of mass of hand weight in FT sensor frame
	ColumnVector<3> F_temp;		// Placeholder to convert forces compesated for offset & weight to vector
	ColumnVector<3> T_temp;		// Placeholder to convert torques compesated for offset & weight to vector
	ColumnVector<3> F_ee_temp;	// Placeholder to convert forces compesated for offset & weight back to array
	ColumnVector<3> T_ee_temp;	// Placeholder to convert torques compesated for offset & weight back to array
	Rw2e = transpose(EE2w_transform3(pos));
	Rh2FT_s =	-0.0065, -0.9991, 0.0421, 
				-0.080, -0.0414, -0.9959, 
				0.9968, -0.0098, -0.0796; // Mushtaq rotation

	/* Offsets and mass below recalibrated by Nick, 8/20/2024 */
	//Mg_w = 0, 0, 0.578 * -9.807;				// 0.578kg measured with ati sensor head + hand + camera & attachments/wires
	Mg_w = 0, 0, _Mg_w[Z];
	//F_offset = -14.334, -16.188, 10.868;		// Force offset, avg when nothing attached to ATI sensor
	//F_offset = -13.338, -11.408, 1.214;		// Force offset, avg when nothing attached to ATI sensor, round 2
	//T_offset = -0.398, 0.392, 0.174;			// Torque offset, avg when nothing attached to ATI sensor

	//r_vect = 0.0021, -0.0038, 0.0781;			// Mushtaq
	r_vect = _R[0], _R[1], _R[2];

	// R (world) to Forcetouch sensor = R_hand to Forcetouch_sensor * R_world to end (effector)
	//Rw2FT_s = Rh2FT_s * Rw2e;
	Rw2FT_s = Rw2e * Rh2FT_s;

	F_grav = Rw2FT_s * Mg_w;
	T_grav = crossProduct(r_vect, F_grav);

	for (int i = 0; i < 3; i++)
	{
		_compensated_FT[i] = _raw_FT[i] - _F_offset[i] - F_grav(i + 1);
		_compensated_FT[i + 3] = _raw_FT[i + 3] - _T_offset[i] - T_grav(i + 1);
	}

	F_temp = _compensated_FT[X], _compensated_FT[Y], _compensated_FT[Z];
	T_temp = _compensated_FT[X + 3], _compensated_FT[Y + 3], _compensated_FT[Z + 3];
	F_ee_temp = transpose(Rh2FT_s) * F_temp;
	T_ee_temp = transpose(Rh2FT_s) * T_temp;

	for (int i = 0; i < 3; i++)
	{
		_F_ee[i] = F_ee_temp(i + 1);
		_T_ee[i] = T_ee_temp(i + 1);
		_FT_ee[i] = _F_ee[i];
		_FT_ee[i + 3] = _T_ee[i];
	}
}

void ForceTorqueManager::compensate_hand_FT_orig()
{
	Matrix<3, 3> Rh2FT_s, Rw2FT_s;
	ColumnVector<3> Mg_w, F_offset, T_offset, F_unb, T_unb, r_vect, F_bias, F_temp, T_temp, F_ee_temp, T_ee_temp;
	Matrix<3, 3> Rw2e;
	Rw2e = transpose(EE2w_transform3(pos));
	//Rh2FT_s = -0.002, -0.998, 0.0617, 0.224, -0.060, -0.972, 0.975, 0.013, 0.224; // from calibration
	//Rh2FT_s = -0.05556, -0.9949, 0.0838, 0.2130, -0.0702, -0.9745, 0.9755, 0.0720, 0.2080; // rotatio
	Rh2FT_s = -0.0065, -0.9991, 0.0421, -0.080, -0.0414, -0.9959, 0.9968, -0.0098, -0.0796; // rotatio
	//Mg_w = 0, 0, -0.623; //  mass of wrist in N  // mushtaq, Feb 2022
	Mg_w = 0, 0, -5.832; // Mushtaq
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
	//r_vect = 0.0021, -0.0038, 0.0781; // Mushtaq
	//r_vect = R[0], R[1], R[2]; // Nick 2024 estimating realtime
	r_vect = 0.678129, 0.088761, -0.161235; // Nick 2024 avg'd value
	//F_offset = 0, 0, 0;
	//T_offset = 0, 0, 0;

	//crossProduct

	// R (world) to Forcetouch sensor = R_hand to Forcetouch_sensor * R_world to end (effector)
	Rw2FT_s = Rh2FT_s * Rw2e;

	F_unb = Rw2FT_s * Mg_w + F_offset;
	F_bias = Rw2FT_s * Mg_w;
	T_unb = crossProduct(r_vect, F_bias) + T_offset;

	for (auto i = 0; i < 3; i++)
	{
		_compensated_FT[i] = _raw_FT[i] - F_unb(i + 1);
		_compensated_FT[i + 3] = _raw_FT[i + 3] - T_unb(i + 1);
	}

	F_temp = _compensated_FT[X], _compensated_FT[Y], _compensated_FT[Z];
	T_temp = _compensated_FT[X+3], _compensated_FT[Y+3], _compensated_FT[Z+3];
	F_ee_temp = transpose(Rh2FT_s) * F_temp;
	T_ee_temp = transpose(Rh2FT_s) * T_temp;

	for (auto i = 0; i < 3; i++)
	{
		_F_ee[i] = F_ee_temp(i + 1);
		_T_ee[i] = T_ee_temp(i + 1);
	}
}

// R is the r vector of the center of mass of the hand/gripper
void ForceTorqueManager::estimate_r(const std::array<double, FT_SIZE> new_ft, std::array<double, 3> &R)
{
	// Solving system of linear equations from Masoud's dissertation
	// t_x = r_y*f_z − r_z*f_y
	// t_y = -r_x*f_z + r_z*f_x
	// t_z = r_x*f_y − r_y*f_z

	// Adjust w/ naked offsets
	//ColumnVector<3> F_offset, T_offset;
	//F_offset = -14.334, -16.188, 10.868; // Force offset, avg when nothing attached to ATI sensor
	//T_offset = -0.398, 0.392, 0.174; // Torque offset, avg when nothing attached to ATI sensor
	
	// R[0] = x, R[1] = y, R[2] = z
	//std::array<double, 3> F = { new_ft[X] - F_offset(X + 1), 
	//							new_ft[Y] - F_offset(Y + 1), 
	//							new_ft[Z] - F_offset(Z + 1) };
	//std::array<double, 3> T = { new_ft[X+3] - T_offset(X + 1), 
	//							new_ft[Y+3] - T_offset(Y + 1), 
	//							new_ft[Z+3] - T_offset(Z + 1) };

	std::array<double, 3> F = { new_ft[X] - _F_offset[X],
								new_ft[Y] - _F_offset[Y],
								new_ft[Z] - _F_offset[Z] };
	std::array<double, 3> T = { new_ft[X + 3] - _T_offset[X],
								new_ft[Y + 3] - _T_offset[Y],
								new_ft[Z + 3] - _T_offset[Z] };

	R[X] = T[Z] / F[Y] + T[Y] / F[Y] + T[Y] / (F[X] - F[Z]) + (F[Z] / F[Y]) * ((T[X] + T[Z]) / (F[X] - F[Z]));

	R[Y] = (T[X] / F[Z]) + (T[Y] * F[Y]) / (F[Z] * (F[X] - F[Z])) + (T[X] + T[Z]) / (F[X] - F[Z]);

	R[Z] = ((T[Y] * F[Y]) + (T[Z] * F[Z]) + (T[X] * F[Z])) / (F[Y] * (F[X] - F[Z]));

	gotoxy(PRINT_R);
	printf("              Rx: %7.3f, Ry: %7.3f, Rz: %7.3f ", R[X], R[Y], R[Z]);
}