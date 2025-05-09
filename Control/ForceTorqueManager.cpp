﻿// Includes
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
#include <vector>
#include <iostream>
#include <sstream>
#include <deque>

#include "Global.h"

#include "Functions.h"
#include "ForceTorqueManager.h"
#include "KDTree.h"


// Defines & Macros
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

//------------------------------------------------------------------------------------------------------------

ForceTorqueManager::ForceTorqueManager()
{
	std::fill(std::begin(_raw_FT), std::end(_raw_FT), 0);
	std::fill(std::begin(_compensated_FT), std::end(_compensated_FT), 0);
	std::fill(std::begin(_FT_ee), std::end(_FT_ee), 0);
	std::fill(std::begin(_F_ee), std::end(_F_ee), 0);
	std::fill(std::begin(_T_ee), std::end(_T_ee), 0);
	_Rw2FT_s = 0, 0, 0,
			   0, 0, 0,
			   0, 0, 0;

	_calibration_status = STOPPED;
	_calibration_pt_file = "C:\\Users\\yroberts\\Desktop\\Nick Leocadio - 1-15-2024\\manus_ft_calibration_cloud.csv";
	_cal_tree.initialize(_calibration_pt_file);
	_plot_file = "C:\\Users\\yroberts\\Desktop\\Nick Leocadio - 1-15-2024\\plotting\\data\\data.csv";

	//Mg_w = 0, 0, 0.578 * -9.807;				// 0.578kg measured with ati sensor head + hand + camera & attachments/wires
	_Mg_w[0] = 0;
	_Mg_w[1] = 0;
	_Mg_w[2] = 0.541 * -9.807; // 0.541kg measured without metal sensor head
	//_Mg_w[2] = 0.571 * -9.807;
	_R[0] = -0.0006;
	_R[1] = 0.0024;
	_R[2] = 0.0778;

	_tension_const = 0;

	_wrist_offset = 14.3;

	_Rh2FT_s = -0.0065, -0.9991, 0.0421,
			   -0.080, -0.0414, -0.9959,
			    0.9968, -0.0098, -0.0796;

	// With nothing attached
	//_F_offset[0] = 5.387;
	//_F_offset[1] = 3.948;
	//_F_offset[2] = 3.731;
	//_T_offset[0] = 0.340;
	//_T_offset[1] = -0.279;
	//_T_offset[2] = 0.216;

	// With hand hanging (not screwed)
	//_F_offset[0] = 5.379;
	//_F_offset[1] = 3.941;
	//_F_offset[2] = 3.781;
	//_T_offset[0] = 0.339;
	//_T_offset[1] = -0.278;
	//_T_offset[2] = 0.216;

	// No hand, hand screws screwed in
	_F_offset[0] = 3.729;
	_F_offset[1] = 4.619;
	_F_offset[2] = 5.491;
	_T_offset[0] = 0.334;
	_T_offset[1] = -0.282;
	_T_offset[2] = 0.217;

	// Hand attached and screwed, no middle pin connector
	//_F_offset[0] = 3.682;
	//_F_offset[1] = 8.183;
	//_F_offset[2] = 6.142;
	//_T_offset[0] = -0.174;
	//_T_offset[1] = 0.085;
	//_T_offset[2] = 0.219;

	// Hand fully attached
	//_F_offset[0] = 2.871;
	//_F_offset[1] = 8.001;
	//_F_offset[2] = -5.091;
	//_T_offset[0] = -0.166;
	//_T_offset[1] = 0.030;
	//_T_offset[2] = 0.207;

	// Different pose
	// Hand attached and screwed, no middle pin connector, hand flat with arm (pitch ~ 0) (weird pose thats why numbers are off)
	//_F_offset[0] = -0.525;
	//_F_offset[1] = 6.115;
	//_F_offset[2] = 7.748;
	//_T_offset[0] = -0.080;
	//_T_offset[1] = -0.225;
	//_T_offset[2] = 0.204;

	// Hand fully attached, hand flat out with arm (pitch ~ 0)
	//_F_offset[0] = -0.141;
	//_F_offset[1] = 6.691;
	//_F_offset[2] = -4.578;
	//_T_offset[0] = -0.087;
	//_T_offset[1] = -0.244;
	//_T_offset[2] = 0.203;


	//// -----------------------------------------------------------------------------------------------------------------
	//// Start with offset from nothing attached, only screws in place pushing on sensor
	//// No hand, hand screws screwed in
	//_F_offset[X] = 3.729;
	//_F_offset[Y] = 4.619;
	//_F_offset[Z] = 5.491;
	//_T_offset[X] = 0.334;
	//_T_offset[Y] = -0.282;
	//_T_offset[Z] = 0.217;

	//double ft_offset_pin[6];
	//// Next, adjust for tension provided by rod & middle pin: diff hand with pin vs hand no pin
	//// Hand attached and screwed, no middle pin connector and flat out (pitch ~ 0)
	//ft_offset_pin[X] = -0.525;
	//ft_offset_pin[Y] = 6.115;
	//ft_offset_pin[Z] = 7.748;
	//ft_offset_pin[X + 3] = -0.080;
	//ft_offset_pin[Y + 3] = -0.225;
	//ft_offset_pin[Z + 3] = 0.204;

	//// Hand fully attached (pitch ~ 0)
	//ft_offset_pin[X] -= -0.141;
	//ft_offset_pin[Y] -= 6.691;
	//ft_offset_pin[Z] -= -4.578;
	//ft_offset_pin[X + 3] -= -0.087;
	//ft_offset_pin[Y + 3] -= -0.244;
	//ft_offset_pin[Z + 3] -= 0.203;
	//
	//// Finally, combine the two
	//_F_offset[X] -= ft_offset_pin[X];
	//_F_offset[Y] -= ft_offset_pin[Y];
	//_F_offset[Z] -= ft_offset_pin[Z];
	//_T_offset[X] -= ft_offset_pin[X + 3];
	//_T_offset[Y] -= ft_offset_pin[Y + 3];
	//_T_offset[Z] -= ft_offset_pin[Z + 3];
	
	//// -----------------------------------------------------------------------------------------------------------------
	// More offsets for left facing position :/
	//_F_offset[X] = 4.165;
	//_F_offset[Y] = 2.67;
	//_F_offset[Z] = -4.342;
	//_T_offset[X] = -0.213;
	//_T_offset[Y] = 0.042;
	//_T_offset[Z] = 0.046;

}

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::update_FT(std::array<double, FT_SIZE> new_FT)
{
	_raw_FT = new_FT;
}

//------------------------------------------------------------------------------------------------------------

std::array<double, FT_SIZE> ForceTorqueManager::get_raw_FT()
{
	return _raw_FT;
}

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::zero_offsets()
{
	_F_offset[X] = _F_offset[X] + _compensated_FT[X];
	_F_offset[Y] = _F_offset[Y] + _compensated_FT[Y];
	_F_offset[Z] = _F_offset[Z] + _compensated_FT[Z];
	_T_offset[X] = _T_offset[X] + _compensated_FT[X+3];
	_T_offset[Y] = _T_offset[Y] + _compensated_FT[Y+3];
	_T_offset[Z] = _T_offset[Z] + _compensated_FT[Z+3];
}

//------------------------------------------------------------------------------------------------------------

// Note: this wrist angle is incorrect.
// This should be the angle between the link before the wrist & the wrist,
// where 0* = hand pointed straight out, in line with the arm.
// Currently, this is using the pitch which is an angle in the world frame and not what we need.
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

	if (STOPPED != _calibration_status)
		update_calibration();

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

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::compensate_hand_FT()
{
	//Matrix<3, 3> Rh2FT_s;		// Rotation end effector (hand) to FT sensor
	//Matrix<3, 3> Rw2FT_s;		// Rotation world to FT sensor
	Matrix<3, 3> Rw2e;			// Rotation world to end effector (hand)
	//ColumnVector<3> Mg_w;		// Weight of hand due to gravity = mass * g = Newtons
	//ColumnVector<3> F_grav;		// Force in the FT sensor frame due to weight of hand & attachments
	//ColumnVector<3> F_tension;	// Force in the FT sensor frame due to weight of hand & attachments
	//ColumnVector<3> F_bias;		// Force in the FT sensor frame due to weight of hand & attachments
	//ColumnVector<3> F_offset;		// Force in the FT sensor frame due to weight of hand & attachments
	//ColumnVector<3> T_grav;		// Torque in the FT sensor frame due to weight of hand & attachments
	//ColumnVector<3> T_bias;		// Torque in the FT sensor frame due to weight of hand & attachments
	//ColumnVector<3> T_offset;		// Torque in the FT sensor frame due to weight of hand & attachments
	//ColumnVector<3> r_vect;		// Center of mass of hand weight in FT sensor frame
	ColumnVector<3> F_temp;		// Placeholder to convert forces compesated for offset & weight to vector
	ColumnVector<3> T_temp;		// Placeholder to convert torques compesated for offset & weight to vector
	ColumnVector<3> F_ee_temp;	// Placeholder to convert forces compesated for offset & weight back to array
	ColumnVector<3> T_ee_temp;	// Placeholder to convert torques compesated for offset & weight back to array

	Rw2e = transpose(EE2w_transform3(pos));
	//Rh2FT_s =	-0.0065, -0.9991, 0.0421, 
	//			-0.080, -0.0414, -0.9959, 
	//			0.9968, -0.0098, -0.0796; // Mushtaq rotation

	std::array<double, FT_SIZE> ft_offsets = _cal_tree.get_ft_offset(pos[3], pos[4], pos[5]);
	for (int i = 0; i < FT_SIZE; i++)
	{
		_compensated_FT[i] = _raw_FT[i] - ft_offsets[i];
	}
	
	// R (world) to Forcetouch sensor = R_hand to Forcetouch_sensor * R_world to end (effector)
	_Rw2FT_s = _Rh2FT_s * Rw2e;
	
	/*
	
	// This should be re-implemented/ re-used when attempting to properly compensate the weight of the hand,
	// tension of MANUS wrist, and any other parts needed to complete the model and completely zero out forces on the hand
	// without relying on a point cloud.

	//Mg_w = 0, 0, _Mg_w[Z];
	//Mg_w = -0.7792, 1.0664, -3.6563;
	Mg_w = _Mg_w[X], _Mg_w[Y], _Mg_w[Z];
	r_vect = _R[X], _R[Y], _R[Z];
	
	F_offset = _F_offset[X], _F_offset[Y], _F_offset[Z];
	T_offset = _T_offset[X], _T_offset[Y], _T_offset[Z];

	F_tension = 0, 0, _tension_const * pow(abs(wrist_angle + _wrist_offset), 2);
	F_grav = _Rw2FT_s * Mg_w;
	F_bias = _Rw2FT_s * Mg_w + F_offset + F_tension;

	T_grav = crossProduct(r_vect, F_grav);
	T_bias = T_grav + T_offset;

	gotoxy(1, 49);
	printf(" F grav:  Fx: %7.3f, Fy: %7.3f, Fz: %7.3f ", F_grav(1), F_grav(2), F_grav(3));
	gotoxy(1, 50);
	printf(" T grav:  Tx: %7.3f, Ty: %7.3f, Tz: %7.3f ", T_grav(1), T_grav(2), T_grav(3));
	gotoxy(1, 51);
	printf(" Rw2FT_s: %7.3f, %7.3f, %7.3f,", _Rw2FT_s(1, 1), _Rw2FT_s(1, 2), _Rw2FT_s(1, 3));
	gotoxy(1, 52);
	printf("          %7.3f, %7.3f, %7.3f,", _Rw2FT_s(2, 1), _Rw2FT_s(2, 2), _Rw2FT_s(2, 3));
	gotoxy(1, 53);
	printf("          %7.3f, %7.3f, %7.3f,", _Rw2FT_s(3, 1), _Rw2FT_s(3, 2), _Rw2FT_s(3, 3));

	for (int i = 0; i < 3; i++)
	{
		_compensated_FT[i] = _raw_FT[i] - F_bias(i + 1);
		_compensated_FT[i + 3] = _raw_FT[i + 3] - T_bias(i + 1);
	}

	*/

	F_temp = _compensated_FT[X], _compensated_FT[Y], _compensated_FT[Z];
	T_temp = _compensated_FT[X + 3], _compensated_FT[Y + 3], _compensated_FT[Z + 3];
	F_ee_temp = transpose(_Rh2FT_s) * F_temp;
	T_ee_temp = transpose(_Rh2FT_s) * T_temp;

	for (int i = 0; i < 3; i++)
	{
		_F_ee[i] = F_ee_temp(i + 1);
		_T_ee[i] = T_ee_temp(i + 1);
		_FT_ee[i] = _F_ee[i];
		_FT_ee[i + 3] = _T_ee[i];
	}
}

//------------------------------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------------------------------
// 
// R is the r vector of the center of mass of the hand/gripper
void ForceTorqueManager::estimate_r(const std::array<double, FT_SIZE> new_ft, std::array<double, 3> &R)
{
	// Solving system of linear equations from Masoud's dissertation
	// t_x = r_y*f_z − r_z*f_y
	// t_y = -r_x*f_z + r_z*f_x
	// t_z = r_x*f_y − r_y*f_z
	
	// R[0] = x, R[1] = y, R[2] = z
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

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::build_calibration_cloud()
{
	if (STOPPED == _calibration_status)
	{
		clear_cal_file();
		_calibration_status = STARTING;
	}
}

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::cancel_calibration()
{
	_calibration_status = STOPPED;
}

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::clear_cal_file()
{
	std::ofstream calibration_file;
	calibration_file.open(_calibration_pt_file, std::ios_base::trunc);
	calibration_file << "X,Y,Z,YAW,PITCH,ROLL,F_X,F_Y,F_Z,T_X,T_Y,T_Z,RX1,RX2,RX3,RY1,RY2,RY3,RZ1,RZ2,RZ3" << std::endl;
	calibration_file.close();
}

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::write_to_cal_file()
{
	std::ofstream calibration_file;
	/*calibration_file.open("C:\\Users\\yroberts\\Desktop\\Nick Leocadio - 1-15-2024\\calibration_cloud.csv", std::ios_base::app);*/
	calibration_file.open(_calibration_pt_file, std::ios_base::app);
	calibration_file << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << "," << pos[4] << "," << pos[5] << ",";
	calibration_file << _raw_FT[0] << "," << _raw_FT[1] << "," << _raw_FT[2] << "," << _raw_FT[3] << "," << _raw_FT[4] << "," << _raw_FT[5] << ",";
	calibration_file << _Rw2FT_s(1, 1) << "," << _Rw2FT_s(1, 2) << "," << _Rw2FT_s(1, 3) << ",";
	calibration_file << _Rw2FT_s(2, 1) << "," << _Rw2FT_s(2, 2) << "," << _Rw2FT_s(2, 3) << ",";
	calibration_file << _Rw2FT_s(3, 1) << "," << _Rw2FT_s(3, 2) << "," << _Rw2FT_s(3, 3) << std::endl;
	calibration_file.close();
}

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::clear_plot_file()
{
	std::ofstream calibration_file;
	calibration_file.open(_plot_file, std::ios_base::trunc);
	calibration_file << "X,Y,Z,YAW,PITCH,ROLL,RAW F_X,RAW F_Y,RAW F_Z,RAW T_X,RAW T_Y,RAW T_Z,F_X,F_Y,F_Z,T_X,T_Y,T_Z,RX1,RX2,RX3,RY1,RY2,RY3,RZ1,RZ2,RZ3" << std::endl;
	calibration_file.close();
}

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::write_to_plot_file(std::array<double, 6>& interact_perceive_FT)
{
	std::ofstream calibration_file;
	calibration_file.open(_plot_file, std::ios_base::app);
	calibration_file << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << "," << pos[4] << "," << pos[5] << ",";
	calibration_file << _raw_FT[0] << "," << _raw_FT[1] << "," << _raw_FT[2] << "," << _raw_FT[3] << "," << _raw_FT[4] << "," << _raw_FT[5] << ",";
	calibration_file << interact_perceive_FT[0] << "," << interact_perceive_FT[1] << "," << interact_perceive_FT[2] << "," << interact_perceive_FT[3] << "," << interact_perceive_FT[4] << "," << interact_perceive_FT[5] << ",";
	calibration_file << _Rw2FT_s(1, 1) << "," << _Rw2FT_s(1, 2) << "," << _Rw2FT_s(1, 3) << ",";
	calibration_file << _Rw2FT_s(2, 1) << "," << _Rw2FT_s(2, 2) << "," << _Rw2FT_s(2, 3) << ",";
	calibration_file << _Rw2FT_s(3, 1) << "," << _Rw2FT_s(3, 2) << "," << _Rw2FT_s(3, 3) << std::endl;
	calibration_file.close();
}

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::update_calibration()
{
	static float home_x = -450.0f;
	static float home_y = 100.0f;
	static float home_z = -5.0f;
	static float home_yaw = 180.0f;
	static float home_pitch = 5.0f;
	static float home_roll = 180.0f;
	float home_pos[6] = { home_x, home_y, home_z, home_yaw, home_pitch, home_roll };

	static float next_roll = 0;
	static float next_pitch = 0;
	static float next_yaw = 0;

	static float min_yaw = 0;
	static float max_yaw = 0;

	static float min_pitch = 0;
	static float max_pitch = 0;

	static float min_roll = 0;
	static float max_roll = 0;

	static int i = 0;
	static int j = 0;
	static int k = 0;

	static bool stop_movement = false;
	static bool do_roll = false;
	static int stop_cntr = 0;
	int num_pts = 15;
	float range = 30.0f;
	int total_pts = num_pts * num_pts;
	if (do_roll) total_pts = total_pts * num_pts;

	switch (_calibration_status)
	{
		case(STARTING):
		{
			gLogger->print_ip_status("Calibration: Going to first home position ...");
			go_to_position(home_pos);

			_calibration_status = FIRST_HOME;
			break;
		} // STARTING
		case(FIRST_HOME):
		{
			if(!new_position_flag)
			{
				min_yaw = home_yaw - range;
				max_yaw = home_yaw + range;
				next_yaw = min_yaw;

				min_pitch = home_pitch - range;
				max_pitch = home_pitch + range;
				next_pitch = min_pitch;

				min_roll = home_roll - range;
				max_roll = home_roll + range;
				next_roll = do_roll ? min_roll : home_roll;

				i = 1;
				j = 1;
				k = 1;

				gLogger->print_ip_status("Calibration: Starting cloud ...");
				_calibration_status = START_CLOUD;
				stop_movement = false;
				stop_cntr = 0;
				float new_pos[6] = { home_x, home_y, home_z, next_yaw, next_pitch, next_roll };
				go_to_position(new_pos);
			}

			break;
		} // FIRST_HOME
		case(START_CLOUD):
		{
			if (!new_position_flag && !home_pos_flag)
			{
				// Make the arm briefly stop to take readings
				if (stop_movement)
				{
					if (stop_cntr++ >= 10)
						stop_movement = false;

					for (int i = 0; i < 8; i++)
						speed[i] = 0;
					new_status = true;
					break;
				}
				stop_movement = true;
				stop_cntr = 0;
				
				write_to_cal_file();

				// If i (yaw) is max, increase pitch or roll
				if (i > num_pts)
				{
					if (j < num_pts)
					{
						i = 1;
						j++;

						next_pitch = min_pitch + (static_cast<float>(j) * range * 2 / num_pts);

						if (next_pitch > 180)
							next_pitch -= 360;
						else if (next_pitch <= -180)
							next_pitch += 360;
					}
					else if (k < num_pts && do_roll)
					{
						i = 1;
						j = 1;
						k++;

						next_roll = min_roll + (static_cast<float>(k) * range * 2 / num_pts);

						if (next_roll > 180)
							next_roll -= 360;
						else if (next_roll <= -180)
							next_roll += 360;
					}
					else // If max number of points reached, get multiple values for the home position
					{
						go_to_position(home_pos);
						i = 1;
						_calibration_status = GET_MULTIPLE_HOMES;
						break;
					}
				}
				// Continue if more points needed
				std::stringstream logstr;
				logstr << "Calibration: Going to position # " << i << ", " << j << ", " << k << ". ";
				logstr << "Progress: " << (i - 1) + (j - 1) * num_pts << "/" << total_pts << std::endl;
				gLogger->print_ip_status(logstr.str());

				if (j % 2)
				{
					next_yaw = min_yaw + (static_cast<float>(i) * range * 2 / num_pts);
				}
				else
				{
					next_yaw = min_yaw + (static_cast<float>(num_pts - i) * range * 2 / num_pts);
				}
					
				if (next_yaw > 180)
					next_yaw -= 360;
				else if (next_yaw <= -180)
					next_yaw += 360;

				i++;
				float new_pos[6] = { home_x, home_y, home_z, next_yaw, next_pitch, next_roll };
				go_to_position(new_pos);
			}

			break;
		} // START_CLOUD
		case (GET_MULTIPLE_HOMES):
		{
			if (!new_position_flag && !home_pos_flag)
			{
				if (i <= 15)
				{
					// Make the arm briefly stop to take readings
					if (stop_movement)
					{
						if (stop_cntr++ >= 10)
							stop_movement = false;

						for (int i = 0; i < 8; i++)
							speed[i] = 0;
						new_status = true;
						break;
					}
					stop_movement = true;
					stop_cntr = 0;

					write_to_cal_file();

					std::stringstream logstr;
					logstr << "Calibration: Getting multiple home calibration points. ";
					logstr << "Progress: " << i << "/" << "10" << std::endl;
					gLogger->print_ip_status(logstr.str());

					i++;
					float new_pos[6];
					new_pos[0] = home_x;
					new_pos[1] = home_y;
					new_pos[2] = home_z;

					int res = i % 8;
					switch (res)
					{
						case 1:
						{
							new_pos[3] = home_yaw + 1.0f;
							new_pos[4] = home_pitch + 1.0f;
							new_pos[5] = home_roll;
							break;
						}
						case 3:
						{
							new_pos[3] = home_yaw + 1.0f;
							new_pos[4] = home_pitch - 1.0f;
							new_pos[5] = home_roll;
							break;
						}
						case 5:
						{
							new_pos[3] = home_yaw - 1.0f;
							new_pos[4] = home_pitch - 1.0f;
							new_pos[5] = home_roll;
							break;
						}
						case 7:
						{
							new_pos[3] = home_yaw - 1.0f;
							new_pos[4] = home_pitch + 1.0f;
							new_pos[5] = home_roll;
							break;
						}
						default: // Even numbers
						{
							new_pos[3] = home_yaw;
							new_pos[4] = home_pitch;
							new_pos[5] = home_roll;
							break;
						}
					}
					go_to_position(new_pos);
				}
				else
				{
					_calibration_status = BUILD_KDTREE;
					go_to_position(home_pos);
				}
			}
			break;

		} // GET_MULTIPLE_HOMES
		case (BUILD_KDTREE):
		{
			gLogger->print_ip_status("Calibration build KD tree...");
			_cal_tree.initialize(_calibration_pt_file);
			_calibration_status = STOPPED;
			gLogger->print_ip_status("Calibration complete!");
			break;
		} // BUILD_KDTREE
	} // end switch
}

//------------------------------------------------------------------------------------------------------------

void ForceTorqueManager::get_ypr_offsets(double yaw, double pitch, double roll, std::array<double, 3>& offsets)
{
	offsets = _cal_tree.get_ypr_offsets(yaw, pitch, roll, _raw_FT);
}

//------------------------------------------------------------------------------------------------------------