#pragma once
// Includes
#include <array>
#include "Matrix.hpp"
#include "ControlLogger.h"
#include "KDTree.h"

// Defines & Macros
#define FT_SIZE 6

// Class definition
class ForceTorqueManager
{
public:
	// Constructor
	ForceTorqueManager();

	// Read FT values from sensor memory
	void ReadForceTorque();

	// Update stored FT value
	void update_FT(std::array<double, FT_SIZE> new_FT);
		
	// Returns compensated Force Torque matrix, compensated based on calibration cloud
	inline std::array<double, FT_SIZE> get_compensated_FT() { return _compensated_FT; };

	inline std::array<double, FT_SIZE> get_FT_ee() { return _FT_ee; };

	inline std::array<double, FT_SIZE / 2> get_F_ee() { return _F_ee; };
	inline std::array<double, FT_SIZE / 2> get_T_ee() { return _T_ee; };

	inline std::array<double, FT_SIZE / 2> get_f_offsets() { return _F_offset; };
	inline void set_f_offset(unsigned int axis, double offset) { axis < 3 ? _F_offset[axis] = offset : printf("ERROR: Invalid axis selected!"); };

	inline std::array<double, FT_SIZE / 2> get_t_offsets() { return _T_offset; };
	inline void set_t_offset(unsigned int axis, double offset) { axis < 3 ? _T_offset[axis] = offset : printf("ERROR: Invalid axis selected!"); };

	//inline double get_weight() { return _Mg_w[2]; };
	inline std::array<double, FT_SIZE / 2> get_weight() { return _Mg_w; };
	//inline void set_weight(double new_weight) { _Mg_w[2] = new_weight; };
	inline void set_weight(unsigned int axis, double new_weight) { axis < 3 ? _Mg_w[axis] = new_weight : printf("ERROR: Invalid axis selected!"); };

	inline std::array<double, FT_SIZE / 2> get_r() { return _R; };
	inline void set_r(unsigned int axis, double offset) { axis < 3 ? _R[axis] = offset : printf("ERROR: Invalid axis selected!"); };

	inline Matrix<3, 3> get_Rw2FT_s() { return _Rw2FT_s; };

	void zero_offsets();

	inline double get_tension_const() { return _tension_const; };
	inline void set_tension_const(double newval) { _tension_const = newval; };

	inline double get_angle_offset() { return _wrist_offset; };
	inline void set_angle_offset(double newval) { _wrist_offset = newval; };

	void build_calibration_cloud();
	void write_to_cal_file();
	void clear_cal_file();
	void cancel_calibration();

	void write_to_plot_file(std::array<double, 6>& interact_perceive_FT);
	void clear_plot_file();

	void get_ypr_offsets(double yaw, double pitch, double roll, std::array<double, 3>& offsets);

private:

	// Returns raw readings from Force Touch sensor
	std::array<double, FT_SIZE> get_raw_FT();

	// This should be evolved to be compensated for weight of hand, tension of robot, and any additional required parts
	void compensate_hand_FT();

	// Unused, original function based on gravity only
	void compensate_hand_FT_orig();

	// Estimates r (center of mass) vector of the hand based on compensated FT readings
	// Does not work correctly as robot has components impacting FT sensor
	// in addition to just gravity's effect on the hand
	void estimate_r(const std::array<double, FT_SIZE> new_ft, std::array<double, 3>& R);

	// Runs the calibration cloud routine
	// Robot will go through a set number of points, and then use the 4 nearest points (ROLL, PITCH, YAW)
	// to determine the sensor offsets to use for the current position.
	// 4 nearest points are given weights based on distance to the current location in ROLL, PITCH, YAW
	void update_calibration();

	std::array<double, FT_SIZE> _raw_FT; // Raw Force & Torque measured by FT sensor
	std::array<double, FT_SIZE> _compensated_FT; // Force & Torque in FT sensor frame, compensated with offsets or calibration tree
	std::array<double, FT_SIZE> _FT_ee; // Force & Torque in end effector frame, compensated with offsets or calibration tree
	std::array<double, FT_SIZE / 2> _F_ee; // Force in end effector frame, compensated with offsets or calibration tree
	std::array<double, FT_SIZE / 2> _T_ee; // Torque in end effector frame, compensated with offsets or calibration tree
	std::array<double, FT_SIZE / 2> _F_offset; // Force offset from attaching sensor (screws, tension, etc)
	std::array<double, FT_SIZE / 2> _T_offset; // Torque offset from attaching sensor (screws, tension, etc)
	std::array<double, FT_SIZE / 2> _Mg_w; // weight of hand in world frame
	std::array<double, FT_SIZE / 2> _R; // center of mass of hand
	std::string _calibration_pt_file; // Used to write/read calibration points to a file so calibration can be re-used
	std::string _plot_file; // Used to write FT readings to a file to be plotted

	KDTree _cal_tree; // Calibration tree used to store calibration points

	Matrix<3, 3> _Rw2FT_s;		// Rotation world to FT sensor
	Matrix<3, 3> _Rh2FT_s;		// Rotation end effector (hand) to FT sensor

	double _tension_const;
	double _wrist_offset;

	enum CAL_STATUS
	{
		STOPPED=0,
		STARTING,
		FIRST_HOME,
		START_CLOUD,
		GET_MULTIPLE_HOMES,
		BUILD_KDTREE
	};

	CAL_STATUS _calibration_status;
};