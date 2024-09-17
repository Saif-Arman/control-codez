#pragma once
#include <array>

#define FT_SIZE 6

class ForceTorqueManager
{
public:
	ForceTorqueManager();

	void ReadForceTorque();

	void update_FT(std::array<double, FT_SIZE> new_FT);
		
	inline std::array<double, FT_SIZE> get_compensated_FT() { return _compensated_FT; };
	inline std::array<double, FT_SIZE> get_FT_ee() { return _FT_ee; };

	inline std::array<double, FT_SIZE / 2> get_F_ee() { return _F_ee; };
	inline std::array<double, FT_SIZE / 2> get_T_ee() { return _T_ee; };

	inline std::array<double, FT_SIZE / 2> get_f_offsets() { return _F_offset; };
	inline void set_f_offset(unsigned int axis, double offset) { axis < 3 ? _F_offset[axis] = offset : printf("ERROR: Invalid axis selected!"); };

	inline std::array<double, FT_SIZE / 2> get_t_offsets() { return _T_offset; };
	inline void set_t_offset(unsigned int axis, double offset) { axis < 3 ? _T_offset[axis] = offset : printf("ERROR: Invalid axis selected!"); };

	inline double get_weight() { return _Mg_w[2]; };
	inline void set_weight(double new_weight) { _Mg_w[2] = new_weight; };

	inline std::array<double, FT_SIZE / 2> get_r() { return _R; };
	inline void set_r(unsigned int axis, double offset) { axis < 3 ? _R[axis] = offset : printf("ERROR: Invalid axis selected!"); };

	void zero_offsets();

	inline double get_tension_angle() { return _tension_angle; };
	inline void set_tension_angle(double angle) { _tension_angle = angle; };

	inline double get_tension_const() { return _tension_const; };
	inline void set_tension_const(double newval) { _tension_const = newval; };

private:

	std::array<double, FT_SIZE> get_raw_FT();

	void compensate_hand_FT();
	void compensate_hand_FT_orig();

	void estimate_r(const std::array<double, FT_SIZE> new_ft, std::array<double, 3>& R);

	std::array<double, FT_SIZE> _raw_FT;
	std::array<double, FT_SIZE> _compensated_FT;
	std::array<double, FT_SIZE> _FT_ee;
	std::array<double, FT_SIZE / 2> _F_ee;
	std::array<double, FT_SIZE / 2> _T_ee;
	std::array<double, FT_SIZE / 2> _F_offset;
	std::array<double, FT_SIZE / 2> _T_offset;
	std::array<double, FT_SIZE / 2> _Mg_w;
	std::array<double, FT_SIZE / 2> _R;

	double _tension_angle;
	double _tension_const;

};