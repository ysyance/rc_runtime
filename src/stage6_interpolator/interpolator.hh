#ifndef _INTERPOLATOR_H_
#define _INTERPOLATOR_H_

#include <vector>
#include "../data_stage4/data_type.hh"

struct interpolation_point
{
	public:
		double distance;
		double velocity;
		double acc;
		double jerk;
	public:
		interpolation_point(double pos, double vel, double acci, double jer): distance(pos), velocity(vel), acc(acci), jerk(jer) {}
		interpolation_point(double pos, double vel, double acci ): distance(pos), velocity(vel), acc(acc), jerk(0.0) {}
		interpolation_point(): distance(0.0), velocity(0.0), acc(0.0),jerk(0.0) {}
		~interpolation_point() {}

};

struct joint_velocity
{
	public:
		double v_a1;
		double v_a2;
		double v_a3;
		double v_a4;
		double v_a5;
		double v_a6;
		double v_aux1;
		double v_aux2;
		double v_aux3;
		double v_aux4;
		double v_aux5;
		double v_aux6;

	public:
		joint_velocity():v_a1(0.0),v_a2(0.0),v_a3(0.0),v_a4(0.0),v_a5(0.0),v_a6(0.0),v_aux1(0.0),v_aux2(0.0),v_aux3(0.0),v_aux4(0.0),v_aux5(0.0),v_aux6(0.0) {}
                joint_velocity(double a1, double a2, double a3, double a4, double a5, double a6):v_a1(a1), v_a2(a2), v_a3(a3), v_a4(a4), v_a5(a5), v_a6(a6), v_aux1(0.0), v_aux2(0.0), v_aux3(0.0), v_aux4(0.0), v_aux5(0.0), v_aux6(0.0) {} 
		~joint_velocity() {}
};

struct joint_acc
{
	public:
		double a_a1;
		double a_a2;
		double a_a3;
		double a_a4;
		double a_a5;
		double a_a6;
		double a_aux1;
		double a_aux2;
		double a_aux3;
		double a_aux4;
		double a_aux5;
		double a_aux6;


	public:
		joint_acc():a_a1(0.0),a_a2(0.0),a_a3(0.0),a_a4(0.0),a_a5(0.0),a_a6(0.0),a_aux1(0.0),a_aux2(0.0),a_aux3(0.0),a_aux4(0.0),a_aux5(0.0),a_aux6(0.0) {}
                joint_acc(double a1, double a2, double a3, double a4, double a5, double a6):a_a1(a1), a_a2(a2), a_a3(a3), a_a4(a4), a_a5(a5), a_a6(a6), a_aux1(0.0), a_aux2(0.0), a_aux3(0.0), a_aux4(0.0), a_aux5(0.0), a_aux6(0.0) {} 
		~joint_acc() {}
};

struct joint_jerk
{
	public:
		double j_a1;
		double j_a2;
		double j_a3;
		double j_a4;
		double j_a5;
		double j_a6;
		double j_aux1;
		double j_aux2;
		double j_aux3;
		double j_aux4;
		double j_aux5;
		double j_aux6;

	public:
		joint_jerk():j_a1(0.0),j_a2(0.0),j_a3(0.0),j_a4(0.0),j_a5(0.0),j_a6(0.0),j_aux1(0.0),j_aux2(0.0),j_aux3(0.0),j_aux4(0.0),j_aux5(0.0),j_aux6(0.0) {}
                joint_jerk(double a1, double a2, double a3, double a4, double a5, double a6):j_a1(a1), j_a2(a2), j_a3(a3), j_a4(a4), j_a5(a5), j_a6(a6), j_aux1(0.0), j_aux2(0.0), j_aux3(0.0), j_aux4(0.0), j_aux5(0.0), j_aux6(0.0) {} 
		~joint_jerk() {}
};

int calculate_center_radius(robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_aux, robot_data_file_process::cartpos p_end, 
		double &center_x, double &center_y, double &center_z, 
		double &radius,
		double &alpha, double &beta, double &gama, double &theta_distance);


int PTP_TMode_interpolator(robot_data_file_process::axispos &p_start, robot_data_file_process::axispos &p_end,
		joint_velocity &v_start, joint_velocity &v_end, joint_velocity &v_target,
		joint_acc &acc, joint_acc &dec,
		double cycle,
		std::vector<robot_data_file_process::axispos> &points_set, std::vector<joint_velocity> &points_velocity, std::vector<joint_acc> &points_acc );

int Lin_TMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_end,
		double v_start, double v_end, double v_target,
		double acc, double dec,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set);
int Circ_TMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_aux, robot_data_file_process::cartpos p_end,
		double vori_start, double vori_end, double vori_target,
		double accori, double decori,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set);



int PTP_SMode_interpolator(robot_data_file_process::axispos &p_start, robot_data_file_process::axispos &p_end,
		joint_velocity &v_start, joint_velocity &v_end, joint_velocity &v_target,
		joint_acc &acc, joint_acc &dec,
		joint_jerk &jerk,
		double cycle,
		std::vector<robot_data_file_process::axispos> &points_set, std::vector<joint_velocity> &points_velocity, std::vector<joint_acc> &points_acc, std::vector<joint_jerk> &points_jerk );
int Lin_SMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_end,
		double v_start, double v_end, double v_target,
		double acc, double dec,
                double jerk,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set);
int Circ_SMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_aux, robot_data_file_process::cartpos p_end,
		double vori_start, double vori_end, double vori_target,
		double accori, double decori,
		double jerkori,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set);



int PTP_TrigMode_interpolator(robot_data_file_process::axispos &p_start, robot_data_file_process::axispos &p_end,
		joint_velocity &v_start, joint_velocity &v_end, joint_velocity &v_target,
		joint_acc &acc, joint_acc &dec,
		double cycle,
		std::vector<robot_data_file_process::axispos> &points_set, std::vector<joint_velocity> &points_velocity, std::vector<joint_acc> &points_acc );
int Lin_TrigMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_end,
		double v_start, double v_end, double v_target,
		double acc, double dec,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set);
int Circ_TrigMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_aux, robot_data_file_process::cartpos p_end,
		double vori_start, double vori_end, double vori_target,
		double accori, double decori,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set);



#endif 
