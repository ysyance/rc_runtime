#include <vector>
#include <cstdio>
#include <iostream>
#include "interpolator.hh"
#include "../data_stage4/data_type.hh"

#define CART_TEST 1
#define JOINT_TEST 0

int main()
{

#if CART_TEST

	robot_data_file_process::cartpos p_start(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	robot_data_file_process::cartpos p_aux(0.0, 80.0, 10.0, 10.0, 30.0, 25.0);
	robot_data_file_process::cartpos p_end(105, 85, 20, 30, 60, 50);
	double v_start = 0.007;
	double v_target = 0.12;
	double v_end = 0.01;
	double acc = 0.02;
	double dec = 0.02;
	//double jerk = 0.01;
	double cycle = 0.1;
	std::vector<interpolation_point> points_set;
	std::vector<robot_data_file_process::cartpos> disp_set;


	int ret = Circ_TrigMode_interpolator(p_start,
										 p_aux,
										 p_end,
										 v_start,
										 v_end,
										 v_target,
										 acc,
										 dec,
										 cycle,
										 points_set,
										 disp_set);
	if(ret != 0)
	{
		std::cout << "error" << std::endl;
	}

	FILE *fp, *fp1;
	fp=fopen("./testdata/acc.txt","w");
	for(std::vector<interpolation_point>::iterator iter = points_set.begin(); iter != points_set.end(); ++iter)
	{
		fprintf(fp, "%lf %lf %lf %lf\n",iter->distance, iter->velocity, iter->acc, iter->jerk );
	}
	fclose(fp);

	fp1=fopen("./testdata/pos.txt","w");
	for(std::vector<robot_data_file_process::cartpos>::iterator iter = disp_set.begin(); iter != disp_set.end(); ++iter)
	{
		fprintf(fp1, "%lf %lf %lf\n",iter->x, iter->y, iter->z);
	}
	fclose(fp1);
#endif /* CART_TEST */







#if JOINT_TEST
	robot_data_file_process::axispos p_start(121.0, -21.0, -95.0, -66.0, -52.0, 98.0);
	robot_data_file_process::axispos p_end(110.0, -73.0, -10.0, -45.0, -62.0, 58.0);
	joint_velocity v_start(0,0,0,0,0,0), v_target(400,250,350,550,650,750), v_end(0,0,0,0,0,0);
	joint_acc acc(2300,2100,2500,2900,3000,4000), dec(2300,2100,2500,2900,3000,4000);
	joint_jerk jerk(1000000, 1000000, 1000000, 1000000, 1000000, 1000000);
	double cycle = 0.001;

	std::vector<robot_data_file_process::axispos> points_set;
	std::vector<joint_velocity> points_velocity;
	std::vector<joint_acc> points_acc;
	std::vector<joint_jerk> points_jerk;

	int ret = PTP_SMode_interpolator(p_start, p_end, v_start, v_end, v_target, acc, dec, jerk, cycle, points_set, points_velocity, points_acc, points_jerk);
	if(ret != 0)
	{
		std::cout << "error" << std::endl;
	}

	FILE *fp_pos, *fp_vel, *fp_acc, *fp_jerk;
	fp_pos=fopen("./testdata/pos_ptp.txt","w");
	for(std::vector<robot_data_file_process::axispos>::iterator iter = points_set.begin(); iter != points_set.end(); ++iter)
	{
		fprintf(fp_pos, "%lf %lf %lf %lf %lf %lf\n",iter->a1, iter->a2, iter->a3, iter->a4, iter->a5, iter->a6);
	}
	fclose(fp_pos);

	fp_vel=fopen("./testdata/vel_ptp.txt","w");
	for(std::vector<joint_velocity>::iterator iter = points_velocity.begin(); iter != points_velocity.end(); ++iter)
	{
		fprintf(fp_vel, "%lf %lf %lf %lf %lf %lf\n",iter->v_a1, iter->v_a2, iter->v_a3, iter->v_a4, iter->v_a5, iter->v_a6);
	}
	fclose(fp_vel);

	fp_acc=fopen("./testdata/acc_ptp.txt","w");
	for(std::vector<joint_acc>::iterator iter = points_acc.begin(); iter != points_acc.end(); ++iter)
	{
		fprintf(fp_acc, "%lf %lf %lf %lf %lf %lf\n",iter->a_a1, iter->a_a2, iter->a_a3, iter->a_a4, iter->a_a5, iter->a_a6);
	}
	fclose(fp_acc);

	fp_jerk=fopen("./testdata/jerk_ptp.txt","w");
	for(std::vector<joint_jerk>::iterator iter = points_jerk.begin(); iter != points_jerk.end(); ++iter)
	{
		fprintf(fp_jerk, "%lf %lf %lf %lf %lf %lf\n",iter->j_a1, iter->j_a2, iter->j_a3, iter->j_a4, iter->j_a5, iter->j_a6);
	}
	fclose(fp_jerk);

#endif /* JOINT_TEST */

	return 0;
}
