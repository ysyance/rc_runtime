
#include <iostream>
#include <cmath>

#include "interpolator.hh"
#include "velocity_curve.hh"

#define rate 57.295779513028
#define rate2 0.017453292519943

int PTP_TMode_interpolator(robot_data_file_process::axispos &p_start, robot_data_file_process::axispos &p_end,
		joint_velocity &v_start, joint_velocity &v_end, joint_velocity &v_target,
		joint_acc &acc, joint_acc &dec,
		double cycle,
		std::vector<robot_data_file_process::axispos> &points_set, std::vector<joint_velocity> &points_velocity, std::vector<joint_acc> &points_acc )
{
	int ret = 0;
	//joint a1
	double distance1 = p_end.a1 - p_start.a1 ;
	double flag1 = 1;
	if(distance1 < 0)
	{
		distance1 = -distance1;
		flag1 = -1;
	}

	TMode_struct ret_value_a1;
	ret = TMode_curve(distance1, v_start.v_a1, v_end.v_a1, v_target.v_a1, acc.a_a1, dec.a_a1, ret_value_a1);
	if(ret < 0)
	{
		std::cout << "joint a1 occurs error" << std::endl;
		return -1;
	}
	//joint a2
	double distance2 = p_end.a2 - p_start.a2 ;
	double flag2 = 1;
	if(distance2 < 0)
	{
		distance2 = -distance2;
		flag2 = -1;
	}
	TMode_struct ret_value_a2;
	ret = TMode_curve(distance2, v_start.v_a2, v_end.v_a2, v_target.v_a2, acc.a_a2, dec.a_a2, ret_value_a2);
	if(ret < 0)
	{
		std::cout << "joint a2 occurs error" << std::endl;
		return -1;
	}
	//joint a3
	double distance3 = p_end.a3 - p_start.a3 ;
	double flag3 = 1;
	if(distance3 < 0)
	{
		distance3 = -distance3;
		flag3 = -1;
	}
	TMode_struct ret_value_a3;
	ret = TMode_curve(distance3, v_start.v_a3, v_end.v_a3, v_target.v_a3, acc.a_a3, dec.a_a3, ret_value_a3);
	if(ret < 0)
	{
		std::cout << "joint a3 occurs error" << std::endl;
		return -1;
	}
	//joint a4
	double distance4 = p_end.a4 - p_start.a4 ;
	double flag4 = 1;
	if(distance4 < 0)
	{
		distance4 = -distance4;
		flag4 = -1;
	}
	TMode_struct ret_value_a4;
	ret = TMode_curve(distance4, v_start.v_a4, v_end.v_a4, v_target.v_a4, acc.a_a4, dec.a_a4, ret_value_a4);
	if(ret < 0)
	{
		std::cout << "joint a4 occurs error" << std::endl;
		return -1;
	}
	//joint a5
	double distance5 = p_end.a5 - p_start.a5 ;
	double flag5 = 1;
	if(distance5 < 0)
	{
		distance5 = -distance5;
		flag5 = -1;
	}
	TMode_struct ret_value_a5;
	ret = TMode_curve(distance5, v_start.v_a5, v_end.v_a5, v_target.v_a5, acc.a_a5, dec.a_a5, ret_value_a5);
	if(ret < 0)
	{
		std::cout << "joint a5 occurs error" << std::endl;
		return -1;
	}
	//joint a6
	double distance6 = p_end.a6 - p_start.a6 ;
	double flag6 = 1;
	if(distance6 < 0)
	{
		distance6 = -distance6;
		flag6 = -1;
	}
	TMode_struct ret_value_a6;
	ret = TMode_curve(distance6, v_start.v_a6, v_end.v_a6, v_target.v_a6, acc.a_a6, dec.a_a6, ret_value_a6);
	if(ret < 0)
	{
		std::cout << "joint a6 occurs error" << std::endl;
		return -1;
	}


	double time_a1 = ret_value_a1.time1 + ret_value_a1.time2 + ret_value_a1.time3;
	double time_a2 = ret_value_a2.time1 + ret_value_a2.time2 + ret_value_a2.time3;
	double time_a3 = ret_value_a3.time1 + ret_value_a3.time2 + ret_value_a3.time3;
	double time_a4 = ret_value_a4.time1 + ret_value_a4.time2 + ret_value_a4.time3;
	double time_a5 = ret_value_a5.time1 + ret_value_a5.time2 + ret_value_a5.time3;
	double time_a6 = ret_value_a6.time1 + ret_value_a6.time2 + ret_value_a6.time3;
	double time_max = 0.0;
	double k1 = 1.0;
	double k2 = 1.0;
	double k3 = 1.0;
	double k4 = 1.0;
	double k5 = 1.0;
	double k6 = 1.0;

	time_max = (time_a1 > time_a2) ? time_a1 : time_a2;
	time_max = (time_a3 > time_max) ? time_a3 : time_max;
	time_max = (time_a3 > time_max) ? time_a4 : time_max;
	time_max = (time_a3 > time_max) ? time_a5 : time_max;
	time_max = (time_a3 > time_max) ? time_a6 : time_max;

	k1 = time_max/time_a1;
	k2 = time_max/time_a2;
	k3 = time_max/time_a3;
	k4 = time_max/time_a4;
	k5 = time_max/time_a5;
	k6 = time_max/time_a6;

	// step2. caculate the interpolation points 

	double v_tmp1_a1 = (2*distance1/k1 - v_start.v_a1*ret_value_a1.time1 - v_end.v_a1*ret_value_a1.time3)/(ret_value_a1.time1+2*ret_value_a1.time2+ret_value_a1.time3);
	ret_value_a1.time1 = ret_value_a1.time1*k1;
	ret_value_a1.time2 = ret_value_a1.time2*k1;
	ret_value_a1.time3 = ret_value_a1.time3*k1;
	double dis_tmp1_a1 = 0.5*(v_start.v_a1 + v_tmp1_a1)*ret_value_a1.time1;
	acc.a_a1 = (v_tmp1_a1 - v_start.v_a1)/(ret_value_a1.time1);
	dec.a_a1 = (v_tmp1_a1 - v_end.v_a1)/(ret_value_a1.time3);
	double dis_tmp2_a1 = dis_tmp1_a1 + v_tmp1_a1*ret_value_a1.time2;



	double v_tmp1_a2 = (2*distance2/k2 - v_start.v_a2*ret_value_a2.time1 - v_end.v_a2*ret_value_a2.time3)/(ret_value_a2.time1+2*ret_value_a2.time2+ret_value_a2.time3);
	ret_value_a2.time1 = ret_value_a2.time1*k2;
	ret_value_a2.time2 = ret_value_a2.time2*k2;
	ret_value_a2.time3 = ret_value_a2.time3*k2;
	double dis_tmp1_a2 = 0.5*(v_start.v_a2 + v_tmp1_a2)*ret_value_a2.time1;
	acc.a_a2 = (v_tmp1_a2 - v_start.v_a2)/(ret_value_a2.time1);
	dec.a_a2 = (v_tmp1_a2 - v_end.v_a2)/(ret_value_a2.time3);
	double dis_tmp2_a2 = dis_tmp1_a2 + v_tmp1_a2*ret_value_a2.time2;

	double v_tmp1_a3 = (2*distance3/k3 - v_start.v_a3*ret_value_a3.time1 - v_end.v_a3*ret_value_a3.time3)/(ret_value_a3.time1+2*ret_value_a3.time2+ret_value_a3.time3);
	ret_value_a3.time1 = ret_value_a3.time1*k3;
	ret_value_a3.time2 = ret_value_a3.time2*k3;
	ret_value_a3.time3 = ret_value_a3.time3*k3;
	double dis_tmp1_a3 = 0.5*(v_start.v_a3 + v_tmp1_a3)*ret_value_a3.time1;
	acc.a_a3 = (v_tmp1_a3 - v_start.v_a3)/(ret_value_a3.time1);
	dec.a_a3 = (v_tmp1_a3 - v_end.v_a3)/(ret_value_a3.time3);
	double dis_tmp2_a3 = dis_tmp1_a3 + v_tmp1_a3*ret_value_a3.time2;

	double v_tmp1_a4 = (2*distance4/k4 - v_start.v_a4*ret_value_a4.time1 - v_end.v_a4*ret_value_a4.time3)/(ret_value_a4.time1+2*ret_value_a4.time2+ret_value_a4.time3);
	ret_value_a4.time1 = ret_value_a4.time1*k4;
	ret_value_a4.time2 = ret_value_a4.time2*k4;
	ret_value_a4.time3 = ret_value_a4.time3*k4;
	double dis_tmp1_a4 = 0.5*(v_start.v_a4 + v_tmp1_a4)*ret_value_a4.time1;
	acc.a_a4 = (v_tmp1_a4 - v_start.v_a4)/(ret_value_a4.time1);
	dec.a_a4 = (v_tmp1_a4 - v_end.v_a4)/(ret_value_a4.time3);
	double dis_tmp2_a4 = dis_tmp1_a4 + v_tmp1_a4*ret_value_a4.time2;

	double v_tmp1_a5 = (2*distance5/k5 - v_start.v_a5*ret_value_a5.time1 - v_end.v_a5*ret_value_a5.time3)/(ret_value_a5.time1+2*ret_value_a5.time2+ret_value_a5.time3);
	ret_value_a5.time1 = ret_value_a5.time1*k5;
	ret_value_a5.time2 = ret_value_a5.time2*k5;
	ret_value_a5.time3 = ret_value_a5.time3*k5;
	double dis_tmp1_a5 = 0.5*(v_start.v_a5 + v_tmp1_a5)*ret_value_a5.time1;
	acc.a_a5 = (v_tmp1_a5 - v_start.v_a5)/(ret_value_a5.time1);
	dec.a_a5 = (v_tmp1_a5 - v_end.v_a5)/(ret_value_a5.time3);
	double dis_tmp2_a5 = dis_tmp1_a5 + v_tmp1_a5*ret_value_a5.time2;

	double v_tmp1_a6 = (2*distance6/k6 - v_start.v_a6*ret_value_a6.time1 - v_end.v_a6*ret_value_a6.time3)/(ret_value_a6.time1+2*ret_value_a6.time2+ret_value_a6.time3);
	ret_value_a6.time1 = ret_value_a6.time1*k6;
	ret_value_a6.time2 = ret_value_a6.time2*k6;
	ret_value_a6.time3 = ret_value_a6.time3*k6;
	double dis_tmp1_a6 = 0.5*(v_start.v_a6 + v_tmp1_a6)*ret_value_a6.time1;
	acc.a_a6 = (v_tmp1_a6 - v_start.v_a6)/(ret_value_a6.time1);
	dec.a_a6 = (v_tmp1_a6 - v_end.v_a6)/(ret_value_a6.time3);
	double dis_tmp2_a6 = dis_tmp1_a6 + v_tmp1_a6*ret_value_a6.time2;


	int num = ceil((time_max)/cycle);
	double t1_a1 = ret_value_a1.time1;
	double t2_a1 = ret_value_a1.time1 + ret_value_a1.time2;
	double t3_a1 = ret_value_a1.time1 + ret_value_a1.time2 +ret_value_a1.time3;

	double t1_a2 = ret_value_a2.time1;
	double t2_a2 = ret_value_a2.time1 + ret_value_a2.time2;
	double t3_a2 = ret_value_a2.time1 + ret_value_a2.time2 +ret_value_a2.time3;

	double t1_a3 = ret_value_a3.time1;
	double t2_a3 = ret_value_a3.time1 + ret_value_a3.time2;
	double t3_a3 = ret_value_a3.time1 + ret_value_a3.time2 +ret_value_a3.time3;

	double t1_a4 = ret_value_a4.time1;
	double t2_a4 = ret_value_a4.time1 + ret_value_a4.time2;
	double t3_a4 = ret_value_a4.time1 + ret_value_a4.time2 +ret_value_a4.time3;

	double t1_a5 = ret_value_a5.time1;
	double t2_a5 = ret_value_a5.time1 + ret_value_a5.time2;
	double t3_a5 = ret_value_a5.time1 + ret_value_a5.time2 +ret_value_a5.time3;

	double t1_a6 = ret_value_a6.time1;
	double t2_a6 = ret_value_a6.time1 + ret_value_a6.time2;
	double t3_a6 = ret_value_a6.time1 + ret_value_a6.time2 +ret_value_a6.time3;
	double t = 0;


	robot_data_file_process::axispos tmp_point; //Attention here!
	joint_velocity tmp_velocity;
	joint_acc tmp_acc;

	for(int i = 0; i < num-1; ++i)
	{
		t = (i+1)*cycle;
		/***********************a1************************************/
		if( t >= 0 && t < t1_a1)
		{
			tmp_acc.a_a1 = acc.a_a1;
			tmp_velocity.v_a1 = v_start.v_a1 + acc.a_a1*t;
			tmp_point.a1 = v_start.v_a1*t + 0.5*acc.a_a1*t*t;
		}
		else if(t >= t1_a1 && t < t2_a1) 
		{
			tmp_acc.a_a1 = 0;
			tmp_velocity.v_a1 = v_tmp1_a1;
			tmp_point.a1 = dis_tmp1_a1 + v_tmp1_a1*(t - t1_a1);
		}
		else if(t >= t2_a1 && t < t3_a1)
		{
			tmp_acc.a_a1 = -dec.a_a1;
			tmp_velocity.v_a1 = v_tmp1_a1 - dec.a_a1*(t-t2_a1);
			tmp_point.a1 = dis_tmp2_a1 + v_tmp1_a1*(t-t2_a1) - 0.5*dec.a_a1*(t-t2_a1)*(t-t2_a1);
		}
		/***********************a2************************************/
		if( t >= 0 && t < t1_a2)
		{
			tmp_acc.a_a2 = acc.a_a2;
			tmp_velocity.v_a2 = v_start.v_a2 + acc.a_a2*t;
			tmp_point.a2 = v_start.v_a2*t + 0.5*acc.a_a2*t*t;
		}
		else if(t >= t1_a2 && t < t2_a2) 
		{
			tmp_acc.a_a2 = 0;
			tmp_velocity.v_a2 = v_tmp1_a2;
			tmp_point.a2 = dis_tmp1_a2 + v_tmp1_a2*(t - t1_a2);
		}
		else if(t >= t2_a2 && t < t3_a2)
		{
			tmp_acc.a_a2 = -dec.a_a2;
			tmp_velocity.v_a2 = v_tmp1_a2 - dec.a_a2*(t-t2_a2);
			tmp_point.a2 = dis_tmp2_a2 + v_tmp1_a2*(t-t2_a2) - 0.5*dec.a_a2*(t-t2_a2)*(t-t2_a2);
		}
		/***********************a3************************************/
		if( t >= 0 && t < t1_a3)
		{
			tmp_acc.a_a3 = acc.a_a3;
			tmp_velocity.v_a3 = v_start.v_a3 + acc.a_a3*t;
			tmp_point.a3 = v_start.v_a3*t + 0.5*acc.a_a3*t*t;
		}
		else if(t >= t1_a3 && t < t2_a3) 
		{
			tmp_acc.a_a3 = 0;
			tmp_velocity.v_a3 = v_tmp1_a3;
			tmp_point.a3 = dis_tmp1_a3 + v_tmp1_a3*(t - t1_a3);
		}
		else if(t >= t2_a3 && t < t3_a3)
		{
			tmp_acc.a_a3 = -dec.a_a3;
			tmp_velocity.v_a3 = v_tmp1_a3 - dec.a_a3*(t-t2_a3);
			tmp_point.a3 = dis_tmp2_a3 + v_tmp1_a3*(t-t2_a3) - 0.5*dec.a_a3*(t-t2_a3)*(t-t2_a3);
		}
		/***********************a4************************************/
		if( t >= 0 && t < t1_a4)
		{
			tmp_acc.a_a4 = acc.a_a4;
			tmp_velocity.v_a4 = v_start.v_a4 + acc.a_a4*t;
			tmp_point.a4 = v_start.v_a4*t + 0.5*acc.a_a4*t*t;
		}
		else if(t >= t1_a4 && t < t2_a4) 
		{
			tmp_acc.a_a4 = 0;
			tmp_velocity.v_a4 = v_tmp1_a4;
			tmp_point.a4 = dis_tmp1_a4 + v_tmp1_a4*(t - t1_a4);
		}
		else if(t >= t2_a4 && t < t3_a4)
		{
			tmp_acc.a_a4 = -dec.a_a4;
			tmp_velocity.v_a4 = v_tmp1_a4 - dec.a_a4*(t-t2_a4);
			tmp_point.a4 = dis_tmp2_a4 + v_tmp1_a4*(t-t2_a4) - 0.5*dec.a_a4*(t-t2_a4)*(t-t2_a4);
		}
		/**********************a5*************************************/
		if( t >= 0 && t < t1_a5)
		{
			tmp_acc.a_a5 = acc.a_a5;
			tmp_velocity.v_a5 = v_start.v_a5 + acc.a_a5*t;
			tmp_point.a5 = v_start.v_a5*t + 0.5*acc.a_a5*t*t;
		}
		else if(t >= t1_a5 && t < t2_a5) 
		{
			tmp_acc.a_a5 = 0;
			tmp_velocity.v_a5 = v_tmp1_a5;
			tmp_point.a5 = dis_tmp1_a5 + v_tmp1_a5*(t - t1_a5);
		}
		else if(t >= t2_a5 && t < t3_a5)
		{
			tmp_acc.a_a5 = -dec.a_a5;
			tmp_velocity.v_a5 = v_tmp1_a5 - dec.a_a5*(t-t2_a5);
			tmp_point.a5 = dis_tmp2_a5 + v_tmp1_a5*(t-t2_a5) - 0.5*dec.a_a5*(t-t2_a5)*(t-t2_a5);
		}
		/**********************a6*************************************/
		if( t >= 0 && t < t1_a6)
		{
			tmp_acc.a_a6 = acc.a_a6;
			tmp_velocity.v_a6 = v_start.v_a6 + acc.a_a6*t;
			tmp_point.a6 = v_start.v_a6*t + 0.5*acc.a_a6*t*t;
		}
		else if(t >= t1_a6 && t < t2_a6) 
		{
			tmp_acc.a_a6 = 0;
			tmp_velocity.v_a6 = v_tmp1_a6;
			tmp_point.a6 = dis_tmp1_a6 + v_tmp1_a6*(t - t1_a6);
		}
		else if(t >= t2_a6 && t < t3_a6)
		{
			tmp_acc.a_a6 = -dec.a_a6;
			tmp_velocity.v_a6 = v_tmp1_a6 - dec.a_a6*(t-t2_a6);
			tmp_point.a6 = dis_tmp2_a6 + v_tmp1_a6*(t-t2_a6) - 0.5*dec.a_a6*(t-t2_a6)*(t-t2_a6);
		}

		tmp_point.a1 = tmp_point.a1*flag1 + p_start.a1;
		tmp_point.a2 = tmp_point.a2*flag2 + p_start.a2;
		tmp_point.a3 = tmp_point.a3*flag3 + p_start.a3;
		tmp_point.a4 = tmp_point.a4*flag4 + p_start.a4;
		tmp_point.a5 = tmp_point.a5*flag5 + p_start.a5;
		tmp_point.a6 = tmp_point.a6*flag6 + p_start.a6;

		points_set.push_back(tmp_point); //Attention here!
		points_velocity.push_back(tmp_velocity);
		points_acc.push_back(tmp_acc);
	}

	//The last point of interpolator
	tmp_point.a1 = p_end.a1;
	tmp_point.a2 = p_end.a2;
	tmp_point.a3 = p_end.a3;
	tmp_point.a4 = p_end.a4;
	tmp_point.a5 = p_end.a5;
	tmp_point.a6 = p_end.a6;

	tmp_velocity.v_a1 = v_end.v_a1;
	tmp_velocity.v_a2 = v_end.v_a2;
	tmp_velocity.v_a3 = v_end.v_a3;
	tmp_velocity.v_a4 = v_end.v_a4;
	tmp_velocity.v_a5 = v_end.v_a5;
	tmp_velocity.v_a6 = v_end.v_a6;

	//tmp_acc.a_a1 = -dec.a_a1;
	//tmp_acc.a_a2 = -dec.a_a2;
	//tmp_acc.a_a3 = -dec.a_a3;
	//tmp_acc.a_a4 = -dec.a_a4;
	//tmp_acc.a_a5 = -dec.a_a5;
	//tmp_acc.a_a6 = -dec.a_a6;

	points_set.push_back(tmp_point); //Attention here!
	points_velocity.push_back(tmp_velocity);
	points_acc.push_back(tmp_acc);

	return 0;

}




int Lin_TMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_end,
		double v_start, double v_end, double v_target,
		double acc, double dec,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set)
{

	// step1. caculate time1, time2, time2, maybe should reset v_start, v_end
	double distance = sqrt((p_start.x - p_end.x)*(p_start.x - p_end.x) + (p_start.y - p_end.y)*(p_start.y - p_end.y) + (p_start.z - p_end.z)*(p_start.z - p_end.z));
	TMode_struct ret_value;
	int ret = TMode_curve(distance, v_start, v_end, v_target, acc, dec, ret_value);
	if(ret < 0)
	{
		return -1;
	}
	double time1 = ret_value.time1;
	double time2 = ret_value.time2;
	double time3 = ret_value.time3;
	v_start = ret_value.v_start;
	v_end = ret_value.v_end;

	double kx = (p_start.x - p_end.x)/distance;
	double ky = (p_start.y - p_end.y)/distance;
	double kz = (p_start.z - p_end.z)/distance;

	// step2. caculate the interpolation points 
	int num = ceil((time1+time2+time3)/cycle);
	double t1 = time1;
	double t2 = time1 + time2;
	double t3 = time1 + time2 +time3;
	double t = 0;



	double v_tmp1 = v_start + acc*time1;
	double dis_tmp1 = v_start*time1 + 0.5*acc*time1*time1;
	double dis_tmp2 = dis_tmp1 + v_tmp1*time2;

	interpolation_point tmp_point; //Attention here!
	robot_data_file_process::cartpos tmp_dis;
	for(int i = 0; i < num-1; ++i)
	{
		t = (i+1)*cycle;
		if( t >= 0 && t < t1)
		{
			tmp_point.acc = acc;
			tmp_point.velocity = v_start + acc*t;
			tmp_point.distance = v_start*t + 0.5*acc*t*t;
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t1 && t < t2) 
		{
			tmp_point.acc = 0;
			tmp_point.velocity = v_tmp1;
			tmp_point.distance = dis_tmp1 + v_tmp1*(t - t1);
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t2 && t < t3)
		{
			tmp_point.acc = -dec;
			tmp_point.velocity = v_tmp1 - dec*(t-t2);
			tmp_point.distance = dis_tmp2 + v_tmp1*(t-t2) - 0.5*dec*(t-t2)*(t-t2);
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}

		points_set.push_back(tmp_point); //Attention here!
		disp_set.push_back(tmp_dis);
	}
	// The last point of interpolator
	//tmp_point.acc = -dec;
	tmp_point.velocity = v_end;
	tmp_point.distance = distance;

	tmp_dis.x = p_end.x;
	tmp_dis.y = p_end.y;
	tmp_dis.z = p_end.z;
	tmp_dis.a = p_end.a;
	tmp_dis.b = p_end.b;
	tmp_dis.c = p_end.c;

	points_set.push_back(tmp_point); //Attention here!
	disp_set.push_back(tmp_dis);



	return 0;

}

int calculate_center_radius(robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_aux, robot_data_file_process::cartpos p_end, 
		double &center_x, double &center_y, double &center_z, 
		double &radius,
		double &alpha, double &beta, double &gama, double &theta_distance)
{
	double a1 = 0.0, a2 = 0.0, a3 = 0.0, b1 = 0.0, b2 = 0.0, b3 = 0.0, c1 = 0.0, c2 = 0.0, c3 = 0.0, d1 = 0.0, d2 = 0.0, d3 = 0.0;
	double x1 = p_start.x, y1 = p_start.y, z1 = p_start.z;
	double x2 = p_aux.x, y2 = p_aux.y, z2 = p_aux.z;
	double x3 = p_end.x, y3 = p_end.y, z3 = p_end.z;

	//step1. calculate circle's center
	a1 = (y1 - y3)*(z2 - z3) - (y2 - y3)*(z1 - z3);
	b1 = (x2 - x3)*(z1 - z3) - (x1 - x3)*(z2 - z3);
	c1 = (x1 - x3)*(y2 - y3) - (x2 - x3)*(y1 - y3);
	d1 = -x3*a1 - y3*b1 - z3*c1;

	double r1 = x1*x1 + y1*y1 +z1*z1;
	double r2 = x2*x2 + y2*y2 +z2*z2;
	double r3 = x3*x3 + y3*y3 +z3*z3;

	a2 = 2*(x2 - x1);
	b2 = 2*(y2 - y1);
	c2 = 2*(z2 - z1);
	d2 = r1 - r2;

	a3 = 2*(x3 - x1);
	b3 = 2*(y3 - y1);
	c3 = 2*(z3 - z1);
	d3 = r1 - r3;

	double det = a1*(b2*c3 - c2*b3) - b1*(a2*c3 - c2*a3) + c1*(a2*b3 - b2*a3);
	if(det < 0.0000001 && det > -0.0000001)
	{
		std::cout << "The value of determinant is zero" << std::endl;
		return -1;
	}

	double vx = d1*(b2*c3 - c2*b3) - b1*(d2*c3 - c2*d3) + c1*(d2*b3 - b2*d3);
	double vy = a1*(d2*c3 - c2*d3) - d1*(a2*c3 - c2*a3) + c1*(a2*d3 - d2*a3);
	double vz = a1*(b2*d3 - d2*b3) - b1*(a2*d3 - d2*a3) + d1*(a2*b3 - b2*a3);

	center_x = -vx/det;
	center_y = -vy/det;
	center_z = -vz/det;

	//step2. calculate circle's radius
	radius = sqrt( (x1 - center_x)*(x1 - center_x) + (y1 - center_y)*(y1 - center_y) + (z1 - center_z)*(z1 - center_z));

	//step3. calculate circle's normal
	double x12 = x2 - x1;
	double y12 = y2 - y1;
	double z12 = z2 - z1;

	double x23 = x3 - x2;
	double y23 = y3 - y2;
	double z23 = z3 - z2;

	double ix = y12*z23 - z12*y23;
	double jy = -x12*z23 + z12*x23;
	double kz = x12*y23 - y12*x23;
	double length = sqrt(ix*ix + jy*jy + kz*kz);
	alpha = ix/length;
	beta = jy/length;
	gama = kz/length;

	//step4. calculate circle's theta_distance
	double theta = 2*asin(0.5*sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)+(z3-z1)*(z3-z1))/radius );

	double x13 = x3 - x1;
	double y13 = y3 - y1;
	double z13 = z3 - z1;

	double x01 = x1 - center_x;
	double y01 = y1 - center_y;
	double z01 = z1 - center_z;

	double ix_n = y13*z01 - z13*y01;
	double jy_n = -x13*z01 + z13*x01;
	double kz_n = x13*y01 - y13*x01;

	double flag_value = ix*ix_n + jy*jy_n + kz*kz_n;
	if(flag_value >= 0)
		theta_distance = theta;
	else 
		theta_distance = 2*PI - theta;

	return 0;
}





int Circ_TMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_aux, robot_data_file_process::cartpos p_end,
		double vori_start, double vori_end, double vori_target,
		double accori, double decori,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set)
{
	double center_x = 0.0, center_y = 0.0, center_z = 0.0, radius = 0.0, alpha = 0.0, beta = 0.0, gama = 0.0, theta_distance;
	int ret = calculate_center_radius(p_start, p_aux, p_end, center_x, center_y, center_z, radius, alpha, beta, gama, theta_distance);
	if(ret < 0)
	{
		std::cout << "Error when calculate the circle's center and radius" << std::endl;
		return -1;
	}

	TMode_struct ret_value;
	ret = TMode_curve(theta_distance, vori_start, vori_end, vori_target, accori, decori, ret_value);
	if(ret < 0)
	{
		return -1;
	}
	double time1 = ret_value.time1;
	double time2 = ret_value.time2;
	double time3 = ret_value.time3;
	vori_start = ret_value.v_start;
	vori_end = ret_value.v_end;

	//test TMode_curve()
	std::cout << "time1: " << time1 << std::endl;
	std::cout << "time2: " << time2 << std::endl;
	std::cout << "time3: " << time3 << std::endl;
	std::cout << "v_start: " << vori_start << std::endl;
	std::cout << "v_end: " << vori_end << std::endl;

	// step2. caculate the interpolation points 
	int num = ceil((time1+time2+time3)/cycle);
	double t1 = time1;
	double t2 = time1 + time2;
	double t3 = time1 + time2 +time3;
	double t = 0;



	double v_tmp1 = vori_start + accori*time1;
	double dis_tmp1 = vori_start*time1 + 0.5*accori*time1*time1;
	double dis_tmp2 = dis_tmp1 + v_tmp1*time2;

	double pso_x = p_start.x - center_x;
	double pso_y = p_start.y - center_y;
	double pso_z = p_start.z - center_z;

	double M11 = 0.0, M12 = 0.0, M13 = 0.0;
	double M21 = 0.0, M22 = 0.0, M23 = 0.0;
	double M31 = 0.0, M32 = 0.0, M33 = 0.0;

	double costheta = 0.0;
	double sintheta = 0.0;


	double k1 = alpha*alpha;
	double k2 = beta*beta;
	double k3 = gama*gama;
	double k12 = alpha*beta;
	double k13 = alpha*gama;
	double k23 = beta*gama;


	interpolation_point tmp_point; //Attention here!
	robot_data_file_process::cartpos tmp_dis;
	for(int i = 0; i < num-1; ++i)
	{
		t = (i+1)*cycle;
		if( t >= 0 && t < t1)
		{
			tmp_point.acc = accori;
			tmp_point.velocity = vori_start + accori*t;
			tmp_point.distance = vori_start*t + 0.5*accori*t*t;

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t1 && t < t2) 
		{
			tmp_point.acc = 0;
			tmp_point.velocity = v_tmp1;
			tmp_point.distance = dis_tmp1 + v_tmp1*(t - t1);

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t2 && t < t3)
		{
			tmp_point.acc = -decori;
			tmp_point.velocity = v_tmp1 - decori*(t-t2);
			tmp_point.distance = dis_tmp2 + v_tmp1*(t-t2) - 0.5*decori*(t-t2)*(t-t2);

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}

		points_set.push_back(tmp_point); //Attention here!
		disp_set.push_back(tmp_dis);
	}
	// The last point of interpolator
	//tmp_point.acc = -decori;
	tmp_point.velocity = vori_end;
	tmp_point.distance = theta_distance;

	tmp_dis.x = p_end.x;
	tmp_dis.y = p_end.y;
	tmp_dis.z = p_end.z;
	tmp_dis.a = p_end.a;
	tmp_dis.b = p_end.b;
	tmp_dis.c = p_end.c;

	points_set.push_back(tmp_point); //Attention here!
	disp_set.push_back(tmp_dis);

	return 0;

}




int PTP_SMode_interpolator(robot_data_file_process::axispos &p_start, robot_data_file_process::axispos &p_end,
		joint_velocity &v_start, joint_velocity &v_end, joint_velocity &v_target,
		joint_acc &acc, joint_acc &dec,
		joint_jerk &jerk,
		double cycle,
		std::vector<robot_data_file_process::axispos> &points_set, std::vector<joint_velocity> &points_velocity, std::vector<joint_acc> &points_acc, std::vector<joint_jerk> &points_jerk )
{
	int ret = 0;
	//joint a1
	double distance1 = p_end.a1 - p_start.a1 ;
	double flag1 = 1;
	if(distance1 < 0)
	{
		distance1 = -distance1;
		flag1 = -1;
	}

	SMode_struct ret_value_a1;
	ret = SMode_curve(distance1, v_start.v_a1, v_end.v_a1, v_target.v_a1, acc.a_a1, dec.a_a1, jerk.j_a1, ret_value_a1);
	if(ret < 0)
	{
		std::cout << "joint a1 occurs error" << std::endl;
		return -1;
	}
	//joint a2
	double distance2 = p_end.a2 - p_start.a2 ;
	double flag2 = 1;
	if(distance2 < 0)
	{
		distance2 = -distance2;
		flag2 = -1;
	}
	SMode_struct ret_value_a2;
	ret = SMode_curve(distance2, v_start.v_a2, v_end.v_a2, v_target.v_a2, acc.a_a2, dec.a_a2, jerk.j_a2, ret_value_a2);
	if(ret < 0)
	{
		std::cout << "joint a2 occurs error" << std::endl;
		return -1;
	}
	//joint a3
	double distance3 = p_end.a3 - p_start.a3 ;
	double flag3 = 1;
	if(distance3 < 0)
	{
		distance3 = -distance3;
		flag3 = -1;
	}
	SMode_struct ret_value_a3;
	ret = SMode_curve(distance3, v_start.v_a3, v_end.v_a3, v_target.v_a3, acc.a_a3, dec.a_a3, jerk.j_a3, ret_value_a3);
	if(ret < 0)
	{
		std::cout << "joint a3 occurs error" << std::endl;
		return -1;
	}
	//joint a4
	double distance4 = p_end.a4 - p_start.a4 ;
	double flag4 = 1;
	if(distance4 < 0)
	{
		distance4 = -distance4;
		flag4 = -1;
	}
	SMode_struct ret_value_a4;
	ret = SMode_curve(distance4, v_start.v_a4, v_end.v_a4, v_target.v_a4, acc.a_a4, dec.a_a4, jerk.j_a4, ret_value_a4);
	if(ret < 0)
	{
		std::cout << "joint a4 occurs error" << std::endl;
		return -1;
	}
	//joint a5
	double distance5 = p_end.a5 - p_start.a5 ;
	double flag5 = 1;
	if(distance5 < 0)
	{
		distance5 = -distance5;
		flag5 = -1;
	}
	SMode_struct ret_value_a5;
	ret = SMode_curve(distance5, v_start.v_a5, v_end.v_a5, v_target.v_a5, acc.a_a5, dec.a_a5, jerk.j_a5, ret_value_a5);
	if(ret < 0)
	{
		std::cout << "joint a5 occurs error" << std::endl;
		return -1;
	}
	//joint a6
	double distance6 = p_end.a6 - p_start.a6 ;
	double flag6 = 1;
	if(distance6 < 0)
	{
		distance6 = -distance6;
		flag6 = -1;
	}
	SMode_struct ret_value_a6;
	ret = SMode_curve(distance6, v_start.v_a6, v_end.v_a6, v_target.v_a6, acc.a_a6, dec.a_a6, jerk.j_a6, ret_value_a6);
	if(ret < 0)
	{
		std::cout << "joint a6 occurs error" << std::endl;
		return -1;
	}


	double time_a1 = ret_value_a1.time1 + ret_value_a1.time2 + ret_value_a1.time3 + ret_value_a1.time4 + ret_value_a1.time5 + ret_value_a1.time6 + ret_value_a1.time7;
	double time_a2 = ret_value_a2.time1 + ret_value_a2.time2 + ret_value_a2.time3+ ret_value_a2.time4 + ret_value_a2.time5 + ret_value_a2.time6 + ret_value_a2.time7;
	double time_a3 = ret_value_a3.time1 + ret_value_a3.time2 + ret_value_a3.time3+ ret_value_a3.time4 + ret_value_a3.time5 + ret_value_a3.time6 + ret_value_a3.time7;
	double time_a4 = ret_value_a4.time1 + ret_value_a4.time2 + ret_value_a4.time3+ ret_value_a4.time4 + ret_value_a4.time5 + ret_value_a4.time6 + ret_value_a4.time7;
	double time_a5 = ret_value_a5.time1 + ret_value_a5.time2 + ret_value_a5.time3+ ret_value_a5.time4 + ret_value_a5.time5 + ret_value_a5.time6 + ret_value_a5.time7;
	double time_a6 = ret_value_a6.time1 + ret_value_a6.time2 + ret_value_a6.time3+ ret_value_a6.time4 + ret_value_a6.time5 + ret_value_a6.time6 + ret_value_a6.time7;
	double time_max = 0.0;

	time_max = (time_a1 > time_a2) ? time_a1 : time_a2;
	time_max = (time_a3 > time_max) ? time_a3 : time_max;
	time_max = (time_a3 > time_max) ? time_a4 : time_max;
	time_max = (time_a3 > time_max) ? time_a5 : time_max;
	time_max = (time_a3 > time_max) ? time_a6 : time_max;


	// step2. caculate the interpolation points 
	/***********************a1************************************/
	double rate_a1 = 0.0;
	double tmp_f1 = ret_value_a1.time1*ret_value_a1.time1*(ret_value_a1.time1 + 1.5*ret_value_a1.time2 + 2*ret_value_a1.time5 + ret_value_a1.time6  )
		+ ret_value_a1.time1*ret_value_a1.time2 * (0.5*ret_value_a1.time2 + 2*ret_value_a1.time5 + ret_value_a1.time6 )
		- ret_value_a1.time5*(ret_value_a1.time5*ret_value_a1.time5 + 1.5*ret_value_a1.time5*ret_value_a1.time6 + 0.5*ret_value_a1.time6*ret_value_a1.time6)
		- (time_a1 - ret_value_a1.time4)*ret_value_a1.time1*(ret_value_a1.time1+ret_value_a1.time2);

	double exp2 = ret_value_a1.time1*(ret_value_a1.time1+ret_value_a1.time2) - ret_value_a1.time5*(ret_value_a1.time5+ret_value_a1.time6);
	double tmp_f2 = (distance1 - v_start.v_a1*time_max)*exp2/(v_end.v_a1 - v_start.v_a1) - time_max*ret_value_a1.time1*(ret_value_a1.time1+ret_value_a1.time2);

	jerk.j_a1 = (v_end.v_a1 - v_start.v_a1)/(rate_a1*rate_a1*exp2);

	rate_a1 = tmp_f2/tmp_f1;
	ret_value_a1.time1 = rate_a1*ret_value_a1.time1;
	ret_value_a1.time2 = rate_a1*ret_value_a1.time2;
	ret_value_a1.time3 = rate_a1*ret_value_a1.time3;
	ret_value_a1.time4 = time_max - rate_a1*(time_a1 - ret_value_a1.time4);
	ret_value_a1.time5 = rate_a1*ret_value_a1.time5;
	ret_value_a1.time6 = rate_a1*ret_value_a1.time6;
	ret_value_a1.time7 = rate_a1*ret_value_a1.time7;

	double v_tmp1_a1 = v_start.v_a1 + 0.5*jerk.j_a1*ret_value_a1.time1*ret_value_a1.time1;
	double v_tmp2_a1 = v_tmp1_a1 + jerk.j_a1*ret_value_a1.time1*ret_value_a1.time2;
	double v_tmp3_a1 = v_tmp2_a1 + 0.5*jerk.j_a1*ret_value_a1.time3*ret_value_a1.time3;
	double v_tmp4_a1 = v_tmp3_a1;
	double v_tmp5_a1 = v_tmp4_a1 - 0.5*jerk.j_a1*ret_value_a1.time5*ret_value_a1.time5;
	double v_tmp6_a1 = v_tmp5_a1 - jerk.j_a1*ret_value_a1.time5*ret_value_a1.time6;
	double dis_tmp1_a1 = v_start.v_a1*ret_value_a1.time1 + jerk.j_a1*ret_value_a1.time1*ret_value_a1.time1*ret_value_a1.time1/6;
	double dis_tmp2_a1 = dis_tmp1_a1 + v_tmp1_a1*ret_value_a1.time2 + 0.5*jerk.j_a1*ret_value_a1.time1*ret_value_a1.time2*ret_value_a1.time2;
	double dis_tmp3_a1 = dis_tmp2_a1 + v_tmp2_a1*ret_value_a1.time3 + 0.5*jerk.j_a1*ret_value_a1.time1*ret_value_a1.time3*ret_value_a1.time3 - jerk.j_a1*ret_value_a1.time3*ret_value_a1.time3*ret_value_a1.time3/6;
	double dis_tmp4_a1 = dis_tmp3_a1 + v_tmp3_a1*ret_value_a1.time4;
	double dis_tmp5_a1 = dis_tmp4_a1 + v_tmp4_a1*ret_value_a1.time5 - jerk.j_a1*ret_value_a1.time5*ret_value_a1.time5*ret_value_a1.time5/6;
	double dis_tmp6_a1 = dis_tmp5_a1 + v_tmp5_a1*ret_value_a1.time6 - 0.5*jerk.j_a1*ret_value_a1.time5*ret_value_a1.time6*ret_value_a1.time6;
	double dis_tmp7_a1 = dis_tmp6_a1 + v_tmp6_a1*ret_value_a1.time7 - 0.5*jerk.j_a1*ret_value_a1.time5*ret_value_a1.time7*ret_value_a1.time7 + jerk.j_a1*ret_value_a1.time7*ret_value_a1.time7*ret_value_a1.time7/6;


	/***********************a2************************************/
	double rate_a2 = 0.0;
	tmp_f1 = ret_value_a2.time1*ret_value_a2.time1*(ret_value_a2.time1 + 1.5*ret_value_a2.time2 + 2*ret_value_a2.time5 + ret_value_a2.time6  )
		+ ret_value_a2.time1*ret_value_a2.time2 * (0.5*ret_value_a2.time2 + 2*ret_value_a2.time5 + ret_value_a2.time6 )
		- ret_value_a2.time5*(ret_value_a2.time5*ret_value_a2.time5 + 1.5*ret_value_a2.time5*ret_value_a2.time6 + 0.5*ret_value_a2.time6*ret_value_a2.time6)
		- (time_a2 - ret_value_a2.time4)*ret_value_a2.time1*(ret_value_a2.time1+ret_value_a2.time2);

	exp2 = ret_value_a2.time1*(ret_value_a2.time1+ret_value_a2.time2) - ret_value_a2.time5*(ret_value_a2.time5+ret_value_a2.time6);
	tmp_f2 = (distance2 - v_start.v_a2*time_max)*exp2/(v_end.v_a2 - v_start.v_a2) - time_max*ret_value_a2.time1*(ret_value_a2.time1+ret_value_a2.time2);

	jerk.j_a2 = (v_end.v_a2 - v_start.v_a2)/(rate_a2*rate_a2*exp2);

	rate_a1 = tmp_f2/tmp_f1;
	ret_value_a2.time1 = rate_a2*ret_value_a2.time1;
	ret_value_a2.time2 = rate_a2*ret_value_a2.time2;
	ret_value_a2.time3 = rate_a2*ret_value_a2.time3;
	ret_value_a2.time4 = time_max - rate_a2*(time_a2 - ret_value_a2.time4);
	ret_value_a2.time5 = rate_a1*ret_value_a2.time5;
	ret_value_a2.time6 = rate_a1*ret_value_a2.time6;
	ret_value_a2.time7 = rate_a1*ret_value_a2.time7;

	double v_tmp1_a2 = v_start.v_a2 + 0.5*jerk.j_a2*ret_value_a2.time1*ret_value_a2.time1;
	double v_tmp2_a2 = v_tmp1_a2 + jerk.j_a2*ret_value_a2.time1*ret_value_a2.time2;
	double v_tmp3_a2 = v_tmp2_a2 + 0.5*jerk.j_a2*ret_value_a2.time3*ret_value_a2.time3;
	double v_tmp4_a2 = v_tmp3_a2;
	double v_tmp5_a2 = v_tmp4_a2 - 0.5*jerk.j_a2*ret_value_a2.time5*ret_value_a2.time5;
	double v_tmp6_a2 = v_tmp5_a2 - jerk.j_a2*ret_value_a2.time5*ret_value_a2.time6;
	double dis_tmp1_a2 = v_start.v_a2*ret_value_a2.time1 + jerk.j_a2*ret_value_a2.time1*ret_value_a2.time1*ret_value_a2.time1/6;
	double dis_tmp2_a2 = dis_tmp1_a2 + v_tmp1_a2*ret_value_a2.time2 + 0.5*jerk.j_a2*ret_value_a2.time1*ret_value_a2.time2*ret_value_a2.time2;
	double dis_tmp3_a2 = dis_tmp2_a2 + v_tmp2_a2*ret_value_a2.time3 + 0.5*jerk.j_a2*ret_value_a2.time1*ret_value_a2.time3*ret_value_a2.time3 - jerk.j_a2*ret_value_a2.time3*ret_value_a2.time3*ret_value_a2.time3/6;
	double dis_tmp4_a2 = dis_tmp3_a2 + v_tmp3_a2*ret_value_a2.time4;
	double dis_tmp5_a2 = dis_tmp4_a2 + v_tmp4_a2*ret_value_a2.time5 - jerk.j_a2*ret_value_a2.time5*ret_value_a2.time5*ret_value_a2.time5/6;
	double dis_tmp6_a2 = dis_tmp5_a2 + v_tmp5_a2*ret_value_a2.time6 - 0.5*jerk.j_a2*ret_value_a2.time5*ret_value_a2.time6*ret_value_a2.time6;
	double dis_tmp7_a2= dis_tmp6_a2 + v_tmp6_a2*ret_value_a2.time7 - 0.5*jerk.j_a2*ret_value_a2.time5*ret_value_a2.time7*ret_value_a2.time7 + jerk.j_a2*ret_value_a2.time7*ret_value_a2.time7*ret_value_a2.time7/6;

	/***********************a3************************************/
	double rate_a3 = 0.0;
	tmp_f1 = ret_value_a3.time1*ret_value_a3.time1*(ret_value_a3.time1 + 1.5*ret_value_a3.time2 + 2*ret_value_a3.time5 + ret_value_a3.time6  )
		+ ret_value_a3.time1*ret_value_a3.time2 * (0.5*ret_value_a3.time2 + 2*ret_value_a3.time5 + ret_value_a3.time6 )
		- ret_value_a3.time5*(ret_value_a3.time5*ret_value_a3.time5 + 1.5*ret_value_a3.time5*ret_value_a3.time6 + 0.5*ret_value_a3.time6*ret_value_a3.time6)
		- (time_a3 - ret_value_a3.time4)*ret_value_a3.time1*(ret_value_a3.time1+ret_value_a3.time2);

	exp2 = ret_value_a3.time1*(ret_value_a3.time1+ret_value_a3.time2) - ret_value_a3.time5*(ret_value_a3.time5+ret_value_a3.time6);
	tmp_f2 = (distance3 - v_start.v_a3*time_max)*exp2/(v_end.v_a3 - v_start.v_a3) - time_max*ret_value_a3.time1*(ret_value_a3.time1+ret_value_a3.time2);

	jerk.j_a3 = (v_end.v_a3 - v_start.v_a3)/(rate_a3*rate_a3*exp2);

	rate_a3 = tmp_f2/tmp_f1;
	ret_value_a3.time1 = rate_a3*ret_value_a3.time1;
	ret_value_a3.time2 = rate_a3*ret_value_a3.time2;
	ret_value_a3.time3 = rate_a3*ret_value_a3.time3;
	ret_value_a3.time4 = time_max - rate_a3*(time_a3 - ret_value_a3.time4);
	ret_value_a3.time5 = rate_a3*ret_value_a3.time5;
	ret_value_a3.time6 = rate_a3*ret_value_a3.time6;
	ret_value_a3.time7 = rate_a3*ret_value_a3.time7;

	double v_tmp1_a3 = v_start.v_a3 + 0.5*jerk.j_a3*ret_value_a3.time1*ret_value_a3.time1;
	double v_tmp2_a3 = v_tmp1_a3 + jerk.j_a3*ret_value_a3.time1*ret_value_a3.time2;
	double v_tmp3_a3 = v_tmp2_a3 + 0.5*jerk.j_a3*ret_value_a3.time3*ret_value_a3.time3;
	double v_tmp4_a3 = v_tmp3_a3;
	double v_tmp5_a3 = v_tmp4_a3 - 0.5*jerk.j_a3*ret_value_a3.time5*ret_value_a3.time5;
	double v_tmp6_a3 = v_tmp5_a3 - jerk.j_a3*ret_value_a3.time5*ret_value_a3.time6;
	double dis_tmp1_a3 = v_start.v_a3*ret_value_a3.time1 + jerk.j_a3*ret_value_a3.time1*ret_value_a3.time1*ret_value_a3.time1/6;
	double dis_tmp2_a3 = dis_tmp1_a3 + v_tmp1_a3*ret_value_a3.time2 + 0.5*jerk.j_a3*ret_value_a3.time1*ret_value_a3.time2*ret_value_a3.time2;
	double dis_tmp3_a3 = dis_tmp2_a3 + v_tmp2_a3*ret_value_a3.time3 + 0.5*jerk.j_a3*ret_value_a3.time1*ret_value_a3.time3*ret_value_a3.time3 - jerk.j_a3*ret_value_a3.time3*ret_value_a3.time3*ret_value_a3.time3/6;
	double dis_tmp4_a3 = dis_tmp3_a3 + v_tmp3_a3*ret_value_a3.time4;
	double dis_tmp5_a3 = dis_tmp4_a3 + v_tmp4_a3*ret_value_a3.time5 - jerk.j_a3*ret_value_a3.time5*ret_value_a3.time5*ret_value_a3.time5/6;
	double dis_tmp6_a3 = dis_tmp5_a3 + v_tmp5_a3*ret_value_a3.time6 - 0.5*jerk.j_a3*ret_value_a3.time5*ret_value_a3.time6*ret_value_a3.time6;
	double dis_tmp7_a3 = dis_tmp6_a3 + v_tmp6_a3*ret_value_a3.time7 - 0.5*jerk.j_a3*ret_value_a3.time5*ret_value_a3.time7*ret_value_a3.time7 + jerk.j_a3*ret_value_a3.time7*ret_value_a3.time7*ret_value_a3.time7/6;

	/***********************a4************************************/
	double rate_a4 = 0.0;
	tmp_f1 = ret_value_a4.time1*ret_value_a4.time1*(ret_value_a4.time1 + 1.5*ret_value_a4.time2 + 2*ret_value_a4.time5 + ret_value_a4.time6  )
		+ ret_value_a4.time1*ret_value_a4.time2 * (0.5*ret_value_a4.time2 + 2*ret_value_a4.time5 + ret_value_a4.time6 )
		- ret_value_a4.time5*(ret_value_a4.time5*ret_value_a4.time5 + 1.5*ret_value_a4.time5*ret_value_a4.time6 + 0.5*ret_value_a4.time6*ret_value_a4.time6)
		- (time_a4 - ret_value_a4.time4)*ret_value_a4.time1*(ret_value_a4.time1+ret_value_a4.time2);

	exp2 = ret_value_a4.time1*(ret_value_a4.time1+ret_value_a4.time2) - ret_value_a4.time5*(ret_value_a4.time5+ret_value_a4.time6);
	tmp_f2 = (distance4 - v_start.v_a4*time_max)*exp2/(v_end.v_a4 - v_start.v_a4) - time_max*ret_value_a4.time1*(ret_value_a4.time1+ret_value_a4.time2);

	jerk.j_a4 = (v_end.v_a4 - v_start.v_a4)/(rate_a4*rate_a4*exp2);

	rate_a4 = tmp_f2/tmp_f1;
	ret_value_a4.time1 = rate_a4*ret_value_a4.time1;
	ret_value_a4.time2 = rate_a4*ret_value_a4.time2;
	ret_value_a4.time3 = rate_a4*ret_value_a4.time3;
	ret_value_a4.time4 = time_max - rate_a4*(time_a4 - ret_value_a4.time4);
	ret_value_a4.time5 = rate_a4*ret_value_a4.time5;
	ret_value_a4.time6 = rate_a4*ret_value_a4.time6;
	ret_value_a4.time7 = rate_a4*ret_value_a4.time7;

	double v_tmp1_a4 = v_start.v_a4 + 0.5*jerk.j_a4*ret_value_a4.time1*ret_value_a4.time1;
	double v_tmp2_a4 = v_tmp1_a4 + jerk.j_a4*ret_value_a4.time1*ret_value_a4.time2;
	double v_tmp3_a4 = v_tmp2_a4 + 0.5*jerk.j_a4*ret_value_a4.time3*ret_value_a4.time3;
	double v_tmp4_a4 = v_tmp3_a4;
	double v_tmp5_a4 = v_tmp4_a4 - 0.5*jerk.j_a4*ret_value_a4.time5*ret_value_a4.time5;
	double v_tmp6_a4 = v_tmp5_a4 - jerk.j_a4*ret_value_a4.time5*ret_value_a4.time6;
	double dis_tmp1_a4 = v_start.v_a4*ret_value_a4.time1 + jerk.j_a4*ret_value_a4.time1*ret_value_a4.time1*ret_value_a4.time1/6;
	double dis_tmp2_a4 = dis_tmp1_a4 + v_tmp1_a4*ret_value_a4.time2 + 0.5*jerk.j_a4*ret_value_a4.time1*ret_value_a4.time2*ret_value_a4.time2;
	double dis_tmp3_a4 = dis_tmp2_a4 + v_tmp2_a4*ret_value_a4.time3 + 0.5*jerk.j_a4*ret_value_a4.time1*ret_value_a4.time3*ret_value_a4.time3 - jerk.j_a4*ret_value_a4.time3*ret_value_a4.time3*ret_value_a4.time3/6;
	double dis_tmp4_a4 = dis_tmp3_a4 + v_tmp3_a4*ret_value_a4.time4;
	double dis_tmp5_a4 = dis_tmp4_a4 + v_tmp4_a4*ret_value_a4.time5 - jerk.j_a4*ret_value_a4.time5*ret_value_a4.time5*ret_value_a4.time5/6;
	double dis_tmp6_a4 = dis_tmp5_a4 + v_tmp5_a4*ret_value_a4.time6 - 0.5*jerk.j_a4*ret_value_a4.time5*ret_value_a4.time6*ret_value_a4.time6;
	double dis_tmp7_a4 = dis_tmp6_a4 + v_tmp6_a4*ret_value_a4.time7 - 0.5*jerk.j_a4*ret_value_a4.time5*ret_value_a4.time7*ret_value_a4.time7 + jerk.j_a4*ret_value_a4.time7*ret_value_a4.time7*ret_value_a4.time7/6;

	/***********************a5************************************/
	double rate_a5 = 0.0;
	tmp_f1 = ret_value_a5.time1*ret_value_a5.time1*(ret_value_a5.time1 + 1.5*ret_value_a5.time2 + 2*ret_value_a5.time5 + ret_value_a5.time6  )
		+ ret_value_a5.time1*ret_value_a5.time2 * (0.5*ret_value_a5.time2 + 2*ret_value_a5.time5 + ret_value_a5.time6 )
		- ret_value_a5.time5*(ret_value_a5.time5*ret_value_a5.time5 + 1.5*ret_value_a5.time5*ret_value_a5.time6 + 0.5*ret_value_a5.time6*ret_value_a5.time6)
		- (time_a5 - ret_value_a5.time4)*ret_value_a5.time1*(ret_value_a5.time1+ret_value_a5.time2);

	exp2 = ret_value_a5.time1*(ret_value_a5.time1+ret_value_a5.time2) - ret_value_a5.time5*(ret_value_a5.time5+ret_value_a5.time6);
	tmp_f2 = (distance5 - v_start.v_a5*time_max)*exp2/(v_end.v_a5 - v_start.v_a5) - time_max*ret_value_a5.time1*(ret_value_a5.time1+ret_value_a5.time2);

	jerk.j_a5 = (v_end.v_a5 - v_start.v_a5)/(rate_a5*rate_a5*exp2);

	rate_a5 = tmp_f2/tmp_f1;
	ret_value_a5.time1 = rate_a5*ret_value_a5.time1;
	ret_value_a5.time2 = rate_a5*ret_value_a5.time2;
	ret_value_a5.time3 = rate_a5*ret_value_a5.time3;
	ret_value_a5.time4 = time_max - rate_a5*(time_a5 - ret_value_a5.time4);
	ret_value_a5.time5 = rate_a5*ret_value_a5.time5;
	ret_value_a5.time6 = rate_a5*ret_value_a5.time6;
	ret_value_a5.time7 = rate_a5*ret_value_a5.time7;

	double v_tmp1_a5 = v_start.v_a5 + 0.5*jerk.j_a5*ret_value_a5.time1*ret_value_a5.time1;
	double v_tmp2_a5 = v_tmp1_a5 + jerk.j_a5*ret_value_a5.time1*ret_value_a5.time2;
	double v_tmp3_a5 = v_tmp2_a5 + 0.5*jerk.j_a5*ret_value_a5.time3*ret_value_a5.time3;
	double v_tmp4_a5 = v_tmp3_a5;
	double v_tmp5_a5 = v_tmp4_a5 - 0.5*jerk.j_a5*ret_value_a5.time5*ret_value_a5.time5;
	double v_tmp6_a5 = v_tmp5_a5 - jerk.j_a5*ret_value_a5.time5*ret_value_a5.time6;
	double dis_tmp1_a5 = v_start.v_a5*ret_value_a5.time1 + jerk.j_a5*ret_value_a5.time1*ret_value_a5.time1*ret_value_a5.time1/6;
	double dis_tmp2_a5 = dis_tmp1_a5 + v_tmp1_a5*ret_value_a5.time2 + 0.5*jerk.j_a5*ret_value_a5.time1*ret_value_a5.time2*ret_value_a5.time2;
	double dis_tmp3_a5 = dis_tmp2_a5 + v_tmp2_a5*ret_value_a5.time3 + 0.5*jerk.j_a5*ret_value_a5.time1*ret_value_a5.time3*ret_value_a5.time3 - jerk.j_a5*ret_value_a5.time3*ret_value_a5.time3*ret_value_a5.time3/6;
	double dis_tmp4_a5 = dis_tmp3_a5 + v_tmp3_a5*ret_value_a5.time4;
	double dis_tmp5_a5 = dis_tmp4_a5 + v_tmp4_a5*ret_value_a5.time5 - jerk.j_a5*ret_value_a5.time5*ret_value_a5.time5*ret_value_a5.time5/6;
	double dis_tmp6_a5 = dis_tmp5_a5 + v_tmp5_a5*ret_value_a5.time6 - 0.5*jerk.j_a5*ret_value_a5.time5*ret_value_a5.time6*ret_value_a5.time6;
	double dis_tmp7_a5 = dis_tmp6_a5 + v_tmp6_a5*ret_value_a5.time7 - 0.5*jerk.j_a5*ret_value_a5.time5*ret_value_a5.time7*ret_value_a5.time7 + jerk.j_a5*ret_value_a5.time7*ret_value_a5.time7*ret_value_a5.time7/6;

	/***********************a6************************************/
	double rate_a6 = 0.0;
	tmp_f1 = ret_value_a6.time1*ret_value_a6.time1*(ret_value_a6.time1 + 1.5*ret_value_a6.time2 + 2*ret_value_a6.time5 + ret_value_a6.time6  )
		+ ret_value_a6.time1*ret_value_a6.time2 * (0.5*ret_value_a6.time2 + 2*ret_value_a6.time5 + ret_value_a6.time6 )
		- ret_value_a6.time5*(ret_value_a6.time5*ret_value_a6.time5 + 1.5*ret_value_a6.time5*ret_value_a6.time6 + 0.5*ret_value_a6.time6*ret_value_a6.time6)
		- (time_a6 - ret_value_a6.time4)*ret_value_a6.time1*(ret_value_a6.time1+ret_value_a6.time2);

	exp2 = ret_value_a6.time1*(ret_value_a6.time1+ret_value_a6.time2) - ret_value_a6.time5*(ret_value_a6.time5+ret_value_a6.time6);
	tmp_f2 = (distance6 - v_start.v_a6*time_max)*exp2/(v_end.v_a6 - v_start.v_a6) - time_max*ret_value_a6.time1*(ret_value_a6.time1+ret_value_a6.time2);

	jerk.j_a6 = (v_end.v_a6 - v_start.v_a6)/(rate_a6*rate_a6*exp2);

	rate_a6 = tmp_f2/tmp_f1;
	ret_value_a6.time1 = rate_a6*ret_value_a6.time1;
	ret_value_a6.time2 = rate_a6*ret_value_a6.time2;
	ret_value_a6.time3 = rate_a6*ret_value_a6.time3;
	ret_value_a6.time4 = time_max - rate_a6*(time_a6 - ret_value_a6.time4);
	ret_value_a6.time5 = rate_a6*ret_value_a6.time5;
	ret_value_a6.time6 = rate_a6*ret_value_a6.time6;
	ret_value_a6.time7 = rate_a6*ret_value_a6.time7;

	double v_tmp1_a6 = v_start.v_a6 + 0.5*jerk.j_a6*ret_value_a6.time1*ret_value_a6.time1;
	double v_tmp2_a6 = v_tmp1_a6 + jerk.j_a6*ret_value_a6.time1*ret_value_a6.time2;
	double v_tmp3_a6 = v_tmp2_a6 + 0.5*jerk.j_a6*ret_value_a6.time3*ret_value_a6.time3;
	double v_tmp4_a6 = v_tmp3_a6;
	double v_tmp5_a6 = v_tmp4_a6 - 0.5*jerk.j_a6*ret_value_a6.time5*ret_value_a6.time5;
	double v_tmp6_a6 = v_tmp5_a6 - jerk.j_a6*ret_value_a6.time5*ret_value_a6.time6;
	double dis_tmp1_a6 = v_start.v_a6*ret_value_a6.time1 + jerk.j_a6*ret_value_a6.time1*ret_value_a6.time1*ret_value_a6.time1/6;
	double dis_tmp2_a6 = dis_tmp1_a6 + v_tmp1_a6*ret_value_a6.time2 + 0.5*jerk.j_a6*ret_value_a6.time1*ret_value_a6.time2*ret_value_a6.time2;
	double dis_tmp3_a6 = dis_tmp2_a6 + v_tmp2_a6*ret_value_a6.time3 + 0.5*jerk.j_a6*ret_value_a6.time1*ret_value_a6.time3*ret_value_a6.time3 - jerk.j_a6*ret_value_a6.time3*ret_value_a6.time3*ret_value_a6.time3/6;
	double dis_tmp4_a6 = dis_tmp3_a6 + v_tmp3_a6*ret_value_a6.time4;
	double dis_tmp5_a6 = dis_tmp4_a6 + v_tmp4_a6*ret_value_a6.time5 - jerk.j_a6*ret_value_a6.time5*ret_value_a6.time5*ret_value_a6.time5/6;
	double dis_tmp6_a6 = dis_tmp5_a6 + v_tmp5_a6*ret_value_a6.time6 - 0.5*jerk.j_a6*ret_value_a6.time5*ret_value_a6.time6*ret_value_a6.time6;
	double dis_tmp7_a6 = dis_tmp6_a6 + v_tmp6_a6*ret_value_a6.time7 - 0.5*jerk.j_a6*ret_value_a6.time5*ret_value_a6.time7*ret_value_a6.time7 + jerk.j_a6*ret_value_a6.time7*ret_value_a6.time7*ret_value_a6.time7/6;


	int num = ceil((time_max)/cycle);
	double t1_a1 = ret_value_a1.time1;
	double t2_a1 = ret_value_a1.time1 + ret_value_a1.time2;
	double t3_a1 = ret_value_a1.time1 + ret_value_a1.time2 +ret_value_a1.time3;
	double t4_a1 = t3_a1 +ret_value_a1.time4;
	double t5_a1 = t4_a1 +ret_value_a1.time5;
	double t6_a1 = t5_a1 +ret_value_a1.time6;
	double t7_a1 = t6_a1 +ret_value_a1.time7;

	double t1_a2 = ret_value_a2.time1;
	double t2_a2 = ret_value_a2.time1 + ret_value_a2.time2;
	double t3_a2 = ret_value_a2.time1 + ret_value_a2.time2 +ret_value_a2.time3;
	double t4_a2 = t3_a2 +ret_value_a2.time4;
	double t5_a2 = t4_a2 +ret_value_a2.time5;
	double t6_a2 = t5_a2 +ret_value_a2.time6;
	double t7_a2 = t6_a2 +ret_value_a2.time7;

	double t1_a3 = ret_value_a3.time1;
	double t2_a3 = ret_value_a3.time1 + ret_value_a3.time2;
	double t3_a3 = ret_value_a3.time1 + ret_value_a3.time2 +ret_value_a3.time3;
	double t4_a3 = t3_a3 +ret_value_a3.time4;
	double t5_a3 = t4_a3 +ret_value_a3.time5;
	double t6_a3 = t5_a3 +ret_value_a3.time6;
	double t7_a3 = t6_a3 +ret_value_a3.time7;

	double t1_a4 = ret_value_a4.time1;
	double t2_a4 = ret_value_a4.time1 + ret_value_a4.time2;
	double t3_a4 = ret_value_a4.time1 + ret_value_a4.time2 +ret_value_a4.time3;
	double t4_a4 = t3_a4 +ret_value_a4.time4;
	double t5_a4 = t4_a4 +ret_value_a4.time5;
	double t6_a4 = t5_a4 +ret_value_a4.time6;
	double t7_a4 = t6_a4 +ret_value_a4.time7;

	double t1_a5 = ret_value_a5.time1;
	double t2_a5 = ret_value_a5.time1 + ret_value_a5.time2;
	double t3_a5 = ret_value_a5.time1 + ret_value_a5.time2 +ret_value_a5.time3;
	double t4_a5 = t3_a5 +ret_value_a5.time4;
	double t5_a5 = t4_a5 +ret_value_a5.time5;
	double t6_a5 = t5_a5 +ret_value_a5.time6;
	double t7_a5 = t6_a5 +ret_value_a5.time7;

	double t1_a6 = ret_value_a6.time1;
	double t2_a6 = ret_value_a6.time1 + ret_value_a6.time2;
	double t3_a6 = ret_value_a6.time1 + ret_value_a6.time2 +ret_value_a6.time3;
	double t4_a6 = t3_a6 +ret_value_a6.time4;
	double t5_a6 = t4_a6 +ret_value_a6.time5;
	double t6_a6 = t5_a6 +ret_value_a6.time6;
	double t7_a6 = t6_a6 +ret_value_a6.time7;
	double t = 0;


	robot_data_file_process::axispos tmp_point; //Attention here!
	joint_velocity tmp_velocity;
	joint_acc tmp_acc;
        joint_jerk tmp_jerk;

	for(int i = 0; i < num-1; ++i)
	{
		t = (i+1)*cycle;
		/***********************a1************************************/
		if( t >= 0 && t < t1_a1)
		{
			tmp_jerk.j_a1 = jerk.j_a1;
			tmp_acc.a_a1 = jerk.j_a1*t;
			tmp_velocity.v_a1 = v_start.v_a1+0.5*jerk.j_a1*t*t;
			tmp_point.a1 = v_start.v_a1*t+jerk.j_a1*t*t*t/6;
		}
		else if(t >= t1_a1 && t < t2_a1) 
		{
			tmp_jerk.j_a1 = 0;
			tmp_acc.a_a1 = jerk.j_a1*ret_value_a1.time1;
			tmp_velocity.v_a1 = v_tmp1_a1+jerk.j_a1*ret_value_a1.time1*(t-t1_a1);
			tmp_point.a1 = dis_tmp1_a1+v_tmp1_a1*(t-t1_a1)+0.5*jerk.j_a1*ret_value_a1.time1*(t-t1_a1)*(t-t1_a1);
		}
		else if(t >= t2_a1 && t < t3_a1)
		{
			tmp_jerk.j_a1 = -jerk.j_a1;
			tmp_acc.a_a1 = jerk.j_a1*(ret_value_a1.time1 - t + t2_a1);
			tmp_velocity.v_a1 = v_tmp2_a1+jerk.j_a1*ret_value_a1.time1*(t-t2_a1)-0.5*jerk.j_a1*(t-t2_a1)*(t-t2_a1);
			tmp_point.a1 = dis_tmp2_a1+v_tmp2_a1*(t-t2_a1)+0.5*jerk.j_a1*ret_value_a1.time1*(t-t2_a1)*(t-t2_a1)-jerk.j_a1*(t-t2_a1)*(t-t2_a1)*(t-t2_a1)/6;
		}
		else if(t >= t3_a1 && t < t4_a1)
		{
			tmp_jerk.j_a1 = 0;
			tmp_acc.a_a1 = 0;
			tmp_velocity.v_a1 = v_tmp3_a1;
			tmp_point.a1 = dis_tmp3_a1+v_tmp3_a1*(t-t3_a1);
		}
		else if(t >= t4_a1 && t < t5_a1)
		{
			tmp_jerk.j_a1 = -jerk.j_a1;
			tmp_acc.a_a1 = -jerk.j_a1*(t-t4_a1);
			tmp_velocity.v_a1 = v_tmp4_a1-0.5*jerk.j_a1*(t-t4_a1)*(t-t4_a1);
			tmp_point.a1 = dis_tmp4_a1+v_tmp4_a1*(t-t4_a1)-jerk.j_a1*(t-t4_a1)*(t-t4_a1)*(t-t4_a1)/6;
		}
		else if(t >= t5_a1 && t < t6_a1)
		{
			tmp_jerk.j_a1 = 0;
			tmp_acc.a_a1 = -jerk.j_a1*ret_value_a1.time5;
			tmp_velocity.v_a1 = v_tmp5_a1-jerk.j_a1*ret_value_a1.time5*(t-t5_a1);
			tmp_point.a1 = dis_tmp5_a1+v_tmp5_a1*(t-t5_a1)-0.5*jerk.j_a1*ret_value_a1.time5*(t-t5_a1)*(t-t5_a1);
		}
		else if(t >= t6_a1 && t < t7_a1)
		{
			tmp_jerk.j_a1 = jerk.j_a1;
			tmp_acc.a_a1 = jerk.j_a1*(-ret_value_a1.time5+t-t6_a1);
			tmp_velocity.v_a1 = v_tmp6_a1-jerk.j_a1*ret_value_a1.time5*(t-t6_a1)+0.5*jerk.j_a1*(t-t6_a1)*(t-t6_a1);
			tmp_point.a1 = dis_tmp6_a1+v_tmp6_a1*(t-t6_a1)-0.5*jerk.j_a1*ret_value_a1.time5*(t-t6_a1)*(t-t6_a1)+jerk.j_a1*(t-t6_a1)*(t-t6_a1)*(t-t6_a1)/6;
		}

		/***********************a2************************************/
		if( t >= 0 && t < t1_a2)
		{
			tmp_jerk.j_a2 = jerk.j_a2;
			tmp_acc.a_a2 = jerk.j_a2*t;
			tmp_velocity.v_a2 = v_start.v_a2+0.5*jerk.j_a2*t*t;
			tmp_point.a2 = v_start.v_a2*t+jerk.j_a2*t*t*t/6;
		}
		else if(t >= t1_a2 && t < t2_a2) 
		{
			tmp_jerk.j_a2 = 0;
			tmp_acc.a_a2 = jerk.j_a2*ret_value_a2.time1;
			tmp_velocity.v_a2 = v_tmp1_a2+jerk.j_a2*ret_value_a2.time1*(t-t1_a2);
			tmp_point.a2 = dis_tmp1_a2+v_tmp1_a2*(t-t1_a2)+0.5*jerk.j_a2*ret_value_a2.time1*(t-t1_a2)*(t-t1_a2);
		}
		else if(t >= t2_a2 && t < t3_a2)
		{
			tmp_jerk.j_a2 = -jerk.j_a2;
			tmp_acc.a_a2 = jerk.j_a2*(ret_value_a2.time1 - t + t2_a2);
			tmp_velocity.v_a2 = v_tmp2_a2+jerk.j_a2*ret_value_a2.time1*(t-t2_a2)-0.5*jerk.j_a2*(t-t2_a2)*(t-t2_a2);
			tmp_point.a2 = dis_tmp2_a2+v_tmp2_a2*(t-t2_a2)+0.5*jerk.j_a2*ret_value_a2.time1*(t-t2_a2)*(t-t2_a2)-jerk.j_a2*(t-t2_a2)*(t-t2_a2)*(t-t2_a2)/6;
		}
		else if(t >= t3_a2 && t < t4_a2)
		{
			tmp_jerk.j_a2 = 0;
			tmp_acc.a_a2 = 0;
			tmp_velocity.v_a2 = v_tmp3_a2;
			tmp_point.a2 = dis_tmp3_a2+v_tmp3_a2*(t-t3_a2);
		}
		else if(t >= t4_a2 && t < t5_a2)
		{
			tmp_jerk.j_a2 = -jerk.j_a2;
			tmp_acc.a_a2 = -jerk.j_a2*(t-t4_a2);
			tmp_velocity.v_a2 = v_tmp4_a2-0.5*jerk.j_a2*(t-t4_a2)*(t-t4_a2);
			tmp_point.a2 = dis_tmp4_a2+v_tmp4_a2*(t-t4_a2)-jerk.j_a2*(t-t4_a2)*(t-t4_a2)*(t-t4_a2)/6;
		}
		else if(t >= t5_a2 && t < t6_a2)
		{
			tmp_jerk.j_a2 = 0;
			tmp_acc.a_a2 = -jerk.j_a2*ret_value_a2.time5;
			tmp_velocity.v_a2 = v_tmp5_a2-jerk.j_a2*ret_value_a2.time5*(t-t5_a2);
			tmp_point.a2 = dis_tmp5_a2+v_tmp5_a2*(t-t5_a2)-0.5*jerk.j_a2*ret_value_a2.time5*(t-t5_a2)*(t-t5_a2);
		}
		else if(t >= t6_a2 && t < t7_a2)
		{
			tmp_jerk.j_a2 = jerk.j_a2;
			tmp_acc.a_a2 = jerk.j_a2*(-ret_value_a2.time5+t-t6_a2);
			tmp_velocity.v_a2 = v_tmp6_a2-jerk.j_a2*ret_value_a2.time5*(t-t6_a2)+0.5*jerk.j_a2*(t-t6_a2)*(t-t6_a2);
			tmp_point.a2 = dis_tmp6_a2+v_tmp6_a2*(t-t6_a2)-0.5*jerk.j_a2*ret_value_a2.time5*(t-t6_a2)*(t-t6_a2)+jerk.j_a2*(t-t6_a2)*(t-t6_a2)*(t-t6_a2)/6;
		}

		/***********************a3************************************/
		if( t >= 0 && t < t1_a3)
		{
			tmp_jerk.j_a3 = jerk.j_a3;
			tmp_acc.a_a3 = jerk.j_a3*t;
			tmp_velocity.v_a3 = v_start.v_a3+0.5*jerk.j_a3*t*t;
			tmp_point.a3 = v_start.v_a3*t+jerk.j_a3*t*t*t/6;
		}
		else if(t >= t1_a3 && t < t2_a3) 
		{
			tmp_jerk.j_a3 = 0;
			tmp_acc.a_a3 = jerk.j_a3*ret_value_a3.time1;
			tmp_velocity.v_a3 = v_tmp1_a3+jerk.j_a3*ret_value_a3.time1*(t-t1_a3);
			tmp_point.a3 = dis_tmp1_a3+v_tmp1_a3*(t-t1_a3)+0.5*jerk.j_a3*ret_value_a3.time1*(t-t1_a3)*(t-t1_a3);
		}
		else if(t >= t2_a3 && t < t3_a3)
		{
			tmp_jerk.j_a3 = -jerk.j_a3;
			tmp_acc.a_a3 = jerk.j_a3*(ret_value_a3.time1 - t + t2_a3);
			tmp_velocity.v_a3 = v_tmp2_a3+jerk.j_a3*ret_value_a3.time1*(t-t2_a3)-0.5*jerk.j_a3*(t-t2_a3)*(t-t2_a3);
			tmp_point.a3 = dis_tmp2_a3+v_tmp2_a3*(t-t2_a3)+0.5*jerk.j_a3*ret_value_a3.time1*(t-t2_a3)*(t-t2_a3)-jerk.j_a3*(t-t2_a3)*(t-t2_a3)*(t-t2_a3)/6;
		}
		else if(t >= t3_a3 && t < t4_a3)
		{
			tmp_jerk.j_a3 = 0;
			tmp_acc.a_a3 = 0;
			tmp_velocity.v_a3 = v_tmp3_a3;
			tmp_point.a3 = dis_tmp3_a3+v_tmp3_a3*(t-t3_a3);
		}
		else if(t >= t4_a3 && t < t5_a3)
		{
			tmp_jerk.j_a3 = -jerk.j_a3;
			tmp_acc.a_a3 = -jerk.j_a3*(t-t4_a3);
			tmp_velocity.v_a3 = v_tmp4_a3-0.5*jerk.j_a3*(t-t4_a3)*(t-t4_a3);
			tmp_point.a3 = dis_tmp4_a3+v_tmp4_a3*(t-t4_a3)-jerk.j_a3*(t-t4_a3)*(t-t4_a3)*(t-t4_a3)/6;
		}
		else if(t >= t5_a3 && t < t6_a3)
		{
			tmp_jerk.j_a3 = 0;
			tmp_acc.a_a3 = -jerk.j_a3*ret_value_a3.time5;
			tmp_velocity.v_a3 = v_tmp5_a3-jerk.j_a3*ret_value_a3.time5*(t-t5_a3);
			tmp_point.a3 = dis_tmp5_a3+v_tmp5_a3*(t-t5_a3)-0.5*jerk.j_a3*ret_value_a3.time5*(t-t5_a3)*(t-t5_a3);
		}
		else if(t >= t6_a3 && t < t7_a3)
		{
			tmp_jerk.j_a3 = jerk.j_a3;
			tmp_acc.a_a3 = jerk.j_a3*(-ret_value_a3.time5+t-t6_a3);
			tmp_velocity.v_a3 = v_tmp6_a3-jerk.j_a3*ret_value_a3.time5*(t-t6_a3)+0.5*jerk.j_a3*(t-t6_a3)*(t-t6_a3);
			tmp_point.a3 = dis_tmp6_a3+v_tmp6_a3*(t-t6_a3)-0.5*jerk.j_a3*ret_value_a3.time5*(t-t6_a3)*(t-t6_a3)+jerk.j_a3*(t-t6_a3)*(t-t6_a3)*(t-t6_a3)/6;
		}

		/***********************a4************************************/
		if( t >= 0 && t < t1_a4)
		{
			tmp_jerk.j_a4 = jerk.j_a4;
			tmp_acc.a_a4 = jerk.j_a4*t;
			tmp_velocity.v_a4 = v_start.v_a4+0.5*jerk.j_a4*t*t;
			tmp_point.a4 = v_start.v_a4*t+jerk.j_a4*t*t*t/6;
		}
		else if(t >= t1_a4 && t < t2_a4) 
		{
			tmp_jerk.j_a4 = 0;
			tmp_acc.a_a4 = jerk.j_a4*ret_value_a4.time1;
			tmp_velocity.v_a4 = v_tmp1_a4+jerk.j_a4*ret_value_a4.time1*(t-t1_a4);
			tmp_point.a4 = dis_tmp1_a4+v_tmp1_a4*(t-t1_a4)+0.5*jerk.j_a4*ret_value_a4.time1*(t-t1_a4)*(t-t1_a4);
		}
		else if(t >= t2_a4 && t < t3_a4)
		{
			tmp_jerk.j_a4 = -jerk.j_a4;
			tmp_acc.a_a4 = jerk.j_a4*(ret_value_a4.time1 - t + t2_a4);
			tmp_velocity.v_a4 = v_tmp2_a4+jerk.j_a4*ret_value_a4.time1*(t-t2_a4)-0.5*jerk.j_a4*(t-t2_a4)*(t-t2_a4);
			tmp_point.a4 = dis_tmp2_a4+v_tmp2_a4*(t-t2_a4)+0.5*jerk.j_a4*ret_value_a4.time1*(t-t2_a4)*(t-t2_a4)-jerk.j_a4*(t-t2_a4)*(t-t2_a4)*(t-t2_a4)/6;
		}
		else if(t >= t3_a4 && t < t4_a4)
		{
			tmp_jerk.j_a4 = 0;
			tmp_acc.a_a4 = 0;
			tmp_velocity.v_a4 = v_tmp3_a4;
			tmp_point.a4 = dis_tmp3_a4+v_tmp3_a4*(t-t3_a4);
		}
		else if(t >= t4_a4 && t < t5_a4)
		{
			tmp_jerk.j_a4 = -jerk.j_a4;
			tmp_acc.a_a4 = -jerk.j_a4*(t-t4_a4);
			tmp_velocity.v_a4 = v_tmp4_a4-0.5*jerk.j_a4*(t-t4_a4)*(t-t4_a4);
			tmp_point.a4 = dis_tmp4_a4+v_tmp4_a4*(t-t4_a4)-jerk.j_a4*(t-t4_a4)*(t-t4_a4)*(t-t4_a4)/6;
		}
		else if(t >= t5_a4 && t < t6_a4)
		{
			tmp_jerk.j_a4 = 0;
			tmp_acc.a_a4 = -jerk.j_a4*ret_value_a4.time5;
			tmp_velocity.v_a4 = v_tmp5_a4-jerk.j_a4*ret_value_a4.time5*(t-t5_a4);
			tmp_point.a4 = dis_tmp5_a4+v_tmp5_a4*(t-t5_a4)-0.5*jerk.j_a4*ret_value_a4.time5*(t-t5_a4)*(t-t5_a4);
		}
		else if(t >= t6_a4 && t < t7_a4)
		{
			tmp_jerk.j_a4 = jerk.j_a4;
			tmp_acc.a_a4 = jerk.j_a4*(-ret_value_a4.time5+t-t6_a4);
			tmp_velocity.v_a4 = v_tmp6_a4-jerk.j_a4*ret_value_a4.time5*(t-t6_a4)+0.5*jerk.j_a4*(t-t6_a4)*(t-t6_a4);
			tmp_point.a4 = dis_tmp6_a4+v_tmp6_a4*(t-t6_a4)-0.5*jerk.j_a4*ret_value_a4.time5*(t-t6_a4)*(t-t6_a4)+jerk.j_a4*(t-t6_a4)*(t-t6_a4)*(t-t6_a4)/6;
		}

		/**********************a5*************************************/
		if( t >= 0 && t < t1_a5)
		{
			tmp_jerk.j_a5 = jerk.j_a5;
			tmp_acc.a_a5 = jerk.j_a5*t;
			tmp_velocity.v_a5 = v_start.v_a5+0.5*jerk.j_a5*t*t;
			tmp_point.a5 = v_start.v_a5*t+jerk.j_a5*t*t*t/6;
		}
		else if(t >= t1_a5 && t < t2_a5) 
		{
			tmp_jerk.j_a5 = 0;
			tmp_acc.a_a5 = jerk.j_a5*ret_value_a5.time1;
			tmp_velocity.v_a5 = v_tmp1_a5+jerk.j_a5*ret_value_a5.time1*(t-t1_a5);
			tmp_point.a5 = dis_tmp1_a5+v_tmp1_a5*(t-t1_a5)+0.5*jerk.j_a5*ret_value_a5.time1*(t-t1_a5)*(t-t1_a5);
		}
		else if(t >= t2_a5 && t < t3_a5)
		{
			tmp_jerk.j_a5 = -jerk.j_a5;
			tmp_acc.a_a5 = jerk.j_a5*(ret_value_a5.time1 - t + t2_a5);
			tmp_velocity.v_a5 = v_tmp2_a5+jerk.j_a5*ret_value_a5.time1*(t-t2_a5)-0.5*jerk.j_a5*(t-t2_a5)*(t-t2_a5);
			tmp_point.a5 = dis_tmp2_a5+v_tmp2_a5*(t-t2_a5)+0.5*jerk.j_a5*ret_value_a5.time1*(t-t2_a5)*(t-t2_a5)-jerk.j_a5*(t-t2_a5)*(t-t2_a5)*(t-t2_a5)/6;
		}
		else if(t >= t3_a5 && t < t4_a5)
		{
			tmp_jerk.j_a5 = 0;
			tmp_acc.a_a5 = 0;
			tmp_velocity.v_a5 = v_tmp3_a5;
			tmp_point.a5 = dis_tmp3_a5+v_tmp3_a5*(t-t3_a5);
		}
		else if(t >= t4_a5 && t < t5_a5)
		{
			tmp_jerk.j_a5 = -jerk.j_a5;
			tmp_acc.a_a5 = -jerk.j_a5*(t-t4_a5);
			tmp_velocity.v_a5 = v_tmp4_a5-0.5*jerk.j_a5*(t-t4_a5)*(t-t4_a5);
			tmp_point.a5 = dis_tmp4_a5+v_tmp4_a5*(t-t4_a5)-jerk.j_a5*(t-t4_a5)*(t-t4_a5)*(t-t4_a5)/6;
		}
		else if(t >= t5_a5 && t < t6_a5)
		{
			tmp_jerk.j_a5 = 0;
			tmp_acc.a_a5 = -jerk.j_a5*ret_value_a5.time5;
			tmp_velocity.v_a5 = v_tmp5_a5-jerk.j_a5*ret_value_a5.time5*(t-t5_a5);
			tmp_point.a5 = dis_tmp5_a5+v_tmp5_a5*(t-t5_a5)-0.5*jerk.j_a5*ret_value_a5.time5*(t-t5_a5)*(t-t5_a5);
		}
		else if(t >= t6_a5 && t < t7_a5)
		{
			tmp_jerk.j_a5 = jerk.j_a5;
			tmp_acc.a_a5 = jerk.j_a5*(-ret_value_a5.time5+t-t6_a5);
			tmp_velocity.v_a5 = v_tmp6_a5-jerk.j_a5*ret_value_a5.time5*(t-t6_a5)+0.5*jerk.j_a5*(t-t6_a5)*(t-t6_a5);
			tmp_point.a5 = dis_tmp6_a5+v_tmp6_a5*(t-t6_a5)-0.5*jerk.j_a5*ret_value_a5.time5*(t-t6_a5)*(t-t6_a5)+jerk.j_a5*(t-t6_a5)*(t-t6_a5)*(t-t6_a5)/6;
		}

		/**********************a6*************************************/
		if( t >= 0 && t < t1_a6)
		{
			tmp_jerk.j_a6 = jerk.j_a6;
			tmp_acc.a_a6 = jerk.j_a6*t;
			tmp_velocity.v_a6 = v_start.v_a6+0.5*jerk.j_a6*t*t;
			tmp_point.a6 = v_start.v_a6*t+jerk.j_a6*t*t*t/6;
		}
		else if(t >= t1_a6 && t < t2_a6) 
		{
			tmp_jerk.j_a6 = 0;
			tmp_acc.a_a6 = jerk.j_a6*ret_value_a6.time1;
			tmp_velocity.v_a6 = v_tmp1_a6+jerk.j_a6*ret_value_a6.time1*(t-t1_a6);
			tmp_point.a6 = dis_tmp1_a6+v_tmp1_a6*(t-t1_a6)+0.5*jerk.j_a6*ret_value_a6.time1*(t-t1_a6)*(t-t1_a6);
		}
		else if(t >= t2_a6 && t < t3_a6)
		{
			tmp_jerk.j_a6 = -jerk.j_a6;
			tmp_acc.a_a6 = jerk.j_a6*(ret_value_a6.time1 - t + t2_a6);
			tmp_velocity.v_a6 = v_tmp2_a6+jerk.j_a6*ret_value_a6.time1*(t-t2_a6)-0.5*jerk.j_a6*(t-t2_a6)*(t-t2_a6);
			tmp_point.a6 = dis_tmp2_a6+v_tmp2_a6*(t-t2_a6)+0.5*jerk.j_a6*ret_value_a6.time1*(t-t2_a6)*(t-t2_a6)-jerk.j_a6*(t-t2_a6)*(t-t2_a6)*(t-t2_a6)/6;
		}
		else if(t >= t3_a6 && t < t4_a6)
		{
			tmp_jerk.j_a6 = 0;
			tmp_acc.a_a6 = 0;
			tmp_velocity.v_a6 = v_tmp3_a6;
			tmp_point.a6 = dis_tmp3_a6+v_tmp3_a6*(t-t3_a6);
		}
		else if(t >= t4_a6 && t < t5_a6)
		{
			tmp_jerk.j_a6 = -jerk.j_a6;
			tmp_acc.a_a6 = -jerk.j_a6*(t-t4_a6);
			tmp_velocity.v_a6 = v_tmp4_a6-0.5*jerk.j_a6*(t-t4_a6)*(t-t4_a6);
			tmp_point.a6 = dis_tmp4_a6+v_tmp4_a6*(t-t4_a6)-jerk.j_a6*(t-t4_a6)*(t-t4_a6)*(t-t4_a6)/6;
		}
		else if(t >= t5_a6 && t < t6_a6)
		{
			tmp_jerk.j_a6 = 0;
			tmp_acc.a_a6 = -jerk.j_a6*ret_value_a6.time5;
			tmp_velocity.v_a6 = v_tmp5_a6-jerk.j_a6*ret_value_a6.time5*(t-t5_a6);
			tmp_point.a6 = dis_tmp5_a6+v_tmp5_a6*(t-t5_a6)-0.5*jerk.j_a6*ret_value_a6.time5*(t-t5_a6)*(t-t5_a6);
		}
		else if(t >= t6_a6 && t < t7_a6)
		{
			tmp_jerk.j_a6 = jerk.j_a6;
			tmp_acc.a_a6 = jerk.j_a6*(-ret_value_a6.time5+t-t6_a6);
			tmp_velocity.v_a6 = v_tmp6_a6-jerk.j_a6*ret_value_a6.time5*(t-t6_a6)+0.5*jerk.j_a6*(t-t6_a6)*(t-t6_a6);
			tmp_point.a6 = dis_tmp6_a6+v_tmp6_a6*(t-t6_a6)-0.5*jerk.j_a6*ret_value_a6.time5*(t-t6_a6)*(t-t6_a6)+jerk.j_a6*(t-t6_a6)*(t-t6_a6)*(t-t6_a6)/6;
		}


		tmp_point.a1 = tmp_point.a1*flag1 + p_start.a1;
		tmp_point.a2 = tmp_point.a2*flag2 + p_start.a2;
		tmp_point.a3 = tmp_point.a3*flag3 + p_start.a3;
		tmp_point.a4 = tmp_point.a4*flag4 + p_start.a4;
		tmp_point.a5 = tmp_point.a5*flag5 + p_start.a5;
		tmp_point.a6 = tmp_point.a6*flag6 + p_start.a6;

		points_set.push_back(tmp_point); //Attention here!
		points_velocity.push_back(tmp_velocity);
		points_acc.push_back(tmp_acc);
                points_jerk.push_back(tmp_jerk);
	}

	//The last point of interpolator
	tmp_point.a1 = p_end.a1;
	tmp_point.a2 = p_end.a2;
	tmp_point.a3 = p_end.a3;
	tmp_point.a4 = p_end.a4;
	tmp_point.a5 = p_end.a5;
	tmp_point.a6 = p_end.a6;

	tmp_velocity.v_a1 = v_end.v_a1;
	tmp_velocity.v_a2 = v_end.v_a2;
	tmp_velocity.v_a3 = v_end.v_a3;
	tmp_velocity.v_a4 = v_end.v_a4;
	tmp_velocity.v_a5 = v_end.v_a5;
	tmp_velocity.v_a6 = v_end.v_a6;

	//tmp_acc.a_a1 = -dec.a_a1;
	//tmp_acc.a_a2 = -dec.a_a2;
	//tmp_acc.a_a3 = -dec.a_a3;
	//tmp_acc.a_a4 = -dec.a_a4;
	//tmp_acc.a_a5 = -dec.a_a5;
	//tmp_acc.a_a6 = -dec.a_a6;

	points_set.push_back(tmp_point); //Attention here!
	points_velocity.push_back(tmp_velocity);
	//points_acc.push_back(tmp_acc);

	return 0;

}






int Lin_SMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_end,
		double v_start, double v_end, double v_target,
		double acc, double dec,
		double jerk,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set)
{
	// step1. caculate time1, time2, time2, maybe should reset v_start, v_end
	double distance = sqrt((p_start.x - p_end.x)*(p_start.x - p_end.x) + (p_start.y - p_end.y)*(p_start.y - p_end.y) + (p_start.z - p_end.z)*(p_start.z - p_end.z));
	SMode_struct ret_value;
	int ret = SMode_curve(distance, v_start, v_end, v_target, acc, dec, jerk, ret_value);
	if(ret < 0)
	{
		return -1;
	}
	double time1 = ret_value.time1;
	double time2 = ret_value.time2;
	double time3 = ret_value.time3;
	double time4 = ret_value.time4;
	double time5 = ret_value.time5;
	double time6 = ret_value.time6;
	double time7 = ret_value.time7;
	v_start = ret_value.v_start;
	v_end = ret_value.v_end;

	//test TrigMode_curve()
	std::cout << "time1: " << time1 << std::endl;
	std::cout << "time2: " << time2 << std::endl;
	std::cout << "time3: " << time3 << std::endl;
	std::cout << "time4: " << time4 << std::endl;
	std::cout << "time5: " << time5 << std::endl;
	std::cout << "time6: " << time6 << std::endl;
	std::cout << "time7: " << time7 << std::endl;
	std::cout << "v_start: " << v_start << std::endl;
	std::cout << "v_end: " << v_end << std::endl;

	double kx = (p_start.x - p_end.x)/distance;
	double ky = (p_start.y - p_end.y)/distance;
	double kz = (p_start.z - p_end.z)/distance;

	// step2. caculate the interpolation points 
	int num = ceil((time1+time2+time3+time4+time5+time6+time7)/cycle);
	double t1 = time1;
	double t2 = time1 + time2;
	double t3 = time1 + time2 +time3;
	double t4 = t3 + time4;
	double t5 = t4 + time5;
	double t6 = t5 + time6;
	double t7 = t6 + time7;
	double t = 0;



	double v_tmp1 = v_start + 0.5*jerk*time1*time1;
	double v_tmp2 = v_tmp1 + jerk*time1*time2;
	double v_tmp3 = v_tmp2 + 0.5*jerk*time3*time3;
	double v_tmp4 = v_tmp3;
	double v_tmp5 = v_tmp4 - 0.5*jerk*time5*time5;
	double v_tmp6 = v_tmp5 - jerk*time5*time6;
	double dis_tmp1 = v_start*time1 + jerk*time1*time1*time1/6;
	double dis_tmp2 = dis_tmp1 + v_tmp1*time2 + 0.5*jerk*time1*time2*time2;
	double dis_tmp3 = dis_tmp2 + v_tmp2*time3 + 0.5*jerk*time1*time3*time3 - jerk*time3*time3*time3/6;
	double dis_tmp4 = dis_tmp3 + v_tmp3*time4;
	double dis_tmp5 = dis_tmp4 + v_tmp4*time5 - jerk*time5*time5*time5/6;
	double dis_tmp6 = dis_tmp5 + v_tmp5*time6 - 0.5*jerk*time5*time6*time6;
	double dis_tmp7 = dis_tmp6 + v_tmp6*time7 - 0.5*jerk*time5*time7*time7 + jerk*time7*time7*time7/6;

	interpolation_point tmp_point; //Attention here!
	robot_data_file_process::cartpos tmp_dis;
	for(int i = 0; i < num-1; ++i)
	{
		t = (i+1)*cycle;
		if( t >= 0 && t < t1)
		{
			tmp_point.jerk = jerk;
			tmp_point.acc = jerk*t;
			tmp_point.velocity = v_start+0.5*jerk*t*t;
			tmp_point.distance = v_start*t+jerk*t*t*t/6;
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t1 && t < t2) 
		{
			tmp_point.jerk = 0;
			tmp_point.acc = jerk*time1;
			tmp_point.velocity = v_tmp1+jerk*time1*(t-t1);
			tmp_point.distance = dis_tmp1+v_tmp1*(t-t1)+0.5*jerk*time1*(t-t1)*(t-t1);
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t2 && t < t3)
		{
			tmp_point.jerk = -jerk;
			tmp_point.acc = jerk*(time1 - t + t2);
			tmp_point.velocity = v_tmp2+jerk*time1*(t-t2)-0.5*jerk*(t-t2)*(t-t2);
			tmp_point.distance = dis_tmp2+v_tmp2*(t-t2)+0.5*jerk*time1*(t-t2)*(t-t2)-jerk*(t-t2)*(t-t2)*(t-t2)/6;
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t3 && t < t4)
		{
			tmp_point.jerk = 0;
			tmp_point.acc = 0;
			tmp_point.velocity = v_tmp3;
			tmp_point.distance = dis_tmp3+v_tmp3*(t-t3);
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t4 && t < t5)
		{
			tmp_point.jerk = -jerk;
			tmp_point.acc = -jerk*(t-t4);
			tmp_point.velocity = v_tmp4-0.5*jerk*(t-t4)*(t-t4);
			tmp_point.distance = dis_tmp4+v_tmp4*(t-t4)-jerk*(t-t4)*(t-t4)*(t-t4)/6;
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t5 && t < t6)
		{
			tmp_point.jerk = 0;
			tmp_point.acc = -jerk*time5;
			tmp_point.velocity = v_tmp5-jerk*time5*(t-t5);
			tmp_point.distance = dis_tmp5+v_tmp5*(t-t5)-0.5*jerk*time5*(t-t5)*(t-t5);
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t6 && t < t7)
		{
			tmp_point.jerk = jerk;
			tmp_point.acc = jerk*(-time5+t-t6);
			tmp_point.velocity = v_tmp6-jerk*time5*(t-t6)+0.5*jerk*(t-t6)*(t-t6);
			tmp_point.distance = dis_tmp6+v_tmp6*(t-t6)-0.5*jerk*time5*(t-t6)*(t-t6)+jerk*(t-t6)*(t-t6)*(t-t6)/6;
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}

		points_set.push_back(tmp_point); //Attention here!
		disp_set.push_back(tmp_dis);
	}
	// The last point of interpolator
	//tmp_point.acc = -dec;
	tmp_point.velocity = v_end;
	tmp_point.distance = distance;

	tmp_dis.x = p_end.x;
	tmp_dis.y = p_end.y;
	tmp_dis.z = p_end.z;
	tmp_dis.a = p_end.a;
	tmp_dis.b = p_end.b;
	tmp_dis.c = p_end.c;

	points_set.push_back(tmp_point); //Attention here!
	disp_set.push_back(tmp_dis);



	return 0;

}





int Circ_SMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_aux, robot_data_file_process::cartpos p_end,
		double vori_start, double vori_end, double vori_target,
		double accori, double decori,
		double jerkori,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set)
{
	double center_x = 0.0, center_y = 0.0, center_z = 0.0, radius = 0.0, alpha = 0.0, beta = 0.0, gama = 0.0, theta_distance;
	int ret = calculate_center_radius(p_start, p_aux, p_end, center_x, center_y, center_z, radius, alpha, beta, gama, theta_distance);
	if(ret < 0)
	{
		std::cout << "Error when calculate the circle's center and radius" << std::endl;
		return -1;
	}

	SMode_struct ret_value;
	ret = SMode_curve(theta_distance, vori_start, vori_end, vori_target, accori, decori, jerkori, ret_value);
	if(ret < 0)
	{
		return -1;
	}
	double time1 = ret_value.time1;
	double time2 = ret_value.time2;
	double time3 = ret_value.time3;
	double time4 = ret_value.time4;
	double time5 = ret_value.time5;
	double time6 = ret_value.time6;
	double time7 = ret_value.time7;
	vori_start = ret_value.v_start;
	vori_end = ret_value.v_end;

	//test TMode_curve()
	std::cout << "time1: " << time1 << std::endl;
	std::cout << "time2: " << time2 << std::endl;
	std::cout << "time3: " << time3 << std::endl;
	std::cout << "time4: " << time4 << std::endl;
	std::cout << "time5: " << time5 << std::endl;
	std::cout << "time6: " << time6 << std::endl;
	std::cout << "time7: " << time7 << std::endl;
	std::cout << "v_start: " << vori_start << std::endl;
	std::cout << "v_end: " << vori_end << std::endl;

	// step2. caculate the interpolation points 
	int num = ceil((time1+time2+time3+time4+time5+time6+time7)/cycle);
	double t1 = time1;
	double t2 = time1 + time2;
	double t3 = time1 + time2 +time3;
	double t4 = t3 + time4;
	double t5 = t4 + time5;
	double t6 = t5 + time6;
	double t7 = t6 + time7;
	double t = 0;



	double v_tmp1 = vori_start + 0.5*jerkori*time1*time1;
	double v_tmp2 = v_tmp1 + jerkori*time1*time2;
	double v_tmp3 = v_tmp2 + 0.5*jerkori*time3*time3;
	double v_tmp4 = v_tmp3;
	double v_tmp5 = v_tmp4 - 0.5*jerkori*time5*time5;
	double v_tmp6 = v_tmp5 - jerkori*time5*time6;
	double dis_tmp1 = vori_start*time1 + jerkori*time1*time1*time1/6;
	double dis_tmp2 = dis_tmp1 + v_tmp1*time2 + 0.5*jerkori*time1*time2*time2;
	double dis_tmp3 = dis_tmp2 + v_tmp2*time3 + 0.5*jerkori*time1*time3*time3 - jerkori*time3*time3*time3/6;
	double dis_tmp4 = dis_tmp3 + v_tmp3*time4;
	double dis_tmp5 = dis_tmp4 + v_tmp4*time5 - jerkori*time5*time5*time5/6;
	double dis_tmp6 = dis_tmp5 + v_tmp5*time6 - 0.5*jerkori*time5*time6*time6;
	double dis_tmp7 = dis_tmp6 + v_tmp6*time7 - 0.5*jerkori*time5*time7*time7 + jerkori*time7*time7*time7/6;

	double pso_x = p_start.x - center_x;
	double pso_y = p_start.y - center_y;
	double pso_z = p_start.z - center_z;

	double M11 = 0.0, M12 = 0.0, M13 = 0.0;
	double M21 = 0.0, M22 = 0.0, M23 = 0.0;
	double M31 = 0.0, M32 = 0.0, M33 = 0.0;

	double costheta = 0.0;
	double sintheta = 0.0;


	double k1 = alpha*alpha;
	double k2 = beta*beta;
	double k3 = gama*gama;
	double k12 = alpha*beta;
	double k13 = alpha*gama;
	double k23 = beta*gama;


	interpolation_point tmp_point; //Attention here!
	robot_data_file_process::cartpos tmp_dis;
	for(int i = 0; i < num-1; ++i)
	{
		t = (i+1)*cycle;
		if( t >= 0 && t < t1)
		{
			tmp_point.jerk = jerkori;
			tmp_point.acc = jerkori*t;
			tmp_point.velocity = vori_start+0.5*jerkori*t*t;
			tmp_point.distance = vori_start*t+jerkori*t*t*t/6;

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t1 && t < t2) 
		{
			tmp_point.jerk = 0;
			tmp_point.acc = jerkori*time1;
			tmp_point.velocity = v_tmp1+jerkori*time1*(t-t1);
			tmp_point.distance = dis_tmp1+v_tmp1*(t-t1)+0.5*jerkori*time1*(t-t1)*(t-t1);

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t2 && t < t3)
		{
			tmp_point.jerk = -jerkori;
			tmp_point.acc = jerkori*(time1 - t + t2);
			tmp_point.velocity = v_tmp2+jerkori*time1*(t-t2)-0.5*jerkori*(t-t2)*(t-t2);
			tmp_point.distance = dis_tmp2+v_tmp2*(t-t2)+0.5*jerkori*time1*(t-t2)*(t-t2)-jerkori*(t-t2)*(t-t2)*(t-t2)/6;

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t3 && t < t4)
		{
			tmp_point.jerk = 0;
			tmp_point.acc = 0;
			tmp_point.velocity = v_tmp3;
			tmp_point.distance = dis_tmp3+v_tmp3*(t-t3);

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t4 && t < t5)
		{
			tmp_point.jerk = -jerkori;
			tmp_point.acc = -jerkori*(t-t4);
			tmp_point.velocity = v_tmp4-0.5*jerkori*(t-t4)*(t-t4);
			tmp_point.distance = dis_tmp4+v_tmp4*(t-t4)-jerkori*(t-t4)*(t-t4)*(t-t4)/6;

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t5 && t < t6)
		{
			tmp_point.jerk = 0;
			tmp_point.acc = -jerkori*time5;
			tmp_point.velocity = v_tmp5-jerkori*time5*(t-t5);
			tmp_point.distance = dis_tmp5+v_tmp5*(t-t5)-0.5*jerkori*time5*(t-t5)*(t-t5);

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t6 && t < t7)
		{
			tmp_point.jerk = jerkori;
			tmp_point.acc = jerkori*(-time5+t-t6);
			tmp_point.velocity = v_tmp6-jerkori*time5*(t-t6)+0.5*jerkori*(t-t6)*(t-t6);
			tmp_point.distance = dis_tmp6+v_tmp6*(t-t6)-0.5*jerkori*time5*(t-t6)*(t-t6)+jerkori*(t-t6)*(t-t6)*(t-t6)/6;

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}

		points_set.push_back(tmp_point); //Attention here!
		disp_set.push_back(tmp_dis);
	}
	// The last point of interpolator
	//tmp_point.jerk = jerkori;
	//tmp_point.acc = -accori;
	tmp_point.velocity = vori_end;
	tmp_point.distance = theta_distance;

	tmp_dis.x = p_end.x;
	tmp_dis.y = p_end.y;
	tmp_dis.z = p_end.z;
	tmp_dis.a = p_end.a;
	tmp_dis.b = p_end.b;
	tmp_dis.c = p_end.c;

	points_set.push_back(tmp_point); //Attention here!
	disp_set.push_back(tmp_dis);

	return 0;

}








int PTP_TrigMode_interpolator(robot_data_file_process::axispos &p_start, robot_data_file_process::axispos &p_end,
		joint_velocity &v_start, joint_velocity &v_end, joint_velocity &v_target,
		joint_acc &acc, joint_acc &dec,
		double cycle,
		std::vector<robot_data_file_process::axispos> &points_set, std::vector<joint_velocity> &points_velocity, std::vector<joint_acc> &points_acc )
{
	int ret = 0;
	//joint a1
	double distance1 = p_end.a1 - p_start.a1 ;
	double flag1 = 1;
	if(distance1 < 0)
	{
		distance1 = -distance1;
		flag1 = -1;
	}

	TMode_struct ret_value_a1;
	ret = TrigMode_curve(distance1, v_start.v_a1, v_end.v_a1, v_target.v_a1, acc.a_a1, dec.a_a1, ret_value_a1);
	if(ret < 0)
	{
		std::cout << "joint a1 occurs error" << std::endl;
		return -1;
	}
	//joint a2
	double distance2 = p_end.a2 - p_start.a2 ;
	double flag2 = 1;
	if(distance2 < 0)
	{
		distance2 = -distance2;
		flag2 = -1;
	}
	TMode_struct ret_value_a2;
	ret = TrigMode_curve(distance2, v_start.v_a2, v_end.v_a2, v_target.v_a2, acc.a_a2, dec.a_a2, ret_value_a2);
	if(ret < 0)
	{
		std::cout << "joint a2 occurs error" << std::endl;
		return -1;
	}
	//joint a3
	double distance3 = p_end.a3 - p_start.a3 ;
	double flag3 = 1;
	if(distance3 < 0)
	{
		distance3 = -distance3;
		flag3 = -1;
	}
	TMode_struct ret_value_a3;
	ret = TrigMode_curve(distance3, v_start.v_a3, v_end.v_a3, v_target.v_a3, acc.a_a3, dec.a_a3, ret_value_a3);
	if(ret < 0)
	{
		std::cout << "joint a3 occurs error" << std::endl;
		return -1;
	}
	//joint a4
	double distance4 = p_end.a4 - p_start.a4 ;
	double flag4 = 1;
	if(distance4 < 0)
	{
		distance4 = -distance4;
		flag4 = -1;
	}
	TMode_struct ret_value_a4;
	ret = TrigMode_curve(distance4, v_start.v_a4, v_end.v_a4, v_target.v_a4, acc.a_a4, dec.a_a4, ret_value_a4);
	if(ret < 0)
	{
		std::cout << "joint a4 occurs error" << std::endl;
		return -1;
	}
	//joint a5
	double distance5 = p_end.a5 - p_start.a5 ;
	double flag5 = 1;
	if(distance5 < 0)
	{
		distance5 = -distance5;
		flag5 = -1;
	}
	TMode_struct ret_value_a5;
	ret = TrigMode_curve(distance5, v_start.v_a5, v_end.v_a5, v_target.v_a5, acc.a_a5, dec.a_a5, ret_value_a5);
	if(ret < 0)
	{
		std::cout << "joint a5 occurs error" << std::endl;
		return -1;
	}
	//joint a6
	double distance6 = p_end.a6 - p_start.a6 ;
	double flag6 = 1;
	if(distance6 < 0)
	{
		distance6 = -distance6;
		flag6 = -1;
	}
	TMode_struct ret_value_a6;
	ret = TrigMode_curve(distance6, v_start.v_a6, v_end.v_a6, v_target.v_a6, acc.a_a6, dec.a_a6, ret_value_a6);
	if(ret < 0)
	{
		std::cout << "joint a6 occurs error" << std::endl;
		return -1;
	}


	double time_a1 = ret_value_a1.time1 + ret_value_a1.time2 + ret_value_a1.time3;
	double time_a2 = ret_value_a2.time1 + ret_value_a2.time2 + ret_value_a2.time3;
	double time_a3 = ret_value_a3.time1 + ret_value_a3.time2 + ret_value_a3.time3;
	double time_a4 = ret_value_a4.time1 + ret_value_a4.time2 + ret_value_a4.time3;
	double time_a5 = ret_value_a5.time1 + ret_value_a5.time2 + ret_value_a5.time3;
	double time_a6 = ret_value_a6.time1 + ret_value_a6.time2 + ret_value_a6.time3;
	double time_max = 0.0;
	double k1 = 1.0;
	double k2 = 1.0;
	double k3 = 1.0;
	double k4 = 1.0;
	double k5 = 1.0;
	double k6 = 1.0;

	time_max = (time_a1 > time_a2) ? time_a1 : time_a2;
	time_max = (time_a3 > time_max) ? time_a3 : time_max;
	time_max = (time_a3 > time_max) ? time_a4 : time_max;
	time_max = (time_a3 > time_max) ? time_a5 : time_max;
	time_max = (time_a3 > time_max) ? time_a6 : time_max;

	k1 = time_max/time_a1;
	k2 = time_max/time_a2;
	k3 = time_max/time_a3;
	k4 = time_max/time_a4;
	k5 = time_max/time_a5;
	k6 = time_max/time_a6;

	// step2. caculate the interpolation points 

	ret_value_a1.time1 = ret_value_a1.time1*k1;
	ret_value_a1.time2 = ret_value_a1.time2*k1;
	ret_value_a1.time3 = ret_value_a1.time3*k1;
	double v_tmp1_a1 = (distance1-0.5*v_start.v_a1-0.5*v_end.v_a1)/(0.5*ret_value_a1.time1+ret_value_a1.time2+0.5*ret_value_a1.time3);
	double dis_tmp1_a1 = 0.5*(v_start.v_a1 + v_tmp1_a1)*ret_value_a1.time1;
	double dis_tmp2_a1 = dis_tmp1_a1 + v_tmp1_a1*ret_value_a1.time2;



	ret_value_a2.time1 = ret_value_a2.time1*k2;
	ret_value_a2.time2 = ret_value_a2.time2*k2;
	ret_value_a2.time3 = ret_value_a2.time3*k2;
	double v_tmp1_a2 = (distance2-0.5*v_start.v_a2-0.5*v_end.v_a2)/(0.5*ret_value_a2.time1+ret_value_a2.time2+0.5*ret_value_a2.time3);
	double dis_tmp1_a2 = 0.5*(v_start.v_a2 + v_tmp1_a2)*ret_value_a2.time1;
	double dis_tmp2_a2 = dis_tmp1_a2 + v_tmp1_a2*ret_value_a2.time2;


	ret_value_a3.time1 = ret_value_a3.time1*k3;
	ret_value_a3.time2 = ret_value_a3.time2*k3;
	ret_value_a3.time3 = ret_value_a3.time3*k3;
	double v_tmp1_a3 = (distance3-0.5*v_start.v_a3-0.5*v_end.v_a3)/(0.5*ret_value_a3.time1+ret_value_a3.time2+0.5*ret_value_a3.time3);
	double dis_tmp1_a3 = 0.5*(v_start.v_a3 + v_tmp1_a3)*ret_value_a3.time1;
	double dis_tmp2_a3 = dis_tmp1_a3 + v_tmp1_a3*ret_value_a3.time2;


	ret_value_a4.time1 = ret_value_a4.time1*k4;
	ret_value_a4.time2 = ret_value_a4.time2*k4;
	ret_value_a4.time3 = ret_value_a4.time3*k4;
	double v_tmp1_a4 = (distance4-0.5*v_start.v_a4-0.5*v_end.v_a4)/(0.5*ret_value_a4.time1+ret_value_a4.time2+0.5*ret_value_a4.time3);
	double dis_tmp1_a4 = 0.5*(v_start.v_a4 + v_tmp1_a4)*ret_value_a4.time1;
	double dis_tmp2_a4 = dis_tmp1_a4 + v_tmp1_a4*ret_value_a4.time2;


	ret_value_a5.time1 = ret_value_a5.time1*k5;
	ret_value_a5.time2 = ret_value_a5.time2*k5;
	ret_value_a5.time3 = ret_value_a5.time3*k5;
	double v_tmp1_a5 = (distance5-0.5*v_start.v_a5-0.5*v_end.v_a5)/(0.5*ret_value_a5.time1+ret_value_a5.time2+0.5*ret_value_a5.time3);
	double dis_tmp1_a5 = 0.5*(v_start.v_a5 + v_tmp1_a5)*ret_value_a5.time1;
	double dis_tmp2_a5 = dis_tmp1_a5 + v_tmp1_a5*ret_value_a5.time2;


	ret_value_a6.time1 = ret_value_a6.time1*k6;
	ret_value_a6.time2 = ret_value_a6.time2*k6;
	ret_value_a6.time3 = ret_value_a6.time3*k6;
	double v_tmp1_a6 = (distance6-0.5*v_start.v_a6-0.5*v_end.v_a6)/(0.5*ret_value_a6.time1+ret_value_a6.time2+0.5*ret_value_a6.time3);
	double dis_tmp1_a6 = 0.5*(v_start.v_a6 + v_tmp1_a6)*ret_value_a6.time1;
	double dis_tmp2_a6 = dis_tmp1_a6 + v_tmp1_a6*ret_value_a6.time2;


	int num = ceil((time_max)/cycle);
	double t1_a1 = ret_value_a1.time1;
	double t2_a1 = ret_value_a1.time1 + ret_value_a1.time2;
	double t3_a1 = ret_value_a1.time1 + ret_value_a1.time2 +ret_value_a1.time3;

	double t1_a2 = ret_value_a2.time1;
	double t2_a2 = ret_value_a2.time1 + ret_value_a2.time2;
	double t3_a2 = ret_value_a2.time1 + ret_value_a2.time2 +ret_value_a2.time3;

	double t1_a3 = ret_value_a3.time1;
	double t2_a3 = ret_value_a3.time1 + ret_value_a3.time2;
	double t3_a3 = ret_value_a3.time1 + ret_value_a3.time2 +ret_value_a3.time3;

	double t1_a4 = ret_value_a4.time1;
	double t2_a4 = ret_value_a4.time1 + ret_value_a4.time2;
	double t3_a4 = ret_value_a4.time1 + ret_value_a4.time2 +ret_value_a4.time3;

	double t1_a5 = ret_value_a5.time1;
	double t2_a5 = ret_value_a5.time1 + ret_value_a5.time2;
	double t3_a5 = ret_value_a5.time1 + ret_value_a5.time2 +ret_value_a5.time3;

	double t1_a6 = ret_value_a6.time1;
	double t2_a6 = ret_value_a6.time1 + ret_value_a6.time2;
	double t3_a6 = ret_value_a6.time1 + ret_value_a6.time2 +ret_value_a6.time3;
	double t = 0;


	robot_data_file_process::axispos tmp_point; //Attention here!
	joint_velocity tmp_velocity;
	joint_acc tmp_acc;

	for(int i = 0; i < num-1; ++i)
	{
		t = (i+1)*cycle;
		/***********************a1************************************/
		if( t >= 0 && t < t1_a1)
		{
			tmp_acc.a_a1 = PI*(v_tmp1_a1 - v_start.v_a1)*sin(t*PI/ret_value_a1.time1)/(2*ret_value_a1.time1);
			tmp_velocity.v_a1 = 0.5*(v_tmp1_a1+v_start.v_a1) - 0.5*(v_tmp1_a1 - v_start.v_a1)*cos(PI*t/ret_value_a1.time1);
			tmp_point.a1 = 0.5*(v_tmp1_a1+v_start.v_a1)*t - ret_value_a1.time1*(v_tmp1_a1 - v_start.v_a1)*sin(PI*t/ret_value_a1.time1)/(2*PI);
		}
		else if(t >= t1_a1 && t < t2_a1) 
		{
			tmp_acc.a_a1 = 0;
			tmp_velocity.v_a1 = v_tmp1_a1;
			tmp_point.a1 = dis_tmp1_a1 + v_tmp1_a1*(t - t1_a1);
		}
		else if(t >= t2_a1 && t < t3_a1)
		{
			tmp_acc.a_a1 = PI*(v_end.v_a1 - v_tmp1_a1)*sin(PI*(t-t2_a1)/ret_value_a1.time3)/(2*ret_value_a1.time3);
			tmp_velocity.v_a1 = 0.5*(v_end.v_a1+v_tmp1_a1) - 0.5*(v_end.v_a1 - v_tmp1_a1)*cos(PI*(t-t2_a1)/ret_value_a1.time3);
			tmp_point.a1 = dis_tmp2_a1 + 0.5*(v_end.v_a1+v_tmp1_a1)*(t-t2_a1) - ret_value_a1.time3*(v_end.v_a1-v_tmp1_a1)*sin(PI*(t-t2_a1)/ret_value_a1.time3)/(2*PI);

		}
		/***********************a2************************************/
		if( t >= 0 && t < t1_a2)
		{
			tmp_acc.a_a2 = PI*(v_tmp1_a2 - v_start.v_a2)*sin(t*PI/ret_value_a2.time1)/(2*ret_value_a2.time1);
			tmp_velocity.v_a2 = 0.5*(v_tmp1_a2+v_start.v_a2) - 0.5*(v_tmp1_a2 - v_start.v_a2)*cos(PI*t/ret_value_a2.time1);
			tmp_point.a2 = 0.5*(v_tmp1_a2+v_start.v_a2)*t - ret_value_a2.time1*(v_tmp1_a2 - v_start.v_a2)*sin(PI*t/ret_value_a2.time1)/(2*PI);
		}
		else if(t >= t1_a2 && t < t2_a2) 
		{
			tmp_acc.a_a2 = 0;
			tmp_velocity.v_a2 = v_tmp1_a2;
			tmp_point.a2 = dis_tmp1_a2 + v_tmp1_a2*(t - t1_a2);
		}
		else if(t >= t2_a2 && t < t3_a2)
		{
			tmp_acc.a_a2 = PI*(v_end.v_a2 - v_tmp1_a2)*sin(PI*(t-t2_a2)/ret_value_a2.time3)/(2*ret_value_a2.time3);
			tmp_velocity.v_a2 = 0.5*(v_end.v_a2+v_tmp1_a2) - 0.5*(v_end.v_a2 - v_tmp1_a2)*cos(PI*(t-t2_a2)/ret_value_a2.time3);
			tmp_point.a2 = dis_tmp2_a2 + 0.5*(v_end.v_a2+v_tmp1_a2)*(t-t2_a2) - ret_value_a2.time3*(v_end.v_a2-v_tmp1_a2)*sin(PI*(t-t2_a2)/ret_value_a2.time3)/(2*PI);

		}
		/***********************a3************************************/
		if( t >= 0 && t < t1_a3)
		{
			tmp_acc.a_a3 = PI*(v_tmp1_a3 - v_start.v_a3)*sin(t*PI/ret_value_a3.time1)/(2*ret_value_a3.time1);
			tmp_velocity.v_a3 = 0.5*(v_tmp1_a3+v_start.v_a3) - 0.5*(v_tmp1_a3 - v_start.v_a3)*cos(PI*t/ret_value_a3.time1);
			tmp_point.a3 = 0.5*(v_tmp1_a3+v_start.v_a3)*t - ret_value_a3.time1*(v_tmp1_a3 - v_start.v_a3)*sin(PI*t/ret_value_a3.time1)/(2*PI);
		}
		else if(t >= t1_a3 && t < t2_a3) 
		{
			tmp_acc.a_a3 = 0;
			tmp_velocity.v_a3 = v_tmp1_a3;
			tmp_point.a3 = dis_tmp1_a3 + v_tmp1_a3*(t - t1_a3);
		}
		else if(t >= t2_a3 && t < t3_a3)
		{
			tmp_acc.a_a3 = PI*(v_end.v_a3 - v_tmp1_a3)*sin(PI*(t-t2_a3)/ret_value_a3.time3)/(2*ret_value_a3.time3);
			tmp_velocity.v_a3 = 0.5*(v_end.v_a3+v_tmp1_a3) - 0.5*(v_end.v_a3 - v_tmp1_a3)*cos(PI*(t-t2_a3)/ret_value_a3.time3);
			tmp_point.a3 = dis_tmp2_a3 + 0.5*(v_end.v_a3+v_tmp1_a3)*(t-t2_a3) - ret_value_a3.time3*(v_end.v_a3-v_tmp1_a3)*sin(PI*(t-t2_a3)/ret_value_a3.time3)/(2*PI);

		}
		/***********************a4************************************/
		if( t >= 0 && t < t1_a4)
		{
			tmp_acc.a_a4 = PI*(v_tmp1_a4 - v_start.v_a4)*sin(t*PI/ret_value_a4.time1)/(2*ret_value_a4.time1);
			tmp_velocity.v_a4 = 0.5*(v_tmp1_a4+v_start.v_a4) - 0.5*(v_tmp1_a4 - v_start.v_a4)*cos(PI*t/ret_value_a4.time1);
			tmp_point.a4 = 0.5*(v_tmp1_a4+v_start.v_a4)*t - ret_value_a4.time1*(v_tmp1_a4 - v_start.v_a4)*sin(PI*t/ret_value_a4.time1)/(2*PI);
		}
		else if(t >= t1_a4 && t < t2_a4) 
		{
			tmp_acc.a_a4 = 0;
			tmp_velocity.v_a4 = v_tmp1_a4;
			tmp_point.a4 = dis_tmp1_a4 + v_tmp1_a4*(t - t1_a4);
		}
		else if(t >= t2_a4 && t < t3_a4)
		{
			tmp_acc.a_a4 = PI*(v_end.v_a4 - v_tmp1_a4)*sin(PI*(t-t2_a4)/ret_value_a4.time3)/(2*ret_value_a4.time3);
			tmp_velocity.v_a4 = 0.5*(v_end.v_a4+v_tmp1_a4) - 0.5*(v_end.v_a4 - v_tmp1_a4)*cos(PI*(t-t2_a4)/ret_value_a4.time3);
			tmp_point.a4 = dis_tmp2_a4 + 0.5*(v_end.v_a4+v_tmp1_a4)*(t-t2_a4) - ret_value_a4.time3*(v_end.v_a4-v_tmp1_a4)*sin(PI*(t-t2_a4)/ret_value_a4.time3)/(2*PI);

		}
		/**********************a5*************************************/
		if( t >= 0 && t < t1_a5)
		{
			tmp_acc.a_a5 = PI*(v_tmp1_a5 - v_start.v_a5)*sin(t*PI/ret_value_a5.time1)/(2*ret_value_a5.time1);
			tmp_velocity.v_a5 = 0.5*(v_tmp1_a5+v_start.v_a5) - 0.5*(v_tmp1_a5 - v_start.v_a5)*cos(PI*t/ret_value_a5.time1);
			tmp_point.a5 = 0.5*(v_tmp1_a5+v_start.v_a5)*t - ret_value_a5.time1*(v_tmp1_a5 - v_start.v_a5)*sin(PI*t/ret_value_a5.time1)/(2*PI);
		}
		else if(t >= t1_a5 && t < t2_a5) 
		{
			tmp_acc.a_a5 = 0;
			tmp_velocity.v_a5 = v_tmp1_a5;
			tmp_point.a5 = dis_tmp1_a5 + v_tmp1_a5*(t - t1_a5);
		}
		else if(t >= t2_a5 && t < t3_a5)
		{
			tmp_acc.a_a5 = PI*(v_end.v_a5 - v_tmp1_a5)*sin(PI*(t-t2_a5)/ret_value_a5.time3)/(2*ret_value_a5.time3);
			tmp_velocity.v_a5 = 0.5*(v_end.v_a5+v_tmp1_a5) - 0.5*(v_end.v_a5 - v_tmp1_a5)*cos(PI*(t-t2_a5)/ret_value_a5.time3);
			tmp_point.a5 = dis_tmp2_a5 + 0.5*(v_end.v_a5+v_tmp1_a5)*(t-t2_a5) - ret_value_a5.time3*(v_end.v_a5-v_tmp1_a5)*sin(PI*(t-t2_a5)/ret_value_a5.time3)/(2*PI);

		}
		/**********************a6*************************************/
		if( t >= 0 && t < t1_a6)
		{
			tmp_acc.a_a6 = PI*(v_tmp1_a6 - v_start.v_a6)*sin(t*PI/ret_value_a6.time1)/(2*ret_value_a6.time1);
			tmp_velocity.v_a6 = 0.5*(v_tmp1_a6+v_start.v_a6) - 0.5*(v_tmp1_a6 - v_start.v_a6)*cos(PI*t/ret_value_a6.time1);
			tmp_point.a6 = 0.5*(v_tmp1_a6+v_start.v_a6)*t - ret_value_a6.time1*(v_tmp1_a6 - v_start.v_a6)*sin(PI*t/ret_value_a6.time1)/(2*PI);
		}
		else if(t >= t1_a6 && t < t2_a6) 
		{
			tmp_acc.a_a6 = 0;
			tmp_velocity.v_a6 = v_tmp1_a6;
			tmp_point.a6 = dis_tmp1_a6 + v_tmp1_a6*(t - t1_a6);
		}
		else if(t >= t2_a6 && t < t3_a6)
		{
			tmp_acc.a_a6 = PI*(v_end.v_a6 - v_tmp1_a6)*sin(PI*(t-t2_a6)/ret_value_a6.time3)/(2*ret_value_a6.time3);
			tmp_velocity.v_a6 = 0.5*(v_end.v_a6+v_tmp1_a6) - 0.5*(v_end.v_a6 - v_tmp1_a6)*cos(PI*(t-t2_a6)/ret_value_a6.time3);
			tmp_point.a6 = dis_tmp2_a6 + 0.5*(v_end.v_a6+v_tmp1_a6)*(t-t2_a6) - ret_value_a6.time3*(v_end.v_a6-v_tmp1_a6)*sin(PI*(t-t2_a6)/ret_value_a6.time3)/(2*PI);

		}

		tmp_point.a1 = tmp_point.a1*flag1 + p_start.a1;
		tmp_point.a2 = tmp_point.a2*flag2 + p_start.a2;
		tmp_point.a3 = tmp_point.a3*flag3 + p_start.a3;
		tmp_point.a4 = tmp_point.a4*flag4 + p_start.a4;
		tmp_point.a5 = tmp_point.a5*flag5 + p_start.a5;
		tmp_point.a6 = tmp_point.a6*flag6 + p_start.a6;

		points_set.push_back(tmp_point); //Attention here!
		points_velocity.push_back(tmp_velocity);
		points_acc.push_back(tmp_acc);
	}

	//The last point of interpolator
	tmp_point.a1 = p_end.a1;
	tmp_point.a2 = p_end.a2;
	tmp_point.a3 = p_end.a3;
	tmp_point.a4 = p_end.a4;
	tmp_point.a5 = p_end.a5;
	tmp_point.a6 = p_end.a6;

	tmp_velocity.v_a1 = v_end.v_a1;
	tmp_velocity.v_a2 = v_end.v_a2;
	tmp_velocity.v_a3 = v_end.v_a3;
	tmp_velocity.v_a4 = v_end.v_a4;
	tmp_velocity.v_a5 = v_end.v_a5;
	tmp_velocity.v_a6 = v_end.v_a6;

	//tmp_acc.a_a1 = -dec.a_a1;
	//tmp_acc.a_a2 = -dec.a_a2;
	//tmp_acc.a_a3 = -dec.a_a3;
	//tmp_acc.a_a4 = -dec.a_a4;
	//tmp_acc.a_a5 = -dec.a_a5;
	//tmp_acc.a_a6 = -dec.a_a6;

	points_set.push_back(tmp_point); //Attention here!
	points_velocity.push_back(tmp_velocity);
	points_acc.push_back(tmp_acc);

	return 0;

}






int Lin_TrigMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_end,
		double v_start, double v_end, double v_target,
		double acc, double dec,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set)
{
	// step1. caculate time1, time2, time2, maybe should reset v_start, v_end
	double distance = sqrt((p_start.x - p_end.x)*(p_start.x - p_end.x) + (p_start.y - p_end.y)*(p_start.y - p_end.y) + (p_start.z - p_end.z)*(p_start.z - p_end.z));
	TMode_struct ret_value;
	int ret = TrigMode_curve(distance, v_start, v_end, v_target, acc, dec, ret_value);
	if(ret < 0)
	{
		return -1;
	}

	double time1 = ret_value.time1;
	double time2 = ret_value.time2;
	double time3 = ret_value.time3;
	v_start = ret_value.v_start;
	v_end = ret_value.v_end;

	//test TrigMode_curve()
	std::cout << "time1: " << time1 << std::endl;
	std::cout << "time2: " << time2 << std::endl;
	std::cout << "time3: " << time3 << std::endl;
	std::cout << "v_start: " << v_start << std::endl;
	std::cout << "v_end: " << v_end << std::endl;

	double kx = (p_start.x - p_end.x)/distance;
	double ky = (p_start.y - p_end.y)/distance;
	double kz = (p_start.z - p_end.z)/distance;

	// step2. caculate the interpolation points 
	int num = ceil((time1+time2+time3)/cycle);
	double t1 = time1;
	double t2 = time1 + time2;
	double t3 = time1 + time2 +time3;
	double t = 0;


	double dis_tmp1 = 0.5*(v_target+v_start)*time1;
	double dis_tmp2 = dis_tmp1 + v_target*time2;

	interpolation_point tmp_point; //Attention here!
	robot_data_file_process::cartpos tmp_dis;
	for(int i = 0; i < num-1; ++i)
	{
		t = (i+1)*cycle;
		if( t >= 0 && t < t1)
		{
			tmp_point.acc = PI*(v_target - v_start)*sin(t*PI/time1)/(2*time1);
			tmp_point.velocity = 0.5*(v_target+v_start) - 0.5*(v_target - v_start)*cos(PI*t/time1);
			tmp_point.distance = 0.5*(v_target+v_start)*t - time1*(v_target - v_start)*sin(PI*t/time1)/(2*PI);
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t1 && t < t2) 
		{
			tmp_point.acc = 0;
			tmp_point.velocity = v_target;
			tmp_point.distance = dis_tmp1 + v_target*(t - t1);
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}
		else if(t >= t2 && t < t3)
		{
			tmp_point.acc = PI*(v_end - v_target)*sin(PI*(t-t2)/time3)/(2*time3);
			tmp_point.velocity = 0.5*(v_end+v_target) - 0.5*(v_end - v_target)*cos(PI*(t-t2)/time3);
			tmp_point.distance = dis_tmp2 + 0.5*(v_end+v_target)*(t-t2) - time3*(v_end-v_target)*sin(PI*(t-t2)/time3)/(2*PI);
			tmp_dis.x = p_start.x + tmp_point.distance * kx;
			tmp_dis.y = p_start.y + tmp_point.distance * ky;
			tmp_dis.z = p_start.z + tmp_point.distance * kz;
			tmp_dis.a = p_start.a + (tmp_point.distance/distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/distance)*(p_end.c - p_start.c);
		}

		points_set.push_back(tmp_point); //Attention here!
		disp_set.push_back(tmp_dis);
	}
	// The last point of interpolator
	tmp_point.acc = -dec;
	tmp_point.velocity = v_end;
	tmp_point.distance = distance;

	tmp_dis.x = p_end.x;
	tmp_dis.y = p_end.y;
	tmp_dis.z = p_end.z;
	tmp_dis.a = p_end.a;
	tmp_dis.b = p_end.b;
	tmp_dis.c = p_end.c;

	points_set.push_back(tmp_point); //Attention here!
	disp_set.push_back(tmp_dis);



	return 0;

}






int Circ_TrigMode_interpolator(
		robot_data_file_process::cartpos p_start, robot_data_file_process::cartpos p_aux, robot_data_file_process::cartpos p_end,
		double vori_start, double vori_end, double vori_target,
		double accori, double decori,
		double cycle, 
		std::vector<interpolation_point> &points_set, std::vector<robot_data_file_process::cartpos> &disp_set)
{
	double center_x = 0.0, center_y = 0.0, center_z = 0.0, radius = 0.0, alpha = 0.0, beta = 0.0, gama = 0.0, theta_distance;
	int ret = calculate_center_radius(p_start, p_aux, p_end, center_x, center_y, center_z, radius, alpha, beta, gama, theta_distance);
	if(ret < 0)
	{
		std::cout << "Error when calculate the circle's center and radius" << std::endl;
		return -1;
	}

	TMode_struct ret_value;
	ret = TrigMode_curve(theta_distance, vori_start, vori_end, vori_target, accori, decori, ret_value);
	if(ret < 0)
	{
		return -1;
	}
	int time1 = ret_value.time1;
	int time2 = ret_value.time2;
	int time3 = ret_value.time3;
	vori_start = ret_value.v_start;
	vori_end = ret_value.v_end;

	//test TMode_curve()
	std::cout << "time1: " << time1 << std::endl;
	std::cout << "time2: " << time2 << std::endl;
	std::cout << "time3: " << time3 << std::endl;
	std::cout << "v_start: " << vori_start << std::endl;
	std::cout << "v_end: " << vori_end << std::endl;

	// step2. caculate the interpolation points 
	int num = ceil((time1+time2+time3)/cycle);
	double t1 = time1;
	double t2 = time1 + time2;
	double t3 = time1 + time2 +time3;
	double t = 0;



	double dis_tmp1 = 0.5*(vori_target+vori_start)*time1;
	double dis_tmp2 = dis_tmp1 + vori_target*time2;

	double pso_x = p_start.x - center_x;
	double pso_y = p_start.y - center_y;
	double pso_z = p_start.z - center_z;

	double M11 = 0.0, M12 = 0.0, M13 = 0.0;
	double M21 = 0.0, M22 = 0.0, M23 = 0.0;
	double M31 = 0.0, M32 = 0.0, M33 = 0.0;

	double costheta = 0.0;
	double sintheta = 0.0;


	double k1 = alpha*alpha;
	double k2 = beta*beta;
	double k3 = gama*gama;
	double k12 = alpha*beta;
	double k13 = alpha*gama;
	double k23 = beta*gama;


	interpolation_point tmp_point; //Attention here!
	robot_data_file_process::cartpos tmp_dis;
	for(int i = 0; i < num-1; ++i)
	{
		t = (i+1)*cycle;
		if( t >= 0 && t < t1)
		{
			tmp_point.acc = PI*(vori_target - vori_start)*sin(t*PI/time1)/(2*time1);
			tmp_point.velocity = 0.5*(vori_target+vori_start) - 0.5*(vori_target - vori_start)*cos(PI*t/time1);
			tmp_point.distance = 0.5*(vori_target+vori_start)*t - time1*(vori_target - vori_start)*sin(PI*t/time1)/(2*PI);

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t1 && t < t2) 
		{
			tmp_point.acc = 0;
			tmp_point.velocity = vori_target;
			tmp_point.distance = dis_tmp1 + vori_target*(t - t1);

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}
		else if(t >= t2 && t < t3)
		{
			tmp_point.acc = PI*(vori_end - vori_target)*sin(PI*(t-t2)/time3)/(2*time3);
			tmp_point.velocity = 0.5*(vori_end+vori_target) - 0.5*(vori_end - vori_target)*cos(PI*(t-t2)/time3);
			tmp_point.distance = dis_tmp2 + 0.5*(vori_end+vori_target)*(t-t2) - time3*(vori_end-vori_target)*sin(PI*(t-t2)/time3)/(2*PI);

			costheta = cos(tmp_point.distance);
			sintheta = sin(tmp_point.distance);

			M11 = costheta*(1-k1) + k1;
			M12 = costheta*(-k13) + sintheta*(-gama) + k13;
			M13 = costheta*(-k13) + sintheta*beta + k13;
			M21 = costheta*(-k12) + sintheta*gama + k12;
			M22 = costheta*(1-k2) + k2;
			M23 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M31 = costheta*(-k13) + sintheta*(-beta) + k13;
			M32 = costheta*(-k23) + sintheta*(-alpha) + k23;
			M33 = costheta*(1-k3) + k3;

			tmp_dis.x = center_x + M11*pso_x + M12*pso_y + M13*pso_z;
			tmp_dis.y = center_y + M21*pso_x + M22*pso_y + M23*pso_z;
			tmp_dis.z = center_z + M31*pso_x + M32*pso_y + M33*pso_z;
			tmp_dis.a = p_start.a + (tmp_point.distance/theta_distance)*(p_end.a - p_start.a);
			tmp_dis.b = p_start.b + (tmp_point.distance/theta_distance)*(p_end.b - p_start.b);
			tmp_dis.c = p_start.c + (tmp_point.distance/theta_distance)*(p_end.c - p_start.c);
		}

		points_set.push_back(tmp_point); //Attention here!
		disp_set.push_back(tmp_dis);
	}
	// The last point of interpolator
	tmp_point.acc = -decori;
	tmp_point.velocity = vori_end;
	tmp_point.distance = theta_distance;

	tmp_dis.x = p_end.x;
	tmp_dis.y = p_end.y;
	tmp_dis.z = p_end.z;
	tmp_dis.a = p_end.a;
	tmp_dis.b = p_end.b;
	tmp_dis.c = p_end.c;

	points_set.push_back(tmp_point); //Attention here!
	disp_set.push_back(tmp_dis);

	return 0;

}
