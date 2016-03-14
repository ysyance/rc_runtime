#ifndef _VELOCITY_CURVE_H_
#define _VELOCITY_CURVE_H_

#define PI 3.14159265358979323846


struct TMode_struct
{
	public:
		double time1;
		double time2;
		double time3;
		double v_start;
		double v_end;

	public:
		TMode_struct():time1(0.0),time2(0.0),time3(0.0),v_start(0.0),v_end(0.0) {}
		~TMode_struct() {}
};

struct SMode_struct
{
	public:
		double time1;
		double time2;
		double time3;
		double time4;
		double time5;
		double time6;
		double time7;
		double v_start;
		double v_end;

	public:
		SMode_struct():time1(0.0),time2(0.0),time3(0.0),time4(0.0),time5(0.0),time6(0.0),time7(0.0),v_start(0.0),v_end(0.0) {}
		~SMode_struct() {}
};


int TMode_curve(
		double distance,
		double v_start, double v_end, double v_target,
		double acc, double dec, 
		TMode_struct &ret_value);
int SMode_curve(
		double distance,
		double v_start, double v_end, double v_target,
		double acc, double dec, 
		double jerk,
		SMode_struct &ret_value);
int TrigMode_curve(
		double distance,
		double v_start, double v_end, double v_target,
		double acc, double dec, 
		TMode_struct &ret_value);




#endif
