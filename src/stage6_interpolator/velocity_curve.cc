#include <iostream>
#include <cmath>

#include "velocity_curve.hh"

int TMode_curve(
		double distance,
		double v_start, double v_end, double v_target,
		double acc, double dec, 
		TMode_struct &ret_value)
{
	// step1. caculate time1, time2, time2, maybe should reset v_start, v_end

	double time1 = 0, time2 = 0, time3 = 0;
	double distance_acc = 0, distance_dec = 0;
	if(v_start >= 0 && v_end >= 0 && v_start < v_target && v_end < v_target)
	{
		time1 = (v_target - v_start)/acc;
		time3 = (v_target- v_end)/dec;

		distance_acc = 0.5*((v_target*v_target - v_start*v_start)/acc);
		distance_dec = 0.5*((v_target*v_target - v_end*v_end)/dec);

		if(distance >= (distance_dec + distance_acc))
		{
			time2 = (distance - distance_acc - distance_dec)/v_target;
		}
		else
		{
			v_target = sqrt((2*acc*dec*distance + dec*v_start*v_start + acc*v_end*v_end)/(acc+dec));
			if(v_start >= 0 && v_end >= 0 && v_start < v_target && v_end < v_target)
			{
				time1 = (v_target - v_start)/acc;
				time2 = 0;
				time3 = (v_target - v_end)/dec;
			}
			else if(v_start > v_end)
			{
				time1 = 0;
				v_target = v_start;
				time3 = (v_target - v_end)/dec;
				distance_dec = (v_target*v_target - v_end*v_end)/(2*dec);
				if(distance_dec <= distance)
				{
					time2 = (distance - distance_dec)/v_target;
				}
				else
				{
					v_target = v_start = sqrt(v_end*v_end + 2*dec*distance);
					std::cout << "should reset v_start velocity" << std::endl;
					time2 = 0;
					time3 = (v_target - v_end)/dec;
				}
			}
			else if(v_start == v_end)
			{
				if(v_start != 0)
				{
					time1 = time3 = 0;
					v_target = v_start;
					time2 = distance/v_target;
				}
				else
				{
					time2 = 0;
					v_target = sqrt(2*acc*dec*distance/(acc+dec));
					time1 = v_target/acc;
					time3 = v_target/dec;
				}
			}
			else
			{
				time3 = 0;
				v_target = v_end;
				time1 = (v_target - v_start)/acc;
				distance_acc = (v_target*v_target - v_start*v_start)/(2*acc);
				if(distance_acc <= distance)
				{
					time2 = (distance - distance_acc)/v_target;
				}
				else
				{
					std::cout << "should reset v_end" << std::endl;
					v_target = v_end = sqrt(v_start*v_start + 2*acc*distance);
					time2 = 0;
					time1 = (v_target - v_start)/acc;
				}
			}

		}

	}
	else if(v_start == v_target && v_end >= 0 && v_end < v_target)
	{

		time1 = 0;
		v_target = v_start;
		time3 = (v_target - v_end)/dec;
		distance_dec = (v_target*v_target - v_end*v_end)/(2*dec);
		if(distance_dec <= distance)
		{
			time2 = (distance - distance_dec)/v_target;
		}
		else
		{
			v_target = v_start = sqrt(v_end*v_end + 2*dec*distance);
			std::cout << "should reset v_start velocity" << std::endl;
			time2 = 0;
			time3 = (v_target - v_end)/dec;
		}
	}
	else if(v_start == v_end && v_start == v_target)
	{

		if(v_start != 0)
		{
			time1 = time3 = 0;
			v_target = v_start;
			time2 = distance/v_target;
		}
		else
		{
			time2 = 0;
			v_target = sqrt(2*acc*dec*distance/(acc+dec));
			time1 = v_target/acc;
			time3 = v_target/dec;
		}
	}
	else if(v_end == v_target && v_start >= 0 && v_start < v_target)
	{

		time3 = 0;
		v_target = v_end;
		time1 = (v_target - v_start)/acc;
		distance_acc = (v_target*v_target - v_start*v_start)/(2*acc);
		if(distance_acc <= distance)
		{
			time2 = (distance - distance_acc)/v_target;
		}
		else
		{
			std::cout << "should reset v_end" << std::endl;
			v_target = v_end = sqrt(v_start*v_start + 2*acc*distance);
			time2 = 0;
			time1 = (v_target - v_start)/acc;
		}
	}
	else
	{
		std::cout << " error input" << std::endl;
		return -1;
	}

	ret_value.time1 = time1;
	ret_value.time2 = time2;
	ret_value.time3 = time3;
	ret_value.v_start = v_start;
	ret_value.v_end = v_end;

	return 0;
}




int SMode_curve(
		double distance,
		double v_start, double v_end, double v_target,
		double acc, double dec, 
		double jerk,
		SMode_struct &ret_value)
{
	double time_m = acc/jerk;
	double time1 = 0.0, time2 = 0.0, time3 = 0.0, time4 = 0.0, time5 = 0.0, time6 = 0.0, time7 = 0.0;
	double distance_acc = 0, distance_dec = 0;

	if(v_start >= 0 && v_end >= 0 && v_start < v_target && v_end < v_target)
	{
		//part 01
		if((v_target - v_start) > acc*time_m && (v_target - v_end) > acc*time_m)
		{
			//part 0101
			time1 = time3 = time5 = time7 = time_m;
			time2 = (v_target - v_start)/acc - time_m;
			time6 = (v_target - v_end)/acc - time_m;

			distance_acc = 0.5*(v_target+v_start)*(time_m + (v_target - v_start)/acc);
			distance_dec = 0.5*(v_target+v_end)*(time_m + (v_target - v_end)/acc);
		}
		else if((v_target - v_start) > acc*time_m && (v_target - v_end) <= acc*time_m)
		{
			//part 0102
			time1 = time3 = time_m;
			time2 = (v_target - v_start)/acc - time_m;
			time6 = 0;
			time5 = time7 = sqrt((v_target - v_end)/jerk);

			distance_acc = 0.5*(v_target + v_start)*(time_m+(v_target-v_start)/acc);
			distance_dec = 2*v_target*time5 - jerk*time5*time5*time5;
		}
		else if((v_target - v_start) <= acc*time_m && (v_target - v_end) > acc*time_m)
		{
			//part 0103
			time2 = 0;
			time1 = time3 = sqrt((v_target - v_start)/jerk);
			time5 = time7 = time_m;
			time6 = (v_target - v_end)/acc - time_m;

			distance_acc = 2*v_start*time1 + jerk*time1*time1*time1;
			distance_dec = 0.5*(v_target+v_end)*(time_m + (v_target-v_end)/acc);
		}
		else if((v_target - v_start) <= acc*time_m && (v_target - v_end) <= acc*time_m)
		{
			//part 0104
			time1 = time3 = sqrt((v_target - v_start)/jerk);
			time2 = 0;
			time5 = time7 = sqrt((v_target - v_end)/jerk);
			time6 = 0;

			distance_acc = 2*v_start*time1 + jerk*time1*time1*time1;
			distance_dec = 2*v_target*time5 - jerk*time5*time5*time5;
		}
		else
		{
			//part 0105
			std::cout << " error input" << std::endl;
			return -1;
		}
		if( distance >= (distance_acc+distance_dec) )
		{
			//part 010101
			time4 = (distance - (distance_acc+distance_dec))/v_target;
		}
		else if(v_start > v_end)
		{
			//part 010102
			time1 = time2 = time3 = 0;
			v_target = v_start;
			if(v_target - v_end > acc*time_m)
			{
				//part 01010201
				time5 = time7 = time_m;
				time6 = (v_target - v_end)/acc - time_m;
				distance_dec = 0.5*(v_target + v_end)*(time_m + (v_target - v_end)/acc);
			}
			else
			{
				//part 01010202
				time6 = 0;
				time5 = time7 = sqrt((v_target - v_start)/jerk);
				distance_dec = 2*v_target*time5 - jerk*time5*time5*time5;
			}
			if(distance > distance_dec)
			{
				//part 01010203
				time4 = (distance - distance_dec)/v_target;
			}
			else
			{
				//part 01010204
				time4 = time6 = 0;
				time5 = time7 = time_m;
				double distance567 = 2*v_end*time_m + acc*time_m*time_m;
				if(distance > distance567)
				{
					std::cout << "should reset v_start" << std::endl;
					time6 = sqrt( (1.5*acc+v_end)*(1.5*acc+v_end) + 2*acc*(distance-2*v_end*time_m-acc*time_m*time_m) - (1.5*acc*time_m + v_end) )/acc;
					v_start = v_target = v_end+acc*(time5+time6);
				}
				else
				{
					std::cout << "should reset v_start" << std::endl;
					double k = distance/(2*jerk);
					double delta = sqrt(k*k + (2*v_end/(3*jerk))*(2*v_end/(3*jerk))*(2*v_end/(3*jerk)));
					time5 = time7 = pow(k+delta, 1.0/3) + pow(k-delta, 1.0/3);
					v_start = v_target = v_end+jerk*time5*time5;
				}
			}
		}
		else if(v_start == v_end)
		{
			//part 010103
			if(v_start != 0)
			{
				time1 = time2 = time3 = time5 = time6 = time7 = 0;
				v_target = v_start;
				time4 = distance/v_target;
			}
			else
			{
				time2 = time6 = 0;
				time1 = time3 = time5 = time7 = time_m;
				if(distance >= 2*acc*time_m*time_m)
				{
					v_target = acc*time_m;
					time4 = (distance - 2*acc*time_m*time_m)/time_m;
				}
				else
				{
					time4 = 0;
					time1 = time3 = time5 = time7 = sqrt(distance/(2*acc));
				}
			}
		}
		else
		{
			//part 010104
			time5 = time6 = time7 = 0;
			v_target = v_end;
			if((v_target - v_start) > acc*time_m)
			{
				time1 = time3 = time_m;
				time2 = (v_target - v_start)/acc - time_m;
				distance_acc = 0.5*(v_target+v_start)*(time_m + (v_target - v_start)/acc);
			}
			else
			{
				time2 = 0;
				time1 = time3 = sqrt((v_target - v_start)/jerk);
				distance_acc = 2*v_start*time1 + jerk*time1*time1*time1;
			}

			if(distance >= distance_acc)
			{
				time4 = (distance - distance_acc)/v_target;
			}
			else
			{
				time2 = time4 = 0;
				time1 = time3 = time_m;
				double distance123 = 2*v_start*time_m + acc*time_m*time_m;
				if(distance > distance123)
				{
					std::cout << "should reset v_start" << std::endl;
					time6 = sqrt( (1.5*acc+v_start)*(1.5*acc+v_start) + 2*acc*(distance-2*v_start*time_m-acc*time_m*time_m) - (1.5*acc*time_m + v_start) )/acc;
					v_end = v_target = v_start+acc*(time1+time2);
				}
				else
				{
					std::cout << "should reset v_start" << std::endl;
					double k = distance/(2*jerk);
					double delta = sqrt(k*k + (2*v_start/(3*jerk))*(2*v_start/(3*jerk))*(2*v_start/(3*jerk)));
					time1 = time3 = pow(k+delta, 1.0/3) + pow(k-delta, 1.0/3);
					v_end = v_target = v_start+jerk*time1*time1;
				}
			}
		}
	}
	else if(v_start == v_target && v_end >= 0 && v_end < v_target)
	{
		//part 02
		//part 010102
		time1 = time2 = time3 = 0;
		v_target = v_start;
		if(v_target - v_end > acc*time_m)
		{
			//part 01010201
			time5 = time7 = time_m;
			time6 = (v_target - v_end)/acc - time_m;
			distance_dec = 0.5*(v_target + v_end)*(time_m + (v_target - v_end)/acc);
		}
		else
		{
			//part 01010202
			time6 = 0;
			time5 = time7 = sqrt((v_target - v_start)/jerk);
			distance_dec = 2*v_target*time5 - jerk*time5*time5*time5;
		}
		if(distance > distance_dec)
		{
			//part 01010203
			time4 = (distance - distance_dec)/v_target;
		}
		else
		{
			//part 01010204
			time4 = time6 = 0;
			time5 = time7 = time_m;
			double distance567 = 2*v_end*time_m + acc*time_m*time_m;
			if(distance > distance567)
			{
				std::cout << "should reset v_start" << std::endl;
				time6 = sqrt( (1.5*acc+v_end)*(1.5*acc+v_end) + 2*acc*(distance-2*v_end*time_m-acc*time_m*time_m) - (1.5*acc*time_m + v_end) )/acc;
				v_start = v_target = v_end+acc*(time5+time6);
			}
			else
			{
				std::cout << "should reset v_start" << std::endl;
				double k = distance/(2*jerk);
				double delta = sqrt(k*k + (2*v_end/(3*jerk))*(2*v_end/(3*jerk))*(2*v_end/(3*jerk)));
				time5 = time7 = pow(k+delta, 1.0/3) + pow(k-delta, 1.0/3);
				v_start = v_target = v_end+jerk*time5*time5;
			}
		}

	}
	else if(v_start == v_end && v_start == v_target)
	{
		//part 03
		//part 010103
		if(v_start != 0)
		{
			time1 = time2 = time3 = time5 = time6 = time7 = 0;
			v_target = v_start;
			time4 = distance/v_target;
		}
		else
		{
			time2 = time6 = 0;
			time1 = time3 = time5 = time7 = time_m;
			if(distance >= 2*acc*time_m*time_m)
			{
				v_target = acc*time_m;
				time4 = (distance - 2*acc*time_m*time_m)/time_m;
			}
			else
			{
				time4 = 0;
				time1 = time3 = time5 = time7 = sqrt(distance/(2*acc));
			}
		}

	}
	else if(v_end == v_target && v_start >= 0 && v_start < v_target)
	{
		//part 04
		//part 010104
		time5 = time6 = time7 = 0;
		v_target = v_end;
		if((v_target - v_start) > acc*time_m)
		{
			time1 = time3 = time_m;
			time2 = (v_target - v_start)/acc - time_m;
			distance_acc = 0.5*(v_target+v_start)*(time_m + (v_target - v_start)/acc);
		}
		else
		{
			time2 = 0;
			time1 = time3 = sqrt((v_target - v_start)/jerk);
			distance_acc = 2*v_start*time1 + jerk*time1*time1*time1;
		}

		if(distance >= distance_acc)
		{
			time4 = (distance - distance_acc)/v_target;
		}
		else
		{
			time2 = time4 = 0;
			time1 = time3 = time_m;
			double distance123 = 2*v_start*time_m + acc*time_m*time_m;
			if(distance > distance123)
			{
				std::cout << "should reset v_start" << std::endl;
				time6 = sqrt( (1.5*acc+v_start)*(1.5*acc+v_start) + 2*acc*(distance-2*v_start*time_m-acc*time_m*time_m) - (1.5*acc*time_m + v_start) )/acc;
				v_end = v_target = v_start+acc*(time1+time2);
			}
			else
			{
				std::cout << "should reset v_start" << std::endl;
				double k = distance/(2*jerk);
				double delta = sqrt(k*k + (2*v_start/(3*jerk))*(2*v_start/(3*jerk))*(2*v_start/(3*jerk)));
				time1 = time3 = pow(k+delta, 1.0/3) + pow(k-delta, 1.0/3);
				v_end = v_target = v_start+jerk*time1*time1;
			}
		}

	}
	else
	{
		//part 05
		std::cout << " error input" << std::endl;
		return -1;
	}

	ret_value.time1 = time1;
	ret_value.time2 = time2;
	ret_value.time3 = time3;
	ret_value.time4 = time4;
	ret_value.time5 = time5;
	ret_value.time6 = time6;
	ret_value.time7 = time7;
	ret_value.v_start = v_start;
	ret_value.v_end = v_end;

        return 0;

}










int TrigMode_curve(
		double distance,
		double v_start, double v_end, double v_target,
		double acc, double dec, 
		TMode_struct &ret_value)
{
	// step1. caculate time1, time2, time2, maybe should reset v_start, v_end

	double time1 = 0, time2 = 0, time3 = 0;
	double distance_acc = 0, distance_dec = 0;
	if(v_start >= 0 && v_end >= 0 && v_start < v_target && v_end < v_target)
	{
		//part 01
		time1 = PI*(v_target - v_start)/(2*acc);
		time3 = PI*(v_target-v_end)/(2*acc);

		distance_acc = 0.25*PI*(v_target*v_target-v_start*v_start)/acc;
		distance_dec = 0.25*PI*(v_target*v_target-v_end*v_end)/acc;

		if(distance >= (distance_acc+distance_dec))
		{
			time2 = (distance - distance_acc - distance_dec)/v_target;
		}
		else
		{
			v_target = sqrt(0.5*(v_start*v_start+v_end*v_end+4*acc*distance/PI));
			if(v_start >= 0 && v_end >= 0 && v_start < v_target && v_end < v_target)
			{
				time1 = PI*(v_target - v_start)/(2*acc);
				time2 = 0;
				time3 = PI*(v_target - v_end)/(2*acc);
			}
			else if(v_start > v_end)
			{
				time1 = 0;
				v_target = v_start;
				time3 = PI*(v_target - v_end)/(2*acc);
				distance_dec = 0.25*PI*(v_target*v_target - v_end*v_end)/acc;

				if(distance >= distance_dec)
				{
					time2 = (distance - distance_dec)/v_target;
				}
				else
				{
					std::cout << "should reset v_start" << std::endl;
					v_start = v_target = sqrt(v_end*v_end + 4*acc*distance/PI);
					time2 = 0;
					time3 = PI*(v_target - v_end)/(2*acc);
				}
			}
			else if(v_start == v_end)
			{
				if(v_start != 0)
				{
					time1 = time3 = 0;
					v_target = v_start;
					time2 = distance/v_target;
				}
				else
				{
					time2 = 0;
					v_target = sqrt(2*acc*distance/PI);
					time1 = time3 = sqrt(PI*distance/(2*acc));
				}
			}
			else
			{
				time3 = 0;
				v_target = v_end;
				time1 = PI*(v_target - v_start)/(2*acc);
				distance_acc = 0.25*PI*(v_target*v_target- v_start*v_start)/acc;

				if(distance >= distance_acc)
				{
					time2 = (distance - distance_acc)/v_target;
				}
				else
				{
					std::cout << "should reset v_end" << std::endl;
					v_end = v_target = sqrt(v_start*v_start + 4*acc*distance/PI);
					time1 = PI*(v_target - v_start)/(2*acc);
					time2 = 0;
				}
			}
		}
	}
	else if(v_start == v_target && v_end >= 0 && v_end < v_target)
	{
		//part 02
		time1 = 0;
		v_target = v_start;
		time3 = PI*(v_target - v_end)/(2*acc);
		distance_dec = 0.25*PI*(v_target*v_target - v_end*v_end)/acc;

		if(distance >= distance_dec)
		{
			time2 = (distance - distance_dec)/v_target;
		}
		else
		{
			std::cout << "should reset v_start" << std::endl;
			v_start = v_target = sqrt(v_end*v_end + 4*acc*distance/PI);
			time2 = 0;
			time3 = PI*(v_target - v_end)/(2*acc);
		}
	}
	else if(v_start == v_end && v_start == v_target)
	{
		//part 03
		if(v_start != 0)
		{
			time1 = time3 = 0;
			v_target = v_start;
			time2 = distance/v_target;
		}
		else
		{
			time2 = 0;
			v_target = sqrt(2*acc*distance/PI);
			time1 = time3 = sqrt(PI*distance/(2*acc));
		}
	}
	else if(v_target == v_end && v_start >= 0 && v_start < v_target)
	{
		//part 04
		time3 = 0;
		v_target = v_end;
		time1 = PI*(v_target - v_start)/(2*acc);
		distance_acc = 0.25*PI*(v_target*v_target - v_start*v_start)/acc;

		if(distance >= distance_acc)
		{
			time2 = (distance - distance_acc)/v_target;
		}
		else
		{
			std::cout << "should reset v_end" << std::endl;
			v_end = v_target = sqrt(v_start*v_start + 4*acc*distance/PI);
			time1 = PI*(v_target - v_start)/(2*acc);
			time2 = 0;
		}
	}
	else
	{
		//part 05
		std::cout << " error input" << std::endl;
		return -1;
	}

	ret_value.time1 = time1;
	ret_value.time2 = time2;
	ret_value.time3 = time3;
	ret_value.v_start = v_start;
	ret_value.v_end = v_end;

        return 0;
}


