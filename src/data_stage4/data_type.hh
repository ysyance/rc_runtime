

#ifndef DATA_TYPE_H_
#define DATA_TYPE_H_

#include <iostream>

namespace robot_data_file_process{

// Part 01-- Position data structure
struct axispos {
	public:
		double a1;
		double a2;
		double a3;
		double a4;
		double a5;
		double a6;

		double aux1;
		double aux2;
		double aux3;
		double aux4;
		double aux5;
		double aux6;
	public:
                axispos():a1(0.0),a2(0.0),a3(0.0),a4(0.0),a5(0.0),a6(0.0),aux1(0.0),aux2(0.0),aux3(0.0),aux4(0.0),aux5(0.0),aux6(0.0) {}
                axispos(double a11, double a12, double a13, double a14, double a15, double a16, double a21, double a22, double a23, double a24, double a25, double a26):a1(a11),a2(a12),a3(a13),a4(a14),a5(a15),a6(a16),aux1(a21),aux2(a22),aux3(a23),aux4(a24),aux5(a25),aux6(a26) {}
                axispos(double a11, double a12, double a13, double a14, double a15, double a16):a1(a11),a2(a12),a3(a13),a4(a14),a5(a15),a6(a16),aux1(0.0),aux2(0.0),aux3(0.0),aux4(0.0),aux5(0.0),aux6(0.0) {}
                ~axispos() {}
		void print()
		{
			std::cout << "a1:  " << a1 << std::endl;
			std::cout << "a2:  " << a2 << std::endl;
			std::cout << "a3:  " << a3 << std::endl;
			std::cout << "a4:  " << a4 << std::endl;
			std::cout << "a5:  " << a5 << std::endl;
			std::cout << "a6:  " << a6 << std::endl;
			std::cout << "aux1:" << aux1 << std::endl;
			std::cout << "aux2:" << aux2 << std::endl;
			std::cout << "aux3:" << aux3 << std::endl;
			std::cout << "aux4:" << aux4 << std::endl;
			std::cout << "aux5:" << aux5 << std::endl;
			std::cout << "aux6:" << aux6 << std::endl;
		}
};

struct cartpos {
	public:
		double x;
		double y;
		double z;
		double a;
		double b;
		double c;

		double aux1;
		double aux2;
		double aux3;
		double aux4;
		double aux5;
		double aux6;

		int mode;
	public:
                cartpos():x(0.0),y(0.0),z(0.0),a(0.0),b(0.0),c(0.0),aux1(0.0),aux2(0.0),aux3(0.0),aux4(0.0),aux5(0.0),aux6(0.0),mode(-1) {}
                cartpos(double a11, double a12, double a13, double a14, double a15, double a16, double a21, double a22, double a23, double a24, double a25, double a26):x(a11),y(a12),z(a13),a(a14),b(a15),c(a16),aux1(a21),aux2(a22),aux3(a23),aux4(a24),aux5(a25),aux6(a26) {}
                cartpos(double a11, double a12, double a13, double a14, double a15, double a16):x(a11),y(a12),z(a13),a(a14),b(a15),c(a16),aux1(0.0),aux2(0.0),aux3(0.0),aux4(0.0),aux5(0.0),aux6(0.0) {}
                ~cartpos() {}
		void print()
		{
			std::cout << "x:   " << x << std::endl;
			std::cout << "y:   " << y << std::endl;
			std::cout << "z:   " << z << std::endl;
			std::cout << "a:   " << a << std::endl;
			std::cout << "b:   " << b << std::endl;
			std::cout << "c:   " << c << std::endl;
			std::cout << "aux1:" << aux1 << std::endl;
			std::cout << "aux2:" << aux2 << std::endl;
			std::cout << "aux3:" << aux3 << std::endl;
			std::cout << "aux4:" << aux4 << std::endl;
			std::cout << "aux5:" << aux5 << std::endl;
			std::cout << "aux6:" << aux6 << std::endl;
		}
};

// axisposext
// cartposext

struct robaxispos {
	public:
		double a1;
		double a2;
		double a3;
		double a4;
		double a5;
		double a6;
	public:
                robaxispos():a1(0.0),a2(0.0),a3(0.0),a4(0.0),a5(0.0),a6(0.0) {}
                robaxispos(double a11, double a12, double a13, double a14, double a15, double a16):a1(a11),a2(a12),a3(a13),a4(a14),a5(a15),a6(a16) {}
                ~robaxispos() {}
		void print()
		{
			std::cout << "a1:  " << a1 << std::endl;
			std::cout << "a2:  " << a2 << std::endl;
			std::cout << "a3:  " << a3 << std::endl;
			std::cout << "a4:  " << a4 << std::endl;
			std::cout << "a5:  " << a5 << std::endl;
			std::cout << "a6:  " << a6 << std::endl;
		}
};

struct robcartpos {
	public:
		double x;
		double y;
		double z;
		double a;
		double b;
		double c;

                int mode;
	public:
                robcartpos():x(0.0),y(0.0),z(0.0),a(0.0),b(0.0),c(0.0),mode(-1) {}
                robcartpos(double a11, double a12, double a13, double a14, double a15, double a16):x(a11),y(a12),z(a13),a(a14),b(a15),c(a16) {}
                ~robcartpos() {}
		void print()
		{
			std::cout << "x:  " << x << std::endl;
			std::cout << "y:  " << y << std::endl;
			std::cout << "z:  " << z << std::endl;
			std::cout << "a:  " << a << std::endl;
			std::cout << "b:  " << b << std::endl;
			std::cout << "c:  " << c << std::endl;
			std::cout << "mode:  " << mode << std::endl;
		}
};

struct auxaxispos {
	public:
		double aux1;
		double aux2;
		double aux3;
		double aux4;
		double aux5;
		double aux6;
	public:
                auxaxispos(): aux1(0.0),aux2(0.0),aux3(0.0),aux4(0.0),aux5(0.0),aux6(0.0) {}
                auxaxispos(double a21, double a22, double a23, double a24, double a25, double a26):aux1(a21),aux2(a22),aux3(a23),aux4(a24),aux5(a25),aux6(a26) {}
                ~auxaxispos() {}
		void print()
		{
			std::cout << "aux1:" << aux1 << std::endl;
			std::cout << "aux2:" << aux2 << std::endl;
			std::cout << "aux3:" << aux3 << std::endl;
			std::cout << "aux4:" << aux4 << std::endl;
			std::cout << "aux5:" << aux5 << std::endl;
			std::cout << "aux6:" << aux6 << std::endl;
		}
};

// Part 02 -- Tool data structure
struct tool {
	public:
		double x;
		double y;
		double z;
		double a;
		double b;
		double c;
		double CGx;
		double CGy;
		double CGz;
	public:
		tool():x(0.0),y(0.0),z(0.0),a(0.0),b(0.0),c(0.0), CGx(0.0), CGy(0.0), CGz(0.0) {}
		tool(double a11, double a12, double a13, double a14, double a15, double a16, double a21, double a22, double a23):x(a11),y(a12),z(a13),a(a14),b(a15),c(a16), CGx(a21), CGy(a22), CGz(a23) {}
		tool(double a11, double a12, double a13, double a14, double a15, double a16):x(a11),y(a12),z(a13),a(a14),b(a15),c(a16), CGx(0.0), CGy(0.0), CGz(0.0) {}
		~tool() {}

		void print()
		{
			std::cout << "x:  " << x << std::endl;
			std::cout << "y:  " << y << std::endl;
			std::cout << "z:  " << z << std::endl;
			std::cout << "a:  " << a << std::endl;
			std::cout << "b:  " << b << std::endl;
			std::cout << "c:  " << c << std::endl;
			std::cout << "CGx:" << CGx << std::endl;
			std::cout << "CGy:" << CGy << std::endl;
			std::cout << "CGz:" << CGz << std::endl;
		}
};

// Part 03 -- Reference system data structure
struct cartrefsys {
	public:
		double x;
		double y;
		double z;
		double a;
		double b;
		double c;
	public:
		cartrefsys():x(0.0),y(0.0),z(0.0),a(0.0),b(0.0),c(0.0) {}
		cartrefsys(double a11, double a12, double a13, double a14, double a15, double a16):x(a11),y(a12),z(a13),a(a14),b(a15),c(a16) {}
		~cartrefsys() {}

		void print()
		{
			std::cout << "x:  " << x << std::endl;
			std::cout << "y:  " << y << std::endl;
			std::cout << "z:  " << z << std::endl;
			std::cout << "a:  " << a << std::endl;
			std::cout << "b:  " << b << std::endl;
			std::cout << "c:  " << c << std::endl;
		}
};

// cartrefsysaxis
// cartrefsysext


// Part 04 -- Percent data structure
struct percent {
	public:
		int perc;
	public:
		percent():perc(0) {}
		percent(int val):perc(val) {}
		~percent() {}

		void print()
		{
			std::cout << "perc:  " << perc << std::endl;
		}
};

struct perc200{
	public:
		int perc2;
		perc200():perc2(100) {}
                perc200(int val):perc2(val) {}
                ~perc200() {}
	public:
		void print()
		{
			std::cout << "perc2:  " << perc2 << std::endl;
		}
};

// Part 05 -- Overlap data structure
struct ovlrel {
	public:
		perc200 ovl;
	public:
                ovlrel():ovl(100) {}
                ~ovlrel() {}
		void print()
		{
			ovl.print();
		}
};

struct ovlabs {
	public:
		double posDist;
		double oriDist;
		double linAxDist;
		double rotAxDist;
		bool vConst;

	public:
                ovlabs():posDist(0.0), oriDist(360.0), linAxDist(10000.0),rotAxDist(360.0), vConst(false) {}
                ovlabs(double a1, double a2, double a3, double a4, bool b1):posDist(a1), oriDist(a2), linAxDist(a3),rotAxDist(a4), vConst(b1) {}
                ~ovlabs() {}
		void print()
		{
			std::cout << "posDist:  " << posDist << std::endl;
			std::cout << "oriDist:  " << oriDist << std::endl;
			std::cout << "linAxDist:  " << linAxDist << std::endl;
			std::cout << "rotAxDist:  " << rotAxDist << std::endl;
			std::cout << "vConst:  " << vConst << std::endl;
		}
};

// Part 06 -- Dynamic data structure
struct dynamic {
	public:
		percent velAxis;
		percent accAxis;
		percent decAxis;
		percent jerkAxis;

		double vel;
		double acc;
		double dec;
		double jerk;

		double velOri;
		double accOri;
		double decOri;
		double jerkOri;
	public:
                dynamic():velAxis(0), accAxis(0), decAxis(0), jerkAxis(0), vel(0.0), acc(0.0), dec(0.0), jerk(0.0), velOri(0.0), accOri(0.0), decOri(0.0), jerkOri(0.0) {}
                dynamic(double a1, double a2, double a3, double a4, double b1, double b2, double b3, double b4, double c1, double c2, double c3, double c4):velAxis(a1), accAxis(a2), decAxis(a3), jerkAxis(a4), vel(b1), acc(b2), dec(b3), jerk(b4), velOri(c1), accOri(c2), decOri(c3), jerkOri(c4) {}
                ~dynamic() {}
		void print()
		{
			std::cout << "velAxis:  ";
			velAxis.print();
			std::cout << "accAxis:  ";
			accAxis.print();
			std::cout << "decAxis:  ";
			decAxis.print();
			std::cout << "jerkAxis:  ";
			jerkAxis.print();
			std::cout << "vel:  " << vel << std::endl;
			std::cout << "acc:  " << acc << std::endl;
			std::cout << "dec:" << dec << std::endl;
			std::cout << "jerk:" << jerk << std::endl;
			std::cout << "velOri:" << velOri << std::endl;
			std::cout << "accOri:" << accOri << std::endl;
			std::cout << "decOri:" << decOri << std::endl;
			std::cout << "jerkOri:" << jerkOri << std::endl;
		}
};



}

#endif
