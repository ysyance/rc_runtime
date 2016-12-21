#ifndef __INST_TYPE_H__
#define __INST_TYPE_H__


#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "plc.h"


typedef Eigen::MatrixXd dmatrix;  
typedef Eigen::VectorXd dvector;
typedef Eigen::VectorXd AxisPos_Deg;
//typedef Eigen::Matrix<double,6,1>  AxisPos_Deg;
typedef Eigen::Matrix<double,6,1>  XyzPose;
typedef Eigen::Matrix<double,4,4>  tmatrix;       // Transfer Matrix type
typedef Eigen::Matrix<double,3,1>  Euler_Deg;     // Euler Angle in Degree



enum INST_TYPE{
	PTP,
	LIN,
	CIRC,
	JOINTJOG,
    CARTJOG
};

struct JogJointParam{
	int jointindex;   // index of axis
    int direction;    // POSITIVE or NEGTIVE
};



struct PosParam{
    AxisPos_Deg apv;
    XyzPose  cpv;
    JogJointParam jjp;
public:
	PosParam(){}
	~PosParam(){}
};


struct ROBOT_INST{
	INST_TYPE  ri_type;
	PosParam args[4];
public:
	ROBOT_INST(){}
	~ROBOT_INST(){}

	ROBOT_INST& operator=(ROBOT_INST& order){
        this->ri_type = order.ri_type;
        if(order.ri_type == PTP){
            this->args[0].apv = order.args[0].apv;
            this->args[1].apv = order.args[1].apv;
        } else if(order.ri_type == JOINTJOG || order.ri_type == CARTJOG) {
        	this->args[0].apv = order.args[0].apv;
        	this->args[0].jjp = order.args[0].jjp;
        } else if(order.ri_type == LIN) {
            this->args[0].cpv = order.args[0].cpv;
            this->args[1].cpv = order.args[1].cpv;
        } else if(order.ri_type == CIRC) {

        }
        return *this;
    }
};

enum JogProc
{
	JOG_INCONSTACC = 0,
	JOG_CONSTVEL,
	JOG_STOP
};


 #define    BASE     1
 #define    TOOL     2

extern ROBOT_INST robot_inst_code;
extern bool flag;









#endif