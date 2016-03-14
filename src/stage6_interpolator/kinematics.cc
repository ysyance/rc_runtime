
struct joint_DH_parameter
{
public:
double theta;
double dist;
double away;
double alpha;
public:
joint_DH_parameter() {}
~joint_DH_parameter() {}
};

struct 6R_Industrial_Robot
{
public:
joint_DH_parameter joint1;
joint_DH_parameter joint2
joint_DH_parameter joint3;
joint_DH_parameter joint4;
joint_DH_parameter joint5;
joint_DH_parameter joint6;
public:
6R_Industrial_Robot() {}
~6R_Industrial_Robot() {}
}



