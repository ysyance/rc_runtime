#ifndef __CALINVERSEKIN_HH__
#define __CALINVERSEKIN_HH__

tmatrix TermPos2TransMatrix(const XyzPose& termPos);
// void calInverseKin_ER4(const tmatrix& transMatrix, const std::vector<Robot_param *> &axes,
// 								const AxisPos_Deg& posLast, AxisPos_Deg& posTar);
int calInverseKin_ER4(const tmatrix& transMatrix, Robot_param*  axes,//const std::vector<Robot_param *> &axes,
								const AxisPos_Deg& posLast, AxisPos_Deg& posTar);

tmatrix calForwardKin(const AxisPos_Deg &PosAxis, Robot_param*  axes, XyzPose &PosCart);

#endif