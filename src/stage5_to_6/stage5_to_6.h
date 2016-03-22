//
// Created by root on 16-3-17.
//

#ifndef RC_RUNTIME_STAGE5_TO_6_H
#define RC_RUNTIME_STAGE5_TO_6_H

#include "ProducerConsumerQueue.h"
#include "data_stage4.hh"
#include "program_stage4.hh"
#include "program_type.hh"

struct ROBOT_ORDER{
    robot_program_file_process::RobotInstructionType ri_type;
    union{
        robot_data_file_process::axispos *apv;
        robot_data_file_process::cartpos *cpv;
        robot_data_file_process::tool *tv;
        robot_data_file_process::cartrefsys *crsv;
        robot_data_file_process::percent *pv;
        robot_data_file_process::perc200 *p2v;
        robot_data_file_process::ovlrel *orv;
        robot_data_file_process::ovlabs *oav;
        robot_data_file_process::dynamic *dynv;
    }args[4];
public:
    ROBOT_ORDER(){}
    ~ROBOT_ORDER(){}

};



static folly::ProducerConsumerQueue<ROBOT_ORDER> order_queue(10);


void order_producer();


void order_consumer();


#endif //RC_RUNTIME_STAGE5_TO_6_H
