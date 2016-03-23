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

public :
    ROBOT_ORDER(ROBOT_ORDER &order){
        this->ri_type = order.ri_type;
        if(order.ri_type == robot_program_file_process::RT_PTP){
            this->args[0].apv = order.args[0].apv;
            this->args[1].apv = order.args[1].apv;
        }
    }
    ROBOT_ORDER& operator=(ROBOT_ORDER& order){
        this->ri_type = order.ri_type;
        if(order.ri_type == robot_program_file_process::RT_PTP){
            this->args[0].apv = order.args[0].apv;
            this->args[1].apv = order.args[1].apv;
        }
        return *this;
    }
    ROBOT_ORDER(ROBOT_ORDER &&order) {
        this->ri_type = order.ri_type;
        if(order.ri_type == robot_program_file_process::RT_PTP){
            this->args[0].apv = order.args[0].apv;
            this->args[1].apv = order.args[1].apv;
        }
    }
    ROBOT_ORDER& operator=(ROBOT_ORDER&& order) {
        this->ri_type = order.ri_type;
        if(order.ri_type == robot_program_file_process::RT_PTP){
            this->args[0].apv = order.args[0].apv;
            this->args[1].apv = order.args[1].apv;
        }
        return *this;
    }

public:
    void print(){
        std::cout << "ROBOT_ORDER TYPE: " << this->ri_type  << std::endl;
        std::cout << "ROBOT_ORDER TYPE: " << robot_program_file_process::RT_PTP  << std::endl;
        if(this->ri_type == robot_program_file_process::RT_PTP){
            this->args[0].apv->print();
            this->args[1].apv->print();
        }
    }

    void clear(){
        this->ri_type = robot_program_file_process::RT_NULL;
        for(int i = 0; i < 4; i++){
            this->args[i].apv = nullptr;
        }
    }

};



extern folly::ProducerConsumerQueue<ROBOT_ORDER> order_queue;




//void order_producer();


void order_consumer();


#endif //RC_RUNTIME_STAGE5_TO_6_H
