//
// Created by root on 16-3-17.
//

#include "stage5_to_6.h"
#include <thread>
#include <unistd.h>

folly::ProducerConsumerQueue<ROBOT_ORDER> order_queue(10);

bool runflag = true;


joint_velocity robot_v_start(0,0,0,0,0,0), robot_v_target(400,250,350,550,650,750), robot_v_end(0,0,0,0,0,0);
joint_acc robot_acc(2300,2100,2500,2900,3000,4000), robot_dec(2300,2100,2500,2900,3000,4000);
joint_jerk robot_jerk(1000000, 1000000, 1000000, 1000000, 1000000, 1000000);
double robot_cycle = 0.001;


std::vector<robot_data_file_process::axispos> points_set;
std::vector<joint_velocity> points_velocity;
std::vector<joint_acc> points_acc;
std::vector<joint_jerk> points_jerk;

//void order_producer(){
//
//}


void order_consumer(){
    int ret = 0;
    int cnt = 1;
    std::cout << "start consumer" << std::endl;
    while(runflag){
        std::cout << "order_consumer" << std::endl;
        ROBOT_ORDER temp_order;
        std::cout << "order_queue size: " << order_queue.sizeGuess() << std::endl;
        while(order_queue.isEmpty()){
            std::this_thread::yield();
            usleep(100);
        }
        if(!order_queue.isEmpty()){
            while(!order_queue.read(temp_order))
                continue;
        }
        switch(temp_order.ri_type){
            case robot_program_file_process::RT_PTP:
//                temp_order.args[0].apv->print();   // just for debug
//                temp_order.args[1].apv->print();   // just for debug
                ret = PTP_TMode_interpolator(*(temp_order.args[0].apv),
                                             *(temp_order.args[1].apv),
                                             robot_v_start,
                                             robot_v_end,
                                             robot_v_target,
                                             robot_acc,
                                             robot_dec,
                                             robot_cycle,
                                             points_set,
                                             points_velocity,
                                             points_acc
                                             );
                if(ret != 0) {
                    std::cout << "error" << std::endl;
                } else {
                    std::cout << "success" << std::endl;
                }
                std::cout << "PTP_TMode_interpolator " << cnt ++ << std::endl;
                break;
            case robot_program_file_process::RT_LIN:
                break;
            case robot_program_file_process::RT_CIRC:
                break;
            case robot_program_file_process::RT_DYN:
                break;
            case robot_program_file_process::RT_DYNOVR:
                break;
            case robot_program_file_process::RT_OVL:
                break;
            case robot_program_file_process::RT_RAMP:
                break;
            case robot_program_file_process::RT_REFSYS:
                break;
            case robot_program_file_process::RT_TOOL:
                break;
            case robot_program_file_process::RT_ORIMODE:
                break;
            case robot_program_file_process::RT_WAITTIME:
                break;
            case robot_program_file_process::RT_STOP:
                break;
            case robot_program_file_process::RT_INFO:
                break;
            case robot_program_file_process::RT_WARNING:
                break;
            case robot_program_file_process::RT_ERROR:
                break;
            case robot_program_file_process::RT_NULL:
                break;
            default:
                break;
        }
        temp_order.clear();
        usleep(10);
    }
}








