//
// Created by root on 16-3-17.
//

#include "stage5_to_6.h"
#include <thread>
#include <unistd.h>

folly::ProducerConsumerQueue<ROBOT_ORDER> order_queue(10);

bool runflag = true;


//void order_producer(){
//
//}


void order_consumer(){

    std::cout << "start consumer" << std::endl;
    while(runflag){
//        std::cout << "order_consumer" << std::endl;
        ROBOT_ORDER temp_order;
//        std::cout << "order_queue size: " << order_queue.sizeGuess() << std::endl;
        if(!order_queue.isEmpty()){
            while(!order_queue.read(temp_order))
                continue;
        }
        switch(temp_order.ri_type){
            case robot_program_file_process::RT_PTP:
                temp_order.args[0].apv->print();
                temp_order.args[1].apv->print();
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
        usleep(100);
    }
}








