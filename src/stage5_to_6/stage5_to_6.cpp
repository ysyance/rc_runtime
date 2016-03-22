//
// Created by root on 16-3-17.
//

#include "stage5_to_6.h"
#include <thread>

//folly::ProducerConsumerQueue<ROBOT_ORDER> order_queue(10);

void order_producer(){

}


void order_consumer(){
    for(;;){
        ROBOT_ORDER temp_order;
        if(!order_queue.isEmpty()){
            order_queue.read(temp_order);
        }
        if(temp_order.ri_type == robot_program_file_process::RT_PTP){
            temp_order.args[0].apv->print();
            temp_order.args[1].apv->print();
        }
    }
}








