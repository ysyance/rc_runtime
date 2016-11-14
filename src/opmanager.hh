#ifndef _OPMANAGER_HH_
#define _OPMANAGER_HH_

#include <native/task.h>
#include <native/event.h>
#include <native/queue.h>

#include <iostream>
#include <string>
#include <setjmp.h>

#include "preprocess.hh"

// RC运行模式
enum RcMode{
    OP_TEACH,           // 示教模式
    OP_RUN              // 再现运行模式
};

// RC运行管理器控制解释器命令
enum RcCommand{
    CMD_START,          // 开始运行
    CMD_STEP,           // 单步运行
    CMD_RUN,            // 自动运行
    CMD_RESET,          // 返回程序起点
    CMD_STOP            // 停止运行
};

// RC运行状态机
enum  RcStatusMachine{
    STATUS_STOP,        // 待机态
    STATUS_READY,       // READY态，此时示教程序已完成词法、语法语义分析生成程序运行数据结构
    STATUS_AUTORUN,     // 自动运行态
    STATUS_STEP,        // 单步运行态
    STATUS_PAUSE        // 暂停态
};

extern RT_QUEUE mq_manager2exec_desc;        // 运行管理器 ---> 解释器 消息队列描述符
extern RT_QUEUE mq_exec2manager_desc;        // 解释器 ---> 运行管理器 消息队列描述符
#define MQ_MANAGER2EXEC    "mq_manager2exec"  // 运行管理器 ---> 解释器 消息队列名
#define MQ_EXEC2MANAGER    "mq_exec2manager"  // 解释器 ---> 运行管理器 消息队列名




// 运行管理器核心数据结构——当前RC运行状态信息数据
struct RCOperationCtrl{
    RcMode  mode;                   // RC运行模式
    bool startup;                   // 是否启动运行，0：停止    1：启动
    int stepflag;                   // 是否进行单步运行，0：非单步   1：单步step in  2:单步step over

    std::string cur_project;        // 当前工程
    std::string cur_program;        // 当前程序
    robot_program_file_process::statement_node *exec_program_head; // 当前程序头指针
    robot_program_file_process::statement_node *pc;     // 程序指针PC
    int cur_linenum;                    // 当前指令对应程序文件行号
    int exec_run_mode;              // 当前程序运行状态，0:待机态，1:执行态，2:等待态
    RcStatusMachine  status;        // RC状态机
public:
    // RCOperationCtrl():mode(OP_TEACH), startup(0), stepflag(0),
    //                 cur_project(NULL), cur_program(NULL), cur_linenum(0),
    //                 exec_program_head(NULL), pc(NULL),exec_run_mode(0)
    // {}

};

extern RCOperationCtrl rc_core;         /* RC核心管理器 */
extern jmp_buf exec_startpoint;			/* 解释执行器起点 */
extern void* rc_cmd;                    /* RC控制命令 */


#define STEPCHECK(linenum)     {if(!rc_core.startup || rc_core.stepflag) {  /* 当前处于停止模式或单步运行模式，进入消息等待*/  \
                                do{                                                                                  \
                                    void *line = rt_queue_alloc(&mq_exec2manager_desc, 4);   /* 分配4个字节内存存放行号 */      \
                                    (*(int*)line) = linenum;                                                          \
                                    rt_queue_send(&mq_exec2manager_desc, line, 1, Q_NORMAL);  /* 返回行号给管理器  */        \
                                    int len = rt_queue_receive(&mq_manager2exec_desc, &rc_cmd, TM_INFINITE);   /* 等待管理器命令 */\
                                    printf("received message> len=%d bytes, cmd=%d\n", len, *((const char *)rc_cmd));       \
                                    char cmd = *((const char *)rc_cmd);                                               \
                                    switch(cmd) {                                                                       \
                                        case CMD_STEP:                                                              \
                                        break;                                                                          \
                                        case CMD_RESET:                                                                 \
                                            rc_core.cur_linenum = 1;                                                \
                                            longjmp(exec_startpoint, 0);        /* 跳转至解释器起点 */                     \
                                        break;                                                                      \
                                        default:                                                                    \
                                        break;                                                                      \
                                    }                                                                               \
                                    rt_queue_free(&mq_manager2exec_desc, rc_cmd);            /* 释放消息内存 */                    \
                                } while(!rc_core.startup);                                                          \
                            }                                                                                       \
                        }





#endif
