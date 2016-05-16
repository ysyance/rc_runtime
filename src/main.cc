/*
 ****************************************************************
 ****************************************************************
 ****************************************************************
 *********                                              *********
 *********                                              *********
 *********   O V E R A L L    A R C H I T E C T U R E   *********
 *********                                              *********
 *********                                              *********
 ****************************************************************
 ****************************************************************
 ****************************************************************

 The compiler works in 4(+1) stages:
 Stage 1   - Lexical analyser      - implemented with flex (iec.flex)
 Stage 2   - Syntax parser         - implemented with bison (iec.y)
 Stage 3   - Semantics analyser    - not yet implemented
 Stage 4   - Code generator        - implemented in C++
 Stage 4+1 - Binary code generator - gcc, javac, etc...


 Data structures passed between stages, in global variables:
 1->2   : tokens (int), and token values (char *)
 2->1   : symbol tables (defined in symtable.hh)
 2->3   : abstract syntax tree (tree of C++ classes, in absyntax.hh file)
 3->4   : Same as 2->3
 4->4+1 : file with program in c, java, etc...


 The compiler works in several passes:
 Pass 1: executes stages 1 and 2 simultaneously
 Pass 2: executes stage 3
 Pass 3: executes stage 4
 Pass 4: executes stage 4+1
 */
#include "preprocess.hh"
#include "plc.h"

#include <native/task.h>

#include <signal.h>
#include <unistd.h>


/* these defines for debug  */
#define TEST_DATA_FILE_RESULT       0
#define TEST_PROGRAM_FILE_RESULT    0
#define RUN_IN_PC  0


#define RC_TASK_NAME "rc_task"		/* RC任务名 */
#define RC_TASK_PRIORITY 80			/* RC任务优先级 */

RT_TASK task_desc;					/*　RC任务描述符 */
RCMem *rc_shm;						/* RC与PLC共享内存区指针 */
RT_HEAP rc_heap_desc;				/* 共享内存区描述符 */
RobotConfig *rc_conf;				/* 机器人配置信息变量指针 */

RT_COND rc_cond_desc;               /* RC/PLC同步对象－－条件变量描述符 */
RT_MUTEX rc_mutex_desc;             /* RC/PLC同步对象－－互斥量描述符 */


/**
 * 函数名：task_routine
 * 函数功能：RC实时任务，其中包含连个线程：解释执行器线程和插补器线程，两个线程通过无锁环形队列（单消费者单生产者模型）联系
 * 参数：cookie  用户给定参数
 * 返回值：无
 */
void task_routine(void *cookie){

	char** argv = (char**)cookie;

    rc_mem_bind(rc_shm, rc_conf);			/* rc_shm绑定共享内存区地址 */
	rc_syncobj_bind(&rc_mutex_desc, RC_MUTEX_NAME, &rc_cond_desc, RC_COND_NAME);    /* 绑定RC/PLC同步对象 */

	/***************************************************/
	/* Part 0101--Key intermediate data structures     */
	/***************************************************/
	robot_data_file_process::DEF_SYM_SYM symtable_of_symtable;
	robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE subprogram_symtable;

#if	RUN_IN_PC
	const char *project_directory = "/root/workspace/robotspace/Coruntime/rc-runtime/test/lab";
	const char *exec_directory = "/root/workspace/robotspace/Coruntime/rc-runtime/test/lab/control";
#else
	const char *project_directory = "/mnt/share/rc-runtime/test/lab";
	const char *exec_directory = "/mnt/share/rc-runtime/test/lab/control";
#endif
	printf("rc task start ...\n");
	/***************************************************/
	/* Part 0102--Process data && program files        */
	/***************************************************/
	int ret = 1;

	ret = data_myftw(argv[1], project_directory, symtable_of_symtable);
	std::cout << "data file return value: " << ret << std::endl;
	ret = program_myftw(argv[1], project_directory, subprogram_symtable, symtable_of_symtable);
	std::cout << "program file return value: " << ret << std::endl;

	auto order_producer = std::bind(stage5, symtable_of_symtable, subprogram_symtable, exec_directory, project_directory);
	std::thread writer(order_producer);
	std::thread reader(order_consumer);

	writer.join();
	reader.join();
}

void cleanup(void){
	rt_task_delete(&task_desc);
}

void sig_handler(int signo){
	if(signo == SIGINT){
	    printf("Receive Signal No: %d \n", signo);
	    cleanup();
	    exit(0);
  }
}



int main(int argc, char **argv) {
	int err;

	signal(SIGINT, sig_handler);
	printf("RC Start ...\n");

	err = rt_task_create(&task_desc, RC_TASK_NAME, 0, RC_TASK_PRIORITY, T_JOINABLE);

	if(!err){
		rt_task_start(&task_desc, &task_routine, argv);
	}
 	rt_task_join(&task_desc);

	return 0;
}
