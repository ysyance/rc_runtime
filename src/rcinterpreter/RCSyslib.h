
#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory.h>

#include "rc_innerdata.h"
#include "rc_exception.h"


extern RT_TASK rc_rsi_desc;                                /* RSI任务描述符 */
extern RT_TASK_INFO  rc_rsi_info;                          /* RSI任务状态 */
void rsi_routine(void *cookie) ;							/* RSI task instance */
extern RT_TASK rc_executor_desc;                           /* RC执行器任务描述符 */
extern RT_TASK_INFO  rc_executor_info;                     /* RC执行器任务状态 */



class RCEntityBase;


typedef struct {
    std::string name;
    int param_count;
    int (*pfun)(std::vector<int>&, RCEntityBase*, RC_SymbolTable&);
} RCLibEntry; /* System-level POU(Library) descriptor of RC*/



/* ** ALL THE IMPLEMENTATION OF LIBRARY FUNCTION IN C/C++ ************** */
inline int rc_sum(std::vector<int>& params, RCEntityBase* config, RC_SymbolTable& symTable) {
	return 0;
}

inline int rc_sub(std::vector<int>& params, RCEntityBase* config, RC_SymbolTable& symTable) {
	return 0;
}

inline int rc_multi(std::vector<int>& params, RCEntityBase* config, RC_SymbolTable& symTable) {
	return 0;
}

inline int rc_div(std::vector<int>& params, RCEntityBase* config, RC_SymbolTable& symTable) {
	return 0;
}

inline int rc_start_rsi(std::vector<int>& params, RCEntityBase* config, RC_SymbolTable& symTable) {
	if( params.size() == 1 && isstring(symTable.addrspace[params[0]]) ) {
		int32_t index = symTable.addrspace[params[0]].v.value_s;
		std::string rsiName = symTable.stringpool[index];
		rt_task_start(&rc_rsi_desc, rsi_routine, &rsiName);
		rt_task_join(&rc_rsi_desc);
	} else {
		throw rc_wrongfuncparams_exception("RC_START_RSI");
	}
	return 0;
}


/* ** THE WHOLE RC FUNCTION LIBRARY HERE ******************************* */
#define VARIABLE_LEN 0
#define RC_LIB_SIZE  5

/* ORDER SENSITIVE */
static const RCLibEntry rcLibEntry[RC_LIB_SIZE] = {
	{"RC_SUM", VARIABLE_LEN, rc_sum},
	{"RC_SUB", 2, rc_sub},
	{"RC_MULTI", 2, rc_multi},
	{"RC_DIV", 2, rc_div},

	{"RC_START_RSI", 1, rc_start_rsi},
};



/* ** the RC Entity type of function block **************************** */
class RCEntityBase {
public:
	RCEntityBase(std::string f) : funcName(f) {}

	virtual int setConfig(std::string key, std::string value) = 0;

	virtual int printInfo() { std::cout << "[> RCEntityBase <]" << std::endl; }

public:
	std::string funcName;
};
