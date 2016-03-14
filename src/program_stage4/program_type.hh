

#ifndef PROGRAM_TYPE_H_
#define PROGRAM_TYPE_H_

namespace robot_program_file_process{

class statement_node; // Attention please here!

enum BasicType{
	TYPE_IDENTIFIER,
	TYPE_BOOL, 
	TYPE_DINT, 
	TYPE_REAL, 
	TYPE_STRING
};

struct value_type{
	BasicType v_type;
	union{
		const char *identifier;
		bool *bv;
		int *dv;
		double *rv;
		std::string *sv;
	}value;
};

struct param_list{
	public:
		value_type *param;
		param_list *next;
	public:
		param_list():param(NULL), next(NULL) {}
		~param_list() {}
};

struct ASSIGN_STMT{
	public:
		const char *variable;
		value_type *value;
	public:
		ASSIGN_STMT():variable(NULL), value(NULL) {}
		~ASSIGN_STMT() {}
};

struct CALL_STMT{
	public:
		const char *subprogram_name;
	public:
		CALL_STMT():subprogram_name(NULL) {}
		~CALL_STMT() {}
};

struct ELSEIF_STMT{
	public:
		value_type *condition;
		statement_node *then_stmt;
		ELSEIF_STMT *next;

	public:
		ELSEIF_STMT():condition(NULL), then_stmt(NULL), next(NULL) {}
		~ELSEIF_STMT() {}
};

struct IF_STMT{
	public:
		value_type *condition;
		statement_node *then_stmt;
		ELSEIF_STMT *elseif_stmt;
		statement_node *else_stmt;
	public:
		IF_STMT():condition(NULL), then_stmt(NULL), elseif_stmt(NULL), else_stmt(NULL) {}
		~IF_STMT() {}
};

struct WHILE_STMT{
	public:
		value_type *condition;
		statement_node *stmt;
	public:
		WHILE_STMT():condition(NULL), stmt(NULL) {}
		~WHILE_STMT() {}
};

struct LOOP_STMT{
	public:
		value_type *const_expression;
		statement_node *stmt;
	public:
		LOOP_STMT():const_expression(NULL), stmt(NULL) {}
		~LOOP_STMT() {}
};

enum RobotInstructionType{
	RT_PTP,
	RT_LIN,
	RT_CIRC,
	RT_DYN,
	RT_DYNOVR,
	RT_OVL,
	RT_RAMP,
	RT_REFSYS,
	RT_TOOL,
	RT_ORIMODE,
	RT_WAITTIME,
	RT_STOP,
	RT_INFO,
	RT_WARNING,
	RT_ERROR
};

struct ROBOT_STMT{
	public:
		RobotInstructionType ri_type;
		param_list *parameter_list;
	public:
		ROBOT_STMT(): parameter_list(NULL) {} 
		~ROBOT_STMT() {}
};

}

#endif



