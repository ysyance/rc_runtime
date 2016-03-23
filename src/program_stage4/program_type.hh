

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
public:
	friend std::ostream &operator<<(std::ostream &os, const value_type contents){
		if(contents.v_type == TYPE_IDENTIFIER){
			os << "TYPE_IDENTIFIER:";
			os << std::string(contents.value.identifier) << " ";
		} else if(contents.v_type == TYPE_BOOL){
			os << "TYPE_BOOL:";
			os << *(contents.value.bv) << " ";
		} else if(contents.v_type == TYPE_DINT){
			os << "TYPE_DINT:";
			os << *(contents.value.dv) << " ";
		} else if(contents.v_type == TYPE_REAL){
			os << "TYPE_REAL:";
			os << *(contents.value.rv) << " ";
		} else {
			os << "TYPE_STRING:";
			os << *(contents.value.sv) << " ";
		}
		return os;
	}
};

struct param_list{
	public:
		value_type *param;
		param_list *next;
	public:
		param_list():param(NULL), next(NULL) {}
		~param_list() {}
	public:
		friend std::ostream &operator<<(std::ostream &os, const param_list contents){
			if(contents.param != nullptr){
				os << "param: ";
				os << *(contents.param);
			}
			if(contents.next != nullptr){
				os << "next: ";
				os << *(contents.next);
			}
			return os;
		}
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
	RT_ERROR,
	RT_NULL //add by yaoshun
};

struct ROBOT_STMT{
	public:
		RobotInstructionType ri_type;
		param_list *parameter_list;
//		robot_data_file_process::DEF_SYMTABLE *datafile; //add by yaoshun
	public:
		ROBOT_STMT(): parameter_list(NULL) {} 
		~ROBOT_STMT() {}
};

}

#endif



