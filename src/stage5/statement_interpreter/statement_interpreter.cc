#include <iostream>
#include <stage5_to_6.h>
#include <program_type.hh>
#include <thread>
//#include <cstring>
#include "../../data_util/symtable.hh"
#include "../../program_util/symtable.hh"
//#include "../program_stage4/program_type.hh"
#include "../stage5.hh"

#include "statement_interpreter.hh"


int assign_interpreter(robot_program_file_process::ASSIGN_STMT *assign_stmt,const  char *exec_directory, const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable)
{
	robot_data_file_process::DEF_SYMTABLE *filename_symtable = symtable_of_symtable.find_value(exec_directory);
	if(filename_symtable == NULL) // Here should be careful! return value!
	{
	}
	robot_data_file_process::sym_element *var =  filename_symtable->find_value(assign_stmt->variable);
	if(var == NULL) // Here should be careful! return value!
	{
	}

	switch(*(var->id_type))
	{
		case robot_data_file_process::TYPE_BOOL:
			switch(assign_stmt->value->v_type)
			{
				case robot_program_file_process::TYPE_BOOL:
					*(var->id_value.bv) = *(assign_stmt->value->value.bv);
					break;
				case robot_program_file_process::TYPE_IDENTIFIER:
					{
						robot_data_file_process::sym_element *right_value =  filename_symtable->find_value(assign_stmt->value->value.identifier);
						if(right_value == var)
							return 0;
						else
						{
							switch(*(right_value->id_type))
							{
								case robot_data_file_process::TYPE_BOOL:
									*(var->id_value.bv) = *(right_value->id_value.bv);
									break;
								default:
									std::cout << "The type of left expression and right expression(variable) is not the same!" << std::endl;
									return -1;
							}
						}
					}
					break;
				default:
					std::cout << "The type of left expression and right expression(value) is not the same!" << std::endl;
					return -1;
					break;
			}
                        std::cout << "The var's new value is: " << *(var->id_value.bv) << std::endl;
			break;
		case robot_data_file_process::TYPE_DINT:
			switch(assign_stmt->value->v_type)
			{
				case robot_program_file_process::TYPE_DINT:
					*(var->id_value.dv) = *(assign_stmt->value->value.dv);
					break;
                                case robot_program_file_process::TYPE_REAL:
					*(var->id_value.dv) = *(assign_stmt->value->value.rv);
					break;
				case robot_program_file_process::TYPE_IDENTIFIER:
					{
						robot_data_file_process::sym_element *right_value =  filename_symtable->find_value(assign_stmt->value->value.identifier);
						if(right_value == var)
							return 0;
						else
						{
							switch(*(right_value->id_type))
							{
								case robot_data_file_process::TYPE_DINT:
									*(var->id_value.dv) = *(right_value->id_value.dv);
									break;
								case robot_data_file_process::TYPE_REAL:
									*(var->id_value.dv) = *(right_value->id_value.rv);
									break;
								default:
									std::cout << "The type of left expression and right expression(variable) is not the same!" << std::endl;
									return -1;
							}
						}
					}
					break;
				default:
					std::cout << "The type of left expression and right expression is not the same!" << std::endl;
					return -1;
					break;
			}
                        std::cout << "The var's new value is: " << *(var->id_value.dv) << std::endl;
			break;
		case robot_data_file_process::TYPE_REAL:
			switch(assign_stmt->value->v_type)
			{
				case robot_program_file_process::TYPE_DINT:
					*(var->id_value.rv) = *(assign_stmt->value->value.dv);
					break;
				case robot_program_file_process::TYPE_REAL:
					*(var->id_value.rv) = *(assign_stmt->value->value.rv);
					break;
				case robot_program_file_process::TYPE_IDENTIFIER:
					{
						robot_data_file_process::sym_element *right_value =  filename_symtable->find_value(assign_stmt->value->value.identifier);
						if(right_value == var)
							return 0;
						else
						{
							switch(*(right_value->id_type))
							{
								case robot_data_file_process::TYPE_DINT:
									*(var->id_value.rv) = *(right_value->id_value.dv);
									break;
								case robot_data_file_process::TYPE_REAL:
									*(var->id_value.rv) = *(right_value->id_value.rv);
									break;
								default:
									std::cout << "The type of left expression and right expression(variable) is not the same!" << std::endl;
									return -1;
							}
						}
					}
					break;
				default:
					std::cout << "The type of left expression and right expression is not the same!" << std::endl;
					return -1;
					break;
			}
                        std::cout << "The var's new value is: " << *(var->id_value.rv) << std::endl;
			break;
		case robot_data_file_process::TYPE_STRING:
			switch(assign_stmt->value->v_type)
			{
				case robot_program_file_process::TYPE_STRING:
					*(var->id_value.sv) = *(assign_stmt->value->value.sv);
					break;
				case robot_program_file_process::TYPE_IDENTIFIER:
					{
						robot_data_file_process::sym_element *right_value =  filename_symtable->find_value(assign_stmt->value->value.identifier);
						if(right_value == var)
							return 0;
						else
						{
							switch(*(right_value->id_type))
							{
								case robot_data_file_process::TYPE_STRING:
									*(var->id_value.sv) = *(right_value->id_value.sv);
									break;
								default:
									std::cout << "The type of left expression and right expression(variable) is not the same!" << std::endl;
									return -1;
							}
						}
					}
					break;
				default:
					std::cout << "The type of left expression and right expression is not the same!" << std::endl;
					return -1;
					break;
			}
                        std::cout << "The var's new value is: " << *(var->id_value.sv) << std::endl;
			break;
		default:
			std::cout << "The value of type other than bool, dint, real, string cannot be assigned!" << std::endl;
			return -1;
			break;

	}
        return 0;
}

int elseif_interpreter(robot_program_file_process::ELSEIF_STMT *elseif_stmt,const  char *exec_directory, const char *project_directory,robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, bool &flag, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable)
{
	robot_data_file_process::DEF_SYMTABLE *filename_symtable = symtable_of_symtable.find_value(exec_directory);
	if(filename_symtable == NULL) // Here should be careful! return value!
	{
	}

	int ret = -1;
	robot_program_file_process::ELSEIF_STMT *pStmt = elseif_stmt;
        if(elseif_stmt == NULL)
        {
                std::cout << "else if statement is NULL in stage5" << std::endl;
                return 0;
        }
	while(pStmt != NULL)
	{
		switch(pStmt->condition->v_type)
		{
			case robot_program_file_process::TYPE_BOOL:
				flag = *(pStmt->condition->value.bv);
				break;
			case robot_program_file_process::TYPE_DINT:
				if(*(pStmt->condition->value.dv) != 0)
					flag = true;
				break;
			case robot_program_file_process::TYPE_REAL:
				if(*(pStmt->condition->value.rv) != 0.0)
					flag = true;
				break;
			case robot_program_file_process::TYPE_IDENTIFIER:
				{
					robot_data_file_process::sym_element *expression_value =  filename_symtable->find_value(pStmt->condition->value.identifier);
					switch(*(expression_value->id_type))
					{
						case robot_data_file_process::TYPE_BOOL:
							flag = *(expression_value->id_value.bv);
							break;
						case robot_data_file_process::TYPE_DINT:
							if(*(expression_value->id_value.dv) != 0);
							flag = true;
							break;
						case robot_data_file_process::TYPE_REAL:
							if(*(expression_value->id_value.rv) != 0.0);
							flag = true;
							break;
						default:
							std::cout << "The type of expression is wrong!" << std::endl;
							return -1;
					}
				}
				break;
			default:
				std::cout << "The type of expression is wrong!" << std::endl;
				return -1;
				break;
		}
		if(flag)
		{
			ret = stage5_core(symtable_of_symtable, subprogram_symtable, pStmt->then_stmt, exec_directory, project_directory);
			return ret;
		}
		else
		{
			pStmt = pStmt->next;
		}
	}
	return ret;
}

int if_interpreter(robot_program_file_process::IF_STMT *if_stmt,const  char *exec_directory, const char *project_directory,robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable)
{
	robot_data_file_process::DEF_SYMTABLE *filename_symtable = symtable_of_symtable.find_value(exec_directory);
	if(filename_symtable == NULL) // Here should be careful! return value!
	{
	}

	bool flag = false;
	int ret = -1;
	switch(if_stmt->condition->v_type)
	{
		case robot_program_file_process::TYPE_BOOL:
			flag = *(if_stmt->condition->value.bv);
			break;
		case robot_program_file_process::TYPE_DINT:
			if(*(if_stmt->condition->value.dv) != 0)
				flag = true;
			break;
		case robot_program_file_process::TYPE_REAL:
			if(*(if_stmt->condition->value.rv) != 0.0)
				flag = true;
			break;
		case robot_program_file_process::TYPE_IDENTIFIER:
			{
				robot_data_file_process::sym_element *expression_value =  filename_symtable->find_value(if_stmt->condition->value.identifier);

				switch(*(expression_value->id_type))
				{
					case robot_data_file_process::TYPE_BOOL:
						flag = *(expression_value->id_value.bv);
						break;
					case robot_data_file_process::TYPE_DINT:
						if(*(expression_value->id_value.dv) != 0);
						flag = true;
						break;
					case robot_data_file_process::TYPE_REAL:
						if(*(expression_value->id_value.rv) != 0.0);
						flag = true;
						break;
					default:
						std::cout << "The type of expression is wrong!" << std::endl;
						return -1;
				}
			}
			break;
		default:
			std::cout << "The type of expression is wrong!" << std::endl;
			return -1;
			break;
	}
	if(flag)
	{
		ret = stage5_core(symtable_of_symtable, subprogram_symtable, if_stmt->then_stmt, exec_directory, project_directory);
		return ret;
	}
	else
	{
		ret = elseif_interpreter(if_stmt->elseif_stmt, exec_directory, project_directory, symtable_of_symtable, flag, subprogram_symtable);
		if(!flag)
			ret = stage5_core(symtable_of_symtable, subprogram_symtable, if_stmt->else_stmt, exec_directory, project_directory);
		return ret;
	}
}
/*
int while_interpreter(robot_program_file_process::WHILE_STMT *while_stmt,const  char *exec_directory, const char *project_directory,robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable)
{
	robot_data_file_process::DEF_SYMTABLE *filename_symtable = symtable_of_symtable.find_value(exec_directory);
	if(filename_symtable == NULL) // Here should be careful! return value!
	{
	}

	while(true)
	{
		bool flag = false;
		int ret = -1;
		switch(while_stmt->condition->v_type)
		{
			case robot_program_file_process::TYPE_BOOL:
				flag = while_stmt->condition->value.bv;
				break;
			case robot_program_file_process::TYPE_DINT:
				if(*(while_stmt->condition->value.dv) != 0)
					flag = true;
				break;
			case robot_program_file_process::TYPE_REAL:
				if(*(while_stmt->condition->value.rv) != 0.0)
					flag = true;
				break;
			case robot_program_file_process::TYPE_IDENTIFIER:
				{
					robot_data_file_process::sym_element *expression_value =  filename_symtable->find_value(while_stmt->condition->value.identifier);

					switch(*(expression_value->id_type))
					{
						case robot_data_file_process::TYPE_BOOL:
							flag = *(expression_value->id_value.bv);
							break;
						case robot_data_file_process::TYPE_DINT:
							if(*(expression_value->id_value.dv) != 0);
							flag = true;
							break;
						case robot_data_file_process::TYPE_REAL:
							if(*(expression_value->id_value.rv) != 0.0);
							flag = true;
							break;
						default:
							std::cout << "The type of expression is wrong!" << std::endl;
							return -1;
					}
				}
				break;
			default:
				std::cout << "The type of expression is wrong!" << std::endl;
				return -1;
				break;
		}
		if(flag)
		{
			ret = stage5_core(symtable_of_symtable, subprogram_symtable, while_stmt->stmt, exec_directory, project_directory);
		}
		else
			break;
	}
	return 0;
}
*/

int loop_interpreter(robot_program_file_process::LOOP_STMT *loop_stmt, const char *exec_directory, const char *project_directory,robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable)
{
	robot_data_file_process::DEF_SYMTABLE *filename_symtable = symtable_of_symtable.find_value(exec_directory);
	if(filename_symtable == NULL) // Here should be careful! return value!
	{
	}

	int loop_num = 0;
	switch(loop_stmt->const_expression->v_type)
	{
		case robot_program_file_process::TYPE_DINT:
			loop_num = *(loop_stmt->const_expression->value.dv);
			break;
		case robot_program_file_process::TYPE_IDENTIFIER:
			{
				robot_data_file_process::sym_element *expression_value =  filename_symtable->find_value(loop_stmt->const_expression->value.identifier);

				switch(*(expression_value->id_type))
				{
					case robot_data_file_process::TYPE_DINT:
						loop_num = *(expression_value->id_value.dv);
						break;
					default:
						std::cout << "The type of expression is wrong!" << std::endl;
						return -1;
				}
			}
			break;
		default:
			std::cout << "The type of expression is wrong!" << std::endl;
			return -1;
			break;
	}
	while(loop_num--)
	{
		int ret = -1;
                std::cout << "The order is:" << loop_num << std::endl;
		ret = stage5_core(symtable_of_symtable, subprogram_symtable, loop_stmt->stmt, exec_directory, project_directory);
	}
	return 0;
}
#include <cstring>
#include <data_stage4.hh>
#include <unistd.h>
#include "stage5_to_6.h"

int robot_interpreter(robot_program_file_process::ROBOT_STMT *robot_stmt,const  char *exec_directory,  robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable)
{

	char *tempstr1 ;
	char *tempstr2 ;
	char *tempstr3 ;
//	std::cout << exec_directory << std::endl;   	// just for debug
	robot_data_file_process::DEF_SYMTABLE *filename_symtable = symtable_of_symtable.find_value(exec_directory);
	robot_data_file_process::sym_element *variable ;
	ROBOT_ORDER temp_order;
	temp_order.ri_type = robot_program_file_process::RT_PTP;
	switch(robot_stmt->ri_type)
	{
		case robot_program_file_process::RT_PTP:

			std::cout << "PTP()" << std::endl;
			std::cout << *(robot_stmt->parameter_list) << std::endl;
			tempstr1 = const_cast<char*>((robot_stmt->parameter_list)->param->value.identifier);
			tempstr2 = const_cast<char*>((robot_stmt->parameter_list)->next->param->value.identifier);
			variable = filename_symtable->find_value(tempstr1);
			if(*(variable->id_type) == robot_data_file_process::TYPE_AXISPOS){
				temp_order.args[0].apv = variable->id_value.apv;
//				std::cout << "=========ap1========" << std::endl;
//				temp_order.args[0].apv->print();
			}
			variable = filename_symtable->find_value(tempstr2);
			if(*(variable->id_type) == robot_data_file_process::TYPE_AXISPOS){
				temp_order.args[1].apv = variable->id_value.apv;
//				std::cout << "=========ap2========" << std::endl;
//				temp_order.args[1].apv->print();
			}


			break;
		case robot_program_file_process::RT_LIN:
			std::cout << "LIN()" << std::endl;
			break;
		case robot_program_file_process::RT_CIRC:
			std::cout << "CIRC()" << std::endl;
			break;
		case robot_program_file_process::RT_DYN:
			std::cout << "DYN()" << std::endl;
			break;
		case robot_program_file_process::RT_DYNOVR:
			std::cout << "DYNOVR()" << std::endl;
			break;
		case robot_program_file_process::RT_OVL:
			std::cout << "OVL()" << std::endl;
			break;
		case robot_program_file_process::RT_RAMP:
			std::cout << "RAMP()" << std::endl;
			break;
		case robot_program_file_process::RT_REFSYS:
			std::cout << "REFSYS()" << std::endl;
			break;
		case robot_program_file_process::RT_TOOL:
			std::cout << "TOOL()" << std::endl;
			break;
		case robot_program_file_process::RT_ORIMODE:
			std::cout << "ORIMODE()" << std::endl;
			break;
		case robot_program_file_process::RT_WAITTIME:
			std::cout << "WAITTIME()" << std::endl;
			break;
		case robot_program_file_process::RT_STOP:
			std::cout << "STOP()" << std::endl;
			break;
		case robot_program_file_process::RT_INFO:
			std::cout << "INFO()" << std::endl;
			break;
		case robot_program_file_process::RT_WARNING:
			std::cout << "WARNING()" << std::endl;
			break;
		case robot_program_file_process::RT_ERROR:
			std::cout << "ERROR()" << std::endl;
			break;
	}
	while(order_queue.isFull()){
		std::this_thread::yield();
		usleep(100);
	}
	if(!order_queue.isFull()){
		while(!order_queue.write(std::move(temp_order)))
			continue;
	}

	usleep(100);
	return 0;
}
