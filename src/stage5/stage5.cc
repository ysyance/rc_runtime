/*
 * (c) 2003 Mario de Sousa
 *
 * Offered to the public under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * This code is made available on the understanding that it will not be
 * used in safety-critical situations without a full and competent review.
 */

/*
 * An IEC 61131-3 IL and ST compiler.
 *
 * Based on the
 * FINAL DRAFT - IEC 61131-3, 2nd Ed. (2001-12-10)
 *
 */


/*
 * This file contains the code that stores the output generated
 * by each specific version of the 4th stage.
 */


#include <cstring>
#include "../data_util/symtable.hh"
#include "../program_util/symtable.hh"
//#include "../data_stage4/data_stage4.hh"
//#include "../program_stage4/program_stage4.hh"
#include "statement_interpreter/statement_interpreter.hh"


#include "stage5.hh"

int stage5(robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable,const char *exec_directory,const char *project_directory)
{
	robot_program_file_process::statement_node *exec_program_head = subprogram_symtable.find_value(exec_directory);
	if(exec_program_head == NULL)
	{
	}
	int ret;
	ret = stage5_core(symtable_of_symtable, subprogram_symtable, exec_program_head, exec_directory, project_directory);

	return ret;
}

int stage5_core(robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_program_file_process::statement_node *exec_program_head, const char *exec_directory, const  char *project_directory) //inline
{
	robot_program_file_process::statement_node *pStmt = exec_program_head;
	int ret = -10;

	while(pStmt != NULL)
	{

		switch(pStmt->stmt_type)
		{
			case robot_program_file_process::ST_ASSIGN:
				ret = assign_interpreter(pStmt->statement.assign_stmt, exec_directory, project_directory, symtable_of_symtable, subprogram_symtable);
				break;
			case robot_program_file_process::ST_CALL:
				{
					char *call_program = new char[strlen(pStmt->statement.call_stmt->subprogram_name) + strlen(project_directory) + 2];
					if(call_program == NULL)
					{

					}
					strcpy(call_program, project_directory);
					call_program = strcat(strcat(call_program, "/"),pStmt->statement.call_stmt->subprogram_name);
					ret = stage5(symtable_of_symtable, subprogram_symtable, call_program, project_directory);
				}
				break;
			case robot_program_file_process::ST_IF:
				ret = if_interpreter(pStmt->statement.if_stmt, exec_directory, project_directory, symtable_of_symtable, subprogram_symtable);
				break;
			//case robot_program_file_process::ST_WHILE:
			//	ret = while_interpreter(pStmt->statement.while_stmt, exec_directory, project_directory, symtable_of_symtable, subprogram_symtable);
			//	break;
			case robot_program_file_process::ST_LOOP:
				ret = loop_interpreter(pStmt->statement.loop_stmt, exec_directory, project_directory, symtable_of_symtable, subprogram_symtable);
				break;
			case robot_program_file_process::ST_ROBOT:
				ret = robot_interpreter(pStmt->statement.robot_stmt, exec_directory, symtable_of_symtable);
				break;
                        default:
                                std::cout << "can not process this type" << std::endl;
                                break;
		}
		if(ret == 0)
			pStmt = pStmt->next;
		else 
			return ret;
	}
	return ret;
}










