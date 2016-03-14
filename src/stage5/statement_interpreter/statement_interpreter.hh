#ifndef _STATEMENT_INTERPRETER_H_
#define _STATEMENT_INTERPRETER_H_

#include "../../data_stage4/data_stage4.hh"
#include "../../program_stage4/program_stage4.hh"

int assign_interpreter(robot_program_file_process::ASSIGN_STMT *assign_stmt,const  char *exec_directory, const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable);
int elseif_interpreter(robot_program_file_process::ELSEIF_STMT *elseif_stmt,const  char *exec_directory, const char *project_directory,robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, bool &flag, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable);
int if_interpreter(robot_program_file_process::IF_STMT *if_stmt,const  char *exec_directory, const char *project_directory,robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable);
//int while_interpreter(robot_program_file_process::WHILE_STMT *while_stmt,const  char *exec_directory, const char *project_directory,robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable);
int loop_interpreter(robot_program_file_process::LOOP_STMT *loop_stmt, const char *exec_directory, const char *project_directory,robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable);
int robot_interpreter(robot_program_file_process::ROBOT_STMT *robot_stmt, const char *exec_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);

#endif
