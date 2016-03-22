#ifndef _PREPROCESS_HH__
#define _PREPROCESS_HH__



#include <cstdlib>  // EXIT_FAILURE
#include <cstring> //wangzhen 0702

#include "data_absyntax/data_absyntax.hh"  // symbol_c type
#include "data_util/symtable.hh"
#include "data_stage4/data_stage4.hh"

#include "program_absyntax/program_absyntax.hh"  // symbol_c type
#include "program_util/symtable.hh"
#include "program_stage4/program_stage4.hh"
#include "stage5/stage5.hh"
#include "stage5_to_6.h"

#include <sys/stat.h>
#include <dirent.h>
#include <limits.h>
#include <regex.h>
#include <sys/types.h>
#include <thread>
#include <functional>

//using namespace std;
//using namespace robot_data_file_process;
//using namespace robot_program_file_process;

/* A macro for printing out internal parser errors... */
#include <iostream> // required for std::cerr
#define ERROR error_exit(__FILE__,__LINE__)


/* forward declarations... */
int data_stage1_2(const char *filename,  robot_data_file_process::symbol_c **tree_root);
bool regex_data_match(char *program_directory, char *program_name);
int data_myftw(char *robot_name, const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);
int data_dopath(const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);

int program_stage1_2(const char *filename,  robot_program_file_process::symbol_c **tree_root);
bool regex_program_match(char *program_directory, char *program_name);
int program_myftw(char *robot_name, const char *project_directory, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);
int program_dopath(const char *project_directory, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);






#endif