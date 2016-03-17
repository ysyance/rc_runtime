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


#define TEST_DATA_FILE_RESULT       0
#define TEST_PROGRAM_FILE_RESULT    1

#include <cstdlib>  // EXIT_FAILURE
#include <cstring> //wangzhen 0702

#include "data_absyntax/data_absyntax.hh"  // symbol_c type
#include "data_util/symtable.hh"
#include "data_stage4/data_stage4.hh"

#include "program_absyntax/program_absyntax.hh"  // symbol_c type
#include "program_util/symtable.hh"
#include "program_stage4/program_stage4.hh"
#include "stage5/stage5.hh"

#include <sys/stat.h>
#include <dirent.h>
#include <limits.h>
#include <regex.h>
#include <sys/types.h>

//using namespace std;
//using namespace robot_data_file_process;
//using namespace robot_program_file_process;

/* A macro for printing out internal parser errors... */
#include <iostream> // required for std::cerr
#define ERROR error_exit(__FILE__,__LINE__)
void error_exit(const char *file_name, int line_no) {
	std::cerr << "\nInternal program error in file " << file_name
		<< " at line " << line_no << "\n\n\n";
	exit(EXIT_FAILURE);
}



/* forward declarations... */
int data_stage1_2(const char *filename,  robot_data_file_process::symbol_c **tree_root);
bool regex_data_match(char *program_directory, char *program_name);
int data_myftw(char *robot_name, const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);
int data_dopath(const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);

int program_stage1_2(const char *filename,  robot_program_file_process::symbol_c **tree_root);
bool regex_program_match(char *program_directory, char *program_name);
int program_myftw(char *robot_name, const char *project_directory, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);
int program_dopath(const char *project_directory, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);

/***************************************************/
/***************************************************/
/*Section 01  : Process data && program files...   */
/*Author      : Wang Zhen                          */
/*Create Date : 2014.12.29                         */
/*Last Update : 2014.12.30                         */
/***************************************************/
/***************************************************/


int main(int argc, char **argv) {

	/***************************************************/
	/* Part 0101--Key intermediate data structures     */
	/***************************************************/
	robot_data_file_process::DEF_SYM_SYM symtable_of_symtable;
	robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE subprogram_symtable;
	const char *project_directory = "/root/workspace/RobotControl/rc_runtime/test/lab";
	const char *exec_directory = "/root/workspace/RobotControl/rc_runtime/test/lab/control";

	/*if(argc < 3){
		printf("lack of arguements !\n");
		exit(1);
	}*/

	/***************************************************/
	/* Part 0102--Process data && program files        */
	/***************************************************/
	int ret = 1; 
	ret = data_myftw(argv[1], project_directory, symtable_of_symtable);
	std::cout << "data file return value: " << ret << std::endl;
	ret = program_myftw(argv[1], project_directory, subprogram_symtable, symtable_of_symtable);
	std::cout << "program file return value: " << ret << std::endl;

	ret = stage5(symtable_of_symtable, subprogram_symtable, exec_directory, project_directory);
	std::cout << "stage5 return value: " << ret << std::endl;


#if TEST_DATA_FILE_RESULT
	/***************************************************/
	/* Part 0103--Test result(data files)              */
	/***************************************************/
	for(robot_data_file_process::DEF_SYM_SYM::iterator i = symtable_of_symtable.begin();
			i != symtable_of_symtable.end();
			i++)
	{
		std::cout << "Data file's name is :" << i->first << "\n";
		std::cout << "=====================\n";
		for(robot_data_file_process::DEF_SYMTABLE::iterator j = i->second->begin(); j != i->second->end(); ++j)
		{
			std::cout << "Variable's name:" << j->first;
			switch(*(j->second->id_type))
			{
				case robot_data_file_process::TYPE_BOOL:
					std::cout << "||Type: BOOL" << std::endl;
					std::cout << *(j->second->id_value.bv) << std::endl;
					break;
				case robot_data_file_process::TYPE_DINT:
					std::cout << "||Type: DINT" << std::endl;
					std::cout << *(j->second->id_value.iv) << std::endl;
					break;
				case robot_data_file_process::TYPE_REAL:
					std::cout << "||Type: REAL" << std::endl;
					std::cout << *(j->second->id_value.dv) << std::endl;
					break;
				case robot_data_file_process::TYPE_STRING:
					std::cout << "||Type: STRING" << std::endl;
					std::cout << *(j->second->id_value.sv) << std::endl;
					break;
				case robot_data_file_process::TYPE_AXISPOS:
					std::cout << "||Type: AXISPOS" << std::endl;
					j->second->id_value.apv->print();
					break;
				case robot_data_file_process::TYPE_CARTPOS:
					std::cout << "||Type: CARTPOS" << std::endl;
					j->second->id_value.cpv->print();
					break;
				case robot_data_file_process::TYPE_ROBAXISPOS:
					std::cout << "||Type: ROBAXISPOS" << std::endl;
					j->second->id_value.rapv->print();
					break;
				case robot_data_file_process::TYPE_AUXAXISPOS:
					std::cout << "||Type: AUXAXISPOS" << std::endl;
					j->second->id_value.aapv->print();
				case robot_data_file_process::TYPE_ROBCARTPOS:
					std::cout << "||Type: ROBCARTPOS" << std::endl;
					j->second->id_value.rcpv->print();
					break;
				case robot_data_file_process::TYPE_CARTREFSYS:
					std::cout << "||Type: CARTREFSYS" << std::endl;
					j->second->id_value.crsv->print();
					break;
				case robot_data_file_process::TYPE_TOOL:
					std::cout << "||Type: TOOL" << std::endl;
					j->second->id_value.tv->print();
					break;
				case robot_data_file_process::TYPE_OVLREL:
					std::cout << "||Type: OVLREL" << std::endl;
					j->second->id_value.orv->print();
					break;
				case robot_data_file_process::TYPE_OVLABS:
					std::cout << "||Type: OVLABS" << std::endl;
					j->second->id_value.oav->print();
					break;
				case robot_data_file_process::TYPE_DYNAMIC:
					std::cout << "||Type: DYNAMIC" << std::endl;
					j->second->id_value.dynv->print();
					break;
				case robot_data_file_process::TYPE_PERCENT:
					std::cout << "||Type: PERCENT" << std::endl;
					j->second->id_value.pv->print();
					break;
				case robot_data_file_process::TYPE_PERC200:
					std::cout << "||Type: PERC200" << std::endl;
					j->second->id_value.p2v->print();
					break;
				default:
					std::cout << "Error type!" << std::endl;
					break;
			}
		}
	}

#endif /*TEST_DATA_FILE_RESULT */

#if TEST_PROGRAM_FILE_RESULT
	/***************************************************/
	/* Part 0104--Test result(program files)           */
	/***************************************************/

	for(robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE::iterator i = subprogram_symtable.begin();
			i != subprogram_symtable.end();
			i++)
	{
		std::cout << "Program file's name is :" << i->first << "\n";
		std::cout << "=====================\n";
	}

#endif /*TEST_PROGRAM_FILE_RESULT */




	return 0;
}

/******************************************/
/******************************************/
/*Section 02  : Process data file...      */
/*Author      : Wang Zhen                 */
/*Create Date : 2014.12.29                */
/*Last Update : 2014.12.29                */
/******************************************/
/******************************************/

bool regex_data_match(char *program_directory, char **program_name)
{
	char *pattern = (char *) std::string("(.+).tid$").c_str();
	char errbuf[1024];
	regex_t reg;
	int err = -1,nm = 2;
	regmatch_t pmatch[2];

	if(regcomp(&reg, pattern, REG_EXTENDED) < 0)
	{
		regerror(err, &reg, errbuf, sizeof(errbuf));
		std::cout << "err: " << errbuf << std::endl;
	}
	err = regexec(&reg,  program_directory, nm, pmatch, 0);
	if( !err )
	{
		for(int i = 1; i < 2 && pmatch[i].rm_so != -1; ++i)
		{
			int len = pmatch[i].rm_eo - pmatch[i].rm_so;
			if(len)
			{
				*program_name = new char[len + 1](); //each member is '\0'
				memcpy(*program_name,  program_directory+pmatch[i].rm_so, len);
			}
		}
		return true;
	}
	else 
		return false;
}

#define pathmax 50

static char *data_fullpath;
static size_t data_pathlen;
static int data_flag = 1;


int data_myftw(char *robot_name, const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable)
{
	data_pathlen = pathmax;
	data_fullpath = new char[pathmax];
	if(data_pathlen <= strlen(robot_name))
	{
		data_pathlen = strlen(robot_name) * 2;
		if( (data_fullpath = (char *)realloc(data_fullpath, data_pathlen)) == NULL )
			std::cout << "realloc failed" << std::endl;
	}
	strcpy(data_fullpath, robot_name);
	return(data_dopath(project_directory, symtable_of_symtable));
}

int data_dopath(const char *project_directory, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable)
{
	robot_data_file_process::symbol_c *tree_root;
	char *program_name;
	struct stat statbuf;
	struct dirent *dirp;
	DIR *dp;
	int ret = 0, n;
	if(lstat(data_fullpath, &statbuf) < 0)
	{
		std::cout << "stat error for " << data_fullpath << std::endl;
		return -1;
	}
	if(S_ISDIR(statbuf.st_mode) == 0)
	{
		std::cout << data_fullpath << " is a file " << std::endl;

		if(regex_data_match(data_fullpath, &program_name)) //.tid data file
		{
			/* 1st Pass */
			if (data_stage1_2(data_fullpath, &tree_root) < 0)
				return -2;

			/* 4rd Pass */
			if (robot_data_file_process::data_stage4(&program_name, &tree_root, symtable_of_symtable) < 0) //Pay attention to the fisrt parameter!
				return -3;
		}
		return 0;
	}
	n = strlen(data_fullpath);
	if(n + NAME_MAX + 2 > data_pathlen)
	{
		data_pathlen *= 2;
		if( (data_fullpath = (char *)realloc(data_fullpath, data_pathlen)) == NULL)
			std::cout << "realloc failed " << std::endl;
	}
	data_fullpath[n++] = '/';
	data_fullpath[n] = 0; //equals '\0'
	if((dp = opendir(data_fullpath)) == NULL)
	{
		std::cout << "can't read directory " << data_fullpath << std::endl;
		return -4;
	}

	if( (!strncmp(data_fullpath,project_directory,8) && data_flag) || !strncmp(data_fullpath,project_directory,strlen(project_directory)))
	{
		data_flag = 0;
		while((dirp = readdir(dp)) != NULL)
		{
			if(strcmp(dirp->d_name, ".") == 0 || strcmp(dirp->d_name, "..") == 0)
				continue;
			strcpy(&data_fullpath[n], dirp->d_name);
			ret = data_dopath(project_directory, symtable_of_symtable);
		}
		data_fullpath[n-1] = 0; //equals '\0'
	}

	if(closedir(dp) < 0)
		std::cout << "can't close directory " << data_fullpath << std::endl;
	return(ret);
}

/******************************************/
/******************************************/
/*Section 03  : Process program file...   */
/*Author      : Wang Zhen                 */
/*Create Date : 2014.12.29                */
/*Last Update : 2014.12.29                */
/******************************************/
/******************************************/
bool regex_program_match(char *program_directory, char **program_name)
{
	char *pattern = (char *) std::string("(.+).tip$").c_str();
	char errbuf[1024];
	regex_t reg;
	int err = -1,nm = 2;
	regmatch_t pmatch[2];

	if(regcomp(&reg, pattern, REG_EXTENDED) < 0)
	{
		regerror(err, &reg, errbuf, sizeof(errbuf));
		std::cout << "err: " << errbuf << std::endl;
	}
	err = regexec(&reg,  program_directory, nm, pmatch, 0);
	if( !err )
	{
		for(int i = 1; i < 2 && pmatch[i].rm_so != -1; ++i)
		{
			int len = pmatch[i].rm_eo - pmatch[i].rm_so;
			if(len)
			{
				*program_name = new char[len + 1](); //each member is '\0'
				memcpy(*program_name,  program_directory+pmatch[i].rm_so, len);
			}
		}
		return true;
	}
	else 
		return false;
}

//#define pathmax 50

static char *program_fullpath;
static size_t program_pathlen;
static int program_flag = 1;

int program_myftw(char *robot_name, const char *project_directory, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable)
{
	program_pathlen = pathmax;
	program_fullpath = new char[pathmax];
	if(program_pathlen <= strlen(robot_name))
	{
		program_pathlen = strlen(robot_name) * 2;
		if( (program_fullpath = (char *)realloc(program_fullpath, program_pathlen)) == NULL )
			std::cout << "realloc failed" << std::endl;
	}
	strcpy(program_fullpath, robot_name);
	return(program_dopath(project_directory, subprogram_symtable, symtable_of_symtable));
}

int program_dopath(const char *project_directory, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable)
{
	robot_program_file_process::symbol_c *tree_root;
	char *program_name;
	struct stat statbuf;
	struct dirent *dirp;
	DIR *dp;
	int ret = 0, n;
	if(lstat(program_fullpath, &statbuf) < 0)
	{
		std::cout << "stat error for " << program_fullpath << std::endl;
		return -1;
	}
	if(S_ISDIR(statbuf.st_mode) == 0)
	{
		std::cout << program_fullpath << " is a file " << std::endl;

		if(regex_program_match(program_fullpath, &program_name)) //.tid data file
		{
			/* 1st Pass */
			if (program_stage1_2(program_fullpath, &tree_root) < 0)
				return -2;

			/* 4rd Pass */
			if (robot_program_file_process::program_stage4(&program_name, &tree_root, subprogram_symtable, symtable_of_symtable) < 0) //Pay attention to the fisrt parameter!
				return -3;
		}
		return 0;
	}
	n = strlen(program_fullpath);
	if(n + NAME_MAX + 2 > program_pathlen)
	{
		program_pathlen *= 2;
		if( (program_fullpath = (char *)realloc(program_fullpath, program_pathlen)) == NULL)
			std::cout << "realloc failed " << std::endl;
	}
	program_fullpath[n++] = '/';
	program_fullpath[n] = 0; //equals '\0'
	if((dp = opendir(program_fullpath)) == NULL)
	{
		std::cout << "can't read directory " << program_fullpath << std::endl;
		return -4;
	}

	if( (!strncmp(program_fullpath,project_directory,8) && program_flag) || !strncmp(program_fullpath,project_directory,strlen(project_directory)))
	{
		program_flag = 0;
		while((dirp = readdir(dp)) != NULL)
		{
			if(strcmp(dirp->d_name, ".") == 0 || strcmp(dirp->d_name, "..") == 0)
				continue;
			strcpy(&program_fullpath[n], dirp->d_name);
			ret = program_dopath(project_directory, subprogram_symtable, symtable_of_symtable);
		}
		program_fullpath[n-1] = 0; //equals '\0'
	}

	if(closedir(dp) < 0)
		std::cout << "can't close directory " << program_fullpath << std::endl;
	return(ret);
}

