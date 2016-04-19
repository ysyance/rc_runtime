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
#define TEST_PROGRAM_FILE_RESULT    0


#include "preprocess.hh"




void error_exit(const char *file_name, int line_no) {
	std::cerr << "\nInternal program error in file " << file_name
		<< " at line " << line_no << "\n\n\n";
	exit(EXIT_FAILURE);
}




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

	/***************************************************/
	/* Part 0102--Process data && program files        */
	/***************************************************/
	int ret = 1; 
	ret = data_myftw(argv[1], project_directory, symtable_of_symtable);
	std::cout << "data file return value: " << ret << std::endl;
	ret = program_myftw(argv[1], project_directory, subprogram_symtable, symtable_of_symtable);
	std::cout << "program file return value: " << ret << std::endl;

//	ret = stage5(symtable_of_symtable, subprogram_symtable, exec_directory, project_directory);
//	std::cout << "stage5 return value: " << ret << std::endl;

	auto order_producer = std::bind(stage5, symtable_of_symtable, subprogram_symtable, exec_directory, project_directory);
	std::thread writer(order_producer);
	std::thread reader(order_consumer);

	reader.join();
	writer.join();
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

