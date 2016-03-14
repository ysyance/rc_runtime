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


#ifndef PROGRAM_STAGE4_H_
#define PROGRAM_STAGE4_H_

#include "../program_absyntax/program_absyntax.hh"
#include "../program_util/symtable.hh"
#include <vector>
#include "program_type.hh"
#include "../data_stage4/data_stage4.hh"

namespace robot_program_file_process{

enum StmtType{ 
	ST_ASSIGN, 
	ST_CALL, 
	ST_IF, 
	ST_WHILE, 
	ST_LOOP, 
	ST_ROBOT
};


class statement_node{
	public:
		StmtType stmt_type;
		union {
			ASSIGN_STMT *assign_stmt;
			CALL_STMT *call_stmt;
			IF_STMT *if_stmt;
			WHILE_STMT *while_stmt;
			LOOP_STMT *loop_stmt;
			ROBOT_STMT *robot_stmt;
		}statement;
		statement_node *next;
		statement_node *outer;

	public:
		statement_node():next(NULL),outer(NULL) {}
		~statement_node() {}
};


typedef symtable_c<statement_node *, 0> DEF_SUBPROGRAM_SYMTABLE;
int program_stage4(char **filename, symbol_c **tree_root, DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable);

}

#endif


