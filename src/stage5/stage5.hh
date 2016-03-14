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


#ifndef _STAGE5_H_
#define _STAGE5_H_

#include "../data_stage4/data_stage4.hh"
#include "../program_stage4/program_stage4.hh"

int stage5(robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable,const  char *exec_directory, const char *project_directory);
int stage5_core(robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable, robot_program_file_process::DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_program_file_process::statement_node *exec_program_head, const char *exec_directory,  const char *project_directory);


#endif


