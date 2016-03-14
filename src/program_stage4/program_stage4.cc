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






// #include <stdio.h>  /* required for NULL */
#include <string>
#include <iostream>
#include "program_stage4.hh"
#include "../data_stage4/data_stage4.hh"

namespace robot_program_file_process{

/***********************************************************************/
/***********************************************************************/
/***********************************************************************/
/***********************************************************************/
/***********************************************************************/
/***********************************************************************/
/***********************************************************************/
/***********************************************************************/
/***********************************************************************/
/***********************************************************************/



/* forward declarations... */
/* These functions will be implemented in generate_XXX.cc */
visitor_c *new_stmt_node(robot_data_file_process::DEF_SYMTABLE *filename_symtable);
void delete_stmt_node(visitor_c *code_generator);
visitor_c *delete_tree();
void delete_visitor(visitor_c *delete_tree_visitor);


int program_stage4(char **filename, symbol_c **tree_root, DEF_SUBPROGRAM_SYMTABLE &subprogram_symtable, robot_data_file_process::DEF_SYM_SYM &symtable_of_symtable) {
  statement_node *statement_head = new statement_node();
  robot_data_file_process::DEF_SYMTABLE *filename_symtable = symtable_of_symtable.find_value(*filename);
  visitor_c *generate_code = new_stmt_node(filename_symtable);
  visitor_c *delete_AST = delete_tree();

  if (NULL == generate_code)
    return -1;
  if (NULL == delete_AST)
    return -2;

  statement_head = (statement_node *)((*tree_root)->accept(*generate_code));
  subprogram_symtable.insert(*filename,statement_head);
  delete_stmt_node(generate_code);

  (*tree_root)->accept(*delete_AST);   
  *tree_root = NULL;
  delete_visitor(delete_AST);

  return 0;
}

}

