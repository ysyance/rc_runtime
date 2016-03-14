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
//#include <string>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include "data_stage4.hh"

namespace robot_data_file_process{


//#define def_symtable(filename) {\
//  symtable_c<sym_element, NULL> filename ## _symtable;
//}

//static symtable_c<int, BOGUS_TOKEN_ID> variable_name_symtable;

/*
 * Join two strings together. Allocate space with malloc(3).
 
static char *strdup2(const char *a, const char *b) {
  char *res = (char *)malloc(strlen(a) + strlen(b) + 1);

  if (!res)
  {
    printf("malloc failed!\n");
    return NULL;
  }
  return strcat(strcpy(res, a), b);  
}
*/

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
visitor_c *new_code_generator(DEF_SYMTABLE *filename_symtable);
void delete_code_generator(visitor_c *code_generator);
visitor_c *delete_tree();
void delete_visitor(visitor_c *delete_tree_visitor);


int data_stage4(char **filename, symbol_c **tree_root, DEF_SYM_SYM &symtable_of_symtable) {
  DEF_SYMTABLE *filename_symtable = new DEF_SYMTABLE();
  visitor_c *generate_code = new_code_generator(filename_symtable);
  visitor_c *delete_AST = delete_tree();

  if (NULL == generate_code)
    return -1;

  (*tree_root)->accept(*generate_code);
  symtable_of_symtable.insert(*filename,filename_symtable);
  delete_code_generator(generate_code);

  (*tree_root)->accept(*delete_AST);
  *tree_root = NULL;
  delete_visitor(delete_AST);

  return 0;
}

}
