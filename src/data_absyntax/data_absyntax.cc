/*
 * Definition of the Abstract Syntax data structure components
 */

#include <stdio.h>
#include <stdlib.h>	/* required for exit() */
#include <string.h>

#include "data_absyntax.hh"
//#include "../stage1_2/iec.hh" /* required for BOGUS_TOKEN_ID, etc... */
#include "data_visitor.hh"

namespace robot_data_file_process{

#define ABORT(str) {printf("ERROR: %s\n", str); exit(0);}

//symbol_c *tree_root = NULL;


/* The base class of all symbols */
symbol_c::symbol_c(void) {
  lineno = 0;
}

symbol_c::symbol_c(long lineno) {
  this->lineno = lineno;
}



token_c::token_c(const char *value) {
  this->value = value;
//  printf("New token: %s\n", value);
}






list_c::list_c(void) {
  n = 0;
  elements = NULL;
}

list_c::list_c(symbol_c *elem) {
  n = 0;
  elements = NULL;
  add_element(elem);
}

/* insert a new element */
void list_c::add_element(symbol_c *elem) {
//printf("list_c::add_element()\n");
  n++;
  elements = (symbol_c **)realloc(elements, n * sizeof(symbol_c *));
  if (elements == NULL)
    ABORT("Out of memory");
  elements[n - 1] = elem;
}





#define SYM_LIST(class_name_c)							\
void *class_name_c::accept(visitor_c &visitor) {return visitor.visit(this);}

#define SYM_TOKEN(class_name_c)							\
class_name_c::class_name_c(const char *value): token_c(value) {}			\
void *class_name_c::accept(visitor_c &visitor) {return visitor.visit(this);}

#define SYM_REF0(class_name_c)			\
void *class_name_c::accept(visitor_c &visitor) {return visitor.visit(this);}


#define SYM_REF2(class_name_c, ref1, ref2)	\
class_name_c::class_name_c(symbol_c *ref1,	\
			   symbol_c *ref2) {	\
  this->ref1 = ref1;				\
  this->ref2 = ref2;				\
}						\
void *class_name_c::accept(visitor_c &visitor) {return visitor.visit(this);}





#include "data_absyntax.def"




#undef SYM_LIST
#undef SYM_TOKEN
#undef SYM_TOKEN
#undef SYM_REF0
#undef SYM_REF2

}

