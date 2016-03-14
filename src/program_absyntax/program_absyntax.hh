/*
 * ABSYNTAX.H
 *
 * This generates the parse tree structure used to bind the components
 * identified by Bison in the correct syntax order. At the end of the
 * Bison analysis the tree is walked in a sequential fashion generating
 * the relavent code.
 */



#ifndef _PROGRAM_ABSYNTAX_HH
#define _PROGRAM_ABSYNTAX_HH


#include <stdio.h> // required for NULL

/* Forward declaration of the visitor interface
 * dclared in the visitor.hh file
 * We cannot include the visitor.hh file, as it will
 * include this same file first, as it too requires references
 * to the abstract syntax classes defined here.
 */

namespace robot_program_file_process{

class visitor_c; // forward declaration


class symbol_c; // forward declaration
//extern symbol_c *tree_root;



/* The base class of all symbols */
class symbol_c {

  public:
    /*
     * Line number for the purposes of error checking
     */
    long lineno;

  public:
    /* default constructor */
    symbol_c(void);
    symbol_c(long lineno);

    /* default destructor */
    /* must be virtual so compiler does not complain... */ 
    virtual ~symbol_c(void) {return;};

    virtual void *accept(visitor_c &visitor) {return NULL;};
};


class token_c: public symbol_c {
  public:
    /* the value of the symbol. */
    const char *value;

  public:
    token_c(const char *value);
};


 /* a list of symbols... */
class list_c: public symbol_c {
  public:
    int n;
    symbol_c **elements;

  public:
    list_c(void);
    list_c(symbol_c *elem);
     /* insert a new element */
    virtual void add_element(symbol_c *elem);
};




#define SYM_LIST(class_name_c)			\
class class_name_c:	public list_c {		\
  public:					\
    virtual void *accept(visitor_c &visitor);	\
};


#define SYM_TOKEN(class_name_c)			\
class class_name_c: 	public token_c {	\
  public:					\
    class_name_c(const char *value);			\
    virtual void *accept(visitor_c &visitor);	\
};


#define SYM_REF0(class_name_c)			\
class class_name_c: public symbol_c {		\
  public:					\
    virtual void *accept(visitor_c &visitor);	\
};


#define SYM_REF2(class_name_c, ref1, ref2)	\
class class_name_c: public symbol_c {		\
  public:					\
    symbol_c *ref1;				\
    symbol_c *ref2;				\
  public:					\
    class_name_c(symbol_c *ref1,		\
		 symbol_c *ref2 = NULL);	\
    virtual void *accept(visitor_c &visitor);	\
};

#define SYM_REF4(class_name_c, ref1, ref2, ref3, ref4)	\
class class_name_c: public symbol_c {			\
  public:						\
    symbol_c *ref1;					\
    symbol_c *ref2;					\
    symbol_c *ref3;					\
    symbol_c *ref4;					\
  public:						\
    class_name_c(symbol_c *ref1,			\
		 symbol_c *ref2,			\
		 symbol_c *ref3,			\
		 symbol_c *ref4 = NULL);		\
    virtual void *accept(visitor_c &visitor);		\
};


#include "program_absyntax.def"



#undef SYM_LIST
#undef SYM_TOKEN
#undef SYM_REF0
#undef SYM_REF2
#undef SYM_REF4


}

#endif /*  _ABSYNTAX_HH */


