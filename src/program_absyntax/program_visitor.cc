/*
 * VISITOR.CC
 *
 * Three base implementations of the visitor interface,
 * that may be later extended to execute a particular algorithm.
 *
 * The null (class null_visitor_c) does nothing.
 *
 * The iterator (class iterator_visitor_c) iterates through
 * every object in the syntax tree.
 *
 * The search class (class search_visitor_c) iterates through
 * every object, until one returns a value != NULL.
 */





#include <unistd.h>

#include <stdio.h>  /* required for NULL */
#include "program_visitor.hh"

#include <iostream>

namespace robot_program_file_process{

/******************/
/* visitor_c      */
/******************/

visitor_c::~visitor_c(void) {return;}


/******************/
/* null_visitor_c */
/******************/

null_visitor_c::~null_visitor_c(void) {return;}

#define SYM_LIST(class_name_c)	\
  void *null_visitor_c::visit(class_name_c *symbol) {return NULL;}

#define SYM_TOKEN(class_name_c)	\
  void *null_visitor_c::visit(class_name_c *symbol) {return NULL;}

#define SYM_REF0(class_name_c)	\
  void *null_visitor_c::visit(class_name_c *symbol) {return NULL;}

#define SYM_REF2(class_name_c, ref1, ref2)	\
  void *null_visitor_c::visit(class_name_c *symbol) {return NULL;}

#define SYM_REF4(class_name_c, ref1, ref2, ref3, ref4)	\
  void *null_visitor_c::visit(class_name_c *symbol) {return NULL;}


#include "program_absyntax.def"




#undef SYM_LIST
#undef SYM_TOKEN
#undef SYM_REF0
#undef SYM_REF2
#undef SYM_REF4





/**********************/
/* iterator_visitor_c */
/**********************/

iterator_visitor_c::~iterator_visitor_c(void) {return;}


void *iterator_visitor_c::visit_list(list_c *list) {
  for(int i = 0; i < list->n; i++) {
    list->elements[i]->accept(*this);
  }
  return NULL;
}


#define SYM_LIST(class_name_c)	\
  void *iterator_visitor_c::visit(class_name_c *symbol) {return visit_list(symbol);}

#define SYM_TOKEN(class_name_c)	\
  void *iterator_visitor_c::visit(class_name_c *symbol) {return NULL;}

#define SYM_REF0(class_name_c)	\
  void *iterator_visitor_c::visit(class_name_c *symbol) {return NULL;}

#define SYM_REF2(class_name_c, ref1, ref2)		\
void *iterator_visitor_c::visit(class_name_c *symbol) {	\
  if (symbol->ref1!=NULL) symbol->ref1->accept(*this);	\
  if (symbol->ref2!=NULL) symbol->ref2->accept(*this);	\
  return NULL;						\
}

#define SYM_REF4(class_name_c, ref1, ref2, ref3, ref4)	\
void *iterator_visitor_c::visit(class_name_c *symbol) {	\
  if (symbol->ref1) symbol->ref1->accept(*this);	\
  if (symbol->ref2) symbol->ref2->accept(*this);	\
  if (symbol->ref3) symbol->ref3->accept(*this);	\
  if (symbol->ref4) symbol->ref4->accept(*this);	\
  return NULL;						\
}


#include "program_absyntax.def"



#undef SYM_LIST
#undef SYM_TOKEN
#undef SYM_REF0
#undef SYM_REF2
#undef SYM_REF4









/********************/
/* search_visitor_c */
/********************/

search_visitor_c::~search_visitor_c(void) {return;}


void *search_visitor_c::visit_list(list_c *list) {
  for(int i = 0; i < list->n; i++) {
    void *res = list->elements[i]->accept(*this);
    if (res != NULL)
      return res;
  }
  return NULL;
}


#define SYM_LIST(class_name_c)	\
  void *search_visitor_c::visit(class_name_c *symbol) {return visit_list(symbol);}

#define SYM_TOKEN(class_name_c)	\
  void *search_visitor_c::visit(class_name_c *symbol) {return NULL;}

#define SYM_REF0(class_name_c)	\
  void *search_visitor_c::visit(class_name_c *symbol) {return NULL;}

#define SYM_REF2(class_name_c, ref1, ref2)			\
void *search_visitor_c::visit(class_name_c *symbol) {		\
  void *res = NULL;						\
  if (symbol->ref1) res =  symbol->ref1->accept(*this);		\
  if (res != NULL)  return res;					\
  if (symbol->ref2) return symbol->ref2->accept(*this);		\
  return NULL;							\
}

#define SYM_REF4(class_name_c, ref1, ref2, ref3, ref4)		\
void *search_visitor_c::visit(class_name_c *symbol) {		\
  void *res = NULL;						\
  if (symbol->ref1) res =  symbol->ref1->accept(*this);		\
  if (res != NULL)  return res;					\
  if (symbol->ref2) res =  symbol->ref2->accept(*this);		\
  if (res != NULL)  return res;					\
  if (symbol->ref3) res =  symbol->ref3->accept(*this);		\
  if (res != NULL)  return res;					\
  if (symbol->ref4) return symbol->ref4->accept(*this);		\
  return NULL;							\
}



#include "program_absyntax.def"



#undef SYM_LIST
#undef SYM_TOKEN
#undef SYM_REF0
#undef SYM_REF2
#undef SYM_REF4



}








