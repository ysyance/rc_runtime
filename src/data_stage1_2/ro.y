/**************************************************************/
/**************************************************************/
/*File Description  : Robot Data files interpreter with bison */
/*Author            : Wang Zhen                               */
/*Create Date       : 2014.06.10                              */
/*Last Update       : 2014.11.28                              */
/**************************************************************/
/**************************************************************/

/******************************************/
/******************************************/
/*Section 01  : Definitions...            */
/*Author      : Wang Zhen                 */
/*Create Date : 2014.06.10                */
/*Last Update : 2014.07.01                */
/******************************************/
/******************************************/

%{
/*******************************************************/
/*Part 0101   : Macro definitions and declarations of  */
/*              funtions and variables that are used   */
/*              in the actions in the grammar rules... */
/*Author      : Wang Zhen                              */
/*Create Date : 2014.06.10                             */
/*Last Update : 2014.07.01                             */
/*******************************************************/

/* required for strdup()  */
#include <string.h>	

/* declare the token parser generated by flex... */
int data_yylex(void);

/* declare the error handler defined at the end of this file */
void data_yyerror (const char *error_msg);

/* produce a more verbose parsing error message */
#define YYERROR_VERBOSE

/* Include debuging code.
 * Printing of debug info must then be activated by setting
 * the variable yydebug to 1.
 */
#define YYDEBUG 0

/* file with declaration of absyntax classes... */
#include "../data_absyntax/data_absyntax.hh"

/* file with declaration of token constants. Generated by bison! */
#include "ro.y.hh"

/* file with the declarations of symbol tables... */
#include "../data_util/symtable.hh"


/*********************************/
/* The global symbol tables...   */
/*********************************/
/* NOTE: declared static because they are not accessed
 *       directly by the lexical parser (flex), but rather
 *       through the function get_identifier_token()
 */
static robot_data_file_process::symtable_c<int, BOGUS_TOKEN_ID> variable_name_symtable;

/*************************/
/* global variables...   */
/*************************/
static robot_data_file_process::symbol_c *tree_root = NULL;

/* The name of the file currently being parsed...
 * Note that flex accesses and updates this global variable
 */
const char *data_current_filename = NULL;

%}

/*******************************************************/
/*Part 0102   : Bison declarations that define         */
/*              terminal and nonterminal symbols,      */
/*              specify precedence, and so on...       */
/*Author      : Wang Zhen                              */
/*Create Date : 2014.06.10                             */
/*Last Update : 2014.07.01                             */
/*******************************************************/


/*******************************************/
/*  - Use another name      */
/*******************************************/

%name-prefix "data_yy"

/*******************************************/
/*  -      */
/*******************************************/
%union {
    robot_data_file_process::symbol_c 	*leaf;
    robot_data_file_process::list_c	*list;
    char 	*ID;	/* token value */
}

/*****************************/
/* Prelimenary constructs... */
/*****************************/
%type <leaf> start
%type <leaf>	any_identifier
%token BOGUS_TOKEN_ID

/***************************/
/* B 0 - Programming Model */
/***************************/
%type <list>	library
 
/*******************************************/
/* B 1.1 - Identifiers                     */
/*******************************************/

%token <ID>	identifier_token 
%type <leaf>	identifier

%token <ID>	prev_declared_variable_name_token
%type <leaf>	prev_declared_variable_name

/*********************/
/* B 1.2 - Constants */
/*********************/
%type <leaf>    constant 

/******************************/
/* B 1.2.1 - Numeric Literals */
/******************************/
%type <leaf>    numeric_literal

//%token <ID>	binary_integer_token
//%type <leaf>    binary_integer

//%token <ID>	octal_integer_token
//%type <leaf>    octal_integer

//%token <ID>	hex_integer_token
//%type <leaf>    hex_integer

%type <leaf>    integer_literal
%type <leaf>    signed_integer

%token <ID>	integer_token 	
%type <leaf>    integer 

%type <leaf>    real_literal
%type <leaf>    signed_real

%token <ID>	real_token
%type <leaf>    real 

//%type <leaf>    bit_string_literal
%type <leaf>    boolean_literal	

%token TRUE
%token FALSE
/*******************************/
/* B 1.2.2 - Character Strings */
/*******************************/
%token <ID>	character_string_token
%type <leaf>    character_string

/**********************/
/* B 1.3 - Data Types */
/**********************/

/***********************************/
/* B 1.3.1 - Elementary Data Types */
/***********************************/
%type <leaf>    elementary_type_name
%type <leaf>    numeric_type_name
%type <leaf>    signed_integer_type_name
%type <leaf>    real_type_name
//%type <leaf>    bit_string_type_name

%type <leaf>    elementary_string_type_name

%token BOOL
%token DINT
%token REAL
//%token DWORD

%token STRING

/********************************/
/* B 1.3.2 - Robot data types */
/********************************/
%type <leaf> robot_data_type_name
%type <leaf> position_data_type_name
%type <leaf> reference_system_data_type_name
%type <leaf> tool_data_type_name
%type <leaf> overlapping_data_type_name
%type <leaf> dynamic_data_type_name
%type <leaf> percentage_data_type_name

%token AXISPOS
%token CARTPOS 
//%token AXISPOSEXT
//%token CARTPOSEXT
%token ROBAXISPOS
%token AUXAXISPOS
%token ROBCARTPOS
//%token WORLDREFSYS
%token CARTREFSYS
//%token CARTREFSYSEXT
//%token CARTREFSYSAXIS
%token TOOL
//%token TOOLSTATIC
%token OVLREL
%token OVLABS
%token DYNAMIC 
%token WORLD
%token PERCENT
%token PERC200

/********************************/
/* B 1.3.3 - Derived data types */
/********************************/

%type  <leaf>	var_init_decl
%type  <leaf>	var1_init_decl
%type  <list>	var1_list

%type <leaf>    simple_spec_init

%type  <leaf>	structured_var_init_decl
%type  <leaf>	initialized_structure
%type <leaf>    structure_initialization 
%type <list>    structure_element_initialization_list 
%type <leaf>    structure_element_initialization 
%type <leaf>    structure_element_name 

%type  <list>	program_var_declarations_list

%token ASSIGN

	


%%
/******************************************/
/******************************************/
/*Section 02  : Grammar rules...          */
/*Author      : Wang Zhen                 */
/*Create Date : 2014.06.10                */
/*Last Update : 2014.07.01                */
/******************************************/
/******************************************/

/*****************************/
/* Prelimenary constructs... */
/*****************************/
start:
  library	{$$ = $1;}
;

any_identifier:
  identifier
| prev_declared_variable_name
;

prev_declared_variable_name: prev_declared_variable_name_token {$$ = new robot_data_file_process::identifier_c($1);};


/***************************/
/* B 0 - Programming Model */
/***************************/
library:
  program_var_declarations_list
;

program_var_declarations_list:
/* empty */
	{ /*if (tree_root == NULL)         Here should be commented! */
	  tree_root = new robot_data_file_process::var_declarations_list_c();
	 $$ = (robot_data_file_process::list_c *)tree_root;
	}
| program_var_declarations_list var_init_decl
        {$$ = $1; $$->add_element($2);}
;


/***************************/
/* B 1.1 - Identifiers     */
/***************************/
identifier:
  identifier_token	{$$ = new robot_data_file_process::identifier_c($1);} /*Attention please! This should not be placed at the first place of this part!*/
;

/*********************/
/* B 1.2 - Constants */
/*********************/
constant:
  numeric_literal
| character_string
//| bit_string_literal
| boolean_literal
;

/******************************/
/* B 1.2.1 - Numeric Literals */
/******************************/
numeric_literal:
  integer_literal
| real_literal
;

integer_literal: 
  signed_integer
        {$$ = new robot_data_file_process::integer_literal_c(new robot_data_file_process::dint_type_name_c(), $1);}
//| binary_integer
//	{$$ = new integer_literal_c(new dint_type_name_c(), $1);}
//| octal_integer
//	{$$ = new integer_literal_c(new dint_type_name_c(), $1);}
//| hex_integer
//	{$$ = new integer_literal_c(new dint_type_name_c(), $1);}
;

signed_integer:
  integer
| '+' integer   {$$ = $2;}
| '-' integer	{$$ = new robot_data_file_process::neg_int_expression_c($2);}
;

real_literal: signed_real
    {$$ = new robot_data_file_process::real_literal_c(new robot_data_file_process::real_type_name_c(), $1);}
;

signed_real:
  real	
| '+' real	{$$ = $2;}
| '-' real	{$$ = new robot_data_file_process::neg_real_expression_c($2);}
;

real:
  real_token		{$$ = new robot_data_file_process::real_c($1);}
;

/* ATTENTION PLEASE! integer will prouce reduce/reduce conflicts! You can use
*            bit_string_literal:
*                bit_string_type_name '#' integer  
*	              {$$ = new bit_string_literal_c($1, $3);}
*to solve this bug!
*/
/*
bit_string_literal:
  '#' integer    
        {$$ = new bit_string_literal_c(new dword_type_name_c(), $2);} 
| '#' binary_integer
	{$$ = new bit_string_literal_c(new dword_type_name_c(), $2);}
| '#' octal_integer
	{$$ = new bit_string_literal_c(new dword_type_name_c(), $2);}
| '#' hex_integer
	{$$ = new bit_string_literal_c(new dword_type_name_c(), $2);}
;
*/

integer:	integer_token		{$$ = new robot_data_file_process::integer_c($1);};

/*
binary_integer:	binary_integer_token	{$$ = new binary_integer_c($1);};
octal_integer:	octal_integer_token	{$$ = new octal_integer_c($1);};
hex_integer:	hex_integer_token	{$$ = new hex_integer_c($1);};
*/

boolean_literal:
  TRUE	{$$ = new robot_data_file_process::boolean_literal_c(new robot_data_file_process::bool_type_name_c(),
  				    new robot_data_file_process::boolean_true_c());}
| FALSE	{$$ = new robot_data_file_process::boolean_literal_c(new robot_data_file_process::bool_type_name_c(),
				    new robot_data_file_process::boolean_false_c());}
;

/*******************************/
/* B 1.2.2 - Character Strings */
/*******************************/
character_string:   character_string_token
        {$$ = new robot_data_file_process::character_string_c($1);}
;

/**********************/
/* B 1.3 - Data Types */
/**********************/

/***********************************/
/* B 1.3.1 - Elementary Data Types */
/***********************************/
elementary_type_name:
  numeric_type_name
//| bit_string_type_name
| elementary_string_type_name
| BOOL		{$$ = new robot_data_file_process::bool_type_name_c();}
;

numeric_type_name:
  signed_integer_type_name
| real_type_name
;

signed_integer_type_name:
  DINT	{$$ = new robot_data_file_process::dint_type_name_c();}
;

real_type_name:
  REAL	{$$ = new robot_data_file_process::real_type_name_c();}
;

/*
bit_string_type_name:
  DWORD	{$$ = new dword_type_name_c();}
;
*/

elementary_string_type_name:
  STRING	{$$ = new robot_data_file_process::string_type_name_c();}
;

/********************************/
/* B 1.3.2 - Robot data types */
/********************************/
robot_data_type_name:
  position_data_type_name
| reference_system_data_type_name
| tool_data_type_name
| overlapping_data_type_name
| dynamic_data_type_name
| percentage_data_type_name
;

position_data_type_name:
  AXISPOS         {$$ = new robot_data_file_process::axispos_type_name_c();}
| CARTPOS         {$$ = new robot_data_file_process::cartpos_type_name_c();}
//| AXISPOSEXT      {$$ = new axisposext_type_name_c();}
//| CARTPOSEXT      {$$ = new cartposext_type_name_c();}
| ROBAXISPOS      {$$ = new robot_data_file_process::robaxispos_type_name_c();}
| AUXAXISPOS      {$$ = new robot_data_file_process::auxaxispos_type_name_c();}
| ROBCARTPOS      {$$ = new robot_data_file_process::robcartpos_type_name_c();}
;

reference_system_data_type_name:
  CARTREFSYS      {$$ = new robot_data_file_process::cartrefsys_type_name_c();}
//| CARTREFSYSEXT   {$$ = new cartrefsysext_type_name_c();}
//| CARTREFSYSAXIS  {$$ = new cartrefsysaxis_type_name_c();}
;

tool_data_type_name:
  TOOL            {$$ = new robot_data_file_process::tool_type_name_c();}
//| TOOLSTATIC      {$$ = new toolstatic_type_name_c();}
;

overlapping_data_type_name:
  OVLREL          {$$ = new robot_data_file_process::ovlrel_type_name_c();}
| OVLABS          {$$ = new robot_data_file_process::ovlabs_type_name_c();}
;

dynamic_data_type_name:
  DYNAMIC         {$$ = new robot_data_file_process::dynamic_type_name_c();}
;

percentage_data_type_name:
  PERCENT           {$$ = new robot_data_file_process::percent_type_name_c();}
| PERC200           {$$ = new robot_data_file_process::perc200_type_name_c();}
;

/********************************/
/* B 1.4 - Variables            */
/********************************/

/******************************************/
/* B 1.4.3 - Declaration & Initialisation */
/******************************************/

var_init_decl:
  var1_init_decl
| structured_var_init_decl
;

var1_init_decl:
  var1_list ':' simple_spec_init 
	{$$ = new robot_data_file_process::var1_init_decl_c($1, $3);}
;

var1_list:
  identifier
	{$$ = new robot_data_file_process::var1_list_c(); $$->add_element($1);
	 variable_name_symtable.insert($1, prev_declared_variable_name_token);
	}
 | var1_list ',' identifier
	{$$ = $1; $$->add_element($3);
	 variable_name_symtable.insert($3, prev_declared_variable_name_token);
	}
;

structured_var_init_decl:
  var1_list ':' initialized_structure 
      {$$ = new robot_data_file_process::structured_var_init_decl_c($1,$3);}
;

initialized_structure:
  robot_data_type_name
	{$$ = new robot_data_file_process::initialized_structure_c($1, NULL);}
| robot_data_type_name ASSIGN structure_initialization
	{$$ = new robot_data_file_process::initialized_structure_c($1, $3);}
;

structure_initialization:
  '(' structure_element_initialization_list ')'
	{$$ = $2;}
;

/* helper symbol for structure_initialization */
structure_element_initialization_list:
  structure_element_initialization
	{$$ = new robot_data_file_process::structure_element_initialization_list_c(); $$->add_element($1);}
| structure_element_initialization_list ',' structure_element_initialization
	{$$ = $1; $$->add_element($3);}
;


structure_element_initialization: // Need to be expanded!
  structure_element_name ASSIGN constant
	{$$ = new robot_data_file_process::structure_element_initialization_c($1, $3);}
| structure_element_name ASSIGN structure_initialization
	{$$ = new robot_data_file_process::structure_element_initialization_c($1, $3);}
;

structure_element_name: any_identifier;

simple_spec_init:
  elementary_type_name
        {$$ = new robot_data_file_process::simple_spec_init_c($1, NULL);}
| elementary_type_name ASSIGN constant
	{$$ = new robot_data_file_process::simple_spec_init_c($1, $3);}
;



%%
/******************************************/
/******************************************/
/*Section 03  : User code...              */
/*Author      : Wang Zhen                 */
/*Create Date : 2014.06.10                */
/*Last Update : 2014.07.01                */
/******************************************/
/******************************************/
#include <stdio.h>	/* required for printf() */
#include <errno.h>
#include "../data_util/symtable.hh"

/* variables defined in code generated by flex... */
extern FILE *data_yyin;
extern int data_yylineno;
extern char *data_yytext;

/* The following function is called automatically by bison whenever it comes across
 * an error. Unfortunately it calls this function before executing the code that handles
 * the error itself, so we cannot print out the correct line numbers of the error location
 * over here.
 * Our solution is to store the current error message in a global variable, and have all
 * error action handlers call the function print_err_msg() after setting the location
 * (line number) variable correctly.
 */

void data_yyerror (const char *error_msg) {
  fprintf(stderr, "In file '%s': error %d: %s happen at line %d with the content is '%s'\n", data_current_filename, data_yynerrs , error_msg, data_yylineno, data_yytext); 
}


/*
 * Join two strings together. Allocate space with malloc(3).
 */
static char *strdup2(const char *a, const char *b) {
  char *res = (char *)malloc(strlen(a) + strlen(b) + 1);

  if (!res)
    return NULL;
  return strcat(strcpy(res, a), b);  /* safe, actually */
}



int data_stage1_2(const char *filename, robot_data_file_process::symbol_c **tree_root_ref) {
  FILE *in_file = NULL;

  if((in_file = fopen(filename, "r")) == NULL) {
    char *errmsg = strdup2("Error opening main file ", filename);
    perror(errmsg);
    free(errmsg);
    return -1;
  }

#if YYDEBUG
  data_yydebug = 1;
#endif

  /* now parse the input file... */
  data_yyin = in_file;
  data_yylineno = 1;
  data_current_filename = filename;
  if (data_yyparse() != 0)
    exit(EXIT_FAILURE);

  if (data_yynerrs > 0) {
    fprintf (stderr, "\nFound %d error(s). Bailing out!\n", data_yynerrs /* global variable */);
    exit(EXIT_FAILURE);
  }

  if (tree_root_ref != NULL)
    *tree_root_ref = tree_root;
 
  /*test the syntax tree*/
  //tree_root_ref->print();
  /*test the symtable*/
  //variable_name_symtable.print12();
  variable_name_symtable.reset();

  fclose(in_file);
  return 0;
}

