/* A Bison parser, made by GNU Bison 3.0.2.  */

/* Bison interface for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2013 Free Software Foundation, Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

#ifndef YY_PRO_YY_ROP_Y_HH_INCLUDED
# define YY_PRO_YY_ROP_Y_HH_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int pro_yydebug;
#endif

/* Token type.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    BOGUS_TOKEN_ID = 258,
    identifier_token = 259,
    integer_token = 260,
    real_token = 261,
    TRUE = 262,
    FALSE = 263,
    character_string_token = 264,
    AND = 265,
    OR = 266,
    NOT = 267,
    LT = 268,
    LE = 269,
    GT = 270,
    GE = 271,
    EQ = 272,
    NE = 273,
    ASSIGN = 274,
    CALL = 275,
    IF = 276,
    THEN = 277,
    ELSE = 278,
    ELSEIF = 279,
    END_IF = 280,
    WHILE = 281,
    DO = 282,
    END_WHILE = 283,
    LOOP = 284,
    END_LOOP = 285,
    standard_function_name_token = 286,
    standard_robot_instruction_token = 287
  };
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE YYSTYPE;
union YYSTYPE
{
#line 114 "rop.y" /* yacc.c:1909  */

    robot_program_file_process::symbol_c *leaf;
    robot_program_file_process::list_c	*list;
    char 	*ID;	/* token value */

#line 93 "rop.y.hh" /* yacc.c:1909  */
};
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif

/* Location type.  */
#if ! defined YYLTYPE && ! defined YYLTYPE_IS_DECLARED
typedef struct YYLTYPE YYLTYPE;
struct YYLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
};
# define YYLTYPE_IS_DECLARED 1
# define YYLTYPE_IS_TRIVIAL 1
#endif


extern YYSTYPE pro_yylval;
extern YYLTYPE pro_yylloc;
int pro_yyparse (void);

#endif /* !YY_PRO_YY_ROP_Y_HH_INCLUDED  */
