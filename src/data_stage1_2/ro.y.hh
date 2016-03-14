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

#ifndef YY_DATA_YY_RO_Y_HH_INCLUDED
# define YY_DATA_YY_RO_Y_HH_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int data_yydebug;
#endif

/* Token type.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    BOGUS_TOKEN_ID = 258,
    identifier_token = 259,
    prev_declared_variable_name_token = 260,
    integer_token = 261,
    real_token = 262,
    TRUE = 263,
    FALSE = 264,
    character_string_token = 265,
    BOOL = 266,
    DINT = 267,
    REAL = 268,
    STRING = 269,
    AXISPOS = 270,
    CARTPOS = 271,
    ROBAXISPOS = 272,
    AUXAXISPOS = 273,
    ROBCARTPOS = 274,
    CARTREFSYS = 275,
    TOOL = 276,
    OVLREL = 277,
    OVLABS = 278,
    DYNAMIC = 279,
    WORLD = 280,
    PERCENT = 281,
    PERC200 = 282,
    ASSIGN = 283
  };
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE YYSTYPE;
union YYSTYPE
{
#line 97 "ro.y" /* yacc.c:1909  */

    robot_data_file_process::symbol_c 	*leaf;
    robot_data_file_process::list_c	*list;
    char 	*ID;	/* token value */

#line 89 "ro.y.hh" /* yacc.c:1909  */
};
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE data_yylval;

int data_yyparse (void);

#endif /* !YY_DATA_YY_RO_Y_HH_INCLUDED  */
