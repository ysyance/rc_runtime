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



#ifndef DATA_STAGE4_H_
#define DATA_STAGE4_H_


#include "../data_absyntax/data_absyntax.hh"
#include "../data_util/symtable.hh"
#include "data_type.hh"
#include <string>

namespace robot_data_file_process{

enum VarType {
	TYPE_BOOL,
	TYPE_DINT,
	TYPE_REAL,			//TYPE_DWORD,

    TYPE_STRING,
	TYPE_AXISPOS,		//TYPE_AXISPOSEXT,
	TYPE_CARTPOS,		// TYPE_CARTPOSEXT,

	TYPE_ROBAXISPOS,
	TYPE_AUXAXISPOS,
	TYPE_ROBCARTPOS,
	TYPE_CARTREFSYS,	//TYPE_CARTREFSYSEXT,// TYPE_CARTREFSYSAXIS,

	TYPE_TOOL,			//TYPE_TOOLSTATIC,
	TYPE_OVLREL,
	TYPE_OVLABS,
	TYPE_DYNAMIC,
	TYPE_PERCENT,
	TYPE_PERC200
};

class sym_element {
public:
	VarType *id_type;
//	std::string id_belongTo_program_name;
	union  {
		bool *bv;
		int *dv;
		double *rv;		//dword
		std::string *sv;
		axispos *apv;	//axisposext
		cartpos *cpv;	//cartposext
		robaxispos *rapv;
		robcartpos *rcpv;
		auxaxispos *aapv;
		tool *tv;		//toolstatic
		cartrefsys *crsv;
		//cartrefsysaxis *crsav; //cartrefsysext
		percent *pv;
		perc200 *p2v;
		ovlrel *orv;
		ovlabs *oav;
		dynamic *dynv;
	} id_value;


public:
	sym_element() {}
	~sym_element() {}
};

#define NULL_ELEMENT 0

typedef symtable_c<sym_element *, NULL_ELEMENT> DEF_SYMTABLE;
typedef symtable_c< symtable_c<sym_element *, NULL_ELEMENT > *, NULL_ELEMENT >  DEF_SYM_SYM;
int data_stage4(char **filename, symbol_c **tree_root, DEF_SYM_SYM &symtable_of_symtable);

}

#endif


