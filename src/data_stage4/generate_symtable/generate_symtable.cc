#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <vector>
#include <utility>
#include <map>
#include "generate_symtable.hh"
#include "../data_stage4.hh"
#include "../data_type.hh"
#include "../../data_util/symtable.hh"

namespace robot_data_file_process{


class generate_symtable_c: public visitor_c {
	private:
		DEF_SYMTABLE &var_symtable;

	public:
		generate_symtable_c(DEF_SYMTABLE *filename_symtable): var_symtable(*filename_symtable) {}
		~generate_symtable_c() {}

	private:
		inline void *visit_list(list_c *list)
		{
			for(int i = 0; i < list->n; ++i)
			{
				list->elements[i] -> accept(*this);
			}
			return NULL;
		}

	public:
		// B 0 - Programming Model
		void *visit(var_declarations_list_c *symbol) {return visit_list(symbol);}
		// B 1.1 - Identifiers
		void *visit(identifier_c *symbol) {
			char *cptr = new char[strlen(symbol->value)];
			if( cptr == NULL )
			{
				std::cout << "identifier_c|cptr: new memory failed !" << std::endl;
				exit(-1);
			}
			strcpy(cptr, symbol->value);
			return static_cast<void *>(cptr);
		} 

		// B 1.2 - Constants

		// B 1.2.1 - Numeric Literals
		void *visit(real_c *symbol) { 
			double *var = new double(atof(symbol->value));
			if( var == NULL )
			{
				std::cout << "real_c|var: new memory failed !" << std::endl;
				exit(-1);
			} 
			return static_cast<void *>(var);
		}
		void *visit(integer_c *symbol) { 
			int *var = new int(atoi(symbol->value)); 
			if( var == NULL )
			{
				std::cout << "integer_c|var: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(var);
		}
		void *visit(neg_int_expression_c *symbol) {
			int *var = static_cast<int *>(symbol->exp->accept(*this)); 
			*var = (*var) * (-1); 
			return static_cast<void *>(var);
		}
		void *visit(neg_real_expression_c *symbol) {
			double *var = static_cast<double *>(symbol->exp->accept(*this)); 
			*var = (*var) * (-1); 
			return static_cast<void *>(var); 
		}

		void *visit(integer_literal_c *symbol) {return symbol->value->accept(*this);}
		void *visit(real_literal_c *symbol) {return symbol->value->accept(*this);}
		//void *visit(bit_string_literal_c *symbol) {;}
		void *visit(boolean_literal_c *symbol) {return symbol->value->accept(*this);}

		void *visit(boolean_true_c *symbol) {
			bool *var = new bool(true); 
			if( var == NULL )
			{
				std::cout << "boolean_true_c|var: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(var);
		}
		void *visit(boolean_false_c *symbol) {
			bool *var = new bool(false); 
			if( var == NULL )
			{
				std::cout << "boolean_false_c|var: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(var);
		}

		void *visit(character_string_c *symbol) {
			std::string *var = new std::string(symbol->value); 
			if( var == NULL )
			{
				std::cout << "character_string_c|var: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(var);
		}

		// B 1.3.1 - Elmentary Data Types
		void *visit(bool_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_BOOL);
			if( id_type == NULL )
			{
				std::cout << "bool_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			} 
			return static_cast<void *>(id_type); 
		}
		void *visit(dint_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_DINT); 
			if( id_type == NULL )
			{
				std::cout << "dint_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type); 
		}
		void *visit(real_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_REAL);
			if( id_type == NULL )
			{
				std::cout << "real_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			} 
			return static_cast<void *>(id_type); 
		}
		//void *visit(dword_type_name_c *symbol) {VarType *id_type = new VarType(TYPE_DWORD); return static_cast<void *>(id_type; }
		void *visit(string_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_STRING); 
			if( id_type == NULL )
			{
				std::cout << "string_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type); 
		}

		// B 1.3.2 - Robot data types
		void *visit(axispos_type_name_c *symbol) { 
			VarType *id_type  = new VarType(TYPE_AXISPOS);
			if( id_type == NULL )
			{
				std::cout << "axispos_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type); 
		}
		void *visit(cartpos_type_name_c *symbol) {
			VarType *id_type  = new VarType(TYPE_CARTPOS);
			if( id_type == NULL )
			{
				std::cout << "cartpos_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type);  
		}    
		//VarType visit(axisposext_type_name_c *symbol) {VarType *id_type  = new VarType(TYPE_BOOL);return static_cast<void *>(id_type;  }
		//VarType visit(cartposext_type_name_c *symbol) {VarType *id_type  = new VarType(TYPE_BOOL);return static_cast<void *>(id_type;  }
		void *visit(robaxispos_type_name_c *symbol) {
			VarType *id_type  = new VarType(TYPE_ROBAXISPOS);
			if( id_type == NULL )
			{
				std::cout << "robaxispos_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type);  
		}
		void *visit(auxaxispos_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_AUXAXISPOS);
			if( id_type == NULL )
			{
				std::cout << "auxaxispos_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type);  
		}
		void *visit(robcartpos_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_ROBCARTPOS);
			if( id_type == NULL )
			{
				std::cout << "robcartpos_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type);  
		}

		void *visit(cartrefsys_type_name_c *symbol) {
			VarType *id_type  = new VarType(TYPE_CARTREFSYS);
			if( id_type == NULL )
			{
				std::cout << "cartrefsys_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type);  
		}
		//VarType visit(cartrefsysext_type_name_c *symbol) {VarType *id_type  = new VarType(TYPE_BOOL);return static_cast<void *>(id_type;  }
		//VarType visit(cartrefsysaxis_type_name_c *symbol) {VarType *id_type = new VarType(YPE_BOOL);return static_cast<void *>(id_type;  }

		void *visit(tool_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_TOOL);
			if( id_type == NULL )
			{
				std::cout << "tool_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type);  
		}
		//VarType visit(toolstatic_type_name_c *symbol) {VarType *id_type = new VarType(TYPE_BOOL);return static_cast<void *>(id_type; }

		void *visit(ovlrel_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_OVLREL);
			if( id_type == NULL )
			{
				std::cout << "ovlrel_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type);  
		}
		void *visit(ovlabs_type_name_c *symbol) {
			VarType *id_type  = new VarType(TYPE_OVLABS);
			if( id_type == NULL )
			{
				std::cout << "ovlabs_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type); 
		}

		void *visit(dynamic_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_DYNAMIC);
			if( id_type == NULL )
			{
				std::cout << "dynamic_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type); 
		}

		void *visit(percent_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_PERCENT);
			if( id_type == NULL )
			{
				std::cout << "percent_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type);  
		}
		void *visit(perc200_type_name_c *symbol) {
			VarType *id_type = new VarType(TYPE_PERC200);
			if( id_type == NULL )
			{
				std::cout << "perc200_type_name_c|id_type: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(id_type);  
		}


		// B 1.4 -  Variables

		// B 1.4.3 - Declarations & Initialisation

		void *visit(var1_init_decl_c *symbol) // Add to symtable
		{
			std::vector<const char *> *var_list =static_cast< std::vector<const char *> * >(symbol->var1_list->accept(*this));
			sym_element *simple_var_element = static_cast<sym_element *>(symbol->spec_init->accept(*this));
			for(std::vector<const char *>::iterator iter = (*var_list).begin(); iter != (*var_list).end(); ++iter )
				var_symtable.insert(*iter, simple_var_element);
			return NULL;
		}

		void *visit(var1_list_c *symbol) 
		{
			std::vector<const char *> *var_list = new std::vector<const char *>(symbol->n);
			if( var_list == NULL )
			{
				std::cout << "var1_list_c|var_list: new memory failed !" << std::endl;
				exit(-1);
			}
			for(int i = 0; i < symbol->n ; ++i)
				(*var_list)[i] = static_cast<char *>(symbol->elements[i]->accept(*this)); 
			return static_cast<void *>(var_list);
		}

		void *visit(simple_spec_init_c *symbol)
		{
			sym_element *simple_var_element = new sym_element();
			if( simple_var_element == NULL )
			{
				std::cout << "simple_spec_init_c|simple_var_element: new memory failed !" << std::endl;
				exit(-1);
			}
			simple_var_element->id_type = static_cast<VarType *>(symbol->simple_specification->accept(*this));
			switch(*(simple_var_element->id_type))
			{
				case TYPE_BOOL:
					simple_var_element->id_value.bv = new bool;
					if( simple_var_element->id_value.bv == NULL )
					{
						std::cout << "simple_spec_init_c|simple_var_element->id_value.bv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->constant != NULL )
						simple_var_element->id_value.bv = static_cast<bool *>(symbol->constant->accept(*this));
					break;
				case TYPE_DINT:
					simple_var_element->id_value.dv = new int;
					if( simple_var_element->id_value.dv == NULL )
					{
						std::cout << "simple_spec_init_c|simple_var_element->id_value.iv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->constant != NULL )
						simple_var_element->id_value.dv = static_cast<int *>(symbol->constant->accept(*this));
					break;
				case TYPE_REAL:
					simple_var_element->id_value.rv = new double;
					if( simple_var_element->id_value.rv == NULL )
					{
						std::cout << "simple_spec_init_c|simple_var_element->id_value.dv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->constant != NULL )
						simple_var_element->id_value.rv = static_cast<double *>(symbol->constant->accept(*this));
					break;
				case TYPE_STRING:
					simple_var_element->id_value.sv = new std::string;
					if( simple_var_element->id_value.sv == NULL )
					{
						std::cout << "simple_spec_init_c|simple_var_element->id_value.sv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->constant != NULL )
						simple_var_element->id_value.sv = static_cast<std::string *>(symbol->constant->accept(*this));
					break;
				default:
					std::cout << "simple_spec_init_c|simple_var_element->id_type: Error id_type, please check your code" << std::endl;
					exit(-1);
					break;
			}
			return static_cast<void *>(simple_var_element);
		}

		void *visit(structured_var_init_decl_c *symbol) // Add to symtable
		{
			std::vector<const char *> *var_list = static_cast< std::vector<const char *> * >(symbol->var1_list->accept(*this));
			sym_element *structure_var_element = static_cast<sym_element *>(symbol->initialized_structure->accept(*this));
			for(std::vector<const char *>::iterator iter = (*var_list).begin(); iter != (*var_list).end(); ++iter)
				var_symtable.insert(*iter, structure_var_element);
			return NULL;
		}

		void *visit(initialized_structure_c *symbol)
		{
			sym_element *structure_var_element = new sym_element();
			if( structure_var_element == NULL )
			{
				std::cout << "initialized_structure_c|structure_var_element: new memory failed !" << std::endl;
				exit(-1);
			}
			structure_var_element->id_type = static_cast<VarType *>(symbol->structure_type_name->accept(*this));
			switch(*(structure_var_element->id_type))
			{
				case TYPE_AXISPOS:
					structure_var_element->id_value.apv = new axispos();
					if( structure_var_element->id_value.apv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.apv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"a1"))
								structure_var_element->id_value.apv->a1 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a2"))
								structure_var_element->id_value.apv->a2 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a3"))
								structure_var_element->id_value.apv->a3 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a4"))
								structure_var_element->id_value.apv->a4 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a5"))
								structure_var_element->id_value.apv->a5 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a6"))
								structure_var_element->id_value.apv->a6 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux1"))
								structure_var_element->id_value.apv->aux1 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux2"))
								structure_var_element->id_value.apv->aux2 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux3"))
								structure_var_element->id_value.apv->aux3 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux4"))
								structure_var_element->id_value.apv->aux4 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux5"))
								structure_var_element->id_value.apv->aux5 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux6"))
								structure_var_element->id_value.apv->aux6 = *(static_cast<double *>((*iter).second)); 
						}
					}
					break;
				case TYPE_CARTPOS:
					structure_var_element->id_value.cpv = new cartpos();
					if( structure_var_element->id_value.cpv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.cpv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast< std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"x"))
								structure_var_element->id_value.cpv->x = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"y"))
								structure_var_element->id_value.cpv->y = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"z"))
								structure_var_element->id_value.cpv->z = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a"))
								structure_var_element->id_value.cpv->a = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"b"))
								structure_var_element->id_value.cpv->b = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"c"))
								structure_var_element->id_value.cpv->c = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux1"))
								structure_var_element->id_value.cpv->aux1 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux2"))
								structure_var_element->id_value.cpv->aux2 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux3"))
								structure_var_element->id_value.cpv->aux3 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux4"))
								structure_var_element->id_value.cpv->aux4 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux5"))
								structure_var_element->id_value.cpv->aux5 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux6"))
								structure_var_element->id_value.cpv->aux6 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"mode"))
								structure_var_element->id_value.cpv->mode = *(static_cast<double *>((*iter).second)); 
						}
					}
					break;
				case TYPE_ROBAXISPOS:
					structure_var_element->id_value.rapv = new robaxispos();
					if( structure_var_element->id_value.rapv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.rapv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * > (symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"a1"))
								structure_var_element->id_value.rapv->a1 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a2"))
								structure_var_element->id_value.rapv->a2 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a3"))
								structure_var_element->id_value.rapv->a3 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a4"))
								structure_var_element->id_value.rapv->a4 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a5"))
								structure_var_element->id_value.rapv->a5 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a6"))
								structure_var_element->id_value.rapv->a6 = *(static_cast<double *>((*iter).second)); 
						}
					}
					break;
				case TYPE_ROBCARTPOS:
					structure_var_element->id_value.rcpv = new robcartpos();
					if( structure_var_element->id_value.rcpv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.rcpv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"x"))
								structure_var_element->id_value.rcpv->x = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"y"))
								structure_var_element->id_value.rcpv->y = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"z"))
								structure_var_element->id_value.rcpv->z = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a"))
								structure_var_element->id_value.rcpv->a = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"b"))
								structure_var_element->id_value.rcpv->b = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"c"))
								structure_var_element->id_value.rcpv->c = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"mode"))
								structure_var_element->id_value.rcpv->mode = *(static_cast<double *>((*iter).second));
						}
					}
					break;
				case TYPE_AUXAXISPOS:
					structure_var_element->id_value.aapv = new auxaxispos();
					if( structure_var_element->id_value.aapv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.aapv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"aux1"))
								structure_var_element->id_value.aapv->aux1 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux2"))
								structure_var_element->id_value.aapv->aux2 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux3"))
								structure_var_element->id_value.aapv->aux3 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux4"))
								structure_var_element->id_value.aapv->aux4 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux5"))
								structure_var_element->id_value.aapv->aux5 = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"aux6"))
								structure_var_element->id_value.aapv->aux6 = *(static_cast<double *>((*iter).second)); 
						}
					}
					break;
				case TYPE_TOOL:
					structure_var_element->id_value.tv = new tool();
					if( structure_var_element->id_value.tv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.tv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"x"))
								structure_var_element->id_value.tv->x = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"y"))
								structure_var_element->id_value.tv->y = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"z"))
								structure_var_element->id_value.tv->z = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a"))
								structure_var_element->id_value.tv->a = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"b"))
								structure_var_element->id_value.tv->b = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"c"))
								structure_var_element->id_value.tv->c = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"CGx"))
								structure_var_element->id_value.tv->CGx = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"CGy"))
								structure_var_element->id_value.tv->CGy = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"CGz"))
								structure_var_element->id_value.tv->CGz = *(static_cast<double *>((*iter).second)); 
						}
					}
					break;
				case TYPE_CARTREFSYS:
					structure_var_element->id_value.crsv = new cartrefsys();
					if( structure_var_element->id_value.crsv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.crsv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"x"))
								structure_var_element->id_value.crsv->x = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"y"))
								structure_var_element->id_value.crsv->y = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"z"))
								structure_var_element->id_value.crsv->z = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"a"))
								structure_var_element->id_value.crsv->a = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"b"))
								structure_var_element->id_value.crsv->b = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"c"))
								structure_var_element->id_value.crsv->c = *(static_cast<double *>((*iter).second)); 
						}
					}
					break;
				case TYPE_PERCENT:
					structure_var_element->id_value.pv = new percent();
					if( structure_var_element->id_value.pv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.pv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"perc"))
								structure_var_element->id_value.pv->perc = *(static_cast<int *>((*iter).second)); 
						}
					}
					break;
				case TYPE_PERC200:
					structure_var_element->id_value.p2v = new perc200();
					if( structure_var_element->id_value.p2v == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.p2v: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"perc2"))
								structure_var_element->id_value.p2v->perc2 = *(static_cast<int *>((*iter).second)); 
						}
					}
					break;
				case TYPE_OVLREL:
					structure_var_element->id_value.orv = new ovlrel();
					if( structure_var_element->id_value.orv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.orv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"ovl"))
								structure_var_element->id_value.orv->ovl = *(static_cast<perc200 *>((*iter).second)); 
						}
					}
					break;
				case TYPE_OVLABS:
					structure_var_element->id_value.oav = new ovlabs();
					if( structure_var_element->id_value.oav == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.oav: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"posDist"))
								structure_var_element->id_value.oav->posDist = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"oriDist"))
								structure_var_element->id_value.oav->oriDist = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"linAxDist"))
								structure_var_element->id_value.oav->linAxDist = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"rotAxDist"))
								structure_var_element->id_value.oav->rotAxDist = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"vConst"))
								structure_var_element->id_value.oav->vConst = *(static_cast<bool *>((*iter).second)); 
						}
					}
					break;
				case TYPE_DYNAMIC:
					structure_var_element->id_value.dynv = new dynamic();
					if( structure_var_element->id_value.dynv == NULL )
					{
						std::cout << "initialized_structure_c|structure_var_element->id_value.dynv: new memory failed !" << std::endl;
						exit(-1);
					}
					if(symbol->structure_initialization != NULL )
					{
						std::map<char *, void *> *element_list = static_cast<   std::map<char *, void *> * >(symbol->structure_initialization->accept(*this));
						for(  std::map<char *, void *>::iterator iter = (*element_list).begin(); iter != (*element_list).end(); ++iter)
						{
							if(!strcmp((*iter).first,"velAxis"))
								structure_var_element->id_value.dynv->velAxis = *(static_cast<percent *>((*iter).second)); 
							else if(!strcmp((*iter).first,"accAxis"))
								structure_var_element->id_value.dynv->accAxis = *(static_cast<percent *>((*iter).second)); 
							else if(!strcmp((*iter).first,"decAxis"))
								structure_var_element->id_value.dynv->decAxis = *(static_cast<percent *>((*iter).second)); 
							else if(!strcmp((*iter).first,"jerkAxis"))
								structure_var_element->id_value.dynv->jerkAxis = *(static_cast<percent *>((*iter).second)); 
							else if(!strcmp((*iter).first,"vel"))
								structure_var_element->id_value.dynv->vel = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"acc"))
								structure_var_element->id_value.dynv->acc = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"dec"))
								structure_var_element->id_value.dynv->dec = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"jerk"))
								structure_var_element->id_value.dynv->jerk = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"velOri"))
								structure_var_element->id_value.dynv->velOri = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"accOri"))
								structure_var_element->id_value.dynv->accOri = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"decOri"))
								structure_var_element->id_value.dynv->decOri = *(static_cast<double *>((*iter).second)); 
							else if(!strcmp((*iter).first,"jerkOri"))
								structure_var_element->id_value.dynv->jerkOri = *(static_cast<double *>((*iter).second)); 
						}
					}
					break;
				default:
					std::cout << "initialized_structure_c|structure_var_element->id_type :Error id_type, please check your code" << std::endl;
					exit(-1);
					break;
			}

			return static_cast<void *>(structure_var_element);
		}

		void *visit(structure_element_initialization_list_c *symbol) 
		{
			std::map<char *, void *> *element_list = new std::map<char *, void *>;
			if( element_list == NULL )
			{
				std::cout << "structure_element_initialization_list_c|element_list: new memory failed !" << std::endl;
				exit(-1);
			}
			for(int i = 0; i < symbol->n ; ++i)
				(*element_list).insert( *(static_cast< std::pair<char *, void *> * >(symbol->elements[i]->accept(*this)))); 
			return static_cast<void *>(element_list);
		}
		void *visit(structure_element_initialization_c *symbol)
		{
			char  *s = static_cast<char *>(symbol->structure_element_name->accept(*this));
			std::pair<char *, void *> *structure_pair = new std::pair<char *, void *>(std::make_pair(s, static_cast<void *>((symbol->value->accept(*this)))));
			if( structure_pair == NULL )
			{
				std::cout << "structure_element_initialization_c|structure_pair: new memory failed !" << std::endl;
				exit(-1);
			}
			return static_cast<void *>(structure_pair);
		}
};/* class generate_iec_c */

visitor_c *new_code_generator(DEF_SYMTABLE *s4o)  {return new generate_symtable_c(s4o);}
void delete_code_generator(visitor_c *code_generator) {delete code_generator;}

}
