#include <iostream>
#include "data_delete_tree.hh"
#include "../data_stage4.hh"

namespace robot_data_file_process{

	class delete_tree_c: public visitor_c {
		public:
			delete_tree_c() {}
			~delete_tree_c() {}

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
			void *visit(var_declarations_list_c *symbol) { 
				visit_list(symbol); 
				delete symbol; 
				symbol = NULL; 
				//std::cout << "delete var_declarations_list_c" << std::endl; 
				return NULL;
			}
			// B 1.1 - Identifiers
			void *visit(identifier_c *symbol) {
				delete symbol; 
				//std::cout << "delete identifier_c" << std::endl;
				return NULL;
			}
			// B 1.2 - Constants

			// B 1.2.1 - Numeric Literals
			void *visit(real_c *symbol) { 
				delete symbol;  
				//std::cout << "delete real_c" << std::endl;
				return NULL;
			}
			void *visit(integer_c *symbol) { 
				delete symbol;  
				//std::cout << "delete integer_c" << std::endl;
				return NULL;
			}
			void *visit(neg_int_expression_c *symbol) {
				symbol->exp->accept(*this); 
				delete symbol;  
				//std::cout << "delete neg_int_expression_c" << std::endl;
				return NULL;
			}
			void *visit(neg_real_expression_c *symbol) {
				symbol->exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete neg_real_expression_c" << std::endl;
				return NULL;
			}

			void *visit(integer_literal_c *symbol) {
				symbol->type->accept(*this);
				symbol->value->accept(*this); 
				delete symbol; 
				//std::cout << "delete var_declarations_list_c" << std::endl;
				return NULL;
			}
			void *visit(real_literal_c *symbol) {
				symbol->type->accept(*this);
				symbol->value->accept(*this); 
				delete symbol;  
				//std::cout << "delete integer_literal_c" << std::endl;
				return NULL;
			}
			//void *visit(bit_string_literal_c *symbol) {;}
			void *visit(boolean_literal_c *symbol) {
				symbol->type->accept(*this);
				symbol->value->accept(*this); 
				delete symbol;  
				//std::cout << "delete boolean_literal_c" << std::endl;
				return NULL;
			}

			void *visit(boolean_true_c *symbol) {
				delete symbol;  
				//std::cout << "delete boolean_true_c" << std::endl;
				return NULL;
			}
			void *visit(boolean_false_c *symbol) {
				delete symbol;  
				//std::cout << "delete boolean_false_c" << std::endl;
				return NULL;
			}

			void *visit(character_string_c *symbol) {
				delete symbol;  
				//std::cout << "delete character_string_c" << std::endl;
				return NULL;
			}

			// B 1.3.1 - Elmentary Data Types
			void *visit(bool_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete bool_type_name_c" << std::endl;
				return NULL;
			}
			void *visit(dint_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete dint_type_name_c" << std::endl;
				return NULL; 
			}
			void *visit(real_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete real_type_name_c" << std::endl;
				return NULL;
			}
			//void *visit(dword_type_name_c *symbol) {delete symbol; symbol = NULL; return NULL; }
			void *visit(string_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete string_type_name_c" << std::endl;
				return NULL; 
			}

			// B 1.3.2 - Robot data types
			void *visit(axispos_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete axispos_type_name_c" << std::endl;
				return NULL; 
			}
			void *visit(cartpos_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete cartpos_type_name_c" << std::endl;
				return NULL; 
			}    
			//VarType visit(axisposext_type_name_c *symbol) {delete symbol; symbol = NULL; return NULL; }
			//VarType visit(cartposext_type_name_c *symbol) {delete symbol; symbol = NULL; return NULL; }
			void *visit(robaxispos_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete robaxispos_type_name_c" << std::endl;
				return NULL; 
			}
			void *visit(auxaxispos_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete auxaxispos_type_name_c" << std::endl;
				return NULL;  
			}
			void *visit(robcartpos_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete robcartpos_type_name_c" << std::endl;
				return NULL;
			}

			void *visit(cartrefsys_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete cartrefsys_type_name_c" << std::endl;
				return NULL; 
			}
			//VarType visit(cartrefsysext_type_name_c *symbol) {delete symbol; symbol = NULL; return NULL;  }
			//VarType visit(cartrefsysaxis_type_name_c *symbol) {delete symbol; symbol = NULL; return NULL; }

			void *visit(tool_type_name_c *symbol) {
				delete symbol; 
				//std::cout << "delete tool_type_name_c" << std::endl;
				return NULL; 
			}
			//VarType visit(toolstatic_type_name_c *symbol) {delete symbol; symbol = NULL; return NULL;}

			void *visit(ovlrel_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete ovlrel_type_name_c" << std::endl;
				return NULL;  
			}
			void *visit(ovlabs_type_name_c *symbol) {
				delete symbol; 
				//std::cout << "delete ovlabs_type_name_c" << std::endl;
				return NULL;
			}

			void *visit(dynamic_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete dynamic_type_name_c" << std::endl;
				return NULL; 
			}

			void *visit(percent_type_name_c *symbol) {
				delete symbol;  
				//std::cout << "delete percent_type_name_c" << std::endl;
				return NULL; 
			}
			void *visit(perc200_type_name_c *symbol) {
				delete symbol; 
				//std::cout << "delete perc200_type_name_c" << std::endl;
				return NULL; 
			}


			// B 1.4 -  Variables

			// B 1.4.3 - Declarations & Initialisation

			void *visit(var1_init_decl_c *symbol) // Add to symtable
			{
				symbol->var1_list->accept(*this);
				symbol->spec_init->accept(*this);
				delete symbol;  
				//std::cout << "delete var1_init_decl_c" << std::endl;
				return NULL;
			}

			void *visit(var1_list_c *symbol) 
			{
				visit_list(symbol); 
				delete symbol;  
				//std::cout << "delete var1_list_c" << std::endl;
				return NULL;
			}

			void *visit(simple_spec_init_c *symbol)
			{
				symbol->simple_specification->accept(*this);
				symbol->constant->accept(*this);
				delete symbol;  
				//std::cout << "delete simple_spec_init_c" << std::endl;
				return NULL;		
			}

			void *visit(structured_var_init_decl_c *symbol) // Add to symtable
			{
				symbol->var1_list->accept(*this);
				symbol->initialized_structure->accept(*this);
				delete symbol;  
				//std::cout << "delete structured_var_init_decl_c" << std::endl;
				return NULL;
			}

			void *visit(initialized_structure_c *symbol)
			{
				symbol->structure_type_name->accept(*this);
				symbol->structure_initialization->accept(*this);
				delete symbol;  
				//std::cout << "delete initialized_structure_c" << std::endl;
				return NULL;
			}

			void *visit(structure_element_initialization_list_c *symbol) 
			{
				visit_list(symbol); 
				delete symbol;  
				//std::cout << "delete structure_element_initialization_list_c" << std::endl;
				return NULL;
			}
			void *visit(structure_element_initialization_c *symbol)
			{
				symbol->structure_element_name->accept(*this);
				symbol->value->accept(*this);
				delete symbol;  
				//std::cout << "delete structure_element_initialization_c" << std::endl;
				return NULL;
			}
	};/* class delete_tree_c */

	visitor_c *delete_tree()  {return new delete_tree_c();}
	void delete_visitor(visitor_c *delete_tree_visitor) {delete delete_tree_visitor;}

}

