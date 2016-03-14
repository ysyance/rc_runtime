#include <iostream>
#include "program_delete_tree.hh"
#include "../program_stage4.hh"

namespace robot_program_file_process{


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
			void *visit(library_c *symbol) { 
				symbol->statement_list_head->accept(*this); 
				delete symbol; 
				symbol = NULL; 
				//std::cout << "delete library_c" << std::endl; 
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



			/***********************/
			/* B 2.1 - Expressions */
			/***********************/
			void *visit(or_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete or_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(and_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete and_expression_c " << std::endl;
				return NULL;
			}
			void *visit(equ_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete equ_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(notequ_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete notequ_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(lt_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete lt_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(gt_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete gt_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(le_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete le_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(ge_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete ge_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(add_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete add_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(sub_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete sub_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(mul_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete mul_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(div_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete div_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(mod_expression_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete mod_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(not_expression_c *symbol) {
				symbol->exp->accept(*this); 
				delete symbol; 
				//std::cout << "delete not_expression_c " << std::endl; 
				return NULL;
			}
			void *visit(function_invocation_c *symbol) {
				symbol->function_name->accept(*this); 
				symbol->parameter_assignment_list->accept(*this); 
				delete symbol; 
				//std::cout << "delete function_invocation_c " << std::endl; 
				return NULL;
			}



			/***************************/
			/* B 2.2 - Statements      */
			/***************************/
			void *visit(statement_list_c *symbol) {
				if(symbol == NULL || symbol->n == 0)
					return NULL;
				else
				{
					visit_list(symbol); 
					delete symbol; 
					//std::cout << "delete statement_list_c" << std::endl; 
					return NULL;
				}
			}

			void *visit(assignment_statement_c *symbol) {
				symbol->l_exp->accept(*this); 
				symbol->r_exp->accept(*this); 
				delete symbol;
				//std::cout << "delete assignment_statement_c " << std::endl;
				return NULL;
			}

			void *visit(subprogram_invocation_c *symbol) {
				symbol->subprogram_name->accept(*this); 
				delete symbol; 
				//std::cout << "delete subprogram_invocation_c" << std::endl; 
				return NULL; 
			}

			void *visit(if_statement_c *symbol) {
				symbol->expression->accept(*this); 
				symbol->statement_list->accept(*this);
				if(symbol->elseif_statement_list == NULL)
					;
				else
					symbol->elseif_statement_list->accept(*this);
				if(symbol->else_statement_list == NULL)
					;
				else
					symbol->else_statement_list->accept(*this);
				delete symbol; 
				//std::cout << "delete subprogram_invocation_c" << std::endl; 
				return NULL; 
			}

			void *visit(elseif_statement_list_c *symbol) {
				if(symbol ==NULL ||symbol->n == 0 )
				{
					return NULL;
				}
				visit_list(symbol); 
				delete symbol; 
				//std::cout << "delete elseif_statement_list_c" << std::endl; 
				return NULL;
			}

			void *visit(elseif_statement_c *symbol) {
				if(symbol == NULL)
				{
					return NULL;
				}
				symbol->expression->accept(*this); 
				symbol->statement_list->accept(*this); 
				delete symbol; 
				//std::cout << "delete elseif_statement_c " << std::endl;
				return NULL;
			}

			void *visit(while_statement_c *symbol) {
				symbol->expression->accept(*this); 
				symbol->statement_list->accept(*this); 
				delete symbol; 
				//std::cout << "delete elseif_statement_c " << std::endl; 
				return NULL;
			}

			void *visit(loop_statement_c *symbol) {
				symbol->const_expression->accept(*this); 
				symbol->statement_list->accept(*this); 
				delete symbol; 
				//std::cout << "delete loop_statement_c " << std::endl; 
				return NULL;
			}

			void *visit(robot_instruction_invocation_c *symbol) {
				symbol->robot_instruction_name->accept(*this); 
				symbol->parameter_assignment_list->accept(*this); 
				delete symbol; 
				//std::cout << "delete loop_statement_c " << std::endl; 
				return NULL;
			}

			void *visit(param_assignment_list_c *symbol) {
				visit_list(symbol); 
				delete symbol; 
				//std::cout << "delete statement_list_c" << std::endl; 
				return NULL;
			}

	};/* class delete_tree_c */











	visitor_c *delete_tree()  {return new delete_tree_c();}
	void delete_visitor(visitor_c *delete_tree_visitor) {delete delete_tree_visitor;}

}

