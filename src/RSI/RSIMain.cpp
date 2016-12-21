#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>

#include "antlr4-runtime.h"
#include "RSILexer.h"
#include "RSIParser.h"
#include "RSIVisitor.h"
#include "RSIBaseVisitor.h"
#include "RSIConstVisitor.h"
#include "RSICodeGenerator.h"
#include "RSIExecutor.h"

#include "RSIXml.h"

using namespace antlr4;
using namespace antlr4::tree;


#ifdef RSI_DEBUG_PRINT
std::unordered_map<int, std::string> rdataIndexMap;   // index --> var
#endif


int RSIMain() {

  std::unordered_map<std::string, int> dataIndexMap;    // parser xml file and generator dataMap; var --> index
  std::unordered_map<std::string, int> constIndexMap;   // the index of all the constants in addr space 
  std::unordered_map<std::string, EntityBase*> fbMap;   // parser xml file and generator functionblock map
  std::unordered_map<std::string, int> funcMap;         // all library function map to check if designated function is existed

  std::vector<IValue> addrspace(1, 0);     // RSI address space ; addrspace[0] is the returned value of all the library function  in global


  RSIXmlLoader loader("config.xml");
  loader.parseXml(addrspace, dataIndexMap, fbMap);

  SymbolTable symTable(addrspace, dataIndexMap, constIndexMap, fbMap, funcMap);
  symTable.create();

  /* read rsi code */
  std::ifstream infile("rsi.code");     
  ANTLRInputStream input(infile);     
  /* rsi code lexer analysis */  
  RSILexer lexer(&input);
  CommonTokenStream tokens(&lexer);
  /* rsi code parser analysis */  
  RSIParser parser(&tokens);
  ParseTree *tree = parser.prog();

  RSIConstVisitor vi(symTable);                // first time, build constant table
  vi.visit(tree);

  CodeShadow code;                  // define rsi code shadow in memory

  RSICodeGenerator CG(code, symTable);        // second time, generate execute model
  CG.visit(tree);

  RSIExecutor app(code);            // create rsi code executor
  app.execute();                    // start execute

  // for(auto elem : dataIndexMap) {
  //   std::cout << elem.first << " : " << addrspace[elem.second] << std::endl;
  // }

  // for(int i = 1; i < addrspace.size(); i++) {
  //   std::cout << i << ": "<< rdataIndexMap[i]<< " = " << addrspace[i] << std::endl;
  // }

  // for(auto &e : fbMap) {
  //   std::cout << e.first << std::endl;
  //   e.second->printInfo();

  //   std::cout << std::endl; 
  // }


  return 0;
}
