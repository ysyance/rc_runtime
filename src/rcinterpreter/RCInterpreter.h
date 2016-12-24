
#pragma once

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <string>

#include "RCRuntimeModel.h"

#include "antlr4-runtime.h"

#include "rc_helper.h"
#include "rc_logger.h"
#include "rc_exception.h"
#include "rc_innerdata.h"
#include "RCdataLexer.h"
#include "RCdataParser.h"
#include "RCdataGenerator.h"
#include "RCcodeLexer.h"
#include "RCcodeParser.h"
#include "RCcodeGenerator.h"

#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace antlr4;
using namespace antlr4::tree;



class RCInterpreter {
public:
	RCInterpreter() {}
	RCInterpreter(std::string pro) : projName(pro) {}
	RCInterpreter(std::string pro, std::string program) : projName(pro), programName(program) {}

public:
	int compile() {
		RC_SymbolTable symbolTable(	addrspace, stringpool, apaddr, 
									cpaddr, tooladdr, cooraddr,
									dataIndexMap, constIndexMap, funcMap);

		for(int i = 0; i < RC_LIB_SIZE; i ++) {
			funcMap.insert({rcLibEntry[i].name, i});
		}

		DIR *dir;
		dir = opendir(projName.c_str());
		if(dir != NULL) {

			std::ifstream dataFile(programName + ".tid");     
			std::ifstream codeFile(programName + ".tip"); 

			compileOnePragam(programName, symbolTable, dataFile, codeFile);
			
		} else {
			throw rc_projectnotfind_exception(projName);
		}
		
	}

	int compileOnePragam(std::string name, RC_SymbolTable &symbolTable, std::ifstream &dataFile, std::ifstream &codeFile) {
		/* *************************   dataFile    ************************** */
		ANTLRInputStream datainput(dataFile);     
		
		RCdataLexer datalexer(&datainput);
		CommonTokenStream datatokens(&datalexer);

		RCdataParser dataparser(&datatokens);
		ParseTree *datatree = dataparser.prog();

		RCdataGenerator DG(symbolTable);
		DG.visit(datatree);
		// std::cout << datatree->toStringTree(&dataparser) << std::endl;
		
		/* *************************   codeFile    ************************** */
		ANTLRInputStream codeInput(codeFile);

		RCcodeLexer codeLexer(&codeInput);
		CommonTokenStream codeTokens(&codeLexer);

		RCcodeParser codeParser(&codeTokens);
		ParseTree *codeTree = codeParser.prog();

		CodeModel code;

		RCcodeGenerator CG(code, symbolTable);
		CG.visit(codeTree);
		// std::cout << codeTree->toStringTree(&codeParser) << std::endl;

		/* *************************   add to progList    ************************** */
		progList.insert({name, code});
		/* JUST FOR DEBUG */
		Utility::printfCodeShadow(code);
	}

public:
	int execute(std::string name = "") {
		if(name == "") name = programName;

		std::cout << std::endl << "| ==== RC executor start ===>" << std::endl << std::endl;

		if(progList.find(name) != progList.end()) {
			CodeModel &code = progList[name];
			for(int i = 0; i < code.size(); i ++) {
				code[i]->execute(NULL);
			}
		} else {
			throw rc_programnotfound_exception(name);
		}
		std::cout << std::endl << " <=== RC executor stop ==== |" << std::endl << std::endl;
		return 0;
	}

public:
	std::string projName;
	std::string programName;

private:
	RCAddressSpace addrspace;     // addrspace[0] is the returned value of all the library function  in global
	
	StringPoolSpace stringpool;
	APDataSpace apaddr;
	CPDataSpace cpaddr;
	ToolDataSpace tooladdr;
	CoorDataSpace cooraddr;

	std::unordered_map<std::string, int> dataIndexMap;		// parser xml file and generator dataMap
	std::unordered_map<std::string, int> constIndexMap;		// the index of all the constants in addr space 
	std::unordered_map<std::string, int> funcMap;  	    	// all library function map to check if designated function is existed

	std::unordered_map<std::string, CodeModel> progList; 	// <programName, code>
};