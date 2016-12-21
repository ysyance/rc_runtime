

#pragma once

#include <vector>
#include <string>

#include "RSIRuntimeModel.h"


class RSIExecutor {
public:
	RSIExecutor(CodeShadow &c):code(c) {}

	int execute() {
		std::cout << std::endl << "| ==== RSI executor start ===>" << std::endl << std::endl;
		for(int i = 0; i < code.size(); i ++) {
			code[i]->execute(NULL);
		}
		std::cout << std::endl << " <=== RSI executor stop ==== |" << std::endl << std::endl;
		return 0;
	}
public:
	CodeShadow &code;		// code model in memory
	int period; 			// period in ms
};


