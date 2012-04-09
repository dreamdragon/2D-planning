#ifndef PREPROCESS_H
#define PREPROCESS_H
#include "environment.h"

class Preprocessor{
public:
	

	Preprocessor();
	~Preprocessor();
	void parseEnvironment(string inputFile, Environment& env);
};

#endif /* PREPROCESS_H */
