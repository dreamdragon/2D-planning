#include<iostream>
#include<fstream>
#include<string>
#include "preprocessor.h"
#include "obstacle.h"
#include "obstacle.cpp"
#include "environment.h"
#include "environment.cpp"


using namespace std;

Preprocessor::Preprocessor(){}
Preprocessor::~Preprocessor(){}

void
Preprocessor::parseEnvironment(string inputFile, Environment& env)
{
	ifstream fin;
	fin.open(inputFile.c_str());
	if (fin.is_open())
	{
		string identifier;
		fin >> identifier >> env.x0 >> env.y0 >> env.x1 >> env.y1;
		string id("boundary");
		if(id.compare(identifier)!=0){
			cout << "ERROR in reading boundary" << endl;
			return;	
		}

		if((env.x1 - env.x0) != (env.y0 - env.y1))
		{
			cout << "Not a square" << endl;
			return;			
		}
	
		id = "start";
		fin >> identifier >> env.x_start >> env.y_start;
		if(id.compare(identifier)!=0){
			cout << "ERROR in reading start" << endl;
			return;	
		}

		id = "end";
		fin >> identifier >> env.x_end >> env.y_end;
		if(id.compare(identifier)!=0){
			cout << "ERROR in reading end" << endl;
			return;	
		}
	
		float x_obs, y_obs, radius_obs;
		while(fin >> identifier){
			fin >> x_obs >> y_obs >> radius_obs;
			env.addObstacle(Obstacle(x_obs, y_obs, radius_obs));
		}

		fin.close();
	}else{
		cout << "file not opened!"<<endl;
	}
}




