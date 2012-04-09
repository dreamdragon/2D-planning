#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include "preprocessor.cpp"
#include "planner.cpp"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


using namespace std;

void testParse(Environment& env);
void writeMapToFile(int **matrix, int matrixSize);
void QuadTreeToLines(visualization_msgs::Marker &line_strip,
										 vector<visualization_msgs::Marker>  &line_list_list,
										 QuadTreeNode *node, int totalDepth);

int main(int argc, char** argv){

	if(argc < 2)
	{
		cout<<"no input file!"<<endl;
		return 1;
	}
	
	string inputFile = argv[1];
	Preprocessor prep;
	
	Environment env;
	
	Planner planner;	

	prep.parseEnvironment(inputFile,env);

	env.radius_robot = 0.3;
/*	
	env.regularCellDecomposite();
	int path_len = planner.a_star(env);
	writeMapToFile(env);
*/

	env.ApproxCellDecomposite(10);
	
	//env.quadTree.showNeighbors();
	//env.quadTree.testNeighbors();
	
	clock_t start_time;

	start_time = clock();
	
	int path_len =  planner.AStarQuadTree(env);
	
	double time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
	
	cout << "Menglong Zhu\n" << "length of path:" << path_len
		<<"\n# of states expanded:"<<planner.numberOfNodeExpand 
		<<"\nplanning time:" << time_in_seconds << "s" << endl;
		
	//writeMapToFile(env.QuadTreeToMatrix(1024),1024);
	
	ofstream fout("path.txt");
	env.outputPath(fout);
	
	
	/* publish the quadtree to rviz*/
	
	ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Publisher marker_pub1 = n.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
  ros::Rate r(0.2);


  float f = 0.0;
  int totalDepth = 0;
  int maxDepth = 7;
  vector<visualization_msgs::Marker> line_list_list;
  while (ros::ok())
  {
		env.ApproxCellDecomposite(totalDepth);
		
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/my_frame";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;



    line_strip.id = 1;
    



    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    



    // Line strip is green
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

		for (uint i = 0; i < maxDepth; i++)
		{
    	// Line list is red
    	visualization_msgs::Marker i_line_list;
    	i_line_list.header.frame_id = "/my_frame";
		  i_line_list.header.stamp = ros::Time::now();
		  i_line_list.ns = "points_and_lines" + i;
		  i_line_list.action = visualization_msgs::Marker::ADD;
		  i_line_list.pose.orientation.w = 1.0;
		  i_line_list.id = 2;
		  i_line_list.type = visualization_msgs::Marker::LINE_LIST;
		  i_line_list.scale.x = 0.1;
		  switch(i%3){
    		case 0: 
    			i_line_list.color.r = 1.0;
    			break;
    		case 1:
    			i_line_list.color.r = 1.0;
    			i_line_list.color.g = 1.0;
    			break;
    		case 2:
    			i_line_list.color.b = 1.0;
    			break;
    	}
    	i_line_list.color.a = 1.0;
    	
    	line_list_list.push_back(i_line_list);
		}


    // Create the vertices for the points and lines
    QuadTreeToLines(line_strip, line_list_list, env.quadTree.root, totalDepth);

    marker_pub.publish(line_strip);
    for (uint i = 0; i < maxDepth; i++)
		{
    	// Line list is red
    	cout << "publish level " << i << endl;
    	marker_pub1.publish(line_list_list[i]);
		}
    
		
    r.sleep();
		line_list_list.clear();
    f += 0.04;
    totalDepth++;
		totalDepth = totalDepth%maxDepth;
  }

	
	return 0;
}

void
writeMapToFile(int **matrix, int matrixSize)
{
	ofstream fout("output.txt");
	for(int y = 0; y<matrixSize; y++)
	{
		for(int x=0; x<matrixSize; x++)
		{
			fout << matrix[y][x] << " ";		
		}
		fout << endl;
	}
}

void testParse(Environment& env)
{
	cout << "boundary\t" << env.x0 << "\t" << env.y0 
	<< "\t" << env.x1 << "\t" << env.y1 << "\n"
	<< "start\t" << env.x_start << "\t" << env.y_start << "\n"
	<< "end\t" << env.x_end << "\t" << env.y_end << "\n"
	<< "number of obstacles:\t" << env.obsContainer.size() << "\n";
	for (unsigned int i = 0; i<env.obsContainer.size(); i++ )
	{
		cout << "obstacle "<< i+1 << ": "
		<< env.obsContainer[i].x_obs << " "
		<< env.obsContainer[i].y_obs << " "  
		<< env.obsContainer[i].radius_obs << endl;
	}	
}

void QuadTreeToLines(visualization_msgs::Marker &line_strip,
										 vector<visualization_msgs::Marker>  &line_list_list,
										 QuadTreeNode *node, int totalDepth)
{
	geometry_msgs::Point *p = new geometry_msgs::Point();
	float scale = 8;
  p->x = node->centerX-25;
  p->y = node->centerY-25;
  p->z = (totalDepth - node->depth)*scale;
	line_strip.points.push_back(*p);
	if(!node->isLeaf)
	{
		QuadTreeToLines(line_strip, line_list_list, node->childOne, totalDepth);
		line_strip.points.push_back(*p);
		QuadTreeToLines(line_strip, line_list_list, node->childTwo, totalDepth);
		line_strip.points.push_back(*p);
		QuadTreeToLines(line_strip, line_list_list, node->childThree, totalDepth);
		line_strip.points.push_back(*p);
		QuadTreeToLines(line_strip, line_list_list, node->childFour, totalDepth);
		line_strip.points.push_back(*p);
	}
	else
	{
		geometry_msgs::Point *s = new geometry_msgs::Point();
		s->x = node->topleftX-25;
		s->y = node->topleftY-25;
		s->z = (totalDepth - node->depth)*scale;
		line_list_list[node->depth].points.push_back(*s);
		geometry_msgs::Point *e = new geometry_msgs::Point();
		e->x = node->topleftX + node->gridSize -25;
		e->y = node->topleftY-25;
		e->z = (totalDepth - node->depth)*scale;
		line_list_list[node->depth].points.push_back(*e);
		
		
		s = e;
		line_list_list[node->depth].points.push_back(*s);
		e = new geometry_msgs::Point();
		e->x = node->topleftX + node->gridSize -25;
		e->y = node->topleftY + node->gridSize -25;
		e->z = (totalDepth - node->depth)*scale;
		line_list_list[node->depth].points.push_back(*e);
		
		s = e;
		line_list_list[node->depth].points.push_back(*s);
		e = new geometry_msgs::Point();
		e->x = node->topleftX -25;
		e->y = node->topleftY + node->gridSize -25;
		e->z = (totalDepth - node->depth)*scale;
		line_list_list[node->depth].points.push_back(*e);
		
		s = e;
		line_list_list[node->depth].points.push_back(*s);
		e = new geometry_msgs::Point();
		e->x = node->topleftX -25;
		e->y = node->topleftY -25;
		e->z = (totalDepth - node->depth)*scale;
		line_list_list[node->depth].points.push_back(*e);
	}
}

