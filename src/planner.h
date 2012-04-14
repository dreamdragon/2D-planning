#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <queue>
#include <set>
#include "environment.h"

struct Point{
    int y;
    int x;
    double cost;
    Point(){}
    Point(int y,int x):y(y),x(x),cost(0){}
    ~Point(){}
};

bool operator< (const Point& point1, const Point &point2)
{
    return point1.cost > point2.cost;	
}

bool operator> (const Point& point1, const Point &point2)
{
    return point1.cost < point2.cost;	
}

bool operator== (const Point& point1, const Point &point2)
{
    return (point1.x == point2.x) && (point1.y == point2.y);	
}

struct QuadTreePoint{
    double centerX;
    double centerY;
    double gridSize;
    double cost;
    QuadTreeNode* node;

    QuadTreePoint(){}
    QuadTreePoint(QuadTreeNode* node):
        centerX(node->centerX),
        centerY(node->centerY),
        gridSize(node->gridSize),
        cost(0),
        node(node){}
    ~QuadTreePoint(){}
};

bool operator< (const QuadTreePoint& point1, 
        const QuadTreePoint& point2)
{
    return point1.cost > point2.cost;
}

bool operator> (const QuadTreePoint& point1, 
        const QuadTreePoint& point2)
{
    return point1.cost < point2.cost;
}

bool operator== (const QuadTreePoint& point1, 
        const QuadTreePoint& point2)
{
    return (point1.centerX == point2.centerX) 
        && (point1.centerY == point2.centerY);
}

class Planner
{
    public:

    //set<Point> closedSet;
    vector<Point> openSetVec;
    int openSetSize;
    priority_queue<Point> openSet;
    int numberOfNodeExpand;
    double *g_score[RESOLUTION];
    //double *f_score[RESOLUTION];
    int *came_fromY[RESOLUTION];
    int *came_fromX[RESOLUTION];
    //int *t[RESOLUTION];
    //1 for open, -1 for close;
    int *open_close[RESOLUTION];


    int a_star(Environment &env);
    int reconstructPath(Environment &env, int y, int x);

    /* Quadtree related*/
    priority_queue<QuadTreePoint> qOpenSet;

    int AStarQuadTree(Environment &env);
    double reconstructPathQuadTree(Environment &env,
            QuadTreeNode* node);
};


#endif /* PLANNER_H */




