#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H
#define RESOLUTION 1000
#include <vector>
#include <stack>
#include <fstream>
#include "obstacle.h"

using namespace std;

class QuadTreeNode
{
    public:
    /* structure */
    enum GridType
    {
        EMPTY = 0,
        MIX = 1,
        FULL = 2,
    };


    double topleftX;
    double topleftY;
    double gridSize;

    double centerX;
    double centerY;
    GridType gridType;

    bool isLeaf;
    int depth;

    QuadTreeNode *childOne;
    QuadTreeNode *childTwo;
    QuadTreeNode *childThree;
    QuadTreeNode *childFour;

    QuadTreeNode *parent;

    vector<QuadTreeNode*> neighbors;


    /* planning */

    double distTrans; // distance to closed obstacle
    double gScore;

    enum Status
    {
        INIT = 0,
        OPEN = 1,
        CLOSE = 2,
    };
    Status status;
    QuadTreeNode *previous;

    /* functions */

    QuadTreeNode();
    QuadTreeNode(double topleftX, double topleftY, 
            double gridSize, int depth);
    ~QuadTreeNode();

    void split();
    bool isEmpty();
    bool isMix();
    bool isFull();

    void setEmpty();
    void setMix();
    void setFull();

    void setOpen();
    void setClose();
    bool isClose();
    bool isOpen();
    bool isUnexplored();

    void setGScore(double score);
    double getGScore();

    void addNeighbors(QuadTreeNode *one,
            QuadTreeNode *Two,
            QuadTreeNode *Three);

    //void getNeighborsFromParent();

    void addNeighborLeafTo(QuadTreeNode *node);
    bool isNeighbor(QuadTreeNode *node);

    void refineNeighbors();

    void nodeToMatrix(int **matrix, int matrixSize, int mapSize);

    void showNeighbors();

    QuadTreeNode* locateNode(double x, double y);
    void testNeighbors();
};

class QuadTree
{
    public:
    vector<QuadTreeNode> roots;
    QuadTreeNode *root;
    int depth;

    // max distance to closed obstacle
    double maxDistTrans;
    // grid be initially devided into resolution * resolution


    QuadTree();
    ~QuadTree();

    void showNeighbors();

    QuadTreeNode* locateNode(double x, double y);
    void testNeighbors();
};

class Environment
{
    public:
    double x0,y0,x1,y1;
    double x_start, y_start;
    double x_end, y_end;
    int baseResolution; // default as 10 in constructor

    double radius_robot; // to be configured

    int grid[RESOLUTION][RESOLUTION];
    double gridSize;

    vector<Obstacle> obsContainer;
    QuadTree quadTree;

    Environment();
    ~Environment();

    void addObstacle(Obstacle obs);

    void regularCellDecomposite();
    /* QuadTree related */
    double qResolution;
    int** qGrid;

    void initialTree();
    void ApproxCellDecomposite(int depth);
    void splitToDepth(QuadTreeNode& node, int depth);
    void labelNode(QuadTreeNode& node);
    double dist(const double ax,const double ay,
            const double bx,const double by)const;

    int** QuadTreeToMatrix(int matrixSize);

    QuadTreeNode* locateNode(double x, double y);

    /* planning */
    stack<QuadTreeNode*> path;
    void outputPath(ofstream& fout);
};


#endif /* ENVIRONMENT_H */
