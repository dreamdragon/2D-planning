#include "environment.h"
#include <cmath>
#include <cassert>

using namespace std;

Environment::Environment():
    x0(0),y0(0),x1(0),y1(0),
    x_start(0), y_start(0),
    x_end(0), y_end(0),baseResolution(2){}

Environment::~Environment(){}

void 
Environment::addObstacle(Obstacle obs){
    obsContainer.push_back(obs);
}

    void
Environment::regularCellDecomposite()
{
    int count=0;
    gridSize = abs(x0-x1)/RESOLUTION;
    for(int y = 0; y<RESOLUTION; y++)
        for(int x=0; x<RESOLUTION; x++)
        {
            grid[y][x] = 0;
        }
    double y_left,x_left,y_right,x_right,distance ;
    vector<Obstacle>::iterator it;
    for ( it=obsContainer.begin(); it < obsContainer.end(); it++ )
    {	
        y_left = ceil((it->y_obs-it->radius_obs-radius_robot)/gridSize);
        x_left = ceil((it->x_obs-it->radius_obs-radius_robot)/gridSize);
        y_right = ceil((it->y_obs+it->radius_obs+radius_robot)/gridSize);
        x_right = ceil((it->x_obs+it->radius_obs+radius_robot)/gridSize);

        y_left = max((double)0,y_left);
        x_left = max((double)0,x_left);
        y_right =min((double)RESOLUTION,y_right);
        x_right =min((double)RESOLUTION,x_right);

        for(int y=y_left; y<y_right; y++)
            for(int x=x_left; x<x_right; x++)
            {

                distance = dist((y+0.5)*gridSize,
                        (x+0.5)*gridSize,
                        it->y_obs,it->x_obs);

                if(distance <= (it->radius_obs+radius_robot))
                {
                    grid[y][x] = 1;
                    count++;
                }
            }
    }
}

    void
Environment::initialTree()
{
    double coarseGridSize = abs(x1 - x0)/baseResolution;
    cout << coarseGridSize << endl;

}

    void
Environment::ApproxCellDecomposite(int depth)
{	
    double mapSize = abs(x1 - x0);
    qResolution = pow(2,depth);

    quadTree.root = new QuadTreeNode(0, 0, mapSize,0);
    labelNode(*quadTree.root);

    cout << "try to split to depth: " << depth << endl;
    if(depth < quadTree.depth)
        return;

    splitToDepth(*quadTree.root,depth);

    quadTree.root->refineNeighbors();

}

    void
Environment::splitToDepth(QuadTreeNode& node, int depth)
{	
    if(node.depth >= depth)
    {
        return;
    }

    if(node.isMix())// only split MIX cell
    {
        node.split();
        labelNode(*(node.childOne));
        labelNode(*(node.childTwo));
        labelNode(*(node.childThree));
        labelNode(*(node.childFour));

        splitToDepth(*(node.childOne), depth);
        splitToDepth(*(node.childTwo), depth);
        splitToDepth(*(node.childThree), depth);
        splitToDepth(*(node.childFour), depth);
    }
}

    void
Environment::labelNode(QuadTreeNode& node)
{
    vector<Obstacle>::iterator it;
    double distance = 0, full_dist, empty_dist, dist_to_obs;
    for ( it=obsContainer.begin(); it < obsContainer.end(); it++ )
    {
        distance = dist(node.centerX, node.centerY,
                it->x_obs, it->y_obs);
        empty_dist = it->radius_obs + radius_robot + node.gridSize/2*1.4;
        full_dist = it->radius_obs + radius_robot - node.gridSize/2*1.4;

        /* cell is default labeled as EMPTY*/		
        if (distance <= full_dist)
        {
            node.setFull();
        }
        else if(distance <= empty_dist)
        {
            if(!node.isFull())
                node.setMix();
        }
        dist_to_obs =  distance - it->radius_obs - radius_robot;
        if(dist_to_obs < node.distTrans)
            node.distTrans = dist_to_obs;
    }
    if(node.distTrans > this->quadTree.maxDistTrans)
        this->quadTree.maxDistTrans = node.distTrans;
}

double
Environment::dist(const double ax,const double ay,
        const double bx,const double by)const
{
    return sqrt((ax-bx)*(ax-bx) + (ay-by)*(ay-by));
}

    int**
Environment::QuadTreeToMatrix(int matrixSize) // for debugging
{
    int **matrix = new int*[matrixSize];
    for(int i = 0; i<matrixSize; i++)
    {
        matrix[i] = new int[matrixSize];
    }
    /*for(unsigned int i=0; i<quadTree.roots.size(); i++)
      {
      quadTree.roots[i].nodeToMatrix(matrix,matrixSize,abs(x0-x1));
      }*/
    quadTree.root->nodeToMatrix(matrix,matrixSize,abs(x0-x1));
    return matrix;
}

    QuadTreeNode*
Environment::locateNode(double x, double y)
{
    return quadTree.locateNode(x,y);
}

    void
Environment::outputPath(ofstream& fout)
{
    fout << x_start << " " << y_start << endl;
    while(!path.empty())
    {
        fout << path.top()->centerX << " " << path.top()->centerY << endl;
        path.pop();
    }
    fout << x_end << " " << y_end << endl;
}


/* QuadTreeNode */
QuadTreeNode::QuadTreeNode(){}

QuadTreeNode::QuadTreeNode(double topleftX, double topleftY, 
        double gridSize, int depth):
    topleftX(topleftX),topleftY(topleftY),gridSize(gridSize),
    centerX(topleftX + gridSize/2),centerY(topleftY + gridSize/2),
    gridType(EMPTY),isLeaf(true),depth(depth),parent(NULL),
    distTrans(10000),status(INIT),previous(NULL)
{
    /*cout << "Construct Node:[" << topleftX << "," << topleftY 
      << "] size:" << gridSize << "*" << gridSize << " depth:"
      << depth << endl;*/
}

QuadTreeNode::~QuadTreeNode(){}

    bool
QuadTreeNode::isMix()
{
    return gridType == MIX;
}

    bool
QuadTreeNode::isFull()
{
    return gridType == FULL;
}

    bool
QuadTreeNode::isEmpty()
{
    return gridType == EMPTY;
}

    void
QuadTreeNode::setEmpty()
{
    gridType = EMPTY;
}

    void
QuadTreeNode::setMix()
{
    gridType = MIX;
}

    void
QuadTreeNode::setFull()
{
    gridType = FULL;
}

    void
QuadTreeNode::setOpen()
{
    status = OPEN;
}

    void
QuadTreeNode::setClose()
{
    status = CLOSE;
}

    bool
QuadTreeNode::isClose()
{
    return status == CLOSE;
}

    bool
QuadTreeNode::isOpen()
{
    return status == OPEN;
}

    bool
QuadTreeNode::isUnexplored()
{
    return status == INIT;
}

    void
QuadTreeNode::setGScore(double score)
{
    gScore = score;
}

    double
QuadTreeNode::getGScore()
{
    return gScore;
}

    void
QuadTreeNode::split()
{
    //cout << "split node ["<< topleftX << "," 
    //	<< topleftY << "]" << endl;
    QuadTreeNode *nodeOne = new QuadTreeNode(topleftX, topleftY, 
            gridSize/2,depth+1);
    QuadTreeNode *nodeTwo = new QuadTreeNode(centerX, topleftY, 
            gridSize/2,depth+1);
    QuadTreeNode *nodeThree = new QuadTreeNode(topleftX, centerY, 
            gridSize/2,depth+1);
    QuadTreeNode *nodeFour = new QuadTreeNode(centerX, centerY, 
            gridSize/2,depth+1);

    this->childOne = nodeOne;
    this->childTwo = nodeTwo;
    this->childThree = nodeThree;
    this->childFour = nodeFour;

    nodeOne->parent = this;
    nodeTwo->parent = this;
    nodeThree->parent = this;
    nodeFour->parent = this;

    /* compute possible neighbor nodes */
    nodeOne->addNeighbors(nodeTwo,nodeThree,nodeFour);
    nodeTwo->addNeighbors(nodeOne,nodeThree,nodeFour);
    nodeThree->addNeighbors(nodeOne,nodeTwo,nodeFour);
    nodeFour->addNeighbors(nodeOne,nodeTwo,nodeThree);

    if(!neighbors.empty())
    {
        vector<QuadTreeNode*>::iterator it;
        for(it=neighbors.begin();
                it<neighbors.end();++it)
        {
            //cout << "parent add neighbor to children" << endl;
            (*it)->addNeighborLeafTo(childOne);
            (*it)->addNeighborLeafTo(childTwo);
            (*it)->addNeighborLeafTo(childThree);
            (*it)->addNeighborLeafTo(childFour);
        }
    }

    /* not a leaf any more*/
    isLeaf = false;	
}


/* recursively add children as node's neighbors if adjacent*/
    void
QuadTreeNode::addNeighborLeafTo(QuadTreeNode *node)
{
    if (this == node->parent)
        return;

    if(isLeaf)
    {
        //cout << "add neighobor leaf to..."  << endl;
        if(this->isNeighbor(node))
            node->neighbors.push_back(this);	
    }
    else
    {

        if(childOne->isNeighbor(node))
            childOne->addNeighborLeafTo(node);
        if(childTwo->isNeighbor(node))
            childTwo->addNeighborLeafTo(node);
        if(childThree->isNeighbor(node))
            childThree->addNeighborLeafTo(node);
        if(childFour->isNeighbor(node))
            childFour->addNeighborLeafTo(node);
    }
}

    void
QuadTreeNode::refineNeighbors()
{
    if (!isLeaf)
    {
        neighbors.clear();
        childOne->refineNeighbors();
        childTwo->refineNeighbors();
        childThree->refineNeighbors();
        childFour->refineNeighbors();
    }
    else
    {
        vector<QuadTreeNode*> tNeighbors = neighbors;
        neighbors.clear();

        vector<QuadTreeNode*>::iterator it;
        for(it=tNeighbors.begin(); it!=tNeighbors.end(); ++it)
        {
            (*it)->addNeighborLeafTo(this);
        }

        // to check if there are duplicate neighbors
        vector<QuadTreeNode*>::iterator it2;
        for(it=neighbors.begin(); it!=neighbors.end(); ++it)
        {
            for(it2=neighbors.begin(); it2!=neighbors.end(); ++it2)
            {	
                if(it == it2) continue;
                if(*it == *it2)
                {
                    cout << (*it)->centerX
                        << "," << (*it)->centerY
                        << (*it)->gridSize << "\t"
                        << (*it2)->centerX
                        << "," << (*it2)->centerY
                        << (*it2)->gridSize << endl;
                }
                assert(*it != *it2); 
            }
        }
    }

}

    bool
QuadTreeNode::isNeighbor(QuadTreeNode *node)
{
    double eps = min(this->gridSize/10, node->gridSize/10);
    double adjDist,xDist,yDist;
    adjDist = (this->gridSize + node->gridSize)/2 + eps;
    xDist = abs(this->centerX - node->centerX);
    yDist = abs(this->centerY - node->centerY);
    if(xDist <= adjDist && yDist <= adjDist)
    {
        return true;
    }
    else
    {
        return false;
    }
}

    void
QuadTreeNode::addNeighbors(QuadTreeNode *one,
        QuadTreeNode *two,
        QuadTreeNode *three)
{
    this->neighbors.push_back(one);
    this->neighbors.push_back(two);
    this->neighbors.push_back(three);
}

    void
QuadTreeNode::nodeToMatrix(int **matrix, int matrixSize, int mapSize)
{
    if(isLeaf){
        int cellSize = (gridSize/(double)mapSize)*matrixSize;

        int y_top = (topleftY/(double)mapSize)*matrixSize;
        int y_bottom = min(y_top+cellSize,matrixSize);

        int x_left = (topleftX/(double)mapSize)*matrixSize;
        int x_right = min(x_left+cellSize,matrixSize);

        for(int y=y_top; y<y_bottom; y++)
            for(int x=x_left; x<x_right; x++)
            {

                if(y==y_top || x==x_left)
                    matrix[y][x] = 10;
                else{
                    matrix[y][x] = gridType*5;
                    if (isOpen())
                    {
                        matrix[y][x] += 20;
                    }else if (isClose())
                    {
                        matrix[y][x] += 20;
                    }else if(isUnexplored())
                    {
                        matrix[y][x] = 0;
                    }
                }
            }
    }else{
        childOne->nodeToMatrix(matrix,matrixSize,mapSize);
        childTwo->nodeToMatrix(matrix,matrixSize,mapSize);
        childThree->nodeToMatrix(matrix,matrixSize,mapSize);
        childFour->nodeToMatrix(matrix,matrixSize,mapSize);
    }
}


    void
QuadTreeNode::showNeighbors() // for debug
{

    double scale = 1000/50;
    if(isLeaf)
    {	
        cout << "----------neighors list-------------------" << endl;
        //if (this->neighbors.size() > 0){
        cout << "node[" << centerX*scale << "," 
            << centerY*scale << "]" << "isLeaf:" << isLeaf << endl;
        for(unsigned int i=0; i<this->neighbors.size(); i++)
        {
            cout << "\tneighbor " << i+1 << ": [" 
                << this->neighbors[i]->centerX*scale << ","
                << this->neighbors[i]->centerY*scale << "]"
                << this->neighbors[i]->gridSize*scale <<" leaf:" 
                << this->neighbors[i]->isLeaf << endl;
        }
        //}
    }
    else
    {
        assert (this->neighbors.size() == 0);
        this->childOne->showNeighbors();
        this->childTwo->showNeighbors();
        this->childThree->showNeighbors();
        this->childFour->showNeighbors();
    }
}

    QuadTreeNode*
QuadTreeNode::locateNode(double x, double y)
{
    /*cout << "locate[" << x << "," << y << "] in [" 
      << centerX << "," << centerY << "," 
      << gridSize << "*" << gridSize << "]" 
      << endl;*/
    if (isLeaf)
    {
        return this;
    }
    else
    {
        // to be optimized...
        if (x >= topleftX && x < centerX &&
                y >= topleftY && y < centerY)
        {
            return childOne->locateNode(x,y);
        }	
        else if (x >= centerX && x < topleftX + gridSize &&
                y >= topleftY && y < centerY)
        {
            return childTwo->locateNode(x,y);
        }	
        else if (x >= topleftX && x < centerX &&
                y >= centerY && y < topleftY + gridSize)
        {
            return childThree->locateNode(x,y);
        }	
        else 
        {
            assert(x >= centerX && x <= topleftX + gridSize &&
                    y >= centerY && y <= topleftY + gridSize);
            return childFour->locateNode(x,y);
        }
    }
}

    void
QuadTreeNode::testNeighbors()
{
    if(isLeaf)
    {
        bool flag = false;
        vector<QuadTreeNode*>::iterator it;
        vector<QuadTreeNode*>::iterator n_it;
        for(it = neighbors.begin(); it != neighbors.end(); ++it)
        {
            QuadTreeNode* iNeighbor= *it;

            //cout << iNeighbor->topleftX << " "
            //	<< iNeighbor->topleftY << " "
            //	<< iNeighbor->gridSize << endl;

            for(n_it = iNeighbor->neighbors.begin(); 
                    n_it != iNeighbor->neighbors.end();
                    ++n_it)
            {
                if (*n_it == this)
                    flag = true;
            }
            assert(flag);
            flag = false;
        }
    }
    else
    {
        childOne->testNeighbors();
        childTwo->testNeighbors();
        childThree->testNeighbors();
        childFour->testNeighbors();
    }
}

/* QuadTree*/
QuadTree::QuadTree():depth(0){}
QuadTree::~QuadTree(){}

    void
QuadTree::showNeighbors() // for debug
{
    /*	for(unsigned int i = 0; i<roots.size(); i++)
        {
        roots[i].showNeighbors();
        }
        */	

    root->showNeighbors();
}

    QuadTreeNode*
QuadTree::locateNode(double x, double y)
{
    return root->locateNode(x,y);
}

    void
QuadTree::testNeighbors()
{
    root->testNeighbors();
}



