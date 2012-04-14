#include "planner.h"

using namespace std;

    int
Planner::a_star(Environment &env)
{	
    numberOfNodeExpand = 0;
    openSetSize = 0;
    //init
    for(int y = 0; y<RESOLUTION; y++){
        open_close[y] = new int[RESOLUTION];
        //f_score[y] = new double[RESOLUTION];
        g_score[y] = new double[RESOLUTION];
        came_fromY[y] = new int[RESOLUTION];
        came_fromX[y] = new int[RESOLUTION];
        for(int x=0; x<RESOLUTION; x++)
        {
            open_close[y][x] = 0;
            //f_score[y][x] = 0;
            //g_score[y][x] = 0;
            came_fromY[y][x] = 0;
            came_fromX[y][x] = 0;
        }
    }

    Point start(ceil(env.y_start/env.gridSize),ceil(env.x_start/env.gridSize));
    Point goal(ceil(env.y_end/env.gridSize),ceil(env.x_end/env.gridSize));

    start.cost = env.dist(start.y,start.x,goal.y,goal.x);

    openSet.push(start);
    openSetSize++;	
    open_close[start.y][start.x] = 1;

    g_score[start.y][start.x] = 0;

    //f_score[start.y][start.x] = start.cost;

    int neighborX[]={-1, 1,-1,1,  0,-1,1,0};
    int neighborY[]={-1,-1, 1,1, -1, 0,0,1};

    Point p;
    while(openSetSize)
    {			
        //int index;
        // find smallest f score
        p = openSet.top();

        /* achieve the goal */
        if(p == goal)
        {
            for(int y = 0; y<RESOLUTION; y++)
                for(int x=0; x<RESOLUTION; x++)
                {
                    if (open_close[y][x] != 0)
                    {
                        env.grid[y][x] = 15;
                        numberOfNodeExpand++;
                    }
                }
            return reconstructPath(env,p.y,p.x);;	
        }

        openSet.pop();
        openSetSize--;
        open_close[p.y][p.x] = -1;

        //openSet.erase(openSet.begin()+index);
        Point *neighbor;
        for(int i=0; i<8; i++)
        {	

            int y = max(0, neighborY[i]+p.y);
            y = min(RESOLUTION-1, y);
            int x = max(0, neighborX[i]+p.x);
            x = min(RESOLUTION-1, x);
            neighbor = new Point(y,x);

            if(open_close[y][x] == -1)
                continue;

            double tentativeScore = g_score[p.y][p.x] + 1 + env.grid[y][x]*10000;
            //if(i<4)tentativeScore += 0.4;

            bool tentative_is_better = false;
            if(open_close[y][x] == 0)
            {
                openSetSize++;
                open_close[y][x] = 1;
                tentative_is_better = true;
            }
            else if(tentativeScore < g_score[y][x])
            {
                tentative_is_better = true;
            }
            else
            {
                tentative_is_better = false;
            }



            if (tentative_is_better)
            {
                came_fromY[y][x] = p.y;
                came_fromX[y][x] = p.x;

                g_score[y][x] = tentativeScore;
                neighbor->cost = g_score[y][x] 
                    + env.dist(y,x,goal.y,goal.x);

                //neighbor.cost = f_score[y][x];
                openSet.push(*neighbor);
            }

        }// end for
    }

    return -1;
}


    int
Planner::reconstructPath(Environment &env, int y, int x)
{
    if (came_fromY[y][x])
    {
        env.grid[y][x] = 30;
        return reconstructPath(env, came_fromY[y][x], came_fromX[y][x])+1;
    }
    env.grid[y][x] = 30;
    return 1;
}

    int
Planner::AStarQuadTree(Environment &env)
{
    numberOfNodeExpand = 0;
    openSetSize = 0;

    QuadTreePoint start(env.locateNode(env.x_start,env.y_start));
    QuadTreePoint goal(env.locateNode(env.x_end,env.y_end));



    start.cost = env.dist(start.centerY,start.centerX,goal.centerY,goal.centerX);
    start.node->setOpen();
    start.node->setGScore(0);

    qOpenSet.push(start);
    openSetSize++;

    QuadTreePoint p;
    QuadTreePoint *neighbor;
    double tentativeScore;
    bool tentative_is_better;
    double stepDistance;

    while(openSetSize)
    {
        //cout << "open set size:" << openSetSize << endl;

        p = qOpenSet.top();
        //cout << "pop out:" << p.centerX << "," << p.centerY 
        //	<< " : " << p.cost << endl;

        if (p == goal)
        {
            // recontruct path
            return reconstructPathQuadTree(env, goal.node);
        }

        qOpenSet.pop(); // This calls the removed element's destructor.
        openSetSize--;

        p.node->setClose();

        vector<QuadTreeNode*>::iterator itr = p.node->neighbors.begin();

        for( ; itr != p.node->neighbors.end(); ++ itr)
        {
            if((*itr)->isClose())
                continue;

            if(!(*itr)->isEmpty())
                continue;

            //cout << "\texpand neighbor:" << (*itr)->centerX 
            //<< "," << (*itr)->centerY << endl;
            //stepDistance = env.dist((*itr)->centerY,
            //						(*itr)->centerX,
            //						p.centerY,
            //						p.centerX);

            stepDistance = ((*itr)->gridSize + p.gridSize)/2;

            tentativeScore = p.node->getGScore() 
                + stepDistance
                + ( env.quadTree.maxDistTrans 
                        - (*itr)->distTrans )*0.1;
            //cout << "\t\ttentativeScore: " << tentativeScore << endl;

            tentative_is_better = false;

            if((*itr)->isUnexplored())
            {
                (*itr)->setOpen();
                tentative_is_better = true;
                numberOfNodeExpand ++;
            }
            else if(tentativeScore < (*itr)->getGScore())
            {
                tentative_is_better = true;
            }
            else
            {
                tentative_is_better = false;
            }

            if (tentative_is_better)
            {
                //	cout << "tentative is better" << endl;
                neighbor = new QuadTreePoint(*itr);

                (*itr)->previous = p.node;
                (*itr)->setGScore(tentativeScore);

                neighbor->cost = tentativeScore 
                    + env.dist((*itr)->centerY,
                            (*itr)->centerX,
                            goal.centerY,
                            goal.centerX);
                //cout << "push:\t" << neighbor->centerX << "," 
                //<< neighbor->centerY << " : " << neighbor->cost << endl;
                openSetSize++;
                qOpenSet.push(*neighbor);
            }
        }
    }
    cout << "start[" <<env.x_start << "," << env.y_start << "] -->["
        << start.centerX << "," << start.centerY << ","
        << start.gridSize << "*" << start.gridSize << "]"
        << start.node->isEmpty() << endl;
    cout << "goal[" <<env.x_end << "," << env.y_end << "] -->["
        << goal.centerX << "," << goal.centerY << ","
        << goal.gridSize << "*" << goal.gridSize << "]"
        << goal.node->neighbors.size() << endl;
    cout << "max dist to obs:" << env.quadTree.maxDistTrans << endl;

    return -1;
}

    double
Planner::reconstructPathQuadTree(Environment &env, QuadTreeNode* node)
{
    env.path.push(node);
    if (node->previous == NULL)
    {
        return 0;
    }
    else
    {
        double step = env.dist(node->centerX,node->centerY,node->previous->centerX,node->previous->centerY);
        return reconstructPathQuadTree(env, node->previous) + step;
    }
}


