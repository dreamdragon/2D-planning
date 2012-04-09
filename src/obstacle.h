#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle{
public:
	float x_obs, y_obs, radius_obs;

	Obstacle(float x,float y,float radius);
	~Obstacle();
};

#endif /* OBSTACLE_H */
