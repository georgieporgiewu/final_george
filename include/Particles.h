#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class Particles {
public:
	Vector3f position;
	Vector3f velocity;
	
	float mass;
	
	bool fixed;
	
	Vector3f normal;

};

#endif