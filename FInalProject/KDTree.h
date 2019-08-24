#ifndef KDTREE_H
#define KDTREE_H

#include <iostream>
#include "scene_io.h"
#include "Render.h"

#define MAX_TRIANGLES 8

struct Triangle;

typedef enum {
	X_SPLIT = 0,
	Y_SPLIT = 1,
	Z_SPLIT = 2
} SplitAxis;

struct Box3d {
	double xmin, xmax, ymin, ymax, zmin, zmax;
	Box3d() {
		xmin = 1e10;
		ymin = 1e10;
		zmin = 1e10;
		xmax = -1e10;
		ymax = -1e10;
		zmax = -1e10;
	}

	bool intersects(const Ray &ray);

	void print() {
		std::cout << "(" << xmin << ", " << ymin << ", " << zmin << ")";
		std::cout << "(" << xmax << ", " << ymax << ", " << zmax << ")" << std::endl;
	}
};

class KDTree
{
	Box3d boundingBox;
	KDTree *child1, *child2;
	Triangle **triangles;
	void sort(int nTriangles, Triangle **t, SplitAxis axis);
public:
	bool intersects(const Ray &ray, double &t, Vector3d &normal, MaterialIO &material);
	KDTree(int nTriangles, Triangle **t, SplitAxis axis);
	~KDTree();
};

#endif