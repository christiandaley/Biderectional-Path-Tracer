#include "stdafx.h"
#include <algorithm>
#include "KDTree.h"
#include "SceneObject.h"


int cmpXmin(const void *t1, const void *t2);
int cmpYmin(const void *t1, const void *t2);
int cmpZmin(const void *t1, const void *t2);

KDTree::KDTree(int nTriangles, Triangle **t, SplitAxis axis)
{

	// First we compute this Tree's bounding box
	this->boundingBox = Box3d();
	for (int i = 0; i < nTriangles; i++) {
		if (t[i]->bb.xmin < boundingBox.xmin)
			boundingBox.xmin = t[i]->bb.xmin;

		if (t[i]->bb.ymin < boundingBox.ymin)
			boundingBox.ymin = t[i]->bb.ymin;

		if (t[i]->bb.zmin < boundingBox.zmin)
			boundingBox.zmin = t[i]->bb.zmin;

		if (t[i]->bb.xmax > boundingBox.xmax)
			boundingBox.xmax = t[i]->bb.xmax;

		if (t[i]->bb.ymax > boundingBox.ymax)
			boundingBox.ymax = t[i]->bb.ymax;

		if (t[i]->bb.zmax > boundingBox.zmax)
			boundingBox.zmax = t[i]->bb.zmax;
	}

	if (nTriangles <= MAX_TRIANGLES) {
		this->triangles = new Triangle *[MAX_TRIANGLES+1];
		for (int i = 0; i < nTriangles; i++) {
			this->triangles[i] = t[i];
			this->triangles[i + 1] = NULL;
		}

		child1 = NULL;
		child2 = NULL;

	}
	else {
		int n1 = nTriangles / 2;
		int n2 = nTriangles - n1;
		sort(nTriangles, t, axis);
		Triangle **t1 = new Triangle*[n1];
		Triangle **t2 = new Triangle*[n2];

		for (int i = 0; i < n1; i++) {
			t1[i] = t[i];
		}

		for (int i = 0; i < n2; i++) {
			t2[i] = t[i + n1];
		}

		SplitAxis childAxis = axis == X_SPLIT ? Y_SPLIT :
			(axis == Y_SPLIT ? Z_SPLIT : X_SPLIT);

		child1 = new KDTree(n1, t1, childAxis);
		child2 = new KDTree(n2, t2, childAxis);

		delete[] t1;
		delete[] t2;

		triangles = NULL;
	}

}

bool KDTree::intersects(const Ray &ray, double &t, Vector3d &normal, MaterialIO &material) {
	if (!this->boundingBox.intersects(ray))
		return false;

	bool intersects = false;
	t = 1e10;
	double d;
	Vector3d n;
	MaterialIO m;

	if (triangles == NULL) {
		if (child1->intersects(ray, d, n, m)) {
			intersects = true;
			t = d;
			normal = n;
			material = m;
		}

		if (child2->intersects(ray, d, n, m)) {
			if (d < t) {
				intersects = true;
				t = d;
				normal = n;
				material = m;
			}
		}

		return intersects;
	}


	for (int i = 0; triangles[i] != NULL; i++) {
		if (triangles[i]->intersects(ray, d, n, m)) {
			if (d < t) {
				t = d;
				normal = n;
				material = m;
				intersects = true;
			}
		}
	}

	return intersects;
}

void KDTree::sort(int nTriangles, Triangle **t, SplitAxis axis) {
	int(*cmp)(const void *t1, const void *t2) = axis == X_SPLIT ? cmpXmin :
		(axis == Y_SPLIT ? cmpYmin : cmpZmin);

	//std::qsort(t, nTriangles, sizeof(Triangle *), cmpXmin);

	
	bool done = false;
	int tries = 0;
	while (!done) {
		done = true;
		tries++;
		for (int i = 0; i < nTriangles - tries; i++) {
			if (cmp(t[i], t[i + 1])) {
				Triangle *temp = t[i];
				t[i] = t[i + 1];
				t[i + 1] = temp;
				done = false;
			}
		}
	}
	
}

KDTree::~KDTree()
{
	if (this->triangles != NULL)
		delete[] this->triangles;
	if (this->child1 != NULL)
		delete this->child1;
	if (this->child2 != NULL)
		delete this->child2;
}

bool Box3d::intersects(const Ray &ray) {
	const Point3d O = ray.O;
	const Vector3d D = ray.D;

	/* First we account for edge cases where the ray is parallel
	to one of the planes that define the box */
	if (D.z == 0.0) {
		if (O.z < zmin || O.z > zmax)
			return false;

		if (O.x < xmin && D.x < 0.0)
			return false;
		if (O.x > xmax && D.x > 0.0)
			return false;
		if (O.y < ymin && D.y < 0.0)
			return false;
		if (O.y > ymax && D.y > 0.0)
			return false;

		return true;
	}
	else if (D.y == 0.0) {
		if (O.y < ymin || O.y > ymax)
			return false;

		if (O.x < xmin && D.x < 0.0)
			return false;
		if (O.x > xmax && D.x > 0.0)
			return false;
		if (O.z < zmin && D.z < 0.0)
			return false;
		if (O.z > zmax && D.z > 0.0)
			return false;

		return true;
	}
	else if (D.x == 0.0) {
		if (O.x < xmin || O.x > xmax)
			return false;

		if (O.y < ymin && D.y < 0.0)
			return false;
		if (O.y > ymax && D.y > 0.0)
			return false;
		if (O.z < zmin && D.z < 0.0)
			return false;
		if (O.z > zmax && D.z > 0.0)
			return false;

		return true;
	}

	/* Now we compute the 6 points where the ray intersects the planes
	that define the box. If any of those points is within the box then 
	the ray intersects the box. Otherwise the ray does not intersect the box
	*/
	
	double t = (zmin - O.z) / D.z;
	Point3d p = O + t * D;
	if (p.x >= xmin && p.x <= xmax &&
		p.y >= ymin && p.y <= ymax &&
		t >= 0.0) {
		return true;
	}

	t = (zmax - O.z) / D.z;
	p = O + t * D;
	if (p.x >= xmin && p.x <= xmax &&
		p.y >= ymin && p.y <= ymax &&
		t >= 0.0) {
		return true;
	}

	t = (ymin - O.y) / D.y;
	p = O + t * D;
	if (p.x >= xmin && p.x <= xmax &&
		p.z >= zmin && p.z <= zmax &&
		t >= 0.0) {
		return true;
	}

	t = (ymax - O.y) / D.y;
	p = O + t * D;
	if (p.x >= xmin && p.x <= xmax &&
		p.z >= zmin && p.z <= zmax &&
		t >= 0.0) {
		return true;
	}

	t = (xmin - O.x) / D.x;
	p = O + t * D;
	if (p.y >= ymin && p.y <= ymax &&
		p.z >= zmin && p.z <= zmax &&
		t >= 0.0) {
		return true;
	}

	t = (xmax - O.x) / D.x;
	p = O + t * D;
	if (p.y >= ymin && p.y <= ymax &&
		p.z >= zmin && p.z <= zmax &&
		t >= 0.0) {
		return true;
	}

	return false;
}

int cmpXmin(const void *t1, const void *t2) {
	return ((Triangle *)t1)->bb.xmin < ((Triangle *)t2)->bb.xmin;
}
int cmpYmin(const void *t1, const void *t2) {
	return ((Triangle *)t1)->bb.ymin < ((Triangle *)t2)->bb.ymin;
}
int cmpZmin(const void *t1, const void *t2) {
	return ((Triangle *)t1)->bb.zmin < ((Triangle *)t2)->bb.zmin;
}