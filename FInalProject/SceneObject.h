#ifndef SCENE_OBJECT_H
#define SCENE_OBJECT_H

#include "scene_io.h"
#include "Render.h"

#include "KDTree.h"

struct Triangle {
	Point3d verts[3];
	Vector3d normals[3];
	Vector3d edge1, edge2;
	MaterialIO *materials[3];
	bool intNormals;
	bool intMaterial;
	double area;
	double diffWeights[3];
	Box3d bb;

	void bound() {
		bb.xmin = MIN(verts[0].x, MIN(verts[1].x, verts[2].x));
		bb.ymin = MIN(verts[0].y, MIN(verts[1].y, verts[2].y));
		bb.zmin = MIN(verts[0].z, MIN(verts[1].z, verts[2].z));
		bb.xmax = MAX(verts[0].x, MAX(verts[1].x, verts[2].x));
		bb.ymax = MAX(verts[0].y, MAX(verts[1].y, verts[2].y));
		bb.zmax = MAX(verts[0].z, MAX(verts[1].z, verts[2].z));
	}

	bool intersects(const Ray &ray, double &t, Vector3d &normal, MaterialIO &material);
};

class SceneObject
{
public:
	ObjType type;
	int id;
	MaterialIO *materials;

	// for spheres
	float radius;
	Point3d origin;

	// for triangle meshes
	int numPolys;
	Triangle *triangles;
	KDTree *tree;

	bool intersects(const Ray &ray, double &t, Vector3d &normal, MaterialIO &material);

	SceneObject(ObjIO *obj, int id);
	~SceneObject();
};

#endif