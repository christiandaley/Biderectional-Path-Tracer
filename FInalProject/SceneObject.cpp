#include "stdafx.h"
#include <cmath>
#include <assert.h>
#include <iostream>
#include "SceneObject.h"
#include "KDTree.h"

double areaOfTriangle(Point3d &v0, Point3d &v1, Point3d &v2) {
	double a, b, c, p;
	a = (v1 - v0).mag();
	b = (v2 - v1).mag();
	c = (v0 - v2).mag();
	p = (a + b + c) / 2.0;

	return sqrt(p * (p - a) * (p - b) * (p - c));
}

SceneObject::SceneObject(ObjIO *obj, int id) {
	type = obj->type;
	this->id = id;
	materials = obj->material;

	if (type == SPHERE_OBJ) {
		SphereIO *sphere = (SphereIO *)obj->data;
		radius = sphere->radius;
		origin = Point3d(sphere->origin);
		
	}
	else if (type == POLYSET_OBJ) {
		PolySetIO *mesh = (PolySetIO *)obj->data;
		numPolys = mesh->numPolys;
		triangles = new Triangle[numPolys];

		for (int i = 0; i < numPolys; i++) {
			assert(mesh->poly[i].numVertices == 3);
			triangles[i].verts[0] = Point3d(mesh->poly[i].vert[0].pos);
			triangles[i].verts[1] = Point3d(mesh->poly[i].vert[1].pos);
			triangles[i].verts[2] = Point3d(mesh->poly[i].vert[2].pos);

			triangles[i].edge1 = triangles[i].verts[1] - triangles[i].verts[0];
			triangles[i].edge2 = triangles[i].verts[2] - triangles[i].verts[0];

			// determine the type of normals
			if (mesh->normType == PER_FACE_NORMAL) {
				triangles[i].normals[0] = triangles[i].edge1.cross(triangles[i].edge2);
				triangles[i].normals[0].normalize();
				triangles[i].intNormals = false;
			}
			else {
				triangles[i].normals[0] = Vector3d(mesh->poly[i].vert[0].norm);
				triangles[i].normals[1] = Vector3d(mesh->poly[i].vert[1].norm);
				triangles[i].normals[2] = Vector3d(mesh->poly[i].vert[2].norm);
				triangles[i].intNormals = true;
			}

			// now determine the type of material binding
			if (mesh->materialBinding == PER_OBJECT_MATERIAL) {
				triangles[i].materials[0] = materials;
				triangles[i].intMaterial = false;
			}
			else {
				
				for (int j = 0; j < 3; j++) {
					triangles[i].materials[j] = &obj->material[mesh->poly[i].vert[j].materialIndex];
				}

				triangles[i].intMaterial = true;
			}

			triangles[i].area = areaOfTriangle(triangles[i].verts[0], 
												triangles[i].verts[1], 
												triangles[i].verts[2]);

			triangles[i].bound();

		}

		Triangle **temp = new Triangle *[numPolys];
		for (int i = 0; i < numPolys; i++) {
			temp[i] = &this->triangles[i];
		}
		this->tree = new KDTree(this->numPolys, temp, X_SPLIT);
		delete[] temp;
	}
	else
		assert(false);
}

bool SceneObject::intersects(const Ray &ray, double &t, Vector3d &normal, MaterialIO &material) {
	bool intersects = false;

	if (type == SPHERE_OBJ) {
		// Get the ray's origin in object space
		Point3d O = ray.O - origin;
		Vector3d D = ray.D;
		// (Dt + O)^2 = R^2
		// D^2t^2 + 2ODt + O^2 - R^2 = 0
		double a, b, c, disc, dist, u, v;
		a = D * D;
		b = 2.0 * O * D;
		c = O*O - radius * radius;
		disc = b*b - 4.0 * a * c;
		if (disc < 0.0)
			return false;

		// try the - solution first
		t = 1e10;
		dist = (-b - sqrt(disc)) / 2.0 / a;
		if (dist > EPSILON) {
			t = dist;
			normal = O + t * D;
			normal.normalize();
			material = materials[0];
			intersects = true;
		}
		// now try the + solution
		dist = (-b + sqrt(disc)) / 2.0 / a;
		if (dist > EPSILON && dist < t) {
			t = dist;
			normal = O + t * D;
			normal.normalize();
			material = materials[0];
			intersects = true;
		}

		return intersects;

	}
	
	assert(type == POLYSET_OBJ);
	// triangle intersection

	return tree->intersects(ray, t, normal, material);
}

SceneObject::~SceneObject()
{
	if (type == POLYSET_OBJ) {
		delete[] triangles;
		delete tree;
	}
}

bool Triangle::intersects(const Ray &ray, double &t, Vector3d &normal, MaterialIO &material) {

	Vector3d h, s, q;
	double a, f, u, v;
	double a1, a2, a3;

	Point3d &v0 = verts[0];
	Point3d &v1 = verts[1];
	Point3d &v2 = verts[2];

	h = ray.D.cross(edge2);
	a = edge1 * h;
	if (a > -EPSILON && a < EPSILON)
		return false;

	f = 1.0 / a;
	s = ray.O - v0;
	u = f * s * h;
	if (u < 0.0 || u > 1.0)
		return false;

	q = s.cross(edge1);
	v = f * ray.D * q;
	if (v < 0.0 || u + v > 1.0)
		return false;

	t = f * edge2 * q;
	if (t > EPSILON) {

		if (intMaterial || intNormals) {
			Point3d p = ray.O + t * ray.D;
			a1 = areaOfTriangle(v1, v2, p) / this->area;
			a2 = areaOfTriangle(v2, v0, p) / this->area;
			a3 = areaOfTriangle(v0, v1, p) / this->area;
		}

		// set the material
		if (intMaterial) {
			for (int i = 0; i < 3; i++) {
				material.ambColor[i] = (a1 * materials[0]->ambColor[i]) +
					(a2 * materials[1]->ambColor[i]) +
					(a3 * materials[2]->ambColor[i]);

				material.diffColor[i] = (a1 * materials[0]->diffColor[i]) +
					(a2 * materials[1]->diffColor[i]) +
					(a3 * materials[2]->diffColor[i]);

				material.specColor[i] = (a1 * materials[0]->specColor[i]) +
					(a2 * materials[1]->specColor[i]) +
					(a3 * materials[2]->specColor[i]);

			}

			material.ktran = (a1 * materials[0]->ktran) + (a2 * materials[1]->ktran)
				+ (a3 * materials[2]->ktran);

			// We store the diffuse weight in the shininess field
			material.shininess = (a1 * diffWeights[0]) 
				+ (a2 * diffWeights[1])
				+ (a3 * diffWeights[2]);

		}
		else {
			material = *materials[0];
			material.shininess = diffWeights[0];
		}

		// set the normal
		if (intNormals) {
			normal = (a1 * normals[0]) + (a2 * normals[1]) + (a3 * normals[2]);
			normal.normalize();
		}
		else {
			normal = normals[0];
		}


		return true;
	}

	return false;
}
