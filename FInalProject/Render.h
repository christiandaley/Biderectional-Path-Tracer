#ifndef RENDER_H
#define RENDER_H

#include <cmath>
#include <iostream>
#include "scene_io.h"

#define MULTITHREAD
//#define NUM_THREADS 2

#define IMPORTANCE_SAMPLING
#define LIGHT_TRACING

#define X_SAMPLES 64
#define Y_SAMPLES 64
#define EPSILON 0.000001
#define RUSSIAN_ROULETTE 0.8
#define MY_PI 3.14159265359
#define MAX_DEPTH 10

#define MIN(a, b) ((a) <= (b) ? (a) : (b))
#define MAX(a, b) ((a) >= (b) ? (a) : (b))

struct Vector3d {
	double x, y, z;
	Vector3d(const float *a) {
		x = a[0];
		y = a[1];
		z = a[2];
	}
	Vector3d() {
		x = 0.0, y = 0.0, z = 0.0;
	}

	Vector3d cross(const Vector3d &v) const {
		Vector3d result;
		result.x = (this->y * v.z) - (this->z * v.y);
		result.y = -((this->x * v.z) - (this->z * v.x));
		result.z = (this->x * v.y) - (this->y * v.x);

		return result;
	}

	void normalize() {
		double m = this->mag();
		this->x /= m;
		this->y /= m;
		this->z /= m;
	}

	double mag() const {
		return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
	}

	double dist2(const Vector3d &v) const {
		double dx = this->x - v.x;
		double dy = this->y - v.y;
		double dz = this->z - v.z;

		return dx * dx + dy * dy + dz * dz;
	}

	void print() const {
		std::cout << x << ", " << y << ", " << z << std::endl;
	}

};

typedef Vector3d Point3d;
typedef Vector3d Color3d;

struct Ray {
	Point3d O;
	Vector3d D;
};

void render(const SceneIO *scene, const int xres, const int yres, const char *name);


Vector3d operator+(const Vector3d &v1, const Vector3d &v2);
Vector3d operator-(const Vector3d &v1, const Vector3d &v2);
double operator*(const Vector3d &v1, const Vector3d &v2);
Vector3d operator*(const Vector3d &v1, const double c);
Vector3d operator*(const double c, const Vector3d &v1);
Vector3d operator-(const Vector3d &v);

#endif
