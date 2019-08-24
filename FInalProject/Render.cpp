#include "stdafx.h"
#include <cmath>
#include <iostream>
#include <assert.h>
#include <vector>
#include <atlimage.h>
#include <Gdiplusimaging.h>
#include <Gdiplus.h>
#include <thread>
#include <time.h>
#include "Render.h"
#include "SceneObject.h"
#include "LightPath.h"
#include "Timer.h"
#include <random>

#include "Render.h"
#include "SceneObject.h"

Point3d cameraPosition, imageCenter;
Vector3d X, Y;
std::vector<SceneObject *> objects;
std::vector<LightIO *> lights;
Color *pixel_buffer;

static void draw_chunk(const int yOff, const int height, const int xres, const int yres);
static Color3d trace_eye_path(const std::vector<LightPath> &paths, const Ray &ray, const double cosTheta, const int depth);
static void trace_light_path(LightPath &lightPath, const Ray &ray, 
	const Color3d &intensity, const double prob, const int depth);
static bool trace_shadow_ray(const Point3d &from, const Point3d &to);
static double shoot_light_ray(const LightIO *light, Ray &ray);
static Vector3d rand_unit_vec();
static Vector3d rand_unit_vec_in_hemisphere(const Vector3d &v);
static Vector3d rand_weighted_unit_vec_in_hemisphere(const Vector3d &v);

double diff_prob(const MaterialIO &m);
static COLORREF make_color_ref(Color c);


void render(const SceneIO *scene, const int xres, const int yres, const char *name) {

	CameraIO *camera = scene->camera;
	srand(time(NULL));

	camera = scene->camera;

	const float c = camera->focalDistance;
	Vector3d V = Vector3d(camera->viewDirection);
	Vector3d U = Vector3d(camera->orthoUp);
	cameraPosition = Point3d(camera->position);
	U.normalize();
	V.normalize();

	Vector3d right = V.cross(U);
	right.normalize();

	Vector3d up = right.cross(V);
	up.normalize();

	imageCenter = cameraPosition + c * V;
	Y = c * tanf(camera->verticalFOV / 2.0f) * up;
	X = c * tanf(camera->verticalFOV * (float)xres / (float)yres / 2.0f) * right;

	objects = std::vector<SceneObject *>();
	int id = 0;
	for (ObjIO *obj = scene->objects; obj != NULL; obj = obj->next, id++) {
		objects.push_back(new SceneObject(obj, id));
	}

	lights = std::vector<LightIO *>();
	for (LightIO *light = scene->lights; light != NULL; light = light->next) {
		if (light->type == POINT_LIGHT)
			lights.push_back(light);
	}

	pixel_buffer = new Color[xres * yres];

#ifdef MULTITHREAD
#ifdef NUM_THREADS
	const int nThreads = NUM_THREADS;
#else
	const int nThreads = std::thread::hardware_concurrency() * 2;
#endif
	std::thread *threads = new std::thread[nThreads];
	int height = yres / nThreads;
	for (int i = 0; i < nThreads; i++) {
		double h = i == nThreads - 1 ? yres - i * height : height;
		threads[i] = std::thread(draw_chunk, i * height, h, xres, yres);
	}

	for (int i = 0; i < nThreads; i++)
		threads[i].join();

	delete[] threads;

#else
	draw_chunk(0, yres, xres, yres);
#endif

	while (objects.size() > 0) {
		delete objects.back();
		objects.pop_back();
	}

	CImage image;
	image.Create(xres, yres, 24, 0);

	for (int j = 0; j < yres; j++) {
		for (int i = 0; i < xres; i++) {
			image.SetPixel(i, yres - j - 1, make_color_ref(pixel_buffer[(j * xres) + i]));
		}
	}

	delete[] pixel_buffer;
	pixel_buffer = NULL;

	wchar_t filename[128];
	swprintf_s(filename, L"%S", name);
	image.Save(filename, Gdiplus::ImageFormatPNG);

}

static void draw_chunk(const int yOff, const int height, const int xres, const int yres) {
	for (int j = yOff; j < yOff + height; j++) {
		for (int i = 0; i < xres; i++) {
			Color color = { 0.0, 0.0, 0.0 };
			for (int x = 0; x < X_SAMPLES; x++) {
				for (int y = 0; y < Y_SAMPLES; y++) {
					double sx, sy;
					sx = (double)i + ((double)x + (double)rand() / RAND_MAX) / X_SAMPLES;
					sx /= xres;
					sx = sx * 2.0 - 1.0;
					sy = (double)j + ((double)y + (double)rand() / RAND_MAX) / Y_SAMPLES;
					sy /= yres;
					sy = sy * 2.0 - 1.0;
					Point3d p = imageCenter + (sx * X) + (sy * Y);
					Ray eyeRay, lightRay;
					eyeRay.O = cameraPosition;
					eyeRay.D = p - cameraPosition;
					eyeRay.D.normalize();
					std::vector<LightPath> paths = std::vector<LightPath>();
					for (int k = 0; k < lights.size(); k++) {
						LightPath lightPath = LightPath(lights[k]);
#ifdef LIGHT_TRACING
						Ray lightRay;
						double prob = shoot_light_ray(lights[k], lightRay);
						trace_light_path(lightPath, lightRay, lights[k]->color, prob, 0);
#endif
						paths.push_back(lightPath);

					}

					Color3d temp = trace_eye_path(paths, eyeRay, 1.0, 0);
					color[0] += temp.x;
					color[1] += temp.y;
					color[2] += temp.z;

				}
			}

			color[0] /= (float)(X_SAMPLES * Y_SAMPLES);
			color[1] /= (float)(X_SAMPLES * Y_SAMPLES);
			color[2] /= (float)(X_SAMPLES * Y_SAMPLES);

			color[0] = MIN(1.0, color[0]);
			color[1] = MIN(1.0, color[1]);
			color[2] = MIN(1.0, color[2]);

			memcpy(pixel_buffer[(j * xres) + i], color, sizeof(Color));

		}
	}
}

static Color3d trace_eye_path(const std::vector<LightPath> &paths, const Ray &ray, const double cosTheta, const int depth) {
	if (depth >= MAX_DEPTH)
		return Color3d();

	SceneObject *obj = NULL;
	Vector3d normal, n;
	MaterialIO material, m;
	Point3d intersection;
	double t;
	double min_dist = DBL_MAX;
	for (std::vector<SceneObject *>::iterator it = objects.begin(); it != objects.end(); it++) {
		if ((*it)->intersects(ray, t, n, m) && t < min_dist) {
			min_dist = t;
			normal = n;
			material = m;
			obj = *it;
			intersection = ray.O + min_dist * ray.D;
		}
	}

	if (obj == NULL)
		return Color3d();


	if (ray.D * normal > 0.0)
		normal = -normal;

	double dot;
	Color3d color = Color3d();

	// Direct lighting

	double diff = diff_prob(material);

	if (diff != 0.0) {
		for (std::vector<LightPath>::const_iterator it = paths.begin(); it != paths.end(); it++) {
			Vector3d dir;
			const LightPath &lightPath = *it;
			if (trace_shadow_ray(intersection, Point3d(lightPath.light->position))) {
				// Direct lighting
				dir = Point3d(lightPath.light->position) - intersection;
				dir.normalize();
				dot = MAX(0.0, dir * normal);

				color.x += dot * lightPath.light->color[0] * material.diffColor[0];
				color.y += dot * lightPath.light->color[1] * material.diffColor[1];
				color.z += dot * lightPath.light->color[2] * material.diffColor[2];
			}

#ifdef LIGHT_TRACING
			for (int i = 0; i < lightPath.length(); i++) {
				const LPEntry &e = lightPath.entryAt(i);
				dir = e.p - intersection;
				dir.normalize();
				dot = MAX(0.0, dir * normal);

				if (trace_shadow_ray(intersection, e.p)) {
					color.x += dot * e.emmittance.x * material.diffColor[0] / e.prob;
					color.y += dot * e.emmittance.y * material.diffColor[1] / e.prob;
					color.z += dot * e.emmittance.z * material.diffColor[2] / e.prob;
				}
			}
#endif
		}
	}

	dot = -ray.D * normal;

	if ((double)rand() / RAND_MAX <= diff && diff != 0.0) {
		// Russian roulette
		if ((double)rand() / RAND_MAX > cosTheta)
			return color;

		Ray scatter;

#ifdef IMPORTANCE_SAMPLING
		scatter.D = rand_weighted_unit_vec_in_hemisphere(normal);
		const double prob = diff * cosTheta / MY_PI;
		scatter.O = intersection + EPSILON * scatter.D;
		Color3d incoming = trace_eye_path(paths, scatter, scatter.D * normal, depth + 1) * (1.0 / prob);
#else
		scatter.D = rand_unit_vec_in_hemisphere(normal);
		const double prob =  diff * cosTheta / (2.0 * MY_PI);
		scatter.O = intersection + EPSILON * scatter.D;
		Color3d incoming = (scatter.D * normal) * trace_eye_path(paths, scatter, scatter.D * normal, depth + 1) * (1.0 / prob);
#endif
		
		color.x += incoming.x * material.diffColor[0] / MY_PI;
		color.y += incoming.y * material.diffColor[1] / MY_PI;
		color.z += incoming.z * material.diffColor[2] / MY_PI;
	}
	else {

		const double prob = 1.0 - diff;
		Ray reflect;
		reflect.D = -2.0 * (ray.D * normal) * normal + ray.D;
		reflect.O = intersection + EPSILON * reflect.D;

		Color3d incoming = trace_eye_path(paths, reflect, 1.0, depth + 1) * (1.0 / prob);
		color.x += incoming.x * material.specColor[0] / MY_PI;
		color.y += incoming.y * material.specColor[1] / MY_PI;
		color.z += incoming.z * material.specColor[2] / MY_PI;
	}

	return color;
}

static void trace_light_path(LightPath &lightPath, const Ray &ray, const Color3d &intensity, 
	const double prob, const int depth) {

	if (depth >= MAX_DEPTH)
		return;

	SceneObject *obj = NULL;
	Vector3d normal, n;
	MaterialIO material, m;
	Point3d intersection;
	double t;
	double min_dist = DBL_MAX;
	for (std::vector<SceneObject *>::iterator it = objects.begin(); it != objects.end(); it++) {
		if ((*it)->intersects(ray, t, n, m) && t < min_dist) {
			min_dist = t;
			normal = n;
			material = m;
			obj = *it;
			intersection = ray.O + min_dist * ray.D;
		}
	}

	if (obj == NULL)
		return;

	if (ray.D * normal > 0.0)
		normal = -normal;

	Color3d emmittance;
	double dot = MAX(0.0, -ray.D * normal);

	emmittance.x = dot * intensity.x * material.diffColor[0] / MY_PI;
	emmittance.y = dot * intensity.y * material.diffColor[1] / MY_PI;
	emmittance.z = dot * intensity.z * material.diffColor[2] / MY_PI;
	
	lightPath.addEntry(intersection, normal, emmittance, prob);

	double diff = diff_prob(material);

	if ((double)rand() / RAND_MAX <= diff && diff != 0.0) {
		if ((double)rand() / RAND_MAX > dot)
			return;

		Ray scatter;

		const double newProb = diff * dot / (2.0 * MY_PI);
		scatter.D = rand_unit_vec_in_hemisphere(normal);
		scatter.O = intersection + EPSILON * scatter.D;

		trace_light_path(lightPath, scatter, emmittance, newProb, depth + 1);
	}
	else {
		Color3d ref;
		ref.x = intensity.x * material.specColor[0] / MY_PI;
		ref.y = intensity.y * material.specColor[1] / MY_PI;
		ref.z = intensity.z * material.specColor[2] / MY_PI;

		const double newProb = 1.0 - diff;
		Ray reflect;
		reflect.D = -2.0 * (ray.D * normal) * normal + ray.D;
		reflect.O = intersection + EPSILON * reflect.D;
		trace_light_path(lightPath, reflect, ref, newProb, depth + 1);
	}

}

static bool trace_shadow_ray(const Point3d &from, const Point3d &to) {
	SceneObject *obj = NULL;
	Vector3d normal, n;
	MaterialIO material, m;
	double t;
	Ray ray;
	ray.O = from;
	ray.D = to - from;
	ray.D.normalize();
	double dist = (to - from).mag();

	for (std::vector<SceneObject *>::iterator it = objects.begin(); it != objects.end(); it++) {
		if ((*it)->intersects(ray, t, n, m) && t < dist) {
			return false;
		}
	}

	return true;
}

static double shoot_light_ray(const LightIO *light, Ray &ray) {
	if (light->type == POINT_LIGHT) {
		ray.O = Point3d(light->position);
		ray.D = rand_unit_vec();
		return 1.0 / (4.0 * MY_PI);
	}


	return 0.0;
}


static Vector3d rand_unit_vec() {
	Vector3d vec;
	double theta = MY_PI * ((double)rand() / RAND_MAX);
	double phi = 2.0 * MY_PI * ((double)rand() / RAND_MAX);
	vec.x = sin(theta) * cos(phi);
	vec.y = sin(theta) * sin(phi);
	vec.z = cos(theta);
	return vec;
}

static Vector3d rand_unit_vec_in_hemisphere(const Vector3d &v) {
	Vector3d orth;

	double theta = MY_PI / 2.0 * ((double)rand() / RAND_MAX);
	double phi = 2.0 * MY_PI * ((double)rand() / RAND_MAX);

	// orth.x * v.x + orth.y * v.y + orth.z * v.Z = 0
	// we let orth.y and orth.z = 1
	// orth.x * v.x + v.y + v.z = 0
	// orth.x = -(v.y + v.z) / v.x
	if (v.x != 0.0) {
		orth.x = -(v.y + v.z) / v.x;
		orth.y = 1.0;
		orth.z = 1.0;
	}
	else if (v.y != 0.0)
	{
		orth.y = -(v.x + v.z) / v.y;
		orth.x = 1.0;
		orth.z = 1.0;
	}
	else if (v.z != 0.0) {
		orth.z = -(v.x + v.y) / v.z;
		orth.x = 1.0;
		orth.y = 1.0;
	}

	orth.normalize();
	Vector3d cross = v.cross(orth);

	Vector3d projX = orth * sin(theta) * cos(phi);
	Vector3d projY = cross * sin(theta) * sin(phi);
	Vector3d projZ = v * cos(theta);

	return projX + projY + projZ;
}

static Vector3d rand_weighted_unit_vec_in_hemisphere(const Vector3d &v) {
	double theta = asin((double)rand() / RAND_MAX);
	double phi = 2.0 * MY_PI * ((double)rand() / RAND_MAX);

	Vector3d orth;
	// orth.x * v.x + orth.y * v.y + orth.z * v.Z = 0
	// we let orth.y and orth.z = 1
	// orth.x * v.x + v.y + v.z = 0
	// orth.x = -(v.y + v.z) / v.x
	if (v.x != 0.0) {
		orth.x = -(v.y + v.z) / v.x;
		orth.y = 1.0;
		orth.z = 1.0;
	}
	else if (v.y != 0.0)
	{
		orth.y = -(v.x + v.z) / v.y;
		orth.x = 1.0;
		orth.z = 1.0;
	} else if (v.z != 0.0) {
		orth.z = -(v.x + v.y) / v.z;
		orth.x = 1.0;
		orth.y = 1.0;
	}

	orth.normalize();
	Vector3d cross = v.cross(orth);
	cross.normalize();

	Vector3d projX = orth * sin(theta) * cos(phi);
	Vector3d projY = cross * sin(theta) * sin(phi);
	Vector3d projZ = v * cos(theta);


	return projX + projY + projZ;
}

static double diff_prob(const MaterialIO &m) {
	double diff = m.diffColor[0] + m.diffColor[1] + m.diffColor[2];
	double d = m.specColor[0] + m.specColor[1] + m.specColor[2] + diff;

	return diff / d;
}

static COLORREF make_color_ref(Color c) {
	unsigned int color = (int)(c[2] * 255.0f);
	color = (color << 8) | (int)(c[1] * 255.0f);
	return (color << 8) | (int)(c[0] * 255.0f);
}

Vector3d operator+(const Vector3d &v1, const Vector3d &v2) {
	Vector3d v;
	v.x = v1.x + v2.x;
	v.y = v1.y + v2.y;
	v.z = v1.z + v2.z;
	return v;
}

Vector3d operator-(const Vector3d &v1, const Vector3d &v2) {
	Vector3d v;
	v.x = v1.x - v2.x;
	v.y = v1.y - v2.y;
	v.z = v1.z - v2.z;
	return v;
}

double operator*(const Vector3d &v1, const Vector3d &v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3d operator*(const Vector3d &v1, const double c) {
	Vector3d v;
	v.x = v1.x * c;
	v.y = v1.y * c;
	v.z = v1.z * c;
	return v;
}

Vector3d operator*(const double c, const Vector3d &v1) {
	return v1 * c;
}

Vector3d operator-(const Vector3d &v) {
	return -1.0 * v;
}