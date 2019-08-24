#ifndef LIGHT_PATH_H
#define LIGHT_PATH_H

#include <vector>
#include "Render.h"

struct LPEntry {
	Point3d p;
	Vector3d n;
	Color3d emmittance;
	double prob;
};

class LightPath
{
public:
	LightIO const *light;
	std::vector<LPEntry> entries;
	void addEntry(const Point3d &p, const Vector3d &n, const Color3d emmittance, const double prob);

	const LPEntry &entryAt(const int i) const;
	int length() const;
	LightPath(const LightIO *light);

	~LightPath();
};

#endif