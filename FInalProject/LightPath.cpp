#include "stdafx.h"
#include "LightPath.h"
#include <assert.h>

LightPath::LightPath(const LightIO *light) {
	this->light = light;
	this->entries = std::vector<LPEntry>();
}

int LightPath::length() const {
	return this->entries.size();
}

void LightPath::addEntry(const Point3d &p, const Vector3d &n,
	const Color3d emmittance, const double prob) {
	LPEntry e;
	e.p = p;
	e.n = n;
	e.prob = prob;
	e.emmittance = emmittance;
	this->entries.push_back(e);
}

const LPEntry &LightPath::entryAt(const int i) const {
	return this->entries[i];
}

LightPath::~LightPath()
{
	
}
