#pragma once
#include <cmath>
#include <utility>

struct point {
	float x, y;
	point(float _x, float _y);
	
	point(const point&& p);
	point(const point& p);
	point();

	point operator+(const point p);
	point operator-(const point p);
	point operator-();
	point& operator=(const point p);
};

float dist(float x1, float y1, float x2, float y2);
float dist(point p1, point p2);

bool linePoint(float x1, float y1, float x2, float y2, float px, float py);
bool linePoint(point pl1, point pl2, point p);


struct CircleObst {
	float x, y, r;

	CircleObst(float _x, float _y, float _r);
	CircleObst();
	bool coll_point(float px, float py, float rr);
	bool coll_point(point p, float rr);

	bool coll_line(float x1, float y1, float x2, float y2, float rr);
	bool coll_line(point p1, point p2, float rr);
};