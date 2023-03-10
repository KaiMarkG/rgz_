#include "ModelStructures.h"

float dist(float x1, float y1, float x2, float y2) {
	float dy = y2 - y1;
	float dx = x2 - x1;
	return std::sqrt(dx * dx + dy * dy);
}

bool linePoint(float x1, float y1, float x2, float y2, float px, float py) {

	float d1 = dist(px, py, x1, y1);
	float d2 = dist(px, py, x2, y2);

	float lineLen = dist(x1, y1, x2, y2);

	float buffer = 0.05;   

	if (d1 + d2 >= lineLen - buffer && d1 + d2 <= lineLen + buffer) {
		return true;
	}
	return false;
}

CircleObst::CircleObst(float _x, float _y, float _r) :
	x{ _x }, y{ _y }, r{ _r }
{
}

CircleObst::CircleObst():CircleObst(0,0,0)
{
}

bool CircleObst::coll_point(float px, float py, float rr) {
	float distX = px - x;
	float distY = py - y;
	float distance = sqrt((distX * distX) + (distY * distY));
	if (distance <= r+rr) {
		return true;
	}
	return false;
}

bool CircleObst::coll_line(float x1, float y1, float x2, float y2, float rr) {
	bool inside1 = coll_point(x1, y1, rr);
	bool inside2 = coll_point(x2, y2, rr);
	if (inside1 || inside2) return true;

	float distX = x1 - x2;
	float distY = y1 - y2;
	float len = sqrt((distX * distX) + (distY * distY));

	float dot = (((x - x1) * (x2 - x1)) + ((y - y1) * (y2 - y1))) / pow(len, 2);

	float closestX = x1 + (dot * (x2 - x1));
	float closestY = y1 + (dot * (y2 - y1));

	bool onSegment = linePoint(x1, y1, x2, y2, closestX, closestY);
	if (!onSegment) return false;

	distX = closestX - x;
	distY = closestY - y;
	float distance = sqrt((distX * distX) + (distY * distY));

	if (distance <= r + rr) {
		return true;
	}
	return false;
}

float dist(point p1, point p2) {
	return dist(p1.x, p1.y, p2.x, p2.y);
}

bool linePoint(point pl1, point pl2, point p) {
	return linePoint(pl1.x, pl1.y, pl2.x, pl2.y, p.x, p.y);
}

bool CircleObst::coll_point(point p, float rr) {
	return coll_point(p.x, p.y, rr);
}

bool CircleObst::coll_line(point p1, point p2, float rr) {
	return coll_line(p1.x, p1.y, p2.x, p2.y, rr);
}

point::point(float _x, float _y):x{_x},y{_y}
{
}

point::point(const point&& p):x{p.x},y{p.y}
{
}

point::point(const point& p):x{p.x},y{p.y}
{
}

point::point():x{0.},y{0.}
{
}

point point::operator+(const point p)
{
	return point(x + p.x, y + p.y);
}

point point::operator-(const point p)
{
	return point(x - p.x, y - p.y);
}

point point::operator-()
{
	return point(-x,-y);
}

point& point::operator=(const point p)
{
	x = p.x;
	y = p.y;
	return *this;
}
