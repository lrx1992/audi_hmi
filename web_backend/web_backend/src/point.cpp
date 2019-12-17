/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: point.cpp
*   Author  : lubing.han
*   Date    : 2018-08-05
*   Describe: point.cpp
*
********************************************************/
#include "utils/point.h"
#include <cassert>


Point2D::Point2D(const Point& p): x(p.x), y(p.y) {

}

Point2D::operator Point() {
  return Point(x, y, 0);
}

Point Point2D::product(const Point2D& p) const {
  return Point(0, 0, x * p.y - y * p.x);
}

Point2D operator+(const Point2D& p1, const Point2D& p2) {
  return Point2D(p1.x + p2.x, p1.y + p2.y);
}

Point2D operator-(const Point2D& p1, const Point2D& p2) {
  return Point2D(p1.x - p2.x, p1.y - p2.y);
}

Point2D operator*(double d, const Point2D& p) {
  return Point2D(d*p.x, d*p.y);
}

Point2D operator*(const Point2D& p, double d) {
  return Point2D(d*p.x, d*p.y);
}
Point2D operator/(const Point2D&p, double r) {
    return Point2D(p.x/r, p.y/r);
}

double operator*(const Point2D& p1, const Point2D& p2) {
  return p1.x * p2.x + p1.y * p2.y;
}

void Point2D::print(const string& prefix, bool line_change) const {
  cout << prefix << "(" << x << ", " << y << ") ";
  if (line_change) cout << endl;
}

Point Point::product(const Point& p) const {
  return Point(y*p.z-z*p.y, z*p.x-x*p.z, x*p.y-y*p.x);
}

Point operator+(const Point& p1, const Point& p2) {
  return Point(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
}

Point operator-(const Point& p1, const Point& p2) {
  return Point(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

Point operator*(double d, const Point& p) {
  return Point(d*p.x, d*p.y, d*p.z);
}

Point operator*(const Point& p, double d) {
  return Point(d*p.x, d*p.y, d*p.z);
}

Point operator/(const Point & p, const double r) {
    Point rst;
    rst.x = p.x / r;
    rst.y = p.y / r;
    rst.z = p.z / r;
    return rst;
}

double operator*(const Point& p1, const Point& p2) {
  return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

void Point::print(const string& prefix, bool line_change) const {
  cout << prefix << "(" << x << ", " << y << ", " << z << ") ";
  if (line_change) cout << endl;
}


Point2D Point2D::vertical() const {
    return Point2D(-y, x);
}

Point2D Point2D::vertical(const Point2D & side) const {
    Point2D v1(-y,x);
    Point2D v2 = v1 * -1;
    return v1*side > v2*side ? v1 : v2;
}

double cross_product(const Point2D & a, const Point2D & b) {
    return a.x * b.y - a.y * b.x;
}

Point cross_product(const Point & a, const Point & b) {
    Point c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

std::ostream & operator<<(std::ostream & out, const Point2D & pt) {
    out << "(" << pt.x << "," << pt.y << ")";
    return out;
}

std::ostream & operator<<(std::ostream & out, const Point & pt) {
    out << "(" << pt.x << "," << pt.y << "," << pt.z <<  ")";
    return out;
}


