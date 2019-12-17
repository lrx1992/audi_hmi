/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: point.h
*   Author  : lubing.han
*   Date    : 2018-08-02
*   Describe: point.h
*
********************************************************/
#ifndef HOBOT_TRANSFORM_POINT_H
#define HOBOT_TRANSFORM_POINT_H

#include <iostream>
//#include "utils/transform_math.h"
using namespace std;


class Point;

class Point2D
{
public:
  Point2D() {}
  Point2D(double _x, double _y): x(_x), y(_y) {}
  Point2D(const Point& p);
  ~Point2D() {}

  operator Point();

  Point2D operator-() const {return Point2D(-x, -y);}
  double length2() const {return x * x + y * y;}
  Point product(const Point2D& p) const;
  virtual void print(const string& prefix = "", bool line_change = true) const;
  bool operator==(const Point2D & other)const {
      return x == other.x && y == other.y;
  }

  Point2D vertical()const;
  Point2D vertical(const Point2D & side)const;/* cloest to side */
  double x = 0, y = 0;
};

Point2D operator+(const Point2D& p1, const Point2D& p2);
Point2D operator-(const Point2D& p1, const Point2D& p2);
Point2D operator*(double d, const Point2D& p);
Point2D operator*(const Point2D& p, double d);
Point2D operator/(const Point2D & p, double r);
double operator*(const Point2D& p1, const Point2D& p2);
double cross_product(const Point2D & a, const Point2D & b);
Point cross_product(const Point & a, const Point & b);

class Point
{
public:
  Point() {}
  Point(double _x, double _y, double _z): x(_x), y(_y), z(_z) {}
  Point(const Point2D& p): x(p.x), y(p.y), z(0.0) {}
  ~Point() {}

  Point operator-() const {return Point(-x, -y, -z);}
  double length2() const {return x * x + y * y + z * z;}
  Point product(const Point& p) const;
  void print(const string& prefix = "", bool line_change = true) const;
  double project(const Point & v)const;
  
  double x = 0, y = 0, z = 0;
};

Point operator+(const Point& p1, const Point& p2);
Point operator-(const Point& p1, const Point& p2);
Point operator*(double d, const Point& p);
Point operator*(const Point& p, double d);
double operator*(const Point& p1, const Point& p2);
Point operator/(const Point & p, const double r);
std::ostream & operator<<(std::ostream & out, const Point2D & pt);
std::ostream & operator<<(std::ostream & out, const Point & pt);


#endif
