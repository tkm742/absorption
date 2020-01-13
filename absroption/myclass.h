#include <bits/stdc++.h>
using namespace std;

struct Point{
    double x;
    double y;
};

class LineBoundary{
public:
    LineBoundary(int id, Point left_end, Point right_end);
    int get_id();
    Point get_left_end();
    Point get_right_end();
    double get_a();
    double get_b();
private:
    int id;
    Point left_end;
    Point right_end;
    double a; // 境界線の傾き
    double b; // 境界線の切片
};

class ArcBoundary{
public:
    ArcBoundary(int id, Point p_start, Point p_end, double r);
    int get_id();
    double get_r();
    double get_start_angle();
    double get_end_angle();
private:
    int id;
    Point p_start;
    Point p_end;
    double r;
    double start_angle;
    double end_angle;
};

class Ray{
public:
    Ray(double intensity, double a, double b, Point p_start, bool direction, int id_medium, vector<Point> trajectory);
    double get_intensity();
    double get_a();
    double get_b();
    Point get_p_start();
    bool get_direction();
    int get_id_medium();
    vector<Point> get_trajectory();
private:
    double intensity;
    double a;
    double b;
    Point p_start;
    bool direction;
    int id_medium;
    vector<Point> trajectory;
};

class Medium{
public:
    Medium(int id, double n, double alpha);
    int get_id();
    double get_n();
    double get_alpha();
private:
    int id;
    double n;
    double alpha;
};

class Collision{
public:
    Collision();
    Collision(Point p_collision, int boundary_type, Ray &ray, LineBoundary *line, ArcBoundary *arc, int id_model);
    Point get_p_collision();
    int get_boundary_type();
    int get_id_medium_next();
    LineBoundary *get_line();
    ArcBoundary *get_arc();
private:
    Point p_collision;
    int boundary_type;
    int id_medium_next;
    LineBoundary *line;
    ArcBoundary *arc;
};