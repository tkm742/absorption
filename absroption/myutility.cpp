
#include <bits/stdc++.h>
#include "parameter.h"
#include "myclass.h"

double conv_rad_to_deg(double rad){
    return rad * 180 / PI;
}

double conv_deg_to_rad(double deg){
    return deg * PI / 180;
}

double calc_distance(Point p1, Point p2){

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    return sqrt(dx*dx + dy*dy);
}

double calc_theta_0_to_2pi_from_slope_and_direction(double a, bool direction){

    if(direction == LEFT){
        if(a >= 0){
            return atan(a);
        }
        else{
            return atan(a) + 2*PI;
        }
    }
    else{
        return atan(a) + PI;
    }
}

Point rotate(Point p, double theta){

    double x = cos(theta)*p.x - sin(theta)*p.y;
    double y = sin(theta)*p.x + cos(theta)*p.y;

    return {x, y};
}

Point normalize(Point p){

    double r = sqrt(p.x*p.x + p.y*p.y);
    Point p_normalized = {p.x/r, p.y/r};

    return p_normalized;
}

double inner_product(Point p1, Point p2){

    return p1.x*p2.x + p1.y*p2.y;
}


double calc_reflection_rate(Ray &ray, Collision &collision, Medium medium_array[], double theta_in){

    // 屈折率の取得
    int id_in  = ray.get_id_medium();
    int id_out = collision.get_id_medium_next();
    double n_in  = medium_array[id_in].get_n();
    double n_out = medium_array[id_out].get_n();

    // cout << id_in << " " << id_out << " " << n_in << " " << n_out << endl;

    // 垂直反射率の計算
    double dn = n_in - n_out;
    double tn = n_in + n_out;
    double f0 = dn*dn/(tn*tn);

    // 角度依存性の計算
    double cos1m  = 1 - cos(theta_in);
    double cos1m2 = cos1m*cos1m;
    return f0 + (1 - f0)*cos1m*cos1m2*cos1m2;
}