#include <bits/stdc++.h>
#include "myclass.h"
#include "parameter.h"

extern string name_boundary[2];
extern string name_direction[2];
extern string name_medium[3];

LineBoundary::LineBoundary(int id, Point left_end, Point right_end){
    this->id = id;
    this->left_end = left_end;
    this->right_end = right_end;
    this->a = (right_end.y - left_end.y) / (right_end.x - left_end.x);
    this->b = (right_end.x * left_end.y - left_end.x * right_end.y) / (right_end.x - left_end.x);
}

int LineBoundary::get_id(){
    return this->id;
}

Point LineBoundary::get_left_end(){
    return this->left_end;
}

Point LineBoundary::get_right_end(){
    return this->right_end;
}

double LineBoundary::get_a(){
    return this->a;
}

double LineBoundary::get_b(){
    return this->b;
}

ArcBoundary::ArcBoundary(int id, Point p_start, Point p_end, double r){
    this->id = id;
    this->p_start = p_start;
    this->p_end = p_end;
    this->r = r;
    this->start_angle  = acos(p_start.x / r);
    this->end_angle = acos(p_end.x / r);
}

int ArcBoundary::get_id(){
    return this->id;
}

double ArcBoundary::get_r(){
    return this->r;
}

double ArcBoundary::get_start_angle(){
    return this->start_angle;
}

double ArcBoundary::get_end_angle(){
    return this->end_angle;
}

Ray::Ray(double intensity, double a, double b, Point p_start, bool direction, int id_medium, vector<Point> trajectory){
    this->intensity = intensity;
    this->a = a;
    this->b = b;
    this->p_start = p_start;
    this->direction = direction;
    this->id_medium = id_medium;
    this->trajectory = trajectory;
}

double Ray::get_intensity(){
    return this->intensity;
}

double Ray::get_a(){
    return this->a;
}

double Ray::get_b(){
    return this->b;
}

Point Ray::get_p_start(){
    return this->p_start;
}

bool Ray::get_direction(){
    return this->direction;
}

int Ray::get_id_medium(){
    return this->id_medium;
}

vector<Point> Ray::get_trajectory(){
    return this->trajectory;
}

Medium::Medium(int id, double n, double alpha){
    this->id = id;
    this->n = n;
    this->alpha = alpha;
}

int Medium::get_id(){
    return this->id;
}

double Medium::get_n(){
    return this->n;
}

double Medium::get_alpha(){
    return this->alpha;
}

Collision::Collision(){
    
}

Collision::Collision(Point p_collision, int boundary_type, Ray &ray, LineBoundary *line, ArcBoundary *arc, int id_model){
    this->p_collision = p_collision;
    this->boundary_type = boundary_type;
    this->line = line;
    this->arc = arc;

    // もし境界が設定されていないならエラー
    if( (boundary_type == LINE && line == NULL) || (boundary_type == ARC && arc == NULL) ){
        cout << "error: invalid operand of Collision class constructor." << endl;
        exit(EXIT_FAILURE);
    }

    int id_medium_now = ray.get_id_medium();

    // モデル１の場合
    if(id_model == MODEL1){
        // 直線との衝突の場合
        if(boundary_type == LINE){
            if(line->get_id() == 0 || line->get_id() == 4){
                if(id_medium_now == AIR){
                    this->id_medium_next = CHLOROPLAST;
                }
                else if(id_medium_now == CHLOROPLAST){
                    this->id_medium_next = AIR;
                }
                else{
                    this->id_medium_next = -1;
                }
            }
            else if(line->get_id() == 1 || line->get_id() == 2 || line->get_id() == 3){
                if(id_medium_now == AIR){
                    this->id_medium_next = WATER;
                }
                else if(id_medium_now == WATER){
                    this->id_medium_next = AIR;
                }
                else{
                    this->id_medium_next = -1;
                }
            }
            else{
                cerr << __FILE__ << " " << __LINE__ << " error: invalid id of LineBoundary." << endl;
                exit(EXIT_FAILURE);
            }
        }
        // 円との衝突の場合
        else{
            if(arc->get_id() == 0){
                if(id_medium_now == AIR){
                    this->id_medium_next = CHLOROPLAST;
                }
                else if(id_medium_now == CHLOROPLAST){
                    this->id_medium_next = AIR;
                }
                else{
                    this->id_medium_next = -1;
                }
            }
            else if(arc->get_id() == 1){
                if(id_medium_now == CHLOROPLAST){
                    this->id_medium_next = WATER;
                }
                else if(id_medium_now == WATER){
                    this->id_medium_next = CHLOROPLAST;
                }
                else{
                    this->id_medium_next = -1;
                }
            }
            else{
                cerr << __FILE__ << " " << __LINE__ << " error: invalid id of ArcBoundary." << endl;
                exit(EXIT_FAILURE);
            }
        }
    }
    // モデル2の場合
    else if(id_model == MODEL2){
        // 直線との衝突の場合
        if(boundary_type == LINE){
            if(line->get_id() == 0 || line->get_id() == 4){
                if(id_medium_now == WATER){
                    this->id_medium_next = CHLOROPLAST;
                }
                else if(id_medium_now == CHLOROPLAST){
                    this->id_medium_next = WATER;
                }
                else{
                    this->id_medium_next = -1;
                }
            }
            else{
                cerr << __FILE__ << " " << __LINE__ << " error: invalid id of LineBoundary." << endl;
                exit(EXIT_FAILURE);
            }
        }
        // 円弧との衝突の場合
        else{
            if(arc->get_id() == 0){
                if(id_medium_now == AIR){
                    this->id_medium_next = CHLOROPLAST;
                }
                else if(id_medium_now == CHLOROPLAST){
                    this->id_medium_next = AIR;
                }
                else{
                    this->id_medium_next = -1;
                }
            }
            else if(arc->get_id() == 1){
                if(id_medium_now == CHLOROPLAST){
                    this->id_medium_next = WATER;
                }
                else if(id_medium_now == WATER){
                    this->id_medium_next = CHLOROPLAST;
                }
                else{
                    this->id_medium_next = -1;
                }
            }
            else if(arc->get_id() == 2){
                if(id_medium_now == AIR){
                    this->id_medium_next = WATER;
                }
                else if(id_medium_now == WATER){
                    this->id_medium_next = AIR;
                }
                else{
                    this->id_medium_next = -1;
                }
            }
            else{
                cerr << __FILE__ << " " << __LINE__ << " error: invalid id of ArcBoundary." << endl;
                exit(EXIT_FAILURE);
            }
        }
    }
    // 存在しないモデルが指定された場合
    else{
        cerr << __FILE__ << " " << __LINE__ << " error: invalid id of model." << endl;
        exit(EXIT_FAILURE);
    }
}

Point Collision::get_p_collision(){
    return this->p_collision;
}

int Collision::get_boundary_type(){
    return this->boundary_type;
}

int Collision::get_id_medium_next(){
    return this->id_medium_next;
}

LineBoundary *Collision::get_line(){
    return this->line;
}

ArcBoundary *Collision::get_arc(){
    return this->arc;
}