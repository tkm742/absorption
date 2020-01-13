#include <bits/stdc++.h>
#include "parameter.h"
#include "myclass.h"
#include "myutility.h"
using namespace std;

// #define DEBUG

string name_boundary[2] = {"line", "arc"};
string name_direction[2] = {"left", "right"};
string name_medium[3] = {"air", "chloroplast", "water"};

const int N_LINE[2] = {N_LINE1, N_LINE2};
const int N_ARC[2] = {N_ARC1, N_ARC2};

// 入射光を生成する（切り欠き部分に入る入射光をランダム生成)
vector<Ray> generate_initial_rays_1(int n_ray_initial, Point end_point_0, Point end_point_5);

// 入射光を生成する（ある特定の角度theta_inで入射する平行光線を生成)
vector<Ray> generate_initial_rays_2(int n_ray_initial, double r, double theta_in);

// 直線境界と光線の衝突を判定し、衝突点を算出する
bool detect_collision_with_line(Collision &collision, Ray &ray, LineBoundary &line, int id_model);

// 円弧境界と光線の衝突を判定し、衝突点を算出する
bool detect_collision_with_arc(Collision &collision, Ray &ray, ArcBoundary &arc, int id_model);

// 光線追跡を幅優先探索で行い吸収量を計算する
double calc_absorption_by_bfs_ray_tracing(Ray &ray_initial, Medium medium_array[], LineBoundary line_boundary_array[], ArcBoundary arc_boundary_array[], int id_model);

// 衝突点候補から、正しい衝突点を抽出する
Collision extract_valid_collision_point(vector<Collision> vec_collision, Point p_start);

// １直進経路における吸収光量を計算する
double calc_single_absorption(Ray &ray, Collision &collision, Medium medium_array[]);

// 入射角の計算
double calc_theta_in(Ray &ray, Collision &collision);

// 屈折角の計算
double calc_theta_out(Ray &ray, Collision &collision, Medium medium_array[], double theta_in);

// 反射光の計算 & キューへの追加
void calc_reflected_ray(queue<Ray> &queue_ray, Ray &ray, Collision &collision, double reflection_rate, double theta_in);

// 屈折光の計算
void calc_refracted_ray(queue<Ray> &queue_ray, Ray &ray, Collision &collision, double reflection_rate, double theta_out);

// 軌跡を出力
void print_trajectory(Ray &ray);


















int main(void){

    cout << endl << "[absorption calculator v1.0]" << endl;

   srand((unsigned int)time(NULL));
    // srand(100);

    // 形状パラメータの設定
    const double h = 0.05; // cm
    const double w = 0.03; // cm
    const double alpha = conv_deg_to_rad(30); // rad
    const double r = 0.1; // cm
    const double d = 0.025; // cm

    // 計算条件の設定
    const int n_ray_initial = 1000;

    // 物理パラメータの設定
    const double n_air = 1.0; // 空気の屈折率
    const double n_chloroplast = 1.333; // 葉緑体領域の屈折率
    const double n_water = 1.333; // 水の屈折率
    const double alpha_air = 0.0; // 空気の吸収係数
    const double alpha_chloroplast = 1e-2; // 葉緑体領域の吸収係数
    const double alpha_water = 1e-4; // 水の吸収係数

    Medium medium_air(AIR, n_air, alpha_air);
    Medium medium_chloroplast(CHLOROPLAST, n_chloroplast, alpha_chloroplast);
    Medium medium_water(WATER, n_water, alpha_water);
    Medium medium_array[3] = {medium_air, medium_chloroplast, medium_water};

    // 境界点
    Point end_point_0 = {-r*sin(alpha/2), r*cos(alpha/2)};
    Point end_point_2 = {-w/2, r-h};
    Point end_point_3 = { w/2, r-h};
    Point end_point_5 = { r*sin(alpha/2), r*cos(alpha/2)};
    double a_temp = (end_point_2.y - end_point_0.y)/(end_point_2.x - end_point_0.x);
    double b_temp = end_point_0.y - a_temp*end_point_0.x;
    double A = 1 + a_temp*a_temp;
    double B = 2*a_temp*b_temp;
    double C = b_temp*b_temp - (r-d)*(r-d);
    double D = B*B - 4*A*C;
    double x_1 = (-B - sqrt(D))/(2*A);
    double y_1 = a_temp*x_1 + b_temp;
    Point end_point_1 = { x_1, y_1};
    Point end_point_4 = {-x_1, y_1}; 
    Point end_point[6] = {end_point_0, end_point_1, end_point_2, end_point_3, end_point_4, end_point_5};

    // 境界線
    LineBoundary line_boundary_0(0, end_point[0], end_point[1]);
    LineBoundary line_boundary_1(1, end_point[1], end_point[2]);
    LineBoundary line_boundary_2(2, end_point[2], end_point[3]);
    LineBoundary line_boundary_3(3, end_point[3], end_point[4]);
    LineBoundary line_boundary_4(4, end_point[4], end_point[5]);
    LineBoundary line_boundary_array1[N_LINE1] = {line_boundary_0, line_boundary_1, line_boundary_2, line_boundary_3, line_boundary_4};
    LineBoundary line_boundary_array2[N_LINE2] = {line_boundary_0, line_boundary_4};

    // 境界円(or円弧)
    ArcBoundary outer_arc_boundary_1(0, end_point[0], end_point[5], r);
    ArcBoundary inner_arc_boundary_1(1, end_point[1], end_point[4], r-d);
    ArcBoundary outer_arc_boundary_2(2, end_point[5], end_point[0], r);
    ArcBoundary arc_boundary_array1[N_ARC1] = {outer_arc_boundary_1, inner_arc_boundary_1};
    ArcBoundary arc_boundary_array2[N_ARC2] = {outer_arc_boundary_1, outer_arc_boundary_2, inner_arc_boundary_1};

    // 入射光生成
    cout << endl;
    cout << "generating initial rays..." << endl;
    // vector<Ray> vec_ray_initial = generate_initial_rays_1(n_ray_initial, end_point[0], end_point[5]);
    vector<Ray> vec_ray_initial = generate_initial_rays_2(n_ray_initial, r, conv_deg_to_rad(89.9));

    // 切り欠きあり形状で吸収効率計算
    int progress = 0;
    double absorption_model_1 = 0.0;
    cout << endl << "calculation with hollow starts." << endl;
    for(int i = 0; i < n_ray_initial; i++){

        // 進捗表示
        double progress_now = ((double)i / n_ray_initial) * 100;
        if(progress < progress_now){
            while(progress < progress_now) progress++;
            cout << "progress " << progress << "%" << endl;
        }

        // 吸収効率計算
        absorption_model_1 += calc_absorption_by_bfs_ray_tracing(vec_ray_initial[i], medium_array, line_boundary_array1, arc_boundary_array1, MODEL1);
    }
    absorption_model_1 /= n_ray_initial; // 光線ごとの平均

    // 切り欠きなし形状で吸収効率計算
    progress = 0;
    double absorption_model_2 = 0.0;
    cout << endl << "calculation without hollow starts." << endl;
    for(int i = 0; i < n_ray_initial; i++){

        // 進捗表示
        double progress_now = ((double)i / n_ray_initial) * 100;
        if(progress < progress_now){
            while(progress < progress_now) progress++;
            cout << "progress " << progress << "%" << endl;
        }

        // 吸収効率計算
        absorption_model_2 += calc_absorption_by_bfs_ray_tracing(vec_ray_initial[i], medium_array, line_boundary_array2, arc_boundary_array2, MODEL2);
    }
    absorption_model_2 /= n_ray_initial; // 光線ごとの平均

    cout << endl;
    cout << "[result]" << endl;
    cout << "absorption efficiency (model_1): " << absorption_model_1 << endl;
    cout << "absorption efficiency (model_2): " << absorption_model_2 << endl;
    cout << "improvement (model_1 / model_2): " << absorption_model_1 / absorption_model_2 << endl;

    return 0;
}



















vector<Ray> generate_initial_rays_1(const int n_ray_initial, Point end_point_0, Point end_point_5){

    double x_left  = end_point_0.x;
    double x_right = end_point_5.x;
    double y_need  = end_point_0.y;

    vector<Ray> vec_ray_initial;

    for(int i = 0; i < n_ray_initial; i++){

        // 入射光生成
        double x_rand = x_left + (x_right - x_left)*(rand() + 1)/(RAND_MAX + 2); // 開口部の範囲のxからランダムに選択
        double rand_uniform = (rand() + 1.0)/(RAND_MAX + 2);
        double theta_rand = PI/2 - asin(rand_uniform); // 0 ~ 90度の範囲で乱数を生成(分布はsin(一様乱数))
        Point p_start = {x_rand + 0.1*cos(theta_rand), y_need + 0.1*sin(theta_rand)}; // 初期光の照射元座標
        double a_start = tan(theta_rand);
        double b_start = p_start.y - a_start*p_start.x;
        vector<Point> trajectory;
        trajectory.push_back(p_start);
        Ray ray_initial(1.0, a_start, b_start, p_start, LEFT, AIR, trajectory);
        vec_ray_initial.push_back(ray_initial);
    }

    return vec_ray_initial;
}

vector<Ray> generate_initial_rays_2(int n_ray_initial, double r, double theta_in){

    vector<Ray> vec_ray_initial;

    // theta_in が 0 だと傾きがinfになるので、小さい値をセットしておく
    if(theta_in == 0) theta_in = 1e-4;

    for(int i = 0; i < n_ray_initial; i++){

        // 入射位置の計算
        double x_start_temp = -r + 2*r*(i + 1)/(n_ray_initial + 1);
        double y_start_temp = 2*r;
        Point p_start = {x_start_temp, y_start_temp};
        p_start = rotate(p_start, -theta_in); // 右にtheta_in回転

        // 傾き、切片の計算
        double a_start = tan(PI/2 - theta_in);
        double b_start = p_start.y - a_start*p_start.x;

        // 光線生成
        vector<Point> trajectory;
        trajectory.push_back(p_start);
        Ray ray_initial(1.0, a_start, b_start, p_start, LEFT, AIR, trajectory);
        vec_ray_initial.push_back(ray_initial);
    }

    return vec_ray_initial;
}

bool detect_collision_with_line(Collision &collision, Ray &ray, LineBoundary &line, int id_model){

    // 交点の算出
    double a_ray = ray.get_a();
    double b_ray = ray.get_b();

    double a_line = line.get_a();
    double b_line = line.get_b();

    double x_collision = (b_line - b_ray) / (a_ray - a_line);
    double y_collision  = a_ray * x_collision + b_ray;

    // もし交点がlineの定義域にあれば、Collision classを生成してtrueを返す
    if(line.get_left_end().x < x_collision && x_collision < line.get_right_end().x && y_collision > 0){
        Point p_collision = {x_collision, y_collision};
        Collision collision_temp(p_collision, LINE, ray, &line, NULL, id_model);
        // もし隣のMediumの算出がうまくいかなかったら、false
        if(collision_temp.get_id_medium_next() == -1){
            return false;
        }
        // もし衝突点が開始点と同一とみなせる場合はfalse
        if(calc_distance(ray.get_p_start(), p_collision) < EPSILON_DISTANCE){
            return false;
        }
        // もし衝突点がrayの進行方向と逆に位置しているならfalse
        double dx = p_collision.x - ray.get_p_start().x;
        if((ray.get_direction() == LEFT && dx > 0) || (ray.get_direction() == RIGHT && dx < 0)){
            return false;
        }
        // うまくいけばcollisionに格納
        collision = collision_temp;
        return true;
    }
    else{
        return false;
    }
}

bool detect_collision_with_arc(Collision &collision, Ray &ray, ArcBoundary &arc, int id_model){

    // 計算に使うパラメータの準備
    double r = arc.get_r();
    double a_ray = ray.get_a();
    double b_ray = ray.get_b();
    double theta_ray = calc_theta_0_to_2pi_from_slope_and_direction(a_ray, ray.get_direction()); // 光線とx軸のなす角（0 ~ 2PI)

    double A = 1 + a_ray*a_ray;
    double B = 2*a_ray*b_ray;
    double C = b_ray*b_ray - r*r;
    double D = B*B - 4*A*C;

    // もし判別式が負（つまり交点がない）なら、false
    if(D < 0) return false;

    // 交点候補を２つ出す
    double x_cand1 = (-B + sqrt(D)) / (2*A);
    double x_cand2 = (-B - sqrt(D)) / (2*A);
    double y_cand1 = a_ray*x_cand1 + b_ray;
    double y_cand2 = a_ray*x_cand2 + b_ray;
    Point p_cand_array[2] = {
        {x_cand1, y_cand1},
        {x_cand2, y_cand2}
    };
    bool is_valid_p_cand[2] = {true, true};

    // rayの始点と同一とみなされる点は除外
    for(int i = 0; i < 2; i++){
        if(calc_distance(p_cand_array[i], ray.get_p_start()) < EPSILON_DISTANCE){
            is_valid_p_cand[i] = false;
        }
    }
    // 切り欠き部分で衝突の場合は除外
    for(int i = 0; i < 2; i++){
        bool direction_collision = ( (p_cand_array[i].x > 0) ? LEFT : RIGHT); // 衝突点と原点の位置関係で入射方向を設定
        double theta_collision = calc_theta_0_to_2pi_from_slope_and_direction(p_cand_array[i].y/p_cand_array[i].x, direction_collision); // 角度を 0 ~ 2PI で計算
        // 円弧が角度0(x軸方向)をまたぐ場合
        if(arc.get_start_angle() > arc.get_end_angle()){
            if(arc.get_end_angle() <= theta_collision && theta_collision <= arc.get_start_angle()){
                is_valid_p_cand[i] = false;
            }
        }
        // 円弧が角度0(x軸方向)をまたがない場合
        else{
            if(theta_collision <= arc.get_start_angle() || arc.get_end_angle() <= theta_collision){
                is_valid_p_cand[i] = false;
            }
        }
    }
    // rayの進行方向と逆の点は除外
    for(int i = 0; i < 2; i++){
        if(ray.get_direction() == LEFT && ray.get_p_start().x < p_cand_array[i].x){
            is_valid_p_cand[i] = false;
        }
        else if(ray.get_direction() == RIGHT && p_cand_array[i].x < ray.get_p_start().x){
            is_valid_p_cand[i] = false;
        }
    }
    // まだ２点生き残っているなら、遠いほうを除外する。
    if(is_valid_p_cand[0] == true && is_valid_p_cand[1] == true){
        double dist1 = calc_distance(ray.get_p_start(), p_cand_array[0]);
        double dist2 = calc_distance(ray.get_p_start(), p_cand_array[1]);
        if(dist1 < dist2){
            is_valid_p_cand[1] = false;
        }
        else{
            is_valid_p_cand[0] = false;
        }
    }

    // 衝突点を格納
    for(int i = 0; i < 2; i++){
        if(is_valid_p_cand[i] == true){
            Collision collision_temp(p_cand_array[i], ARC, ray, NULL, &arc, id_model);
            if(collision_temp.get_id_medium_next() == -1){
                return false;
            }
            collision = collision_temp;
            return true;
        }
    }

    // 衝突点がないならfalseを返す
    return false;
}

double calc_absorption_by_bfs_ray_tracing(Ray &ray_initial, Medium medium_array[], LineBoundary line_boundary_array[], ArcBoundary arc_boundary_array[], int id_model){

    // 準備
    queue<Ray> queue_ray;
    queue_ray.push(ray_initial);
    double absorption = 0.0;

    // 幅優先探索
    while(!queue_ray.empty()){
        Ray ray_now = queue_ray.front();
        queue_ray.pop();

#ifdef DEBUG
        cout << endl;
        cout << "[ray info]" << endl;
        cout << "from: " << ray_now.get_p_start().x << " " << ray_now.get_p_start().y << endl;
        cout << "direction: " << name_direction[ray_now.get_direction()] << endl;
        cout << "slope: " << ray_now.get_a() << endl;
        cout << "intercept: " << ray_now.get_b() << endl;
        cout << "intensity: " << ray_now.get_intensity() << endl;
        cout << "medium:" << name_medium[ray_now.get_id_medium()] << endl;
#endif

        // もし光強度が閾値より小さいなら、次の探索へcontinue
        if(ray_now.get_intensity() < EPSILON_INTENSITY){
#ifdef DEBUG
            print_trajectory(ray_now);
#endif
            continue;
        }

        vector<Collision> vec_collision;
        Collision collision;
        bool is_collided = false;

        // 直線との衝突判定
#ifdef DEBUG
        cout << endl;
        cout << "[collision detection]" << endl;
#endif
        for(int i_line = 0; i_line < N_LINE[id_model]; i_line++){
            if(detect_collision_with_line(collision, ray_now, line_boundary_array[i_line], id_model) == true){
                vec_collision.push_back(collision);
                is_collided = true;
#ifdef DEBUG
                cout << "on the line " << i_line << " at ";
                cout << collision.get_p_collision().x << " " << collision.get_p_collision().y << endl;
#endif
            }
        }
        // 円との衝突判定
        for(int i_arc = 0; i_arc < N_ARC[id_model]; i_arc++){
            if(detect_collision_with_arc(collision, ray_now, arc_boundary_array[i_arc], id_model) == true){
                vec_collision.push_back(collision);
                is_collided = true;
#ifdef DEBUG
                cout << "on the arc " << i_arc << " at ";
                cout << collision.get_p_collision().x << " " << collision.get_p_collision().y << endl;
#endif
            }
        }

        // もし衝突しなかった場合は、それまでの軌跡を出力して、次の探索へcontinue
        if(is_collided == false){
#ifdef DEBUG
            print_trajectory(ray_now);
#endif
            continue;
        }

        // 衝突点候補から正しい衝突点を抽出
        Collision valid_collision = extract_valid_collision_point(vec_collision, ray_now.get_p_start());
#ifdef DEBUG
        cout << "selected boundary: " << name_boundary[valid_collision.get_boundary_type()] << " ";
        if(valid_collision.get_boundary_type() == LINE){
            cout << valid_collision.get_line()->get_id() << endl;
        }
        else{
            cout << valid_collision.get_arc()->get_id() << endl;
        }
        int id1 = ray_now.get_id_medium();
        int id2 = valid_collision.get_id_medium_next();
        cout << id1 << " " << id2 << endl;
        cout << name_medium[id1] << " to " << name_medium[id2] << endl;
#endif


        // 吸収を計算
        double absorption_temp = calc_single_absorption(ray_now, valid_collision, medium_array);
        if(ray_now.get_id_medium() == CHLOROPLAST){
            absorption += absorption_temp;
        }

        // 入射角、屈折角、反射率を計算
        double theta_in  = calc_theta_in(ray_now, valid_collision);
        double reflection_rate = calc_reflection_rate(ray_now, valid_collision, medium_array, theta_in);
        double theta_out = calc_theta_out(ray_now, valid_collision, medium_array, theta_in);
        // もし全反射なら、反射率を1にする
        if(isnan(theta_out) == true) reflection_rate = 1.0;

        // 反射光・屈折光を計算
        calc_reflected_ray(queue_ray, ray_now, valid_collision, reflection_rate, theta_in);
        if(isnan(theta_out) == false){ // 全反射じゃなければ屈折光を計算
           calc_refracted_ray(queue_ray, ray_now, valid_collision, reflection_rate, theta_out);
        }
    }

    return absorption;
}

Collision extract_valid_collision_point(vector<Collision> vec_collision, Point p_start){

    double dist_min = 1e100;
    Collision valid_collision;

    for(auto collision_cand : vec_collision){
        double dist_temp = calc_distance(collision_cand.get_p_collision(), p_start);
        if(dist_temp < dist_min){
            dist_min = dist_temp;
            valid_collision = collision_cand;
        }
    }

    return valid_collision;
}

double calc_single_absorption(Ray &ray, Collision &collision, Medium medium_array[]){

    double intensity = ray.get_intensity();
    int id_medium = ray.get_id_medium();
    double alpha = medium_array[id_medium].get_alpha();
    double dist = calc_distance(ray.get_p_start(), collision.get_p_collision());

    return intensity * (1 - exp(-alpha * dist));
}

double calc_theta_in(Ray &ray, Collision &collision){

    double a_in = ray.get_a();
    double x_in_temp = ( (ray.get_direction() == LEFT) ? 1 : -1);
    double y_in_temp = a_in*x_in_temp;
    double r_temp = sqrt(x_in_temp*x_in_temp + y_in_temp*y_in_temp);
    double x_in = x_in_temp / r_temp;
    double y_in = y_in_temp / r_temp;
    Point p_in = {x_in, y_in};
    Point p_in_rotated;

    if(collision.get_boundary_type() == LINE){
        double theta_rot = -atan(collision.get_line()->get_a()); // 境界面の傾きを直すのに必要な角度
        p_in_rotated = rotate(p_in, theta_rot);
    }
    else if(collision.get_boundary_type() == ARC){
        double a_vertical = collision.get_p_collision().y / collision.get_p_collision().x; // 境界面の垂線の傾き
        double theta_vertical = atan(a_vertical); // 境界面の垂線とy軸がなす角(-PI/2 ~ PI/2);
        double theta_rot = ( (theta_vertical > 0) ? PI/2 - theta_vertical : -PI/2 - theta_vertical);
        p_in_rotated = rotate(p_in, theta_rot);
    }

    return acos(abs(p_in_rotated.y));
}


double calc_theta_out(Ray &ray, Collision &collision, Medium medium_array[], double theta_in){

    int id_in  = ray.get_id_medium();
    int id_out = collision.get_id_medium_next();
    double n_in  = medium_array[id_in].get_n();
    double n_out = medium_array[id_out].get_n();

    return asin(n_in/n_out * sin(theta_in));
}

void calc_reflected_ray(queue<Ray> &queue_ray, Ray &ray, Collision &collision, double reflection_rate, double theta_in){

    double a_in = ray.get_a(); // 入射光の傾き
    double a_vert; // 境界面に対する垂線の傾き
    // boundary が line の場合
    if(collision.get_boundary_type() == LINE){
        if(collision.get_line()->get_id() == 2){
            a_vert = 1e100; // 本当はvertical lineが鉛直でa_vertは定義できないが、誤差を許容し、とても大きい数で代用。
        }
        else{
            double a_boundary = collision.get_line()->get_a();
            a_vert = -1/a_boundary;
        }
    }
    // boundary が arc の場合
    else{
        a_vert = collision.get_p_collision().y/collision.get_p_collision().x;
    }

    // 入射光と仮想単位円の交点を算出
    double x_in_cand1 = 1/(1 + a_in*a_in);
    double x_in_cand2 = -x_in_cand1;
    double x_in = (ray.get_direction() == LEFT) ? x_in_cand1 : x_in_cand2;
    double y_in = a_in*x_in;
    Point p_in = {x_in, y_in};
    p_in = normalize(p_in);

    // 境界線の垂線と仮想単位円の交点を２つ算出
    double x_vert_1 = 1/(1 + a_vert*a_vert);
    double y_vert_1 = a_vert*x_vert_1;
    double x_vert_2 = -x_vert_1;
    double y_vert_2 = -y_vert_1;
    Point p_vert_1 = {x_vert_1, y_vert_1};
    Point p_vert_2 = {x_vert_2, y_vert_2};
    p_vert_1 = normalize(p_vert_1);
    p_vert_2 = normalize(p_vert_2);
    double dist1 = calc_distance(p_in, p_vert_1);
    double dist2 = calc_distance(p_in, p_vert_2);
    Point p_vert_near = (dist1 < dist2) ? p_vert_1 : p_vert_2; // 入射光側の点をp_vert_nearとする

    // 反射光と仮想単位円の交点の候補を算出
    Point p_reflected_cand1 = rotate(p_vert_near,  theta_in);
    Point p_reflected_cand2 = rotate(p_vert_near, -theta_in);
    dist1 = calc_distance(p_in, p_reflected_cand1);
    dist2 = calc_distance(p_in, p_reflected_cand2);
    Point p_reflected = (dist1 < dist2) ? p_reflected_cand2 : p_reflected_cand1;

    // 反射光の各パラメータを計算
    double intensity_reflected = reflection_rate*ray.get_intensity();
    double a_reflected = p_reflected.y/p_reflected.x;
    double b_reflected = collision.get_p_collision().y - a_reflected*collision.get_p_collision().x;
    bool direction_new = (p_reflected.x < 0) ? LEFT : RIGHT; // もしx_outが負なら、反射光の進行方向はLEFT。正ならRIGHT。
    Point p_start_new = collision.get_p_collision(); // 光線の始点は、進行方向にわずかにずらしておく
    p_start_new.x += 0.01*EPSILON_DISTANCE*p_reflected.x;
    p_start_new.y += 0.01*EPSILON_DISTANCE*p_reflected.y;
    vector<Point> trajectory_new = ray.get_trajectory(); // trajectoryに衝突点を追加
    trajectory_new.push_back(collision.get_p_collision());
    
    // 反射光を生成してqueueに追加
    Ray ray_reflected(intensity_reflected, a_reflected, b_reflected, p_start_new, direction_new, ray.get_id_medium(), trajectory_new);
    queue_ray.push(ray_reflected);

#ifdef DEBUG
    cout << endl;
    cout << "[reflection info]" << endl;
    cout << p_in.x << " " << p_in.y << endl;
    cout << p_vert_1.x << " " << p_vert_1.y << endl;
    cout << p_vert_2.x << " " << p_vert_2.y << endl;
    cout << p_reflected.x << " " << p_reflected.y << endl;
#endif
}

void calc_refracted_ray(queue<Ray> &queue_ray, Ray &ray, Collision &collision, double reflection_rate, double theta_out){

    double a_in = ray.get_a(); // 入射光の傾き
    double a_vert; // 境界面に対する垂線の傾き
    // boundary が line の場合
    if(collision.get_boundary_type() == LINE){
        if(collision.get_line()->get_id() == 2){
            a_vert = 1e100; // 本当はvertical lineが鉛直でa_vertは定義できないが、誤差を許容し、とても大きい数で代用。
        }
        else{
            double a_boundary = collision.get_line()->get_a();
            a_vert = -1/a_boundary;
        }
    }
    // boundary が arc の場合
    else{
        a_vert = collision.get_p_collision().y/collision.get_p_collision().x;
    }

    // 入射光と仮想単位円の交点 x_in, y_in を算出
    double x_in_cand1 = 1/(1 + a_in*a_in);
    double x_in_cand2 = -x_in_cand1;
    double x_in = (ray.get_direction() == LEFT) ? x_in_cand1 : x_in_cand2;
    double y_in = a_in*x_in;
    Point p_in = {x_in, y_in};
    p_in = normalize(p_in);
    
    // 界面で仮に直進した場合の直進光と仮想単位円の交点を算出
    Point p_straight_out = {-p_in.x, -p_in.y};

    // 境界面の垂線と仮想単位円の交点のうち、x_in, y_in から遠い方を算出
    double x_vert_cand1 = 1/(1 + a_vert*a_vert);
    double x_vert_cand2 = -x_vert_cand1;
    double y_vert_cand1 = a_vert*x_vert_cand1;
    double y_vert_cand2 = -y_vert_cand1;
    Point p_vert_1 = {x_vert_cand1, y_vert_cand1};
    Point p_vert_2 = {x_vert_cand2, y_vert_cand2};
    p_vert_1 = normalize(p_vert_1);
    p_vert_2 = normalize(p_vert_2);
    double dist1 = calc_distance(p_in, p_vert_1);
    double dist2 = calc_distance(p_in, p_vert_2);
    Point p_vert_far = (dist1 < dist2) ? p_vert_2 : p_vert_1; 

    // 屈折光と仮想単位円の交点を算出
    Point p_refracted_cand1 = rotate(p_vert_far,  theta_out); // 交点の候補を算出
    Point p_refracted_cand2 = rotate(p_vert_far, -theta_out);
    dist1 = calc_distance(p_refracted_cand1, p_straight_out); // p_straight_outと近いほうが正しい交点
    dist2 = calc_distance(p_refracted_cand2, p_straight_out);
    Point p_refracted = (dist1 < dist2) ? p_refracted_cand1 : p_refracted_cand2;
    
    // 屈折光の各パラメータを算出
    double intensity_refracted = (1 - reflection_rate)*ray.get_intensity(); // 屈折光強度は透過率×入射強度
    double a_refracted = p_refracted.y/p_refracted.x;
    double b_refracted = collision.get_p_collision().y - a_refracted*collision.get_p_collision().x; 
    Point p_start_new = collision.get_p_collision(); // 屈折光の出射位置は進行方向にわずかにずらして設定する
    p_start_new.x += 0.01*EPSILON_DISTANCE*p_refracted.x;
    p_start_new.y += 0.01*EPSILON_DISTANCE*p_refracted.y;
    bool direction_new = (p_refracted.x < 0) ? LEFT : RIGHT; 
    vector<Point> trajectory_new = ray.get_trajectory(); // trajectoryに衝突点を追加
    trajectory_new.push_back(collision.get_p_collision());

    // 屈折光を生成してqueueに追加
    Ray ray_refracted(intensity_refracted, a_refracted, b_refracted, p_start_new, direction_new, collision.get_id_medium_next(), trajectory_new);
    queue_ray.push(ray_refracted);
    
#ifdef DEBUG
    cout << endl;
    cout << "[refraction info]" << endl;
    cout << p_in.x << " " << p_in.y << endl;
    cout << p_vert_1.x << " " << p_vert_1.y << endl;
    cout << p_vert_2.x << " " << p_vert_2.y << endl;
    cout << p_refracted.x << " " << p_refracted.y << endl;
#endif

    return;
}


void print_trajectory(Ray &ray){

    vector<Point> trajectory = ray.get_trajectory();
    
    cout << endl;
    cout << "[trajectory]" << endl;
    for(Point p : trajectory){
        cout << p.x << " " << p.y << endl;
    }

    return;
}