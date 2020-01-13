
// radian を degree に変換
double conv_rad_to_deg(double rad);

// degree を radian に変換
double conv_deg_to_rad(double deg);

// 2点間の距離を算出
double calc_distance(Point p1, Point p2);

// 直線の方向も加味して、直線とx軸がなす角を 0 ~ 2PI で返す
double calc_theta_0_to_2pi_from_slope_and_direction(double a, bool direction);

// 座標をtheta(rad)回転させる
Point rotate(Point p, double theta);

// 座標を原点からの距離１に規格化する
Point normalize(Point p);

// 内積を計算する
double inner_product(Point p1, Point p2);

// 近似フレネル式で反射率を計算
double calc_reflection_rate(Ray &ray, Collision &collision, Medium medium_array[], double theta_in);