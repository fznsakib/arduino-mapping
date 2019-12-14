#ifndef _Kinematics
#define _Kinematics_h

//You may want to use some/all of these variables
//const float WHEEL_DIAMETER    = ??;
//const float WHEEL_RADIUS      = ??;
const float WHEEL_SEPERATION  = 922; // 926.66
//const float GEAR_RATIO        = ??;
//const float COUNTS_PER_SHAFT_REVOLUTION = ??;
//const float COUNTS_PER_WHEEL_REVOLUTION =  ??;
const float COUNTS_PER_MM               = 6.6666666;


// Build up your Kinematics class.
class Kinematics {
  public:
    
    Kinematics();   // Constructor, required.

    void update(long left_encoder_count, long right_encoder_count);  // should calculate an update to pose.
    float get_x();
    float get_y();
    float get_d();
    float get_theta();
    void set_theta(float t);
    
  private:
    
    float x;
    float y;
    float d;
    float left_d;
    float right_d;

    float theta;

    long old_left_encoder_count;
    long old_right_encoder_count;
    
};


// Required constructor.  Initialise variables.
Kinematics::Kinematics() {
  x = 0;
	y = 0;
  d = 0;
  left_d  = 0;
  right_d = 0;
  theta = 0;

  old_left_encoder_count  = 0;
  old_right_encoder_count = 0;
}

void Kinematics::update(long left_encoder_count, long right_encoder_count) {
  left_d  = (float) (left_encoder_count  - old_left_encoder_count );
  right_d = (float) (right_encoder_count - old_right_encoder_count);
  d = (float) (left_d + right_d) / 2.0f;
  theta = theta + (float) (left_d - right_d) / (float) WHEEL_SEPERATION;
  x = x + d * cos(theta);
  y = y + d * sin(theta);
  old_left_encoder_count  = left_encoder_count;
  old_right_encoder_count = right_encoder_count;
}

float Kinematics::get_x() {
  return x;
}

float Kinematics::get_y() {
  return y;
}

float Kinematics::get_d() {
  return d;
}

float Kinematics::get_theta() {
  return theta;
}

void Kinematics::set_theta(float t) {
  theta = t;
}

#endif