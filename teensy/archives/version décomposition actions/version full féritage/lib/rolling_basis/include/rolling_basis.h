#include <Arduino.h>
#include <action.h>

class Rolling_Basis {

private:
    float prevT;

public :
    inline float radius() { return this->center_distance / 2.0; };
    inline float wheel_perimeter() { return this->wheel_diameter * PI; };
    inline float wheel_unit_tick_cm() { return this->wheel_perimeter() / this->encoder_resolution; };

    // Rolling basis's motors
    Motor *right_motor;
    Motor *left_motor;

    // Odometrie
    float X = 0.0;
    float Y = 0.0;
    float THETA = 0.0;
    bool IS_RUNNING = false;

    // Inactive check
    long last_running_check = 0;
    long last_position_update = 0;
    long running_check_right = 0;
    long running_check_left = 0;
    long inactive_delay = 0;

    // Ticks
    long left_ticks = 0;
    long right_ticks = 0;

    // Trajectory precision
    unsigned short corrector_error_auth;

    // Rolling basis params
    unsigned short encoder_resolution;
    float center_distance;
    float wheel_diameter;

    byte max_pwm;

    // Constructor
    Rolling_Basis(unsigned short encoder_resolution, float center_distance, float wheel_diameter);

    // Inits function
    void init_right_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor, byte threshold_pwm_value);
    void init_left_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor, byte threshold_pwm_value);
    void init_motors();
    void init_rolling_basis(float x, float y, float theta, long inactive_delay, unsigned short corrector_error_auth, byte max_pwm);

    void odometrie_handle();

    void is_running_update();

    float trajectory_theta_calculator(float target_x, float target_y);
    float delta_theta_calculator(float target_theta);
    float trajectory_distance_calculator(float target_x, float target_y);

    // Angle to ticks
    void theta_to_ticks(float theta_to_rotate, long *ticks, short *r_sign, short *l_sign);
    // Distance to ticks
    void distance_to_ticks(float distance, long *ticks, short *r_sign, short *l_sign);

    // Keep current position
    void keep_position(long current_right_ticks, long current_left_ticks);

    // Shut down motors
    void shutdown_motor();
};