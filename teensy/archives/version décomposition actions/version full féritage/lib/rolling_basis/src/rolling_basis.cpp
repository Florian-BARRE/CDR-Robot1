#include <rolling_basis.h>
#include <Arduino.h>
#include <util/atomic.h>

void Rolling_Basis::init_rolling_basis(float x, float y, float theta, long inactive_delay, unsigned short corrector_error_auth, byte max_pwm)
{
    this->X = x;
    this->Y = y;
    this->THETA = theta;
    this->inactive_delay = inactive_delay;
    this->corrector_error_auth = corrector_error_auth;
    this->max_pwm = max_pwm;
}

void Rolling_Basis::init_motors(){
    this->right_motor->init();
    this->left_motor->init();
}

Rolling_Basis::Rolling_Basis(unsigned short encoder_resolution, float center_distance, float wheel_diameter)
{
    this->encoder_resolution = encoder_resolution;
    this->center_distance = center_distance;
    this->wheel_diameter = wheel_diameter;
}

void Rolling_Basis::init_right_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor = 1.0, byte threshold_pwm_value=0)
{
    this->right_motor = new Motor(enca, encb, pwm, in2, in1, kp, kd, ki, correction_factor, threshold_pwm_value);
}

void Rolling_Basis::init_left_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor = 1.0, byte threshold_pwm_value = 0)
{
    this->left_motor = new Motor(enca, encb, pwm, in2, in1, kp, kd, ki, correction_factor, threshold_pwm_value);
}

void Rolling_Basis::odometrie_handle(){
    /* Determine the position of the robot */
    long delta_left  = this->left_motor->ticks - this->left_ticks;
    this->left_ticks = this->left_ticks + delta_left;

    long delta_right  = this->right_motor->ticks - this->right_ticks;
    this->right_ticks = this->right_ticks + delta_right;
    
    float left_move  = delta_left * this->wheel_unit_tick_cm();
    float right_move = delta_right * this->wheel_unit_tick_cm();

    float movement_difference = right_move - left_move;
    float movement_sum = (right_move + left_move) / 2;

    THETA = THETA + (movement_difference / this->center_distance);
    this->X = this->X + (cos(this->THETA) * movement_sum);
    this->Y = this->Y + (sin(this->THETA) * movement_sum);
}

void Rolling_Basis::is_running_update(){
    if ((millis() - this->last_running_check) > 10){
        this->last_running_check = millis();
        long delta_right = abs(this->running_check_right - this->right_motor->ticks);
        long delta_left = abs(this->running_check_left - this->left_motor->ticks);

        if((delta_left > 3) || (delta_right > 3))
            this->last_position_update = millis();
        
        this->running_check_right = this->right_motor->ticks;
        this->running_check_left  = this->left_motor->ticks;

        this->IS_RUNNING = ((millis() - this->last_position_update) < this->inactive_delay);
    }
}

float Rolling_Basis::trajectory_theta_calculator(float target_x, float target_y)
{
    /*
    float x_error = target_x - this->X;
    float y_error = target_y - this->Y;
    float cmd_theta;

    // Extremum rotation
    if (x_error == 0 && y_error > 0)
        cmd_theta = PI / 2.0;
    else if (x_error == 0 && y_error < 0)
        cmd_theta = -(PI / 2.0);
    
    // If don't need to rotate
    if (y_error == 0)
        cmd_theta = this->THETA;
    else
        cmd_theta = atan2(y_error, x_error) - this->THETA;

    //*cmd_theta = (*cmd_theta > PI || *cmd_theta < -PI) ? fmod(*cmd_theta, PI) : *cmd_theta; // Noah a eu la merveilleuse idée d'employé un ?
    //cmd_theta = (cmd_theta > PI || cmd_theta < -PI) ? -fmod(cmd_theta, PI) : cmd_theta;
    return cmd_theta;
    */

    float delta_x = fabs(fabs(target_x) - fabs(this->X));
    float delta_y = fabs(fabs(target_y) - fabs(this->Y));
    float cmd_theta;

    // Extremum rotation
    // delta y null
    if (delta_y == 0){
        if (target_x > this->X)
            cmd_theta = 0.0;
        else if (target_x < this->X)
            cmd_theta = PI;
    }
    // delta x null
    else if (delta_x == 0)
    {
        if (target_y > this->Y)
            cmd_theta = PI / 2.0;
        else if (target_x < this->X)
            cmd_theta = -(PI / 2.0);
    }

    // Any cases
    else{
        float adelta_x = target_x - this->X;
        float adelta_y = target_y - this->Y;

        float target_angle = atan2(adelta_y, adelta_x);
        cmd_theta = fmod((target_angle - this->THETA + PI), (2 * PI)) - PI;
    }
    return cmd_theta;
}

float Rolling_Basis::trajectory_distance_calculator(float target_x, float target_y)
{
    /*
    float x_error = target_x - this->X;
    float y_error = target_y - this->Y;
    float cmd_dist;

    cmd_dist = sqrt((x_error * x_error) + (y_error * y_error));
    return cmd_dist;
    */
    // No negative dist
    float delta_x = fabs(fabs(target_x) - fabs(this->X));
    float delta_y = fabs(fabs(target_y) - fabs(this->Y));
    float cmd_dist;

    cmd_dist = sqrt((delta_x * delta_x) + (delta_y * delta_y));

    return cmd_dist;
}

float Rolling_Basis::delta_theta_calculator(float target_theta){
    float current_theta = fmod(this->THETA, 2.0 * PI);
    float delta_theta = abs(abs(target_theta) - abs(this->THETA));

    short sign = 1;
    // W peut etre > au lieu de '<'
    if (target_theta < current_theta)
        sign = -1;
    
    return sign * delta_theta;
}

void Rolling_Basis::theta_to_ticks(float theta_to_rotate, long *ticks, short *r_sign, short *l_sign){
    float distance = (theta_to_rotate * this->radius());
    *ticks = abs((this->encoder_resolution / this->wheel_perimeter()) * distance);

    if (theta_to_rotate > 0.0)
    {
        *r_sign = 1;
        *l_sign = -1;
    }
    else
    {
        *r_sign = -1;
        *l_sign = 1;
    }
}

void Rolling_Basis::distance_to_ticks(float distance, long *ticks, short *r_sign, short *l_sign){
    *ticks = (this->encoder_resolution / this->wheel_perimeter()) * distance;

    if (*ticks > 0)
    {
        *r_sign = 1;
        *l_sign = 1;
    }
    else
    {
        *r_sign = -1;
        *l_sign = -1;
    }
}

void Rolling_Basis::keep_position(long current_right_ticks, long current_left_ticks) {
    this->right_motor->handle(current_right_ticks, this->max_pwm);
    this->left_motor->handle(current_left_ticks, this->max_pwm);
}

void Rolling_Basis::shutdown_motor()
{
    this->right_motor->set_motor(1, 0);
    this->left_motor->set_motor(1, 0);
}