#include <step_action.h>

// Generic Step Action class
void Step_Action::check_end_of_action(Ticks current_ticks)
{
    long right_error = abs(this->right_ref + this->right_sign * this->total_ticks) - abs(current_ticks.right);
    long left_error  = abs(this->left_ref  + this->left_sign * this->total_ticks)  - abs(current_ticks.left);

    // Check if the robot is near the target position
    if ((long)this->precision_params->error_precision >= right_error && (long)this->precision_params->error_precision >= left_error)
        (this->end_movement_cpt)++;

    // Checks if the robot has been in the right position long enough end the action
    if (this->end_movement_cpt >= this->precision_params->end_movement_presicion)
        this->state = finished;
    else
        this->state = in_progress;
}

void Step_Action::update_action_cursor(Ticks current_ticks)
{
    long right_error = this->ticks_cursor - abs(current_ticks.right - this->right_ref);
    long left_error = this->ticks_cursor - abs(current_ticks.left - this->left_ref);

    // Do we need to correct the trajectory ?
    if (abs(right_error) <= this->precision_params->trajectory_precision && abs(left_error) <= this->precision_params->trajectory_precision)
    {
        if (abs(this->ticks_cursor - this->total_ticks) < this->precision_params->error_precision)
            this->ticks_cursor = this->total_ticks;
        else
            this->ticks_cursor += this->precision_params->trajectory_precision;

        if (this->ticks_cursor > this->total_ticks)
            this->ticks_cursor = this->total_ticks;
    }
}

void Step_Action::handle(Point current_point, Ticks current_ticks, Rolling_Basis_Ptrs *rolling_basis_ptrs)
{
    // Check if the action is computed
    if(!this->is_computed)
        this->compute(current_ticks, rolling_basis_ptrs->rolling_basis_params);

    // Handle the action 
    else if (this->state == not_started || this->state == in_progress){
      
        // Set motor order
        unsigned int addr = &(rolling_basis_ptrs->right_motor);
     
        rolling_basis_ptrs->right_motor->handle((this->right_ref + this->right_sign * this->ticks_cursor), *this->speed);
  
        rolling_basis_ptrs->left_motor->handle((this->left_ref + this->left_sign * this->ticks_cursor), *this->speed);

        // Update action progression cursor
        this->update_action_cursor(current_ticks);
  
    }

    // Check if the action is finished
    this->check_end_of_action(current_ticks);
}

// Move Forward or Backward
Step_Forward_Backward::Step_Forward_Backward(float distance, byte *speed, Precision_Params *precision_params)
{
    this->distance = fabs(distance);
    this->speed = speed;
    this->direction = (distance < 0) ? backward : forward;
    this->precision_params = precision_params;
}

void Step_Forward_Backward::compute(Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params)
{
    this->total_ticks = (rolling_basis_params->encoder_resolution / rolling_basis_params->wheel_perimeter) * this->distance;
    if (this->direction == forward)
    {
        this->right_sign = 1;
        this->left_sign = 1;
    }
    else if (this->direction == backward)
    {
        this->right_sign = -1;
        this->left_sign = -1;
    }

    this->right_ref = current_ticks.right;
    this->left_ref = current_ticks.left;
    this->is_computed = true;
}

// Move Rotation
Step_Rotation::Step_Rotation(float theta, byte *speed, Precision_Params *precision_params)
{
    this->theta = fabs(theta);
    this->speed = speed;
    this->rotation_sens = (theta < 0) ? clockwise : counterclockwise;
    this->precision_params = precision_params;
}

void Step_Rotation::compute(Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params)
{
    float distance = (fabs(this->theta) * rolling_basis_params->radius);
    this->total_ticks = (rolling_basis_params->encoder_resolution / rolling_basis_params->wheel_perimeter) * distance;

    if (this->rotation_sens == counterclockwise)
    {
        this->right_sign = 1;
        this->left_sign = -1;
    }
    else if (this->rotation_sens == clockwise)
    {
        this->right_sign = -1;
        this->left_sign = 1;
    }

    this->right_ref = current_ticks.right;
    this->left_ref = current_ticks.left;
    this->is_computed = true;
}
