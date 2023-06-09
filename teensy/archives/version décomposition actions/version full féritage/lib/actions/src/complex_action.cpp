#include <complex_action.h>
/*
void Complex_Action::alloc_memory(short nb_actions)
{
    this->nb_basic_movements = nb_actions;
    this->basic_movements = new Basic_Movement *[nb_actions];
}

void Complex_Action::handle(Point current_point, long current_right_ticks, long current_left_ticks, Motor *right_motor, Motor *left_motor, Rolling_Basis_Params *rolling_basis_params)
{
    // Compute the complex action
    if (!this->is_computed)
        this->compute(
            current_point,
            current_right_ticks, current_left_ticks,
            rolling_basis_params
        );

    // Handle the complex action
    if (0 <= this->movement_index && this->movement_index < this->nb_basic_movements)
    {
        this->state = in_progress;

        // Compute the next under action
        if (!this->basic_movements[this->movement_index]->is_computed)
        {
            this->basic_movements[this->movement_index]->compute(
                current_point,
                current_right_ticks, current_left_ticks,
                rolling_basis_params
            );
        }
        // Handle action
        else if (
            this->basic_movements[this->movement_index]->is_computed &&
            (this->basic_movements[this->movement_index]->state == not_started || this->basic_movements[this->movement_index]->state == in_progress))
            this->basic_movements[this->movement_index]->handle(current_right_ticks, current_left_ticks, right_motor, left_motor);

        // End of the action
        else if (this->basic_movements[this->movement_index]->is_finished())
            this->movement_index++;
    }
    else
        this->state = finished;

}

Go_To::Go_To(Point target_point, direction dir, byte speed, unsigned int end_movement_presicion, unsigned int error_precision, unsigned int trajectory_precision)
{
    this->target_point = target_point;
    this->dir = dir;
    this->speed = speed;
    this->precision_params.end_movement_presicion = end_movement_presicion;
    this->precision_params.error_precision = error_precision;
    this->precision_params.trajectory_precision = trajectory_precision;
}

void Go_To::compute(Point current_point, long current_right_ticks, long current_left_ticks, Rolling_Basis_Params *rolling_basis_params)
{
    //  Go to is simple move
    //  1°) rotate to be in front of the target point
    //  2°) go forward to the target point
    this->alloc_memory(2);
    this->basic_movements[0] = new Get_Orientation(
        this->target_x,
        this->target_y,
        this->dir,
        this->speed,
        this->params.end_movement_presicion,
        this->params.error_precision,
        this->params.trajectory_precision);
    this->basic_actions[1] = new Move_Straight(
        this->target_x,
        this->target_y,
        this->dir,
        this->speed,
        this->params.end_movement_presicion,
        this->params.error_precision,
        this->params.trajectory_precision);
    this->is_computed = true;

    this->basic_movements[0]
}

*/