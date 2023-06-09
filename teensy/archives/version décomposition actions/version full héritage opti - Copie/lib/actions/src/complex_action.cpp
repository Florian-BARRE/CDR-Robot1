#include <complex_action.h>

void Complex_Action::alloc_memory(short nb_actions)
{
    this->nb_basic_movements = nb_actions;
    this->basic_movements = new Basic_Action *[nb_actions];
}

void Complex_Action::handle(Point current_point, Ticks current_ticks, Rolling_Basis_Ptrs *rolling_basis_ptrs)
{
    // Compute the complex action
    if (!this->is_computed)
        this->compute(
            current_point,
            current_ticks,
            rolling_basis_ptrs->rolling_basis_params
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
                current_ticks,
                rolling_basis_ptrs->rolling_basis_params
            );
        }
        // Handle action
        else if (
            this->basic_movements[this->movement_index]->is_computed &&
            (this->basic_movements[this->movement_index]->state == not_started || this->basic_movements[this->movement_index]->state == in_progress)){
            this->basic_movements[this->movement_index]->handle(
                current_point,
                current_ticks,
                rolling_basis_ptrs
            );
          }

            // End of the action
            else if (this->basic_movements[this->movement_index]->is_finished()) this->movement_index++;
    }
    else
        this->state = finished;
}

Go_To::Go_To(Point target_point, Direction direction, byte speed, Precision_Params precision_params)
{
    this->target_point = target_point;
    this->direction = direction;
    this->speed = speed;
    this->precision_params = precision_params;
}

void Go_To::compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params)
{
    //  Go to is simple move
    //  1°) rotate to be in front of the target point
    //  2°) go forward to the target point
    this->alloc_memory(2);
    this->basic_movements[0] = new Get_Orientation(
        this->target_point.x, this->target_point.y,
        &this->direction,
        &this->speed,
        &this->precision_params
    );

    this->basic_movements[1] = new Move_Straight(
        this->target_point.x, this->target_point.y,
        &this->direction,
        &this->speed,
        &this->precision_params
    );

    this->is_computed = true;
}

