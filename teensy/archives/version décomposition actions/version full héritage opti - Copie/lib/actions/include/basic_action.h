#include <step_action.h>

// Generic Basic Action class
class Basic_Action : public Action
{
public:
    Precision_Params *precision_params;
    byte *speed;
    Direction *direction;

    Step_Action *step_action;

    Basic_Action() = default;
    virtual void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) = 0;
    void handle(Point current_point, Ticks current_ticks, Rolling_Basis_Ptrs *rolling_basis_ptrs) override;
};

// Move Straight
class Move_Straight : public Basic_Action
{
public:
    void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) override;
    float target_x, target_y;
    Move_Straight(float target_x, float target_y, Direction* direction, byte* speed,Precision_Params *precision_params);
};

// Get Orientation in front of a point (turn on itself)
class Get_Orientation : public Basic_Action
{
public:
    void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) override;
    float target_x, target_y;
    Get_Orientation(float target_x, float target_y, Direction *direction, byte *speed, Precision_Params *precision_params);
};

// Do a Rotation (turn on itself)
class Move_Rotation : public Basic_Action
{
public:
    void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) override;
    float target_theta;
    Move_Rotation(float target_theta, Direction *direction, byte *speed, Precision_Params *precision_params);
};