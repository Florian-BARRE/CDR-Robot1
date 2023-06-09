#include <step_action.h>

// Generic Basic Action class
class Basic_Action : public Action
{
private:
    virtual void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) = 0;
    Step_Action *step_action;

public:
    Precision_Params *precision_params;
    byte *speed;
    Direction *direction;

    Basic_Action() = default;
    void handle(Point current_point, Ticks current_ticks, Rolling_Basis_Ptrs *rolling_basis_ptrs);
};

// Move Straight
class Move_Straight : public Basic_Action
{
private:
    void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) override;
    float target_x, target_y;

public:
    Move_Straight(float target_x, float target_y, Direction* direction, byte* speed,Precision_Params *precision_params);
};

// Get Orientation in front of a point (turn on itself)
class Get_Orientation : public Basic_Action
{
private: 
    void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) override;
    float target_x, target_y;

public:
    Get_Orientation(float target_x, float target_y, Direction *direction, byte *speed, Precision_Params *precision_params);
};

// Do a Rotation (turn on itself)
class Move_Rotation : public Basic_Action
{
private:
    void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) override;
    float target_theta;

public:
    Move_Rotation(float target_theta, Direction *direction, byte *speed, Precision_Params *precision_params);
};