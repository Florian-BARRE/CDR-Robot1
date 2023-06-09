#include <action.h>

// Generic Step Action class
class Step_Action : public Action
{
public:
    void check_end_of_action(Ticks current_ticks);
    void update_action_cursor(Ticks current_ticks);

    Precision_Params *precision_params;
    byte *speed;

    long total_ticks;
    long ticks_cursor = 0;

    long right_ref;
    long left_ref;
    short right_sign = 1;
    short left_sign = 1;

    unsigned int end_movement_cpt = 0;

    Step_Action() = default;
    virtual void compute(Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) = 0;
    void handle(Point current_point, Ticks current_ticks, Rolling_Basis_Ptrs *rolling_basis_ptrs) override;
};

// Move Forward or Backward class
class Step_Forward_Backward : public Step_Action
{
public:
    void compute(Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) override;
    float distance;
    Direction direction;
    Step_Forward_Backward(float distance, byte *speed, Precision_Params *precision_params);
};

// Rotation class
class Step_Rotation : public Step_Action
{
public:
    void compute(Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) override;
    float theta;
    Sens rotation_sens;
    Step_Rotation(float theta, byte *speed, Precision_Params *precision_params);
};
