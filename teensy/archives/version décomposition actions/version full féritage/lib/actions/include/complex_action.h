/*#include "action.h"

class Complex_Action : public Action
{
    // Les classes de plus haut niveau stocke les données de précisions de traj
    // Les classes bas niveau y accède via pointeur

    // Les données de la base roulante sont toujours en stocké en pointeur car elles sont constantes pas
    // besion de les changer donc 1 varaible pour tout le programme

public:
    // Rolling_Basis_Params rolling_basis_params; on le donne que au moment du compute
    Precision_Params precision_params;
    byte speed;
    Direction dir;

    Basic_Movement **basic_movements;
    short nb_basic_movements;
    short movement_index = 0;

    void alloc_memory(short nb_actions);
    void handle(Point current_point, long current_right_ticks, long current_left_ticks, Motor *right_motor, Motor *left_motor, Rolling_Basis_Params *rolling_basis_params);
    void compute(Point current_point, long current_right_ticks, long current_left_ticks, Rolling_Basis_Params *rolling_basis_params);
};

class Go_To : public Complex_Action
{
public:
    Point target_point;
    Go_To(Point target_point, Direction dir, byte speed, unsigned int end_movement_presicion, unsigned int error_precision, unsigned int trajectory_precision);
    void compute(Point current_point, long current_right_ticks, long current_left_ticks, Rolling_Basis_Params *rolling_basis_params);
};

*/