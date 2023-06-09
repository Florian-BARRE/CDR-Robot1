#include <basic_action.h>

class Complex_Action : public Action
{
    // Les classes de plus haut niveau stocke les données de précisions de traj
    // Les classes bas niveau y accède via pointeur

    // Les données de la base roulante sont toujours en stocké en pointeur car elles sont constantes pas
    // besoin de les changer donc 1 varaible pour tout le programme

public:
    // Rolling_Basis_Params rolling_basis_params; on le donne que au moment du compute
    Precision_Params precision_params;
    byte speed;
    Direction direction;

    Basic_Action **basic_movements;
    short nb_basic_movements;
    short movement_index = 0;

    void alloc_memory(short nb_actions);

    Complex_Action() = default;
    virtual void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params) = 0;
    void handle(Point current_point, Ticks current_ticks, Rolling_Basis_Ptrs *rolling_basis_ptrs) override;
};

class Go_To : public Complex_Action
{
public:
    Point target_point;
    Go_To(Point target_point, Direction direction, byte speed, Precision_Params precision_params);
    void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params);
};

class Curve_Go_To : public Complex_Action
{
public:
    Point target_point;
    Point center_point;
    unsigned short interval;
    Curve_Go_To(Point target_point, Point center_point, unsigned short interval, Direction direction, byte speed, Precision_Params precision_params);
    void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params);
};



