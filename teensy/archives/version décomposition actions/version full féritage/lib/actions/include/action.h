#include <Arduino.h>
#include <motors_driver.h>
#include <structures.h>

class Action{
    private:
        virtual void compute() = 0;

    public:
        bool is_computed = false;
        Action_state state = not_started;

        Action() = default;
        bool is_finished();
        virtual void handle() = 0; // A définir en globale ailleurs ?
};

/*
class Complex_Action : public Action
{
        // Les classes de plus haut niveau stocke les données de précisions de traj
        // Les classes bas niveau y accède via pointeur

        // Les données de la base roulante sont toujours en stocké en pointeur car elles sont constantes pas
        // besion de les changer donc 1 varaible pour tout le programme

    private:
        void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params);

    public:
        // Rolling_Basis_Params rolling_basis_params; on le donne que au moment du compute
        Precision_Params precision_params;
        byte speed;
        Direction dir;

        Basic_Movement **basic_movements;
        short nb_basic_movements;
        short movement_index = 0;

        Complex_Action() = default;
        void alloc_memory(short nb_actions);
        void handle(Point current_point, Ticks current_ticks, Motor *right_motor, Motor *left_motor, Rolling_Basis_Params *rolling_basis_params);
};

class Basic_Movement : public Action
{
    private:
        void compute(Point current_point, Ticks current_ticks, Rolling_Basis_Params *rolling_basis_params);

    public:
        Precision_Params *precision_params;
        byte *speed;
        direction *dir;

        Basic_Movement() = default;
        void handle(Point current_point, Ticks current_ticks, Motor *right_motor, Motor *left_motor, Rolling_Basis_Params *rolling_basis_params);
};


Action
/-> Step Movement
/--> Basic Movement
/---> Complex Movement


*/