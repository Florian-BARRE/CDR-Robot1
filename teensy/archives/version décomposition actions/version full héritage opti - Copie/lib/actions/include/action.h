#include <Arduino.h>
#include <motors_driver.h>
#include <structures.h>

class Action{
    public:
        bool is_computed = false;
        Action_state state = not_started;

        Action() = default;
        bool is_finished();
        //virtual void compute() = 0;
        virtual void handle(Point current_point, Ticks current_ticks, Rolling_Basis_Ptrs *rolling_basis_ptrs) = 0; 
};