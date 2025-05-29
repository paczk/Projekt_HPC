#include <vector>
#include "bodies.hpp"

class Constraint
{
    public:
        Constraint(int id, Body *body1, Body *body2, const std::vector<double> &constraint_placement)
            : id(id), body1(body1), body2(body2), constraint_placement(constraint_placement) {}

        virtual double ConstrainingFunction()
        {
            return 0.0;
        }
    
    protected:
        int id;
        Body *body1;
        Body *body2;
        std::vector<double> constraint_placement;
        
};

