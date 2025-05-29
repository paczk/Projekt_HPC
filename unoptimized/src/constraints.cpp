#include <vector>
#include "bodies.hpp"

class Constraint
{
    public:
        Constraint(int id, int body1_id, int body2_id)
            : id(id), body1_id(body1_id), body2_id(body2_id) {}

        virtual double ConstrainingFunction()
        {
            return 0.0;
        }
    
    protected:
        int id;
        int body1_id;
        int body2_id;
        
};


class DistanceConstraint : public Constraint
{
    public:
        DistanceConstraint(int id, int body1_id, int body2_id, const std::vector<double>& body1_point, 
                           const std::vector<double>& body2_point, 
                           std::vector<double> (*distance)(double t))
            : Constraint(id, body1_id, body2_id), body1_point(body1_point), body2_point(body2_point), distance(distance) {}

    private:
        std::vector<double> body1_point;
        std::vector<double> body2_point;
        std::vector<double> (*distance)(double t);
};
