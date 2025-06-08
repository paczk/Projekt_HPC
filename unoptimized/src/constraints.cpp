#include <eigen3/Eigen/Dense>
#include <vector>
#include "bodies.hpp"
#include "quaternion_operations.hpp"
#include "constraints.hpp"

const Eigen::VectorXd ground {0, 0, 0, 1, 0, 0, 0};

Eigen::Map<const Eigen::VectorXd> get_body_position(const Eigen::VectorXd& q, int id, const std::vector<int>& body_ids)
{
    if(id == 0)
    {
        return Eigen::Map<const Eigen::VectorXd>(ground.data(), 3);
    }
    else
    {
        auto it = std::find(body_ids.begin(), body_ids.end(), id);
        int i = std::distance(body_ids.begin(), it);
        return Eigen::Map<const Eigen::VectorXd>(q.data() + i * 7, 3);
    }
}

Eigen::Map<const Eigen::VectorXd> get_body_rotation(const Eigen::VectorXd& q, int id, const std::vector<int>& body_ids)
{
    if(id == 0)
    {
        return Eigen::Map<const Eigen::VectorXd>(ground.data(), 3);
    }
    else
    {
        auto it = std::find(body_ids.begin(), body_ids.end(), id);
        int i = std::distance(body_ids.begin(), it);
        return Eigen::Map<const Eigen::VectorXd>(q.data() + i * 7 + 3, 4);
    }
}

class Constraint
{
    public:
        Constraint(int id, int body1_id, int body2_id)
            : id(id), body1_id(body1_id), body2_id(body2_id) {}

        virtual Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids);

        virtual double equations_number();

    protected:
        int id;
        int body1_id;
        int body2_id;    
};

class DistanceConstraint : public Constraint
{
    public:
        DistanceConstraint(int id, int body1_id, int body2_id, const Eigen::Vector3d& body1_point, 
                           const Eigen::Vector3d& body2_point, 
                           const Eigen::Vector3d& (*distance)(double t))
            : Constraint(id, body1_id, body2_id), body1_point(body1_point), body2_point(body2_point), distance(distance) {}
        
        Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override
        {
            auto e1 = get_body_rotation(q, body1_id, body_ids);
            auto e2 = get_body_rotation(q, body2_id, body_ids);
            auto r1 = get_body_position(q, body1_id, body_ids);
            auto r2 = get_body_position(q, body2_id, body_ids);
            
            Eigen::Vector3d functions;

            Eigen::Vector3d dist;
            dist = distance(t);

            functions = r2 + R(e2) * body2_point - (r1 + R(e1) * body1_point) - dist;
            return functions;
        }

        double equations_number() override
        {
            return 3;
        }

    private:
        const Eigen::Vector3d& body1_point;
        const Eigen::Vector3d& body2_point;
        const Eigen::Vector3d& (*distance)(double t);
};

class FixedParameterConstraint : public Constraint
{
    public:
        FixedParameterConstraint(int id, int body_id, int parameter_index)
            : Constraint(id, body_id, 0), parameter_index(parameter_index) {}

        Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override
        {
            Eigen::VectorXd functions(1);
            auto e = get_body_rotation(q, body1_id, body_ids);
            auto r = get_body_position(q, body1_id, body_ids);

            if (parameter_index < 3) {
                functions(0) = r(parameter_index);
            } else {
                functions(0) = e(parameter_index - 3);
            }
            return functions;
        }

        double equations_number() override
        {
            return 1;
        }

    private:
        int parameter_index;

};

class FixedOrientationConstraint : public Constraint
{
    public:
        FixedOrientationConstraint(int id, int body_id, const Eigen::Vector3d& orientation)
            : Constraint(id, body_id, 0), orientation(orientation) {}

        Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override
        {
            Eigen::VectorXd functions(3);
            auto e = get_body_rotation(q, body1_id, body_ids);
            functions = R(e).transpose() * orientation;
            return functions;
        }

        double equations_number() override
        {
            return 4;
        }

    private:
        const Eigen::Vector3d& orientation;
};

class FixedPositionConstraint : public Constraint
{
    public:
        FixedPositionConstraint(int id, int body_id, const Eigen::Vector3d& position)
            : Constraint(id, body_id, 0), position(position) {}

        Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override
        {
            Eigen::VectorXd functions(3);
            auto r = get_body_position(q, body1_id, body_ids);
            functions = r - position;
            return functions;
        }

        double equations_number() override
        {
            return 3;
        }

    private:
        const Eigen::Vector3d& position;
};

class BallJointConstraint : public Constraint
{
    public:
        BallJointConstraint(int id, int body1_id, int body2_id, const Eigen::Vector3d& body1_point, 
                            const Eigen::Vector3d& body2_point)
            : Constraint(id, body1_id, body2_id), body1_point(body1_point), body2_point(body2_point) {}

        Eigen::VectorXd ConstrainingFunctions(const Eigen::VectorXd& q, double t, const std::vector<int>& body_ids) override
        {
            auto e1 = get_body_rotation(q, body1_id, body_ids);
            auto e2 = get_body_rotation(q, body2_id, body_ids);
            auto r1 = get_body_position(q, body1_id, body_ids);
            auto r2 = get_body_position(q, body2_id, body_ids);

            Eigen::VectorXd functions(3);
            functions = (r2 + R(e2) * body2_point) - (r1 + R(e1) * body1_point);
            return functions;
        }
        double equations_number() override
        {
            return 3;
        }
    private:
        const Eigen::Vector3d& body1_point;
        const Eigen::Vector3d& body2_point;
};


