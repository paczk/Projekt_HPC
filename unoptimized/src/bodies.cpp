#include <vector>
#include "bodies.hpp"

class Body
{
    public:
        Body(int id, double x, double y, double z, const std::vector<double>& euler_angles)
            : id(id), x(x), y(y), z(z), euler_angles(euler_angles) {}

        void setPosition(const std::vector<double>& position)
        {
            if (position.size() < 7) return;
            x = position[0];
            y = position[1];
            z = position[2];
            euler_angles.clear();
            euler_angles.insert(euler_angles.end(), position.begin() + 3, position.end());
        }

        const std::vector<double>& getPosition() const
        {
            std::vector<double> position;
            position.push_back(x);
            position.push_back(y);
            position.push_back(z);
            position.insert(position.end(), euler_angles.begin(), euler_angles.end());
            return position;
        }
        
    private:
        int id;
        double x;
        double y;
        double z;
        std::vector<double> euler_angles;
};

