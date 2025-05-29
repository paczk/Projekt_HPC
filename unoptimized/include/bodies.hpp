#ifndef BODIES_HPP
#define BODIES_HPP

class Body
{
    public:
        Body(int id, double x, double y, double z, const std::vector<double>& euler_angles);

        void setPosition(const std::vector<double>& position);

        const std::vector<double>& getPosition();
        
    private:
        int id;
        double x;
        double y;
        double z;
        std::vector<double> euler_angles;
};

#endif