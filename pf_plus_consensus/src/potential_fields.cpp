#include <cmath>

class PotentialField {

    public:

    double x;
    double y;
    double z;
    double gain;
    double radius;
    double spread;
	double vx;
	double vy;
	double vz;

    PotentialField() {
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->vx = 0.0;
        this->vy = 0.0;
        this->vz = 0.0;
        this->gain = 0.0;
        this->radius = 0.0;
        this->spread  = 0.0;
    }

    // calculo da forca exercida pelo ponto objetivo
    PotentialField attForce(PotentialField target, PotentialField robot)
    {
        PotentialField temp;
        double distance = std::pow(std::pow(target.x-robot.x, 2)+std::pow(target.y-robot.y, 2), 0.5);
        double psi = std::atan2(target.y-robot.y, target.x-robot.x);
        double deltaX, deltaY;
        if (distance < target.radius) {
            deltaX = 0;
            deltaY = 0;
        } else if (distance <= target.spread + target.radius) {
            deltaX = target.gain * (distance - target.radius) * std::cos(psi);
            deltaY = target.gain * (distance - target.radius) * std::sin(psi);
        } else {
            deltaX = target.gain * target.spread * std::cos(psi);
            deltaY = target.gain * target.spread * std::sin(psi);
        }
        temp.x = deltaX;
        temp.y = deltaY;
        temp.z = 0.0;
        return temp;
    }

    // calculo da forca exercida por um obstaculo
    PotentialField repForce(PotentialField obs, PotentialField robot)
    {
        PotentialField temp;
        double distance = std::pow(std::pow(robot.x-obs.x, 2)+std::pow(robot.y-obs.y, 2), 0.5);
        double psi = std::atan2(obs.y-robot.y, obs.x-robot.x);
        double deltaX, deltaY;
        if (distance > obs.spread + obs.radius) {
            deltaX = 0;
            deltaY = 0;
        } else if (distance >= obs.radius) {
            deltaX = -obs.gain * (obs.spread + obs.radius - distance) * std::cos(psi);
            deltaY = -obs.gain * (obs.spread + obs.radius - distance) * std::sin(psi);
        } else {
            deltaX = -copysign(1.0, std::cos(psi)) * 9999;
            deltaY = -copysign(1.0, std::sin(psi)) * 9999;
        }
        temp.x = deltaX;
        temp.y = deltaY;
        temp.z = 0.0;
        return temp;
    }

    // funcao para somar dois campos potenciais
    void add(PotentialField a) 
    {
        this->x += a.x;
        this->y += a.y;
        this->z += a.z;
    }
};
