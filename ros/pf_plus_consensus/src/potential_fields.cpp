#include <cmath>
#include <vector>

using namespace std;

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

    PotentialField(double a, double b) {
        this->x = a;
        this->y = b;
        this->z = 0.0;
        this->vx = 0.0;
        this->vy = 0.0;
        this->vz = 0.0;
        this->gain = 0.0;
        this->radius = 0.0;
        this->spread  = 0.0;
    }

    // calculo da forca exercida pelo ponto objetivo
    PotentialField attForce(PotentialField target, PotentialField robot, double gain_pf = 1.0)
    {
        PotentialField temp;
        double distance = std::pow(std::pow(target.x-robot.x, 2)+std::pow(target.y-robot.y, 2), 0.5);
        double psi = std::atan2(target.y-robot.y, target.x-robot.x);
        double deltaX, deltaY;
        if (distance < target.radius) {
            deltaX = 0;
            deltaY = 0;
        } else if (distance <= target.spread + target.radius) {
            //deltaX = target.gain * 0.5 * std::pow(distance - target.radius, 2) * std::cos(psi);
            //deltaY = target.gain * 0.5 * std::pow(distance - target.radius, 2) * std::sin(psi);
            deltaX = target.gain * (distance - target.radius) * std::cos(psi);
            deltaY = target.gain * (distance - target.radius) * std::sin(psi);
        } else {
            deltaX = target.gain * target.spread * std::cos(psi);
            deltaY = target.gain * target.spread * std::sin(psi);
        }
        temp.x = deltaX * gain_pf;
        temp.y = deltaY * gain_pf;
        temp.z = 0.0 * gain_pf;
        return temp;
    }

    // calculo da forca exercida por um obstaculo
    PotentialField repForce(PotentialField obs, PotentialField robot, double gain_pf = 1.0)
    {
        PotentialField temp;
        double distance = std::pow(std::pow(robot.x-obs.x, 2)+std::pow(robot.y-obs.y, 2), 0.5);
        double psi = std::atan2(obs.y-robot.y, obs.x-robot.x);
        double deltaX, deltaY;
        if (distance > obs.spread + obs.radius) {
            deltaX = 0;
            deltaY = 0;
        } else if (distance >= obs.radius) {
            //deltaX = -obs.gain * (1 / std::pow(obs.spread + obs.radius - distance, 0.5)) * std::cos(psi);
            //deltaY = -obs.gain * (1 / std::pow(obs.spread + obs.radius - distance, 0.5)) * std::sin(psi);
            deltaX = -obs.gain * (obs.spread + obs.radius - distance) * std::cos(psi);
            deltaY = -obs.gain * (obs.spread + obs.radius - distance) * std::sin(psi);
        } else {
            deltaX = -copysign(1.0, std::cos(psi)) * 9999;
            deltaY = -copysign(1.0, std::sin(psi)) * 9999;
        }
        temp.x = deltaX * gain_pf;
        temp.y = deltaY * gain_pf;
        temp.z = 0.0;
        return temp;
    }
    
    // calculo da forca exercida por um retangulo
    PotentialField boxForce(PotentialField robot, double x1, double y1, double x2, double y2, double gain_pf = 1.0)
    {
        PotentialField temp;
        PotentialField even_temp;
        even_temp.gain = 1.0;
        even_temp.radius = 0.2;
        even_temp.spread = 0.4;
        double now = x2;
        while (now < x1) {
            even_temp.x = now;
            even_temp.y = y1;
            temp.add(repForce(even_temp, robot));
            even_temp.y = y2;
            temp.add(repForce(even_temp, robot));
            now += 0.1;
        }
        now = y2;
        while (now < y1) {
            even_temp.x = x1;
            even_temp.y = now;
            temp.add(repForce(even_temp, robot));
            even_temp.x = x2;
            temp.add(repForce(even_temp, robot));
            now += 0.1;
        }
        
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
