#include <cmath>
#include <vector>

#define PI 3.14159265

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
        this->spread = 0.0;
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
        this->spread = 0.0;
    }

    // calculo da forca exercida pelo ponto objetivo

    PotentialField attForce(PotentialField target, PotentialField robot, double gain_pf = 1.0) {
        PotentialField temp;
        double distance = std::pow(std::pow(target.x - robot.x, 2) + std::pow(target.y - robot.y, 2), 0.5);
        double psi = std::atan2(target.y - robot.y, target.x - robot.x);
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

    PotentialField repForce(PotentialField obs, PotentialField robot, double gain_pf = 1.0, int type = 0) {
        PotentialField temp;
        double distance = std::pow(std::pow(robot.x - obs.x, 2) + std::pow(robot.y - obs.y, 2), 0.5);
        double psi = std::atan2(obs.y - robot.y, obs.x - robot.x);
        double deltaX, deltaY;
        if (distance > obs.spread + obs.radius) {
            deltaX = 0;
            deltaY = 0;
        } else if (distance >= obs.radius) {
            deltaX = -obs.gain * (1 / std::pow(obs.spread + obs.radius - distance, 0.5)) * std::cos(psi);
            deltaY = -obs.gain * (1 / std::pow(obs.spread + obs.radius - distance, 0.5)) * std::sin(psi);
            //            deltaX = -obs.gain * (obs.spread + obs.radius - distance) * std::cos(psi);
            //            deltaY = -obs.gain * (obs.spread + obs.radius - distance) * std::sin(psi);
        } else {
            deltaX = -copysign(1.0, std::cos(psi)) * 9999;
            deltaY = -copysign(1.0, std::sin(psi)) * 9999;
        }

        temp.x = deltaX * gain_pf;
        temp.y = deltaY * gain_pf;
        temp.z = 0.0;

        return temp;
    }
    // calculo da forca exercida por um obstaculo

    PotentialField rotateRepForce(PotentialField obs, PotentialField robot, double gain_pf = 1.0) {
        PotentialField temp;
        double distance = std::pow(std::pow(robot.x - obs.x, 2) + std::pow(robot.y - obs.y, 2), 0.5);
        double psi = std::atan2(obs.y - robot.y, obs.x - robot.x);
        double deltaX, deltaY;
        if (distance > obs.spread + obs.radius) {
            deltaX = 0;
            deltaY = 0;
        } else if (distance >= obs.radius) {
            deltaX = -obs.gain * (1 / std::pow(obs.spread + obs.radius - distance, 0.5)) * std::cos(psi);
            deltaY = -obs.gain * (1 / std::pow(obs.spread + obs.radius - distance, 0.5)) * std::sin(psi);
            //            deltaX = -obs.gain * (obs.spread + obs.radius - distance) * std::cos(psi);
            //            deltaY = -obs.gain * (obs.spread + obs.radius - distance) * std::sin(psi);
        } else {
            deltaX = -copysign(1.0, std::cos(psi)) * 9999;
            deltaY = -copysign(1.0, std::sin(psi)) * 9999;
        }

        psi = -0.75 * (PI / 2); //std::atan2(deltaY, deltaX) + (PI / 2);
        temp.x = ((deltaX * std::cos(psi) - deltaY * std::sin(psi))) * gain_pf;
        temp.y = ((deltaX * std::sin(psi) + deltaY * std::cos(psi))) * gain_pf;
        temp.z = 0.0;

        return temp;
    }

    // calculo da forca exercida por um retangulo

    PotentialField rotateBoxForce(PotentialField robot, double x1, double y1, double x2, double y2) {
        PotentialField temp;
        PotentialField even_temp;
        even_temp.gain = 1.0;
        even_temp.radius = 0.25;
        even_temp.spread = 1.0;
        double now = x2;
        while (now < x1) {
            even_temp.x = now;
            even_temp.y = y1;
            temp.add(rotateRepForce(even_temp, robot));
            even_temp.y = y2;
            temp.add(rotateRepForce(even_temp, robot));
            now += 0.1;
        }
        now = y2;
        while (now < y1) {
            even_temp.x = x1;
            even_temp.y = now;
            temp.add(rotateRepForce(even_temp, robot));
            even_temp.x = x2;
            temp.add(rotateRepForce(even_temp, robot));
            now += 0.1;
        }

        return temp;
    }
    // calculo da forca exercida por um retangulo

    PotentialField boxForce(PotentialField robot, double x1, double y1, double x2, double y2, double gain_pf = 1.0) {
        PotentialField temp;
        PotentialField even_temp;
        even_temp.gain = 1.0;
        even_temp.radius = 0.25;
        even_temp.spread = 1.0;
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

    // Given three colinear points p, q, r, the function checks if 
    // point q lies on line segment 'pr' 

    bool onSegment(PotentialField p, PotentialField q, PotentialField r) {
        if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
                q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
            return true;

        return false;
    }

    // To find orientation of ordered triplet (p, q, r). 
    // The function returns following values 
    // 0 --> p, q and r are colinear 
    // 1 --> Clockwise 
    // 2 --> Counterclockwise 

    int orientation(PotentialField p, PotentialField q, PotentialField r) {
        // See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
        // for details of below formula. 
        int val = (q.y - p.y) * (r.x - q.x) -
                (q.x - p.x) * (r.y - q.y);

        if (val == 0) return 0; // colinear 

        return (val > 0) ? 1 : 2; // clock or counterclock wise 
    }

    // The main function that returns true if line segment 'p1q1' 
    // and 'p2q2' intersect. 

    bool doIntersect(PotentialField p1, PotentialField q1, PotentialField p2, PotentialField q2) {
        // Find the four orientations needed for general and 
        // special cases 
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case 
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases 
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;

        // p1, q1 and q2 are colinear and q2 lies on segment p1q1 
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;

        // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;

        // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // Doesn't fall in any of the above cases 
    }

    // funcao para somar dois campos potenciais

    void add(PotentialField a) {
        this->x += a.x;
        this->y += a.y;
        this->z += a.z;
    }

    //saturacao

    void saturate(double a) {
        double rx = abs(this->x / a);
        double ry = abs(this->y / a);

        double rmax = max(rx, ry);

        if (rmax > 1) {
            this->x = this->x / rmax;
            this->y = this->y / rmax;
        }
    }
};
