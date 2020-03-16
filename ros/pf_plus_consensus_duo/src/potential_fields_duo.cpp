#include <cmath>
#include <vector>
#include <iostream>

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
            deltaX = target.gain * 0.5 * std::pow(distance - target.radius, 2) * std::cos(psi);
            deltaY = target.gain * 0.5 * std::pow(distance - target.radius, 2) * std::sin(psi);
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
    PotentialField repForceWall(PotentialField wall, PotentialField robot, double gain_pf = 1.0)
    {
        PotentialField temp, w;
        w.add(nearestPointFromWall(robot,wall));
        
        double distance = std::pow(std::pow(robot.x-w.x, 2)+std::pow(robot.y-w.y, 2), 0.5);
        double psi = std::atan2(w.y-robot.y, w.x-robot.x);
        double deltaX, deltaY;
        if (distance > wall.spread + wall.radius) {
            deltaX = 0;
            deltaY = 0;
        } else if (distance >= wall.radius) {
            deltaX = -wall.gain * (1 / std::pow(distance - wall.radius, 2)) * std::cos(psi);
            deltaY = -wall.gain * (1 / std::pow(distance - wall.radius, 2)) * std::sin(psi);
        } else {
            deltaX = -copysign(1.0, std::cos(psi)) * 10;
            deltaY = -copysign(1.0, std::sin(psi)) * 10;
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
            deltaX = -obs.gain * (1 / std::pow(obs.spread + obs.radius - distance, 0.5)) * std::cos(psi);
            deltaY = -obs.gain * (1 / std::pow(obs.spread + obs.radius - distance, 0.5)) * std::sin(psi);
        } else {
            deltaX = -copysign(1.0, std::cos(psi)) * 10;
            deltaY = -copysign(1.0, std::sin(psi)) * 10;
        }
        temp.x = deltaX * gain_pf;
        temp.y = deltaY * gain_pf;
        temp.z = 0.0 * gain_pf;
        return temp;
    }

    // funcao para somar dois campos potenciais
    void add(PotentialField a) 
    {
        this->x += a.x;
        this->y += a.y;
        this->z += a.z;
    }
    
    PotentialField nearestPointFromWall(PotentialField robot, PotentialField wall)
    {
        PotentialField temp;
        if (robot.x >= wall.x && robot.x <= wall.vx) {
            temp.x = robot.x;
        } else if (robot.x < wall.x) {
            temp.x = wall.x;
        } else {
            temp.x = wall.vx;
        }
        if (robot.y >= wall.vy && robot.y <= wall.y) {
            temp.y = robot.y;
        } else if (robot.y < wall.vy) {
            temp.y = wall.vy;
        } else {
            temp.y = wall.y;
        }
        return temp;
    }

    // example of this in action here: http://itp.nyu.edu/~gab305/files/gif_with_blurred_borders.gif

    float distanceFromLine(PotentialField p, PotentialField l1, PotentialField l2){
        float xDelta = l2.x - l1.x;
        float yDelta = l2.y - l1.y;

        //	final double u = ((p3.getX() - p1.getX()) * xDelta + (p3.getY() - p1.getY()) * yDelta) / (xDelta * xDelta + yDelta * yDelta);
        float u = ((p.x - l1.x) * xDelta + (p.y - l1.y)*yDelta) / (xDelta * xDelta + yDelta * yDelta);
        
        PotentialField closestPointOnLine;
        if (u < 0) {
	        closestPointOnLine = l1;
	    } else if (u > 1) {
	        closestPointOnLine = l2;
	    } else {
                closestPointOnLine = PotentialField(l1.x + u * xDelta, l1.y + u * yDelta);
	    }
        
       
        PotentialField d = PotentialField(p.x - closestPointOnLine.x, p.y - closestPointOnLine.y);
        return sqrt(d.x * d.x + d.y * d.y); // distance
    }


    float distanceFromPoly(PotentialField p, vector<PotentialField> poly){
        float result = 10000;
        
        // check each line
        for(int i = 0; i < poly.size(); i++){
            int previousIndex = i -1;
            if(previousIndex < 0){
                previousIndex = poly.size() - 1;
            }
            
            PotentialField currentPoint = poly.at(i);
            PotentialField previousPoint = poly.at(previousIndex);
            
            float segmentDistance = distanceFromLine(PotentialField(p.x,p.y), previousPoint, currentPoint);
            
            if(segmentDistance < result){
                result = segmentDistance;
            }
        }
        
        return result;
    }
};
