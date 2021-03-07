#include <iostream>
#include <queue>
#include <stack>
#include <cmath>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Emitter.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace webots;
using namespace cv;

Robot* robot = new Robot();
Motor* leftMotor = robot->getMotor("left wheel motor");
Motor* rightMotor = robot->getMotor("right wheel motor");
PositionSensor* leftEncoders = leftMotor->getPositionSensor();
PositionSensor* rightEncoders = rightMotor->getPositionSensor();
DistanceSensor* infrared[8];
Camera* colorCam = robot->getCamera("colour_sensor");
Gyro* gyro = robot->getGyro("gyro");
LightSensor* leftHeat = robot->getLightSensor("left_heat_sensor");
LightSensor* rightHeat = robot->getLightSensor("right_heat_sensor");
Emitter* emitter = robot->getEmitter("emitter");
GPS* gps = robot->getGPS("gps");
Camera *camR = robot->getCamera("camera_right");
Camera *camL = robot->getCamera("camera_left");
Camera *camC = robot->getCamera("camera_centre");
DistanceSensor* lds = robot->getDistanceSensor("lds");
DistanceSensor* rds = robot->getDistanceSensor("rds");
const double radpermeter = 1/0.0205;
double angle = 0;
void updateGyro(int timeStep) {
  angle += (timeStep / 1000.0) * (gyro->getValues())[0];
}
double getAngle() {
    return angle * 180 / 3.1415;
}
void turnL(double deg, int timestep)
{
  int start = getAngle();
      leftMotor->setVelocity(-2);
    rightMotor->setVelocity(2);
  while (robot->step(timestep) != -1) {
    updateGyro(timestep);
//    printf("angle: %f\n", getAngle());
    if(getAngle()-start>=deg+0.05)
      break;
    

  }
}
void turnR(double deg, int timestep)
{
  int start = getAngle();
      leftMotor->setVelocity(2);
    rightMotor->setVelocity(-2);
  while (robot->step(timestep) != -1) {
    updateGyro(timestep);
//    printf("angle: %f\n", getAngle());
    if(start-getAngle()>=deg-3.5){
      break;
    }
  }
}
void goforward(double dist, int timeStep)
{
  double start = leftEncoders->getValue();

  leftMotor->setVelocity(4);   //Set motors to spin at 10 radians per second
  rightMotor->setVelocity(4);
  
  while (robot->step(timeStep) != -1) {
    updateGyro(timeStep);

    leftMotor->setVelocity(4);
    rightMotor->setVelocity(4);
    if (leftEncoders->getValue() - start >= radpermeter*dist+0.01)
    {
      break;  
    }

  }
}
//check when moving forward

int checkObstable()//returns: 0 = no obs; 1 = obs on right; 2 = obs on left; 3 = large obs
{
    if(abs(lds->getValue()- rds->getValue()) < 0.3 || lds->getValue() < 0.1 || rds->getValue() < 0.1)
        return 3;
    else if(lds->getValue()- rds->getValue() > 0.3 && rds->getValue() < 0.15)
        return 1;
    else if(rds->getValue() - lds->getValue() > 0.3 && lds->getValue() < 0.15)
        return 2;
    else
        return 0;

}

int main( int argc, char** argv )
{

int timeStep = (int)robot->getBasicTimeStep();

leftEncoders->enable(timeStep);
rightEncoders->enable(timeStep);

gyro->enable(timeStep);
gps->enable(timeStep);
leftHeat->enable(timeStep);
rightHeat->enable(timeStep);
colorCam->enable(timeStep);
camL->enable(timeStep);
camR->enable(timeStep);
camC->enable(timeStep);
    
lds->enable(timeStep);
rds->enable(timeStep);

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    bool avoiding = false;
    bool turning = false;
    bool advancing = false;
    const double motorSpeed = 5.0;
    double turnTarget = 0.0;
    double targetAngle = 0.0;
    double forwardTarget = 0.0;
    double startOfMoveGPS = 0.0;
    int shifts = 0;
    const double shiftDist = 0.01;
    int obs = 0;
    while (robot->step(timeStep) != -1) {
        cout << "left: " << lds->getValue() << "\n";
        cout << "right: " << rds->getValue() << "\n";
        
        if(avoiding)
        {
            if(checkObstable() == 1){        //shift left
              turnL(90, timeStep);
              if(checkObstable() == 3 || shifts > 5) // if out of tile
              {
                  //wall off
                  cout << "cannot pass\n";
                  //go back to previous tile
                  
                  
                  //center
                  turnR(90, timeStep);
                  goforward(shiftDist*shifts, timeStep);
                  turnL(90, timeStep);
                  avoiding = false;
              }
              goforward(0.01, timeStep);
              turnR(90, timeStep);
              obs = 1;
              shifts++;
            }
            else if(checkObstable() == 2){        //shift right
              turnR(90, timeStep);
              if(checkObstable() == 3 || shifts > 5)
              {
                  //wall off
                  
                  //go back to previous tile
                  cout << "cannot pass\n";
                  //center
                  turnL(90, timeStep);
                  goforward(shiftDist*shifts, timeStep);
                  turnR(90, timeStep);
                  avoiding = false;
              }
              goforward(0.01, timeStep);
              turnR(90, timeStep);
              obs = 2;
              shifts++;
            }
            else if (checkObstable() == 3){
                //wall off and go back to center
                if(obs == 1){
                    turnR(90, timeStep);
                    goforward(shiftDist*shifts, timeStep);
                    turnL(90, timeStep);
                }
                else if(obs == 2){
                    turnL(90, timeStep);
                    goforward(shiftDist*shifts, timeStep);
                    turnR(90, timeStep);
                }
            }
            else if(checkObstable() == 0){
                //go to next tile and center
                advancing = true;
                goforward(0.15, timeStep);
                if(obs == 1){
                    turnR(90, timeStep);
                    goforward(shiftDist*shifts, timeStep);
                    turnL(90, timeStep);
                }
                else if(obs == 2){
                    turnL(90, timeStep);
                    goforward(shiftDist*shifts, timeStep);
                    turnR(90, timeStep);
                }
                avoiding = false;
            }
        }
        else if(checkObstable() == 1 || checkObstable() == 2)
        {
            avoiding = true;
            cout << "avoiding\n";
        }
        else{
            advancing = true;
            leftMotor->setVelocity(2);
            rightMotor->setVelocity(2);
        }
    };
    
    delete robot;
    return 0;
}