//robot.h: defines all the motors and sensors

Robot* robot = new Robot();
Motor* leftMotor = robot->getMotor("wheel2 motor");
Motor* rightMotor = robot->getMotor("wheel1 motor");
DistanceSensor* infrared[7];
Gyro* gyro = robot->getGyro("gyro");
Emitter* emitter = robot->getEmitter("emitter");
GPS* gps = robot->getGPS("gps");
Camera *camR = robot->getCamera("rCam");
Camera *camL = robot->getCamera("lCam");


//PositionSensor* leftEncoders = leftMotor->getPositionSensor();
//PositionSensor* rightEncoders = rightMotor->getPositionSensor();
//Camera* colorCam = robot->getCamera("colour_sensor");
//LightSensor* leftHeat = robot->getLightSensor("left_heat_sensor");
//LightSensor* rightHeat = robot->getLightSensor("right_heat_sensor");

int init() {
    int timeStep = (int)robot->getBasicTimeStep();

    for (int i = 0; i < 7; i++) {
        infrared[i] = robot->getDistanceSensor("ps" + to_string(i));
        infrared[i]->enable(timeStep);
    }
    
    gyro->enable(timeStep);
    gps->enable(timeStep);
    camL->enable(timeStep);
    camR->enable(timeStep);
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    //leftEncoders->enable(timeStep);
    //rightEncoders->enable(timeStep);
    //leftHeat->enable(timeStep);
    //rightHeat->enable(timeStep);
    //colorCam->enable(timeStep);
    //camC->enable(timeStep);

    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    
    for (int i = 0; i < 10; i++) robot->step(timeStep);
  
    return timeStep;
}

void setMotors(double lVal, double rVal){
    //cout << "Motors set to: " << lVal << ", " << rVal << endl;
    leftMotor->setVelocity(-lVal);
    rightMotor->setVelocity(rVal);
}

double lVel(){ return -leftMotor->getVelocity();}
double rVel(){ return rightMotor->getVelocity();}