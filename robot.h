//robot.h: defines all the sensors and motors

int timeStep;
Robot *robot = new Robot();
Motor *leftMotor = robot->getMotor("left wheel motor");
Motor *rightMotor = robot->getMotor("right wheel motor");
PositionSensor *leftEncoders = leftMotor->getPositionSensor();
PositionSensor *rightEncoders = rightMotor->getPositionSensor();
DistanceSensor *infrared[8];
Camera *colorCam = robot->getCamera("colour_sensor");
Gyro *gyro = robot->getGyro("gyro");
LightSensor *leftHeat = robot->getLightSensor("left_heat_sensor");
LightSensor *rightHeat = robot->getLightSensor("right_heat_sensor");
Emitter *emitter = robot->getEmitter("emitter");
GPS *gps = robot->getGPS("gps");
Camera *camR = robot->getCamera("camera_right");
Camera *camL = robot->getCamera("camera_left");
Camera *camC = robot->getCamera("camera_centre");
Receiver *rec = robot->getReceiver("receiver");

int init()
{
  int timeStep = (int) robot->getBasicTimeStep();

  for(int i = 0; i < 8; i++)
  {
    infrared[i] = robot->getDistanceSensor("ps" + to_string(i));
    infrared[i]->enable(timeStep);
  }
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
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);

  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);

  rec->enable(timeStep);

  for(int i = 0; i < 10; i++) robot->step(timeStep);

  return timeStep;
}
