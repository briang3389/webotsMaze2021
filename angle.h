//angle.h: deals with the angle the robot is at

double angle = 0.00; //angle the robot is at based on the Gyro sensor

void updateGyro(int timeStep) { 
    //updates angle with the Gyro sensor (should be called every timestep)
    angle -= (timeStep / 1000.0) * (gyro->getValues())[0];
    angle = fmod((angle + 6.283), 6.283);
}

double getAngle() {
    //return number of degrees based on Gyro sensor
    return angle * 180.0 / 3.1415;
}

const int roundAngle(int i) {
    //rounds angle to the nearest 
    int ang = (i + 360) % 360;
    if (ang > 315 || ang <= 45) return 0;
    if (ang > 45 && ang <= 135) return 90;
    if (ang > 135 && ang <= 225) return 180;
    if (ang > 225 && ang <= 315) return 270;
    return -1;
}

int compassDirection() {
    //returns rounded angle in the form of an integer from 0-3
    //0 is up, 1 is right, 2 is down, 3 is left
    return roundAngle(getAngle()) / 90;
}

const bool goClockwise(int startDirection, int endDirection) { 
    //checks if it's fastest to turn clockwise (or counterclockwise)
    if (endDirection - startDirection == -1 || endDirection - startDirection == 3) {
        return false;
    }
    return true;
}
