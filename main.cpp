#include <includes.h>
#include <robot.h>
#include <angle.h>
#include <tile.h>
#include <environment.h>
#include <message.h>
#include <bfs.h>


int main(){
    int timeStep = init();

    //states
    bool turning = false;
    bool advancing = false;
    bool stopping = false;
    bool reversing = false;
    bool ending = false;
    bool skipTurn = false;
    bool visualChecking = false;


    //constants
    const double motorSpeed = 5.0;
    const double stoppingConst = 250.0;

    //movement variables
    double turnTarget = 0.0;
    double targetAngle = 0.0;
    double forwardTarget = 0.0;
    double startOfMoveGPS = 0.0;

    //other stuff
    pair<double, double> motorPrevious;
    pair<int, int> targetTile;

    int timer = 0;

    stack<pair<int, int>> steps;

    startingGPS.first = gps->getValues()[0];
    startingGPS.second = gps->getValues()[2];
    cout << "Starting GPS values: " << startingGPS.first << " " << startingGPS.second << endl;

    wallScan(timeStep);
    printBoard();
    while (robot->step(timeStep) != -1) {
        updateGyro(timeStep);

        //stop for victim
        if (stopping) {
            if (timer == stoppingConst) {
                sendMessage();
            }
            if (timer >= stoppingConst + 30) {
                timer = 0;
                setMotors(motorPrevious.first, motorPrevious.second);
                stopping = false;
            }
            timer++;
        }

        //reverse out of hole
        else if (reversing) {
            if (direction == 0 || direction == 2) {
                if (fabs(gps->getValues()[2] - startOfMoveGPS) < .01) reversing = false;

            }
            if (direction == 1 || direction == 3) {
                if (fabs(gps->getValues()[0] - startOfMoveGPS) < .01) reversing = false;
            }
        }

        //turn
        else if (turning) {
            if (fabs(getAngle() - turnTarget) < 5.0 || fabs(getAngle() - turnTarget + 360) < 1.0 || skipTurn) {
                moveStart = make_pair(gps->getValues()[0], gps->getValues()[2]);
                skipTurn = false;
                turning = false;
                setMotors(min(leftMotor->getMaxVelocity(), motorSpeed), min(leftMotor->getMaxVelocity(), motorSpeed));
                targetAngle = roundAngle(getAngle());
                if (direction == 0 || direction == 2) startOfMoveGPS = gps->getValues()[2];
                else startOfMoveGPS = gps->getValues()[0];
            }
            else {
                if (checkAllVictims()) {
                    timer = 0;
                    motorPrevious = make_pair(leftMotor->getVelocity(), rightMotor->getVelocity());
                    leftMotor->setVelocity(0);
                    rightMotor->setVelocity(0);
                    stopping = true;
                }

            }
        }

        //go forward
        else if (advancing) {
            //cout << "Delta: " << getDelta() << " Target: " << forwardTarget << endl;
            if (getDelta() >= forwardTarget || getDirection(loc, getCoords()) == -1) {
                advancing = false;
                setMotors(0, 0);

                loc = getCoords();
                direction = compassDirection();
            }
            else {
                //cout << getDelta() << endl;
                double relativeDirection = fmod((360.0 + targetAngle - getAngle()), 360.0);
                //cout << "Angle = " << relativeDirection << endl;
                if (relativeDirection > .2 && relativeDirection < 90.0) {
                    rightMotor->setVelocity(min(rightMotor->getMaxVelocity(), rightMotor->getVelocity() - .005));
                    leftMotor->setVelocity(min(rightMotor->getMaxVelocity(), leftMotor->getVelocity() + .005));
                    setMotors(min(rightMotor->getMaxVelocity(), lVel() + .005), min(rightMotor->getMaxVelocity(), rVel() - .005));
                }
                if (relativeDirection < 359.8 && relativeDirection > 270.0) {
                    setMotors(min(rightMotor->getMaxVelocity(), lVel() - .005), min(rightMotor->getMaxVelocity(), rVel() + .005));
                }
                //if (checkHeatVictim(message) || checkVisualVictim(true, message) || checkVisualVictim(false, message)) {
                if (checkAllVictims()) {
                    motorPrevious = make_pair(lVel(), rVel());
                    setMotors(0, 0);
                    stopping = true;
                }

                if (checkHole()) {
                    cout << "There's a hole!" << endl;
                    reversing = true;
                    advancing = false;
                    setMotors(-motorSpeed / 2, -motorSpeed / 2);
                    board[targetTile.first][targetTile.second].visited = true;
                    board[targetTile.first][targetTile.second].isHole = true;
                }
            }
        }

        //find new path
        else {
            if (!board[loc.first][loc.second].visited) {
                wallScan(timeStep);
            }
            if (steps.empty()) {
                if (!ending) {
                    pair<int, int> tempPair = runBFS(loc);
                    //cout << "Finished BFS!" << endl;
                    while (tempPair != loc && tempPair.first != -1) {
                        steps.push(tempPair);
                        tempPair = boardParents[tempPair.first][tempPair.second];
                    }
                    if (tempPair.first == -1) {
                        ending = true;
                        board[startingCoord.first][startingCoord.second].visited = false;
                        while (!steps.empty()) steps.pop();
                        tempPair = runBFS(loc);

                        while (tempPair != loc && tempPair.first != -1) {
                            steps.push(tempPair);
                            tempPair = boardParents[tempPair.first][tempPair.second];
                        }
                        cout << "Exiting maze..." << endl;
                    }
                }
                else {
                    message[8] = 'E';
                    emitter->send(message, 9);
                    break;
                }
            }
            cout << "Coordinates: " << loc.first << " " << loc.second << endl;
            if (!visualChecking) {
                targetTile = steps.top();
                cout << "Traveling to " << targetTile.first << " " << targetTile.second << ", ";
                steps.pop();
                turning = advancing = true;
                int getDirVal = getDirection(loc, targetTile);
                //cout << "Direction: " << getDirVal << endl;
                int dirTarget = (4 + getDirVal) % 4;
                turnTarget = dirTarget * 90;
                if (dirTarget == direction) turning = false;
                forwardTarget = getDeltaTarget(targetTile);
                //cout << "Angle: " << getAngle() << endl;
                if (turning){
                    if (goClockwise(direction, dirTarget)) {
                        cout << "Turning clockwise" << endl;
                        setMotors(motorSpeed, -motorSpeed);
                    }
                    else {
                        cout << "Turning counterclockwise" << endl;
                        setMotors(-motorSpeed, motorSpeed);
                    }
                }
                else {
                    cout << "Heading straight" << endl;
                    
                    moveStart = make_pair(gps->getValues()[0], gps->getValues()[2]);
                    setMotors(min(leftMotor->getMaxVelocity(), motorSpeed), min(leftMotor->getMaxVelocity(), motorSpeed));
                    targetAngle = roundAngle(getAngle());
                    if (direction == 0 || direction == 2) startOfMoveGPS = gps->getValues()[2];
                    else startOfMoveGPS = gps->getValues()[0];
                    cout << "Target angle: " << targetAngle << endl;
                }
                printBoard();
                direction = getDirVal;

            }
        }
    }
}
