#include <includes.h>
#include <robot.h>
#include <angle.h>
#include <tile.h>
#include <navigation.h>
#include <bfs.h>
#include <message.h>
#include <recognition.h>
#include <victims.h>



int main(int argc, char** argv) {
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

    
    angle = 0;

    while (robot->step(timeStep) != -1) {
        updateGyro(timeStep);
        if (stopping){
            if (timer == stoppingConst) {
                sendMessage();
            }
            if (timer >= stoppingConst + 30) {
                timer = 0;
                rightMotor->setVelocity(motorPrevious.first);
                leftMotor->setVelocity(motorPrevious.second);
                stopping = false;
            }
            timer++;
        }
        
        else if (reversing) {
            if (direction == 0 || direction == 2) {
                if (fabs(gps->getValues()[2] - startOfMoveGPS) < .01) reversing = false;

            }
            if (direction == 1 || direction == 3) {
                if (fabs(gps->getValues()[0] - startOfMoveGPS) < .01) reversing = false;
            }
        }
        else if (turning) {
            if (fabs(getAngle() - turnTarget) < 5.0 || fabs(getAngle() - turnTarget + 360) < 1.0 || skipTurn) {
                //timer = 0;
                moveStart = make_pair(gps->getValues()[0], gps->getValues()[2]);
                skipTurn = false;
                turning = false;
                leftMotor->setVelocity(min(leftMotor->getMaxVelocity(), motorSpeed));
                rightMotor->setVelocity(min(leftMotor->getMaxVelocity(), motorSpeed));
                targetAngle = roundAngle(getAngle());
                if (direction == 0 || direction == 2) startOfMoveGPS = gps->getValues()[2];
                else startOfMoveGPS = gps->getValues()[0];
                //cout << "Target = " << targetAngle << endl;
                //cout << "Turned!" << endl;
            }
            else {
                if (checkHeatVictim() || checkVisualVictim(0) || checkVisualVictim(1) || checkVisualVictim(2)){
                    timer = 0;
                    motorPrevious = make_pair(rightMotor->getVelocity(), leftMotor->getVelocity());
                    leftMotor->setVelocity(0);
                    rightMotor->setVelocity(0);
                    stopping = true;
                }

            }
        }
        else if (advancing) {
            //cout << "Delta: " << getDelta() << endl;
            if (getDelta() >= forwardTarget || getDirection(loc, getCoords()) == -1) {
                advancing = false;
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);

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
                }
                if (relativeDirection < 359.8 && relativeDirection > 270.0) {
                    rightMotor->setVelocity(min(rightMotor->getMaxVelocity(), rightMotor->getVelocity() + .005));
                    leftMotor->setVelocity(min(rightMotor->getMaxVelocity(), leftMotor->getVelocity() - .005));
                }
                //timer++;
                //if (checkHeatVictim(message) || checkVisualVictim(true, message) || checkVisualVictim(false, message)) {
                if (checkHeatVictim()) {
                    motorPrevious = make_pair(rightMotor->getVelocity(), leftMotor->getVelocity());
                    leftMotor->setVelocity(0);
                    rightMotor->setVelocity(0);
                    stopping = true;
                }
                
                if (checkHole()) {
                    cout << "There's a hole!" << endl;
                    reversing = true;
                    advancing = false;
                    rightMotor->setVelocity(-motorSpeed / 2);
                    leftMotor->setVelocity(-motorSpeed / 2);
                    board[targetTile.first][targetTile.second].visited = true;
                    board[targetTile.first][targetTile.second].isHole = true;
                }
            }
        }
        else {
           
            if (!board[loc.first][loc.second].visited) {
                scan(loc);
                board[loc.first][loc.second].visited = true;
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
            if (!visualChecking){
                targetTile = steps.top();
                //cout << "Traveling to " << targetTile.first << " " << targetTile.second << ", ";
                steps.pop();
                int getDirVal = getDirection(loc, targetTile);
                //cout << "Direction: " << getDirVal << endl;
                int dirTarget = (4 + getDirVal) % 4;
                turnTarget = dirTarget * 90;
                if (dirTarget == direction) skipTurn = true;
                forwardTarget = getDeltaTarget(targetTile);
                //cout << "Angle: " << getAngle() << endl;
                if (goClockwise(direction, dirTarget)) {
                    leftMotor->setVelocity(motorSpeed);
                    rightMotor->setVelocity(-motorSpeed);
                }
                else {
                    leftMotor->setVelocity(-motorSpeed);
                    rightMotor->setVelocity(motorSpeed);
                }
                direction = getDirVal;
    
    
                //cout << "Started moving." << endl;
                //if (!skipTurn) cout << "Started turning." << endl;
                turning = advancing = true;
            }
        }
    }

    delete robot;
    return 0;
}
