#include <includes.h>
#include <robot.h>
#include <angle.h>
#include <tile.h>
#include <message.h>
#include <environment.h>
#include <bfs.h>

int timeStep;

//states
bool statesArr[]={false,false,false,false,false,false,false};
bool& turning = statesArr[0];
bool& advancing = statesArr[1];
bool& stopping = statesArr[2];
bool& reversing = statesArr[3];
bool& ending = statesArr[4];
bool& skipTurn = statesArr[5];
bool& visualChecking = statesArr[6];

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


const bool resetMapForLOP=true;
Tile boardBackup[boardSize][boardSize];
pair<int, int> boardParentsBackup[boardSize][boardSize];
bool traveledBackup[boardSize][boardSize];


#include <debugstuff.h>


int doTimeStep()
{
  #ifdef DEBUGSTUFF
    debugStuff();
  #endif
  int ret=robot->step(timeStep);
  updateGyro(timeStep);
  return ret;
}


pair<int,int> prevCoords={-1,-1};
//vector<pair<int,int>> allKnownCheckpoints;

int main(int argc, char **argv)
{
  timeStep = init();

  startingGPS.first = gps->getValues()[0];
  startingGPS.second = gps->getValues()[2];
  cout << "Starting GPS values: " << startingGPS.first << " " << startingGPS.second << endl;

  int numLOP = 0;
  //angle = 0;
  
  wallScan(timeStep);
  //printBoard();

  while(doTimeStep() != -1)
  {
    if(rec->getQueueLength() > 0) //lack of progress just happened
    {
      cout<<"LOP "<< ++numLOP <<endl;
      //char c = *(char*) 
      rec->getData(); //TODO: have it reset the map to last checkpoint
      rec->nextPacket();
      //printf("%c\t%d\n", c, numLOP);

      //reset state
      turning = false;
      advancing = false;
      stopping = false;
      reversing = false;
      ending = false;
      skipTurn = false;
      visualChecking = false;
      
      turnTarget = 0.0;
      targetAngle = 0.0;
      forwardTarget = 0.0;
      startOfMoveGPS = 0.0;
      
      timer = 0;
      
      rightMotor->setVelocity(0);
      leftMotor->setVelocity(0);
      //angle=0.0;
      while(steps.size()>0)steps.pop();
      while(bfsQueue.size()>0)bfsQueue.pop();
      //change position to last visited checkpoint (or start tile)
      //loc=getCoords(); //this uses gps so manually keeping track of checkpoint locations is not necessary
      
      if(resetMapForLOP)
      { //restorebackup
        for(int i=0;i<boardSize;i++)
        {
          for(int j=0;j<boardSize;j++)
          {
            board[i][j]=boardBackup[i][j];
            boardParents[i][j]=boardParentsBackup[i][j];
            traveled[i][j]=traveledBackup[i][j];
          }
        }
      }
      
      for(int i = 0; i < 10; i++)
      {
        cout<<i+1<<" ";
        fflush(stdout);
        doTimeStep();
      }
      cout<<endl;
      
      angle=0.0;
      direction=0;
      loc=getCoords();
      continue;
    }
    if(resetMapForLOP) //STILL NEED TO TEST (including if LOP happens before any checkpoint is hit)
    { 
      pair<int,int> tmploc=getCoords();
      if(tmploc != prevCoords)
      {
        prevCoords=tmploc;
        if(getTileType(colorCam->getImage())==1) //is checkpoint
        {
          cout<<"saving map and stuff"<<endl;
          //savebackup
          for(int i=0;i<boardSize;i++)
          {
            for(int j=0;j<boardSize;j++)
            {
              boardBackup[i][j]=board[i][j];
              boardParentsBackup[i][j]=boardParents[i][j];
              traveledBackup[i][j]=traveled[i][j];
            }
          }
        }
        /*
        bool alreadyBeenThere=false;
        for(pair<int,int> pairr:allKnownCheckpoints)
        {
          if(tmploc==pairr)
          {
            alreadyBeenThere=true;
            break;
          }
        }
        */
      }
    }
    
    

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
                setMotors(min(motorMax(), motorSpeed), min(motorMax(), motorSpeed));
                targetAngle = roundAngle(getAngle());
                if (direction == 0 || direction == 2) startOfMoveGPS = gps->getValues()[2];
                else startOfMoveGPS = gps->getValues()[0];
            }
            else {
                if (checkVisualVictim(camR) || checkVisualVictim(camL)) {
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
                    rightMotor->setVelocity(min(motorMax(), rightMotor->getVelocity() - .005));
                    leftMotor->setVelocity(min(motorMax(), leftMotor->getVelocity() + .005));
                    setMotors(min(motorMax(), lVel() + .005), min(motorMax(), rVel() - .005));
                }
                if (relativeDirection < 359.8 && relativeDirection > 270.0) {
                    setMotors(min(motorMax(), lVel() - .005), min(motorMax(), rVel() + .005));
                }
                //if (checkHeatVictim(message) || checkVisualVictim(true, message) || checkVisualVictim(false, message)) {
                if (checkVisualVictim(camR) || checkVisualVictim(camL)) {
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
                    setMotors(min(motorMax(), motorSpeed), min(motorMax(), motorSpeed));
                    targetAngle = roundAngle(getAngle());
                    if (direction == 0 || direction == 2) startOfMoveGPS = gps->getValues()[2];
                    else startOfMoveGPS = gps->getValues()[0];
                    cout << "Target angle: " << targetAngle << endl;
                }
                //printBoard();
                direction = getDirVal;

            }
        }
    }
}
