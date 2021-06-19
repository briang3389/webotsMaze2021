#include "includes.h"

int doTimeStep();

#include "robot.h"
#include "angle.h"
#include "tile.h"
#include "message.h"
#include "environment.h"
#include "bfs.h"

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
const double stoppingConst = 100.0;

//movement variables
double turnTarget = 0.0;
double targetAngle = 0.0;
double forwardTarget = 0.0;
double startOfMoveGPS = 0.0;

//other stuff
pair<double, double> motorPrevious;
pair<int, int> targetTile;
bool doLOPstuff=false;
int timer = 0;

stack<pair<int, int>> steps;


const bool resetMapForLOP=false;
Tile boardBackup[boardSize][boardSize];
pair<int, int> boardParentsBackup[boardSize][boardSize];
bool traveledBackup[boardSize][boardSize];


//#include "debugstuff.h"


int doTimeStep()
{
  #ifdef DEBUGSTUFF
    //debugStuff();
  #endif
  int ret=robot->step(timeStep);
  updateGyro(timeStep);
  //cout<<getAngle()<<endl;
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

  orient(timeStep);
  cout << "Done Orienting!" << endl;

  int numLOP = 0;
  //angle = 0;
  
  wallScan(timeStep);
  
  setMotors(1.5,-1.5);
  while((int)getAngle()!=(direction*90+180)%360)
  {
      doTimeStep();
  }
  setMotors(0,0);
  
  direction = (direction + 2) % 4;
  
  wallScan(timeStep);

  while(doTimeStep() != -1)
  {
      if(rec->getQueueLength() > 0 || doLOPstuff) //lack of progress just happened
      {
        cout<<"LOP "<< ++numLOP <<endl;
        if(rec->getQueueLength()>0)
        {
        //char c = *(char*) 
        rec->getData(); //TODO: have it reset the map to last checkpoint
        rec->nextPacket();
        //printf("%c\t%d\n", c, numLOP);
        }
      //reset state
      turning = false;
      advancing = false;
      stopping = false;
      reversing = false;
      ending = false;//if(!doLOPstuff)
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
      while(steps.size()>0)steps.pop();//if(!doLOPstuff)if(!ending)
      while(bfsQueue.size()>0)bfsQueue.pop();//if(!doLOPstuff)if(!ending)
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
      
      for(int i = 0; i < 100; i++)
      {
        cout<<i+1<<" ";
        fflush(stdout);
        doTimeStep();
      }
      cout<<endl;
      
      if(!doLOPstuff)angle=0.0;
      direction=0;
      loc=getCoords();
      doLOPstuff=false;
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
                for(int i = 0; i < 50; i++) //50
                {
                    doTimeStep();
                }
                boardLoc(getCoords()).victimChecked=true;
                cout << "Done stopping!" << endl;
                /*
                setMotors(1.5,-1.5);
                while((int)getAngle()!=0)
                {
                    doTimeStep();
                }
                */
                /*
                if(getAngle()<180.0)setMotors(motorSpeed,-motorSpeed);
                else setMotors(-motorSpeed,motorSpeed);
                while(!(fabs(getAngle() - 0.0) < 5.0 || fabs(getAngle() - 0.0 + 360) < 1.0))
                {
                  doTimeStep();
                }
                doLOPstuff=true;
                continue;
                */
            }
            if (timer >= stoppingConst + 30) {
                timer = 0;
                setMotors(motorPrevious.first, motorPrevious.second);
                stopping = false;
                cout << "Done more stopping!" << endl;
            }
            timer++;
        }

        //reverse out of hole
        else if (reversing) {
            
            if (direction == 0 || direction == 2) {
                if (fabs(gps->getValues()[2] - startOfMoveGPS) < .002) reversing = false;

            }
            if (direction == 1 || direction == 3) {
                if (fabs(gps->getValues()[0] - startOfMoveGPS) < .002) reversing = false;
            }
        }

        //turn
        else if (turning) {
            if (fabs(getAngle() - turnTarget) < 5.0 || fabs(getAngle() - turnTarget + 360) < 1.0 || skipTurn) {
                moveStart = make_pair(gps->getValues()[0], gps->getValues()[2]);
                skipTurn = false;
                turning = false;
                if (direction == 0 || direction == 2) startOfMoveGPS = gps->getValues()[2];
                else startOfMoveGPS = gps->getValues()[0];
                setMotors(min(motorMax(), motorSpeed), min(motorMax(), motorSpeed));
                targetAngle = roundAngle(getAngle());
            }
            else {
                if (checkVisualVictim(camR) || checkVisualVictim(camL)) {
                    timer = 0;
                    motorPrevious = make_pair(lVel(), rVel());//make_pair(leftMotor->getVelocity(), rightMotor->getVelocity());
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
                //cout << "Angle = " << relativeDirection << ", target is " << targetAngle << endl;
                if (relativeDirection > .2 && relativeDirection < 90.0) {
                    rightMotor->setVelocity(min(motorMax(), rightMotor->getVelocity() - .01));
                    leftMotor->setVelocity(min(motorMax(), leftMotor->getVelocity() + .01));
                    setMotors(min(motorMax(), lVel() + .01), min(motorMax(), rVel() - .01));
                }
                if (relativeDirection < 359.8 && relativeDirection > 270.0) {
                    setMotors(min(motorMax(), lVel() - .01), min(motorMax(), rVel() + .01));
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
                    pair<int, int> neighborThing = neighborTile(targetTile, direction);
                    if(direction==1) neighborThing = neighborTile(neighborThing, 0);
                    if(direction==2) neighborThing = neighborTile(targetTile, 1);
                    if (direction==3) neighborThing=targetTile;
                    pair<int, int> leftNeighbor = neighborTile(neighborThing, (direction + 1) % 4);
                    pair<int, int> farNeighbor = neighborTile(neighborThing, direction);
                    pair<int, int> farLeftNeighbor = neighborTile(leftNeighbor, direction);
                    
                    cout<<neighborThing.first<<" "<<neighborThing.second<<"    "<<leftNeighbor.first<<" "<<leftNeighbor.second<<"    "<<farNeighbor.first<<" "<<farNeighbor.second<<"    "<<farLeftNeighbor.first<<" "<<farLeftNeighbor.second<<endl;
                    
                    board[neighborThing.first][neighborThing.second].visited = true;
                    board[neighborThing.first][neighborThing.second].isHole = true;
                    board[leftNeighbor.first][leftNeighbor.second].visited = true;
                    board[leftNeighbor.first][leftNeighbor.second].isHole = true;
                    board[farNeighbor.first][farNeighbor.second].visited = true;
                    board[farNeighbor.first][farNeighbor.second].isHole = true;
                    board[farLeftNeighbor.first][farLeftNeighbor.second].visited = true;
                    board[farLeftNeighbor.first][farLeftNeighbor.second].isHole = true;
                }
            }
        }

        //find new path
        else {
            if (!board[loc.first][loc.second].visited) {
                wallScan(timeStep);
                #ifdef DEBUGSTUFF
                debugStuff();
                #endif
            }
            if (steps.empty()) {
                if (!ending) {
                    pair<int, int> tempPair = runBFS(loc);
                    cout << "Finished BFS!" <<tempPair.first<<" "<<tempPair.second<< endl;
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
                    cout<<"send exit signal"<<endl;
                    
                    doMapCSV();
                    
                    char exitMessage='E';
                    emitter->send(&exitMessage,1);
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
                    targetAngle = roundAngle(getAngle());
                    if (direction == 0 || direction == 2) startOfMoveGPS = gps->getValues()[2];
                    else startOfMoveGPS = gps->getValues()[0];
                    cout << "Target angle: " << targetAngle << endl;
                    setMotors(min(motorMax(), motorSpeed), min(motorMax(), motorSpeed));
                }
                //printBoard();
                direction = getDirVal;

            }
        }
    }
    #ifdef DEBUGSTUFF
    debugStuff(true);
    #endif
    for(int i = 0; i < 50; i++)
    {
      doTimeStep();
    }
}