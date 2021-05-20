//environment.h: scanning for walls, holes, and victims

const double coordToGPS = 0.06; //the gps measurement that corresponds to one tile
pair<int, int> getCoords() {
    //use GPS coordinates to get integer coordinates of the robot
    pair<double, double> diff;
    pair<int, int> newCoords = startingCoord;
    diff.first = gps->getValues()[0] - startingGPS.first;
    diff.second = gps->getValues()[2] - startingGPS.second;
    newCoords.first += round(diff.first / coordToGPS);
    newCoords.second -= round(diff.second / coordToGPS);
    //cout << "At coords: " << newCoords.first << " " << newCoords.second << ", direction: " << direction <<  endl;
    return newCoords;
}

double getDeltaTarget(pair<int, int> targetCoordinates) {
    //find the maximum GPS distance in a cardinal direction from current location to a certain coordinate
    //for example, if the function is called from (0,0) with target (-100, 1), it'll return 100 converted into GPS distance
    pair<double, double> targetGPS;
    targetGPS.first = startingGPS.first + (targetCoordinates.first - startingCoord.first) * coordToGPS;
    targetGPS.second = startingGPS.second - (targetCoordinates.second - startingCoord.second) * coordToGPS;
    return max(fabs(targetGPS.first - gps->getValues()[0]), fabs(targetGPS.second - gps->getValues()[2]));
}

pair<double, double> moveStart; //where the move started - updated in main()

double getDelta() {
    //distance moved in a cardinal direction from where the move began
    return max(fabs(moveStart.first - gps->getValues()[0]), fabs(moveStart.second - gps->getValues()[2]));
}

double wallThreshold = 0.2;
double paraThreshold = 0.1;
int scanningMax = 30;

void wallScan(int timeStep){
    double startScan = getAngle();
    boardLoc(loc).visited = true;
    boardLoc(loc).open[0] = boardLoc(loc).open[1] = boardLoc(topLeft(loc)).open[1] = boardLoc(topLeft(loc)).open[2] = true;
    boardLoc(topRight(loc)).open[2] = boardLoc(topRight(loc)).open[3] = boardLoc(bottomRight(loc)).open[3] = boardLoc(bottomRight(loc)).open[0] = true;
    vector<pair<int, int>> psDirections {loc, topLeft(loc), topRight(loc), bottomRight(loc)};
    rotate(psDirections.begin(), psDirections.begin() + direction, psDirections.end());
    cout << "Direction: " << direction << endl;
    cout << infrared[0]->getValue() << ' ' << infrared[1]->getValue() << ' ' << infrared[2]->getValue() << ' ' << infrared[4]->getValue() << ' ' << infrared[5]->getValue() << ' ' << infrared[6]->getValue() << endl;
    int ctr = 0;
    
    int doRight = 0, doLeft = 0, doCenter = 0;
    if (infrared[0]->getValue() > wallThreshold) {
        boardLoc(psDirections[0]).open[(direction + 3) % 4] = true;
        pair<int, int> neighbor = neighborTile(psDirections[0], (direction + 3) % 4);
        board[neighbor.first][neighbor.second].open[(direction + 1) % 4] = true;
        doLeft++;
    }
    if (infrared[1]->getValue() > wallThreshold) {
        boardLoc(psDirections[1]).open[(direction + 3) % 4] = true;
        pair<int, int> neighbor = neighborTile(psDirections[1], (direction + 3) % 4);
        board[neighbor.first][neighbor.second].open[(direction + 1) % 4] = true;
        doLeft++;
    }
    if (infrared[2]->getValue() > wallThreshold) {
        boardLoc(psDirections[1]).open[direction % 4] = true;
        pair<int, int> neighbor = neighborTile(psDirections[1], direction % 4);
        board[neighbor.first][neighbor.second].open[(direction + 2) % 4] = true;
        doCenter++;
    }
    if (infrared[4]->getValue() > wallThreshold) {
        boardLoc(psDirections[2]).open[direction % 4] = true;
        pair<int, int> neighbor = neighborTile(psDirections[2], direction % 4);
        board[neighbor.first][neighbor.second].open[(direction + 2) % 4] = true;
        doCenter++;
    }
    if (infrared[5]->getValue() > wallThreshold) {
        boardLoc(psDirections[2]).open[(direction + 1) % 4] = true;
        pair<int, int> neighbor = neighborTile(psDirections[2], (direction + 1) % 4);
        board[neighbor.first][neighbor.second].open[(direction + 3) % 4] = true;
        doRight++;
    }
    if (infrared[6]->getValue() > wallThreshold) {
        boardLoc(psDirections[3]).open[(direction + 1) % 4] = true;
        pair<int, int> neighbor = neighborTile(psDirections[3], (direction + 1) % 4);
        board[neighbor.first][neighbor.second].open[(direction + 3) % 4] = true;
        doRight++;
    }
    
    if (doLeft < 2 && doCenter < 2 && doRight < 2){
        return;
    }
    bool leftSide = false, frontSide = false, rightSide = false;
    setMotors(-3, 3);
    while (robot->step(timeStep) != -1 && ctr < scanningMax) {
        updateGyro(timeStep);
        if (infrared[1]->getValue() < paraThreshold) leftSide = true;
        if (infrared[6]->getValue() < paraThreshold) rightSide = true;
        if (infrared[4]->getValue() < paraThreshold) frontSide = true;
        //cout << "Left:" << infrared[1]->getValue() << "Right:" << infrared[6]->getValue() << "Front:" << infrared[4]->getValue() << endl;
        ctr++;
    }
    setMotors(3, -3);
    while (robot->step(timeStep) != -1 && (fabs(getAngle() - startScan) > 1.5 )) {updateGyro(timeStep);}
    setMotors(0, 0);
    
    
    if (!leftSide && doLeft == 2){
        cout << "Left!" << endl;
        pair<int, int> neighbor = neighborTile(psDirections[0], (direction + 3) % 4); 
        board[neighbor.first][neighbor.second].open[direction] = true;
        pair<int, int> nNeighbor = neighborTile(neighbor, direction);
        board[nNeighbor.first][nNeighbor.second].open[(direction+2) % 4] = true;
    }
    if (!rightSide && doRight == 2){
        cout << "Right!" << endl;
        pair<int, int> neighbor = neighborTile(psDirections[3], (direction + 1) % 4); 
        board[neighbor.first][neighbor.second].open[direction] = true;
        pair<int, int> nNeighbor = neighborTile(neighbor, direction);
        board[nNeighbor.first][nNeighbor.second].open[(direction+2) % 4] = true;
    }if (!frontSide && doCenter == 2){
        cout << "Center!" << endl;
        pair<int, int> neighbor = neighborTile(psDirections[1], (direction) % 4); 
        board[neighbor.first][neighbor.second].open[(direction + 1) % 4] = true;
        pair<int, int> nNeighbor = neighborTile(neighbor, (direction + 1) % 4);
        board[nNeighbor.first][nNeighbor.second].open[(direction+3) % 4] = true;
    }
}

double motorMax(){
    return min(leftMotor->getMaxVelocity(), rightMotor->getMaxVelocity());
}

bool checkAllVictims(){return false;}

string tileTypeStr;
int getTileType(const unsigned char* img)
{ //0normal 1checkpoint 2pit 3red 4blue 5purple

    int r=Camera::imageGetRed(img,1,0,0);
    int g=Camera::imageGetGreen(img,1,0,0);
    int b=Camera::imageGetBlue(img,1,0,0);
  
    const char* str;
    int ret;
    if(r>230 && g>230 && b>230)
    {
      str="checkpoint";
      ret=1;
    }
    else if(r<100 && g<100 && b<100)
    {
      str="pit";
      cout << "Found a hole!" << endl;
      ret=2;
    }
    else if(r>190 && g<100 && b<100)
    {
      str="red";
      ret=3;
    }
    else if(r<100 && g<100 && b>190)
    {
      str="blue";
      ret=4;
    }
    else if(r>80 && g<90 && b>130)
    {
      str="purple";
      ret=5;
    }
    else
    {
      str="normal";
      ret=0;
    }
    //(void)str;//this silences the unused variable warning
    tileTypeStr=to_string(r)+" "+to_string(g)+" "+to_string(b)+"    "+str;
    //cout<<r<<" "<<g<<" "<<b<<"    "<<str<<endl;
    return ret;
}

bool checkHole(){
    return getTileType(colorCam->getImage()) == 2;
}