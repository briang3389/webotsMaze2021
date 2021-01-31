//navigation.h: a collection of helper functions for navigating the map
//also hole because why not

const double coordToGPS = 0.12; //the gps measurement that corresponds to one tile
pair<int, int> getCoords() {
    //use GPS coordinates to get integer coordinates of the robot
    pair<double, double> diff;
    pair<int, int> newCoords = startingCoord;
    diff.first = gps->getValues()[0] - startingGPS.first;
    diff.second = gps->getValues()[2] - startingGPS.second;
    newCoords.first += round(diff.first / coordToGPS);
    newCoords.second -= round(diff.second / coordToGPS);
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
void scan(pair<int, int>& tile) {
    //scan for walls (does not scan backwards), marks node as visited
    board[tile.first][tile.second].visited = true;
    if (infrared[0]->getValue() > wallThreshold) {
        board[tile.first][tile.second].open[direction % 4] = true;
        pair<int, int> neighbor = neighborTile(tile, direction % 4);
        board[neighbor.first][neighbor.second].open[(direction + 2) % 4] = true;
    }
    if (infrared[2]->getValue() > wallThreshold) {
        board[tile.first][tile.second].open[(direction + 1) % 4] = true;
        pair<int, int> neighbor = neighborTile(tile, (direction + 1) % 4);
        board[neighbor.first][neighbor.second].open[(direction + 3) % 4] = true;
    }
    if (infrared[5]->getValue() > wallThreshold) {
        board[tile.first][tile.second].open[(direction + 3) % 4] = true;
        pair<int, int> neighbor = neighborTile(tile, (direction + 3) % 4);
        board[neighbor.first][neighbor.second].open[(direction + 1) % 4] = true;
    }
}

bool checkHole() {
    //returns true if there's a hole
    const unsigned char* img = colorCam->getImage();
    if (img[1] < 65) { 
        return true;
    }
    return false;
}
