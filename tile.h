//tile.h: deals with the tile data structure

struct Tile {
    bool open[4]; //whether there's a wall on each of the sides
    bool visited; //whether the robot has been to the tile
    bool isHole; //whether the tile is a hole
    bool victimChecked; //whether there was a  victim already seen on the tile
};

const int boardSize = 100; //size of board (should be double the maxSize)
Tile board[boardSize][boardSize]; 
pair<int, int> loc{ boardSize / 2, boardSize / 2 }; //current location of the robot
//GPS Locations refer to the bottom left corner of the robot (lower x and y)
const pair<int, int> startingCoord = loc; //where the robot started
pair<double, double> startingGPS; //GPS location of startingGPS member

void printBoard() {
    //prints the board to the webots console
    for (int i = boardSize - 1; i >= 0; i--) {
        for (int j = 0; j < boardSize; j++) {
            if (!board[j][i].open[2]) cout << "_";
            else cout << " ";
            if (!board[j][i].open[1]) cout << "|";
            else cout << " ";
        }
        cout << endl;
    }
    cout << "Printing complete!" << endl;
}

pair<int, int> neighborTile(pair<int, int> tile, int dir) {
    //returns the coordinates of a tile in a given direction
    if (dir == 0) return make_pair(tile.first, tile.second + 1);
    if (dir == 1) return make_pair(tile.first + 1, tile.second);
    if (dir == 2) return make_pair(tile.first, tile.second - 1);
    if (dir == 3) return make_pair(tile.first - 1, tile.second);
    return make_pair(-1, -1);
}

int getDirection(pair<int, int> first, pair<int, int> second) {
    //returns the direction that a second tile is relative to the first (0-3, 10 for same tile, -1 for too far away)
    for (int i = 0; i < 4; i++) {
        if (neighborTile(first, i) == second) return i;
    }
    if (first == second) return 10;
    return -1;
}

Tile& boardLoc(pair<int, int> tileCoords){
    return board[tileCoords.first][tileCoords.second];
}

pair<int, int> topRight(pair<int, int> &startTile){
    return make_pair(startTile.first + 1, startTile.second + 1);
}

pair<int, int> topLeft(pair<int, int> &startTile){
    return make_pair(startTile.first, startTile.second + 1);
}

pair<int, int> bottomRight(pair<int, int> &startTile){
    return make_pair(startTile.first + 1, startTile.second);
}

bool traversible(pair<int, int> &startTile){
    return (!boardLoc(startTile).isHole && !boardLoc(topLeft(startTile)).isHole && !boardLoc(topRight(startTile)).isHole && !boardLoc(bottomRight(startTile)).isHole && boardLoc(startTile).open[0] && boardLoc(startTile).open[1] && boardLoc(topRight(startTile)).open[2] && boardLoc(topRight(startTile)).open[3]);
}


const Tile defaultTile=board[0][0];

bool tilesEqual(Tile t1, Tile t2)
{
  return t1.open[0]==t2.open[0] && t1.open[1]==t2.open[1] && t1.open[2]==t2.open[2] && t1.open[3]==t2.open[3] && t1.visited==t2.visited && t1.isHole==t2.isHole && t1.victimChecked==t2.victimChecked;
}

bool isBlankCol(int x)
{
  for(int tmpy=0;tmpy<boardSize;tmpy++)
  {
    if(!tilesEqual(board[x][tmpy],defaultTile))return false;
  }
  return true;
}
bool isBlankRow(int y)
{
  for(int tmpx=0;tmpx<boardSize;tmpx++)
  {
    if(!tilesEqual(board[tmpx][y],defaultTile))return false;
  }
  return true;
}

char specialTiles[boardSize][boardSize];
char& specialTilesLoc(pair<int, int> tileCoords){
    return specialTiles[tileCoords.first][tileCoords.second];
}
void fullSpecialTileSet(pair<int, int> tileCoords, char val)
{
  specialTiles[tileCoords.first][tileCoords.second]=val;
  specialTiles[tileCoords.first+1][tileCoords.second]=val;
  specialTiles[tileCoords.first][tileCoords.second+1]=val;
  specialTiles[tileCoords.first+1][tileCoords.second+1]=val;
}

void doMapCSV()
{
  //coords in this are like math coordinates
  
  int startX=0;
  while(isBlankCol(startX))startX++;
  int endX=startX;
  while(!isBlankCol(endX))endX++;
  endX--;
  
  int startY=0;
  while(isBlankRow(startY))startY++;
  int endY=startY;
  while(!isBlankRow(endY))endY++;
  endY--;
  
  const int csvWidth=(endX-startX+1)*2+1;
  const int csvHeight=(endY-startY+1)*2+1;
  //cout<<startX<<" "<<endX<<" "<<csvWidth<<"    "<<startY<<" "<<endY<<" "<<csvHeight<<endl;
  vector<vector<string>> csvMap(csvWidth,vector<string>(csvHeight,string("")));
  
  for(int x=startX,csvX=1;x<=endX;x++,csvX+=2)
  {
    for(int y=startY,csvY=1;y<=endY;y++,csvY+=2)
    {
      //cout<<x<<" "<<y<<"    "<<csvX<<" "<<csvY<<endl;
      /*
      if(!board[x][y].visited)
      {
        csvMap[csvX][csvY]="0";
        //continue;
      }
      */
      if((x==startingCoord.first && y==startingCoord.second) || (x==startingCoord.first+1 && y==startingCoord.second) || (x==startingCoord.first && y==startingCoord.second+1) || (x==startingCoord.first+1 && y==startingCoord.second+1))
      {
        cout<<x<<" "<<y<<"    "<<csvX<<" "<<csvY<<endl;
        csvMap.at(csvX).at(csvY)="5";
      }
      else if(board[x][y].isHole)
      {
        csvMap[csvX][csvY]="2";
        //continue;
      }
      else if(specialTiles[x][y]!=0)
      {
        char tmpchararray[2]={0};
        tmpchararray[0]=specialTiles[x][y];
        csvMap.at(csvX).at(csvY)=string(tmpchararray);//to_string(specialTiles[x][y]);
      }
      else
      {
        csvMap.at(csvX).at(csvY)="0";
      }
      
      //NEED TO KNOW WHERE VICTIMS FOUND
      csvMap.at(csvX).at(csvY+1)=board[x][y].open[0]?"0":"1";//temporary just assume no victims on walls
      csvMap.at(csvX+1).at(csvY)=board[x][y].open[1]?"0":"1";
      csvMap.at(csvX).at(csvY-1)=board[x][y].open[2]?"0":"1";
      csvMap.at(csvX-1).at(csvY)=board[x][y].open[3]?"0":"1";
    }
  }
  
  for(int x=0;x<csvWidth;x++)
  {
    for(int y=0;y<csvHeight;y++)
    {
      if(csvMap[x][y]=="")csvMap[x][y]="0";//maybe this should be 1
    }
  }
  
  for(int x=0;x<csvWidth;x+=2) //verticy is 1 if connected to a wall
  {
    for(int y=0;y<csvHeight;y+=2)
    {
      if((y+1<csvHeight&&csvMap[x][y+1]!="0") || (x+1<csvWidth&&csvMap[x+1][y]!="0") || (y-1>=0&&csvMap[x][y-1]!="0") || (x-1>=0&&csvMap[x-1][y]!="0"))
      {
        csvMap[x][y]="1";
      }
      else
      {
        csvMap[x][y]="0";
      }
    }
  }
  
  

  for(int i = csvHeight-1; i >= 0; i--) {
    for(int j = 0; j < csvWidth; j++) {
      cout<<csvMap[j][i]<<" ";
    }
    cout<<endl;
  }
  /*cout<<endl<<endl;
  for(int i = 99; i >= 0; i--) {
    for(int j = 0; j < 100; j++) {
      cout<<(specialTiles[j][i]?specialTiles[j][i]:'0')<<" ";
    }
    cout<<endl;
  }*/
  
  string flattened = "";
  for(int i = 0; i < csvWidth; i++) {
    for(int j = csvHeight-1; j >= 0; j--) {
      flattened += csvMap[i][j] + ",";
    }
  }
  flattened.pop_back();
  char *message=(char*)malloc(8 + flattened.size());
  memcpy(message, &csvWidth, sizeof(csvWidth));
  memcpy(&message[4], &csvHeight, sizeof(csvHeight));
  memcpy(&message[8], flattened.c_str(), flattened.size());
  emitter->send(message, 8 + flattened.size()); // Send map data
  char msg = 'M'; // Send map evaluate request
  emitter->send(&msg, sizeof(msg));
}