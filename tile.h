//tile.h: deals with the tile data structure

struct Tile
{
  bool open[4]; //whether there's a wall on each of the sides
  bool visited; //whether the robot has been to the tile
  bool isHole; //whether the tile is a hole
  bool hasVisualVictim; //used  for "tile ahead" visual victim; TODO: remove the tile ahead code
  bool victimChecked; //whether there was a  victim already seen on the tile
};

const int boardSize = 100; //size of board (should be double the maxSize)
Tile board[boardSize][boardSize];
pair<int, int> loc { boardSize / 2, boardSize / 2 }; //current location of the robot
int direction = 0; //0 is up, 1 is right, 2 is down, 3 is left
const pair<int, int> startingCoord = loc; //where the robot started
pair<double, double> startingGPS; //GPS location of startingGPS member

void printBoard()
{
  //prints the board to the webots console
  for(int i = boardSize - 1; i >= 0; i--)
  {
    for(int j = 0; j < boardSize; j++)
    {
      if(!board[j][i].open[2]) cout << "_";
      else cout << " ";
      if(!board[j][i].open[1]) cout << "|";
      else cout << " ";
    }
    cout << endl;
  }
  cout << "Printing complete!" << endl;
}

pair<int, int> neighborTile(pair<int, int> tile, int dir)
{
  //returns the coordinates of a tile in a given direction
  if(dir == 0) return make_pair(tile.first, tile.second + 1);
  if(dir == 1) return make_pair(tile.first + 1, tile.second);
  if(dir == 2) return make_pair(tile.first, tile.second - 1);
  if(dir == 3) return make_pair(tile.first - 1, tile.second);
  return make_pair(-1, -1);
}

int getDirection(pair<int, int> first, pair<int, int> second)
{
  //returns the direction that a second tile is relative to the first (0-3, 10 for same tile, -1 for too far away)
  for(int i = 0; i < 4; i++)
  {
    if(neighborTile(first, i) == second) return i;
  }
  if(first == second) return 10;
  return -1;
}
