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

void orient(int timeStep){
    setMotors(1, 1);
    for (int i = 0; i < 5; i++) robot->step(timeStep);
    
    double xValue= fabs(gps->getValues()[0] - startingGPS.first);
    double yValue = fabs(gps->getValues()[2] - startingGPS.second);
    
    if (xValue > yValue){
        if (gps->getValues()[0] > startingGPS.first) direction = 1;
        else direction = 3;
    }
    else{
        if (gps->getValues()[2] > startingGPS.second) direction = 2;
        else direction = 0;
    }
    angle = 1.5708 * direction;
    setMotors(-1, -1);
    for (int i = 0; i < 5; i++) robot->step(timeStep);
    setMotors(0, 0);
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
double paraThreshold = 0.15; //0.1
int scanningMax = 20;

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
    while (doTimeStep() != -1 && ctr < scanningMax) {
        
        if (infrared[1]->getValue() < paraThreshold) leftSide = true;
        if (infrared[6]->getValue() < paraThreshold) rightSide = true;
        if (infrared[4]->getValue() < paraThreshold) frontSide = true;
        //cout << "Left:" << infrared[1]->getValue() << "Right:" << infrared[6]->getValue() << "Front:" << infrared[4]->getValue() << endl;
        ctr++;
    }
    setMotors(3, -3);
    //change 1.5 to 3 to fix spin issue
    while (doTimeStep() != -1 && (fabs(getAngle() - startScan) > 3 )) {;}
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

const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";

int low_H = 150, low_S = 60, low_V = 60;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int low_Hy = 0;
int high_Hy = 40;
int PosX;
int PosZ;
  
int thresh = 192;
const int max_thresh = 255;

//blur
int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;
char window_name[] = "Smoothing Demo";
int display_caption( const char* caption );
int display_dst( int delay );
char getLetter(double Values[3])
{
    //{top, mid, bottom}
    double Data[6][3] = {{0.159763, 0.230769, 0.191321}, //Hdata top: 0.159763, mid: 0.230769, bot: 0.191321
                        {0.230769, 0.333333, 0.349112},  //Udata top: 0.177515, mid: 0.252465, bot: 0.094675
                        {0.177515, 0.252465, 0.094675},  //Udata2 top: 0.177515, mid: 0.252465, bot: 0.094675
                        {0.112426, 0.106509, 0.161736},  //Sdata top: 0.112426, mid: 0.106509, bot: 0.161736
                        {0.595661, 0.439625, 0.651854},  //C top: 0.595661, mid: 0.169625, bot: 0.581854
                        {0.558974, 0.208521, 0.558560}}; //P top: 0.408974, mid: 0.248521, bot: 0.408560
   
    //{mid-top, mid-bottom, bottom-top}
    double diffs[6][3];
    for(int n = 0; n < 6; n++)
    {
        diffs[n][0] = 10*(Data[n][1] - Data[n][0]);
        diffs[n][1] = 10*(Data[n][1] - Data[n][2]);
        diffs[n][2] = 10*(Data[n][2] - Data[n][0]);
    }
    double valDiffs[3] = {10*(Values[1] - Values[0]), 10*(Values[1] - Values[2]), 10*(Values[2] - Values[0])};
//    printf("valDiffs %f, %f, %f\n", valDiffs[0], valDiffs[1], valDiffs[2]);
    
    double dist[6] = { 0 };
    for(int n = 0; n < 6; n++)
    {
        for(int i = 0; i < 3; i++)
        {
            dist[n] += (Values[i]-Data[n][i])*(Values[i]-Data[n][i]);
        }
//        dist[n] = sqrt(dist[n]);
    }
    for(int n = 0; n < 6; n++)
    {
        for(int i = 0; i < 3; i++)
        {
            dist[n] += 2*(valDiffs[i]-diffs[n][i])*(valDiffs[i]-diffs[n][i]);
        }
        dist[n] = sqrt(dist[n]);
    }
    int letter = 0;
    if(dist[1] < dist[letter])
        letter = 1;
    if(dist[2] < dist[letter])
        letter = 2;
    if(dist[3] < dist[letter])
        letter = 3;
    if(dist[4] < dist[letter])
        letter = 4;
    if(dist[5] < dist[letter])
        letter = 4;
    if(letter == 0){//Closest to H    
        printf("H victim\n");
        return 'H';
    }
    else if(letter == 1 || letter == 2){//Closest to U 
        printf("U victim\n");
        return 'U';
    }
    else if(letter == 3){//Closest to S
        printf("S victim\n");
        return 'S';
    }
    else if(letter == 4){//Closest to C
        printf("C victim\n");
        return 'C';
    }
    else{//Closest to P
        printf("P victim\n");
        return 'P';
    }
}

bool checkVisualVictim(Camera* cam)
{   
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    Mat frame_HSV, frame_red, frame_yellow;
    Mat frame(cam->getHeight(), cam->getWidth(), CV_8UC4, (void*)cam->getImage());;
    
    if( frame.empty() )
    {
      cout << "Could not open or find the image!\n" << endl;
      return false;
    }
    Mat clone = frame.clone();
    Mat drawing = frame.clone();
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_red);
        inRange(frame_HSV, Scalar(low_Hy, low_S, low_V), Scalar(high_Hy, high_S, high_V), frame_yellow);
        // Show the frames
        imshow(window_capture_name, frame);
        imshow("red", frame_red);
        imshow("yellow", frame_yellow);
    waitKey(1);
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( frame_red, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
    if(contours.size() == 0) // if see no red
    {
      cvtColor(frame,clone,COLOR_BGR2GRAY); //grayscale
      
      //blur( clone, clone, Size(3,3) );
//      Mat inverted;
      bitwise_not ( clone, clone );
      GaussianBlur(clone, clone, cv::Size(0, 0), 0.5);
      addWeighted(clone, 1.5, clone, -0.5, 0, clone);
      imshow("blured", clone);
      bitwise_not ( clone, clone );

      threshold(clone,clone,thresh,max_thresh,THRESH_BINARY); //threshold
      imshow("thresh", clone);
      Mat canny_output;
      vector<vector<Point>> contours;
      vector<vector<Point>> invContours;
      vector<Vec4i> hierarchy;
      
      findContours( clone, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
      for( size_t i = 0; i < contours.size(); i++ )
      {
        Scalar color = Scalar( 0, 255, 255);
  
        drawContours( clone, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
        Rect roi = boundingRect(contours[i]);
        imshow("conts", clone);
        double width = (double)roi.width;
        double height = (double)roi.height;
        
        // printf("width: %f, height: %f\n", width, height);
        // printf("width/height: %f\n", width/height);

        if(width < 120 && width > 25 && height < 120 && height > 25 && width/height < 1.2  &&  width/height > 0.66)
        {
          double area = 39*13;
          rectangle(frame, roi, color,1);
          Mat crop = frame(roi);
          Mat reCrop;
          resize(crop, reCrop, Size(), 45.0*3/width, 45.0/height);
          reCrop = reCrop(Rect(9, 3, 39*3, 39));
          imshow("reCrop", reCrop);
          Rect slicet(0, 0, 39*3, 13);
          Mat slice1 = reCrop(slicet);
          imshow("slice1", slice1);
          Rect slicemh(0, 13, 39*3, 13);
          Mat slice2 = reCrop(slicemh);
          imshow("slice2", slice2);
          Rect sliceb(0, 26, 39*3, 13);
          Mat slice3 = reCrop(sliceb);
          imshow("slice3", slice3);
          
          Vec3b t;
          Vec3b m;
          Vec3b b;
          double top = 0;
          double mid = 0;
          double bottom = 0;
          int valThresh = 140;
          for(int y = 0; y < 39; y++)//get horizontal slices
          {
            if(y<13){
              for(int x = 0; x < 39; x++)
              {
                t = reCrop.at<Vec3b>(Point(x, y));
                if(t.val[0] < valThresh){
                  top+=(1/area);
                }
              }
            }
            else if(y<26){
              for(int x = 0; x < 39; x++)
              {
                m = reCrop.at<Vec3b>(Point(x, y));
                if(m.val[0] < valThresh){
                  mid+=(1/area);
                }
              }
            }
            else if(y<39){
              for(int x = 0; x < 39; x++)
              {
                b = reCrop.at<Vec3b>(Point(x, y));
                if(b.val[0] < valThresh){
                  bottom+=(1/area);
                }
              }
            } 
          }
          printf("top: %f, mid: %f, bot: %f\n", top, mid, bottom);
          
          PosX = gps->getValues()[0]*100;
          PosZ = gps->getValues()[2]*100;
         if(top > 0 && top < 0.9 && mid > 0 && mid < 0.8 && bottom > 0 && bottom < 0.9)//exclude noisy info
          {
              double img[3] = {top, mid, bottom};
              changeMessage(PosX, PosZ, getLetter(img));
              return true;
          }
        }
      }
    }
    else{// yes red
      for( size_t i = 0; i < contours.size(); i++ )
      {
        Scalar color = Scalar( 0, 0, 255);
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
        Rect roi = boundingRect(contours[i]);
        
        double width = (double)roi.width;
        double height = (double)roi.height;
        
        if(width > 30 && width < 100 && height > 15 && height < 100 && (width/height > 0.9 && width/height < 2.1))
        {
          PosX = gps->getValues()[0]*100;
          PosZ = gps->getValues()[2]*100;
        
          findContours( frame_yellow, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
          if(contours.size() == 0){ //if no yellow
              changeMessage(PosX, PosZ, 'F');
          }
          else{
              changeMessage(PosX, PosZ, 'O');
          }
          return true;
        }
      }
    }
    return false;
}

bool checkAllVictims(){return false;}

string tileTypeStr;
int getTileType(const unsigned char* img)
{ //0normal 1checkpoint 2pit 3red 4blue 5purple 6swamp

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
    else if(r>170&&r<178 && g>136&&g<144 && b>74&&b<82) //+-4 from 174 140 78
    {
      str="swamp";
      ret=6;
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