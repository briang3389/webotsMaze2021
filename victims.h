//victims.h: identify victims, both heat and visual

void doVictimOffset(int &posX, int &posZ, bool leftSide, bool heat)
{
  //apply offset to the broadcast of the direction (basically, if you just broadcast the robot's position,
  //it says you're too far away)
  int currentDir = compassDirection();

  if(heat)
  {
    if(currentDir == 0) posZ -= 2;
    if(currentDir == 1) posX += 2;
    if(currentDir == 2) posZ += 2;
    if(currentDir == 3) posX -= 2;

    if(leftSide)
    {
      if(currentDir == 0) posX -= (100 * coordToGPS) / 2;
      if(currentDir == 1) posZ -= (100 * coordToGPS) / 2;
      if(currentDir == 2) posX += (100 * coordToGPS) / 2;
      if(currentDir == 3) posZ += (100 * coordToGPS) / 2;
    }

    else
    {
      if(currentDir == 0) posX += (100 * coordToGPS) / 2;
      if(currentDir == 1) posZ += (100 * coordToGPS) / 2;
      if(currentDir == 2) posX -= (100 * coordToGPS) / 2;
      if(currentDir == 3) posZ -= (100 * coordToGPS) / 2;
    }
  }
}

const int robotID = 1;
const int heatThreshold = 35;
bool checkHeatVictim()
{
  //check if there's a heat victim, either on the right or on the left
  if((leftHeat->getValue() > heatThreshold || rightHeat->getValue() > heatThreshold))
  {
    int PosX = gps->getValues()[0] * 100;
    int PosZ = gps->getValues()[2] * 100;
    doVictimOffset(PosX, PosZ, leftHeat->getValue() > heatThreshold, true);
    cout << "Heat Victim at GPS Values: " << PosX << " " << PosZ << endl;
    changeMessage(PosX, PosZ, 't');
    return true;
  }
  return false;
}

int thresh = 170;
const int max_thresh = 255;
RNG rng(12345);
double top;
double mid;
double bottom;

//blur
int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;
char window_name[] = "Smoothing Demo";
int display_caption(const char *caption);
int display_dst(int delay);

bool checkVisualVictim(int cameraNum = 0)
{
  //check for visual victim in the current tile
  //camera 0 is center, 1 is left, 2 is right
  //probably needs an update, not sure though
  if(board[getCoords().first][getCoords().second].victimChecked) return false;

  int PosX = gps->getValues()[0] * 100;
  int PosZ = gps->getValues()[2] * 100;

  Camera *detectionCam = camC;
  if(cameraNum == 1) detectionCam = camL;
  if(cameraNum == 2) detectionCam = camR;
  Mat src(detectionCam->getHeight(), detectionCam->getWidth(), CV_8UC4, (void*) detectionCam->getImage());
  ;

  if(src.empty())
  {
    cout << "Could not open or find the image!\n" << endl;
    return false;
  }
  Mat drawing = src.clone();
  cvtColor(src, src, COLOR_BGR2GRAY); //grayscale

  blur(src, src, Size(3, 3));
  threshold(src, src, thresh, max_thresh, THRESH_BINARY); //threshold
  imshow("thresh", src);
  Mat canny_output;
  vector < vector<Point> > contours;
  vector < Vec4i > hierarchy;
  findContours(src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  imshow("Contours", drawing);

  for(size_t i = 0; i < contours.size(); i++)
  {
    Scalar color = Scalar(0, 0, 255);
    drawContours(drawing, contours, (int) i, color, 2, LINE_8, hierarchy, 0);
    Rect roi = boundingRect(contours[i]);

    int width = roi.width;
    int height = roi.height;
    double area = width * height / 3;
    if(roi.width < 130 && roi.width > 50 && roi.height < 130 && roi.height > 50 && roi.width / roi.height < 1.1 && roi.width / roi.height > 0.9)
    {
      cout << "Contour of ok size found" << endl;
      rectangle(src, roi, color, 1);
      Mat crop = src(roi);
      imshow("crop", crop);
      Rect slicet(roi.x, roi.y, roi.width, roi.height / 3);
      Mat slice1 = src(slicet);
      imshow("slice1", slice1);
      Rect slicem(roi.x, roi.y + (roi.height / 3), roi.width, roi.height / 3);
      Mat slice2 = src(slicem);
      imshow("slice2", slice2);
      Rect sliceb(roi.x, roi.y + (roi.height * 2 / 3), roi.width, roi.height / 3);
      Mat slice3 = src(sliceb);
      imshow("slice3", slice3);
      Vec3b t;
      Vec3b m;
      Vec3b b;

      top = mid = bottom = 0;
      for(int y = 0; y < roi.height; y++)
      {
        if(y < roi.height / 3)
        {
          for(int x = 0; x < roi.width; x++)
          {
            t = crop.at < Vec3b > (Point(x, y));
            if(t.val[0] < 200) top += (1 / area);
          }
        }
        else if(y < roi.height * 2 / 3)
        {
          for(int x = 0; x < roi.width; x++)
          {
            m = crop.at < Vec3b > (Point(x, y));
            if(m.val[0] < 200) mid += (1 / area);
          }
        }
        else if(y < roi.height)
        {
          for(int x = 0; x < roi.width; x++)
          {
            b = crop.at < Vec3b > (Point(x, y));
            if(b.val[0] < 200) bottom += (1 / area);
          }
        }
      }

      PosX = gps->getValues()[0] * 100;
      PosZ = gps->getValues()[2] * 100;
      doVictimOffset(PosX, PosZ, false, false);

      cout << "Visual Victim at GPS Values " << PosX << " " << PosZ << endl;
      if(bottom - top >= 0.08 && bottom - mid >= 0.03 && mid - top >= 0.03)
      { //If bottom is darker than top and mid
        changeMessage(PosX, PosZ, 'U');
        cout << "U victim\n" << endl;
        board[getCoords().first][getCoords().second].victimChecked = true;
        return true;
      }
      else if(mid - top >= 0.04 && mid - bottom >= 0.04)
      { //If mid is darker than top and bottom
        changeMessage(PosX, PosZ, 'H');
        cout << "H victim\n" << endl;
        board[getCoords().first][getCoords().second].victimChecked = true;
        return true;
      }
      else if(mid - top < 0.04 && bottom - mid < 0.04)
      {
        changeMessage(PosX, PosZ, 'S');
        cout << "S victim\n" << endl;
        board[getCoords().first][getCoords().second].victimChecked = true;
        return true;
      }

    }
  }
  waitKey(1);
  return false;
}

bool checkVisualVictimAhead(bool onLeft)
{
  //check for visual victims in the next tile ahead - should be removed eventually
  Camera *detectionCam = onLeft ? camL : camR;

  Mat src(detectionCam->getHeight(), detectionCam->getWidth(), CV_8UC4, (void*) detectionCam->getImage());
  ;

  if(src.empty())
  {
    cout << "Could not open or find the image!\n" << endl;
    //cout << "Usage: " << argv[0] << " <Input image>" << endl;
    //continue;
  }
  Mat drawing = src.clone();
  cvtColor(src, src, COLOR_BGR2GRAY); //grayscale

  blur(src, src, Size(3, 3));
  threshold(src, src, thresh, max_thresh, THRESH_BINARY); //threshold
  imshow("thresh", src);
  Mat canny_output;
  vector < vector<Point> > contours;
  vector < Vec4i > hierarchy;
  findContours(src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  imshow("Contours", drawing);

  for(size_t i = 0; i < contours.size(); i++)
  {
    Scalar color = Scalar(0, 0, 255);
    drawContours(drawing, contours, (int) i, color, 2, LINE_8, hierarchy, 0);
    Rect roi = boundingRect(contours[i]);

    double width = (double) roi.width;
    double height = (double) roi.height;

    //printf("width: %f, height: %f\n", width, height);
    //printf("width/height: %f\n", width/height);
    //double area = width*height/3;
    //check side walls
    if(roi.width < 35 && roi.width > 20 && roi.height < 45 && roi.height > 25 && width / height < 0.8 && width / height > 0.65)
    {
      //printf("go to next tile and turn\n");
      return true;
      //switch camera to centre camera    
    }
  }
  waitKey(1);
  return false;
}

