//victims.h: identify victims, both heat and visual

void doVictimOffset(int& posX, int& posZ, bool leftSide, bool heat) {
    //apply offset to the broadcast of the direction (basically, if you just broadcast the robot's position,
    //it says you're too far away)
    int currentDir = compassDirection();

    if (heat){
        if (currentDir == 0) posZ -= 2;
        if (currentDir == 1) posX += 2;
        if (currentDir == 2) posZ += 2;
        if (currentDir == 3) posX -= 2;
    

        if (leftSide) {
            if (currentDir == 0) posX -= (100 * coordToGPS) / 2;
            if (currentDir == 1) posZ -= (100 * coordToGPS) / 2;
            if (currentDir == 2) posX += (100 * coordToGPS) / 2;
            if (currentDir == 3) posZ += (100 * coordToGPS) / 2;
        }
    
        else {
            if (currentDir == 0) posX += (100 * coordToGPS) / 2;
            if (currentDir == 1) posZ += (100 * coordToGPS) / 2;
            if (currentDir == 2) posX -= (100 * coordToGPS) / 2;
            if (currentDir == 3) posZ -= (100 * coordToGPS) / 2;
        }
    }
}

const int robotID = 1;
const int heatThreshold = 35;
bool checkHeatVictim() {
    //check if there's a heat victim, either on the right or on the left
    if ( (leftHeat->getValue() > heatThreshold || rightHeat->getValue() > heatThreshold)) {
        int PosX = gps->getValues()[0] * 100;
        int PosZ = gps->getValues()[2] * 100;
        doVictimOffset(PosX, PosZ, leftHeat->getValue() > heatThreshold, true);
        cout << "Heat Victim at GPS Values: " << PosX << " " << PosZ << endl;
        changeMessage(PosX, PosZ, 't');
        return true;
    }
    return false;
}

int thresh = 190;
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
int display_caption(const char* caption);
int display_dst(int delay);

bool checkVisualVictim(int camNum)
{
    Camera *cam;
    if (camNum == 0){ cam = camC; cout << "center: " << endl;}
    else if (camNum == 1){ cam = camL; cout << "left: " << endl;}
    else if (camNum == 2){ cam = camR; cout << "right: " << endl;}
    else if (camNum == 3){ cam = camLFront; cout << "left front: " << endl;}
    else if (camNum == 4){ cam = camRFront; cout << "right front: " << endl;}
    else return false;
    Mat src(cam->getHeight(), cam->getWidth(), CV_8UC4, (void*)cam->getImage());;
    
    if( src.empty() )
    {
      cout << "Could not open or find the image!\n" << endl;
      return false;
    }
    Mat drawing = src.clone();
    cvtColor(src,src,COLOR_BGR2GRAY); //grayscale
    
    blur( src, src, Size(3,3) );
    threshold(src,src,thresh,max_thresh,THRESH_BINARY); //threshold

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
    imshow( "Contours", drawing );


    for( size_t i = 0; i< contours.size(); i++ )
    {
      Scalar color = Scalar( 0, 0, 255);
      drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
      Rect roi = boundingRect(contours[i]);
      
      double width = (double)roi.width;
      double height = (double)roi.height;
      
      printf("width: %f, height: %f\n", width, height);
      printf("width/height: %f\n", width/height);
      double area = width*height/3;
      if(width < 90 && width > 45 && height < 90 && height > 45 && width/height < 1.1  && width/height > 0.9)
      {
        
        rectangle(src,roi, color,1);
        Mat crop = src(roi);
        imshow("crop", crop);
        Rect slicet(roi.x, roi.y, roi.width, roi.height/3);
        Mat slice1 = src(slicet);
        imshow("slice1", slice1);
        Rect slicem(roi.x, roi.y+(roi.height/3), roi.width, roi.height/3);
        Mat slice2 = src(slicem);
        imshow("slice2", slice2);
        Rect sliceb(roi.x, roi.y+(roi.height*2/3), roi.width, roi.height/3);
        Mat slice3 = src(sliceb);
        imshow("slice3", slice3);
        Vec3b t;
        Vec3b m;
        Vec3b b;

        double top = 0;
        double mid = 0;
        double bottom = 0;
        for(int y = 0; y < height; y++)
        {
          if(y<height/3){
            for(int x = 0; x < width; x++)
            {
              t = crop.at<Vec3b>(Point(x, y));
              if(t.val[0] < 200)
                top+=(1/area);
            }
          }
          else if(y<height*2/3){
            for(int x = 0; x < roi.width; x++)
            {
              m = crop.at<Vec3b>(Point(x, y));
              if(m.val[0] < 200)
                mid+=(1/area);
            }
          }
          else if(y<height){
            for(int x = 0; x < width; x++)
            {
              b = crop.at<Vec3b>(Point(x, y));
              if(b.val[0] < 200)
                bottom+=(1/area); 
            }
          } 
        }
        printf("top: %f, mid: %f, bottom: %f\n", top, mid, bottom);

        int PosX = gps->getValues()[0]*100;
        int PosZ = gps->getValues()[2]*100;
        if(top > 0.43 && top < 0.6 && mid > 0.43 && mid < 0.7 && bottom > 0.43 && bottom < 0.6)//exclude noisy info
        {
        
          if(mid-top >= 0.06 && mid-bottom >= 0.06){//If mid is darker than top and bottom
            changeMessage(PosX, PosZ, 'H');
            printf("H victim on camera %d\n", camNum);
            return true;
          }
          if(bottom-top >= 0.04 && mid-top >= 0.015 && mid-top < 0.05){//If bottom is darker than top and mid
            changeMessage(PosX, PosZ, 'U');
            printf("U victim on camera %d\n", camNum);
            return true;
          }
          if(mid-top < 0.03 || (bottom-mid < 0.03 && bottom - top < 0.04)){
           changeMessage(PosX, PosZ, 'S');
            printf("S victim on camera %d\n", camNum);
            return true;
          }
        }
      }
    }

    return false;
}
