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

int thresh = 150;
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

char getLetter(double Values[3])
{
    //{top, mid, bottom}
    double Data[3][3] = {{0.230769, 0.394477, 0.246548}, //Hdata top: 0.230769, mid: 0.394477, bottom: 0.246548
                        {0.268383, 0.280079, 0.337278},  //Udata top: 0.258383, mid: 0.280079, bottom: 0.337278
                        {0.332051, 0.328107, 0.363748}}; //Sdata top: 0.302051, mid: 0.278107, bottom: 0.343748
    
    //{mid-top, mid-bottom, bottom-top}
    double diffs[3][3];
    for(int n = 0; n < 3; n++)
    {
        diffs[n][0] = 10*(Data[n][1] - Data[n][0]);
        diffs[n][1] = 10*(Data[n][1] - Data[n][2]);
        diffs[n][2] = 10*(Data[n][2] - Data[n][0]);
    }
    double valDiffs[3] = {10*(Values[1] - Values[0]), 10*(Values[1] - Values[2]), 10*(Values[2] - Values[0])};
//    printf("valDiffs %f, %f, %f\n", valDiffs[0], valDiffs[1], valDiffs[2]);
    
    double dist[3] = { 0 };
    for(int n = 0; n < 3; n++)//compare values
    {
        for(int i = 0; i < 3; i++)
        {
            dist[n] += (Values[i]-Data[n][i])*(Values[i]-Data[n][i]);
        }
    }
    for(int n = 0; n < 3; n++)//compare diffs
    {
        for(int i = 0; i < 3; i++)
        {
            dist[n] += (valDiffs[i]-diffs[n][i])*(valDiffs[i]-diffs[n][i]);
        }
        dist[n] = sqrt(dist[n]);
    }
    int letter = 0;
    if(dist[1] < dist[letter])
        letter = 1;
    if(dist[2] < dist[letter])
        letter = 2;
    if(letter == 0){//Closest to H    
        printf("H victim\n");
        return 'H';
    }
    else if(letter == 1){//Closest to U 
        printf("U victim\n");
        return 'U';
    }
    else{//Closest to S
        printf("S victim\n");
        return 'S';
    }
}
bool checkVisualVictim(int camNum)
{
    Camera *cam;
    if (camNum == 0){ cam = camC; cout << "center"<< endl;}
    else if (camNum == 1){ cam = camL; cout << "left" << endl;}
    else if (camNum == 2){ cam = camR; cout << "right" << endl;}
//    else if (camNum == 3) cam = camLFront;
//    else if (camNum == 4) cam = camRFront;
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
      double area = 39*13;
      if(width < 80 && width > 33 && height < 80 && height > 33 && width/height < 1.08  && width/height > 0.94)
      {
        rectangle(src,roi, color,1);
        Mat crop = src(roi);
        Mat reCrop;
        resize(crop, reCrop, Size(), 39.0*3/width, 39.0/height);
     
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

        for(int y = 0; y < 39; y++)//get horizontal slices
        {
          if(y<13){
            for(int x = 0; x < 39; x++)
            {
              t = reCrop.at<Vec3b>(Point(x, y));
              if(t.val[0] < thresh)
                top+=(1/area);
            }
          }
          else if(y<26){
            for(int x = 0; x < 39; x++)
            {
              m = reCrop.at<Vec3b>(Point(x, y));
              if(m.val[0] < thresh)
                mid+=(1/area);
            }
          }
          else if(y<39){
            for(int x = 0; x < 39; x++)
            {
              b = reCrop.at<Vec3b>(Point(x, y));
              if(b.val[0] < thresh)
                bottom+=(1/area);
            }
          } 
        }
        printf("top: %f, mid: %f, bottom: %f\n", top, mid, bottom);
        int PosX = gps->getValues()[0]*100;
        int PosZ = gps->getValues()[2]*100;
        if(top > 0.1 && top < 0.45 && mid > 0.1 && mid < 0.6 && bottom > 0.1 && bottom < 0.5)//exclude noisy info
        {
            double img[3] = {top, mid, bottom};
            char letter = getLetter(img);
            changeMessage(PosX, PosZ, letter);
            printf("%c victim on camera %d\n", letter, camNum);
            return true;
        }
      }
    }

    return false;
}
