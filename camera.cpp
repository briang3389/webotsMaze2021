#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
using namespace cv;
using namespace std;
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <webots/GPS.hpp>
#include <webots/LightSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>

using namespace webots;


Robot *robot = new Robot();
Motor *leftMotor = robot->getMotor("wheel1 motor");
Motor *rightMotor = robot->getMotor("wheel2 motor");
GPS *gps = robot->getGPS("gps");
Emitter *emitter = robot->getEmitter("emitter");
Gyro* gyro = robot->getGyro("gyro");
DistanceSensor* infrared[4];
Camera *camL = robot->getCamera("lCam");
Camera *camR = robot->getCamera("rCam");

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
  
int thresh = 140;
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
    double Data[3][3] = {{0.240769, 0.344477, 0.296548}, //Hdata top: 0.230769, mid: 0.394477, bottom: 0.246548
                        {0.258383, 0.280079, 0.357278},  //Udata top: 0.258383, mid: 0.280079, bottom: 0.337278
                        {0.282051, 0.258107, 0.343748}}; //Sdata top: 0.302051, mid: 0.278107, bottom: 0.343748
    
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
    for(int n = 0; n < 3; n++)
    {
        for(int i = 0; i < 3; i++)
        {
            dist[n] += (Values[i]-Data[n][i])*(Values[i]-Data[n][i]);
        }
//        dist[n] = sqrt(dist[n]);
    }
    for(int n = 0; n < 3; n++)
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

bool checkVisualVictim(Camera* cam, char (&mess)[9])
{
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    Mat frame_HSV, frame_red, frame_yellow;

    Mat frame(camR->getHeight(), camR->getWidth(), CV_8UC4, (void*)camR->getImage());;
    
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
      
      blur( clone, clone, Size(3,3) );
      threshold(clone,clone,thresh,max_thresh,THRESH_BINARY); //threshold
      imshow("thresh", clone);
      Mat inverted;
      bitwise_not ( clone, inverted );
      imshow("inverted", inverted);
      Mat canny_output;
      vector<vector<Point>> contours;
      vector<vector<Point>> invContours;
      vector<Vec4i> hierarchy;
      
      findContours( clone, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
      findContours( inverted, invContours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
      for( size_t i = 0; i < contours.size(); i++ )
      {
        Scalar color = Scalar( 0, 255, 255);
  
        drawContours( clone, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
        Rect roi = boundingRect(contours[i]);
        imshow("conts", clone);
        double width = (double)roi.width;
        double height = (double)roi.height;
        
        //printf("width: %f, height: %f\n", width, height);
        //printf("width/height: %f\n", width/height);
        double area = 39*13;
        if(width < 100 && width > 50 && height < 100 && height > 50 && width/height < 1.1  && width/height > 0.9)
        {
          rectangle(clone,roi, color,1);
          Mat crop = clone(roi);
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
                if(t.val[0] < thresh){
                  top+=(1/area);
                }
              }
            }
            else if(y<26){
              for(int x = 0; x < 39; x++)
              {
                m = reCrop.at<Vec3b>(Point(x, y));
                if(m.val[0] < thresh){
                  mid+=(1/area);
                }
              }
            }
            else if(y<39){
              for(int x = 0; x < 39; x++)
              {
                b = reCrop.at<Vec3b>(Point(x, y));
                if(b.val[0] < thresh){
                  bottom+=(1/area);
                }
              }
            } 
          }
          //printf("top: %f, mid: %f, bot: %f\n", top, mid, bottom);
          waitKey(1);
          PosX = gps->getValues()[0]*100;
          PosZ = gps->getValues()[2]*100;

          if(top > 0.18 && top < 0.38 && mid > 0.22 && mid < 0.44 && bottom > 0.2 && bottom < 0.42)//exclude noisy info
          {
              double img[3] = {top, mid, bottom};
              memcpy(&mess[0], &PosX, 4);
              memcpy(&mess[4], &PosZ, 4);
              mess[8] = getLetter(img);
              return true;
          }
          else if(top > 0.5 && bottom > 0.5)
          {
              memcpy(&mess[0], &PosX, 4);
              memcpy(&mess[4], &PosZ, 4);
              if(bottom > 0.8){
                cout << "C hazard\n";
                mess[8] = 'C';
              }
              else{
                cout << "P hazard\n";//top: 0.715976, mid: 0.337278, bot: 0.721893
                mess[8] = 'P';
              }
              memcpy(&mess[0], &PosX, 4);
              memcpy(&mess[4], &PosZ, 4);
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
        
        if(width > 40 && width < 100 && height > 20 && height < 100 && (width/height > 0.9 && width/height < 2.1))
        {
          //printf("width: %f, height: %f\n", width, height);
          //printf("width/height: %f\n", width/height);
  
          findContours( frame_yellow, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
          if(contours.size() == 0){ //if no yellow
            cout << "F hazard\n";
            mess[8] = 'F';
          }
          else{
            cout << "O hazard\n";
            mess[8] = 'O';
          }
          return true;
        }
      }
    }
    return false;
}

int main(int argc, char **argv) {
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
 
  char mess[9];
  gyro->enable(timeStep);
  gps->enable(timeStep);
  camR->enable(timeStep);
  while (robot->step(timeStep) != -1) {  
      checkVisualVictim(camR, mess);
  }
  delete robot;
  return 0;
}
