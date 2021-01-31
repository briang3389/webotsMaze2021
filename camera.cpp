#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
using namespace cv;
using namespace std;

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/Emitter.hpp>

using namespace webots;


  // create the Robot instance.
  Robot *robot = new Robot();
  Motor *rightMotor = robot->getMotor("right wheel motor");
  Motor *leftMotor = robot->getMotor("left wheel motor");

  Camera *camC = robot->getCamera("camera_centre");
  Camera *camL = robot->getCamera("camera_left");
  GPS *gps = robot->getGPS("gps");
  Emitter *emitter = robot->getEmitter("emitter");
int PosX;
int PosZ;
  
int thresh = 165;
const int max_thresh = 255;

//blur
int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;
char window_name[] = "Smoothing Demo";
int display_caption( const char* caption );
int display_dst( int delay );
bool checkVisualVictim(Camera* cam, char (&mess)[9])
{
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
    imshow("thresh", src);
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
      if(width < 120 && width > 50 && height < 120 && height > 50 && width/height < 1.1  && width/height > 0.9)
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

        PosX = gps->getValues()[0]*100;
        PosZ = gps->getValues()[2]*100;
        if(top < 0.6 && mid < 0.7 && bottom < 0.6)//exclude noisy info
        {
        
          if(mid-top >= 0.06 && mid-bottom >= 0.06){//If mid is darker than top and bottom
            memcpy(&mess[0], &PosX, 4);
            memcpy(&mess[4], &PosZ, 4);
       
            mess[8] = 'H';
            printf("H victim\n");
            return true;
          }
          if(bottom-top >= 0.04 && mid-top >= 0.015){//If bottom is darker than top and mid
            memcpy(&mess[0], &PosX, 4);
            memcpy(&mess[4], &PosZ, 4);
            mess[8] = 'U';
            printf("U victim\n");
            return true;
          }

          if(mid-top < 0.03 || bottom-mid < 0.03 && bottom - top < 0.04){
            memcpy(&mess[0], &PosX, 4);
            memcpy(&mess[4], &PosZ, 4);
     
            mess[8] = 'S';
            printf("S victim\n");
            return true;
          }
        }
      }
    }

    return false;
}


int main( int argc, char** argv )
{
  char message[9];
  int timeStep = (int)robot->getBasicTimeStep();
  

  camC->enable(timeStep);

  while (robot->step(timeStep) != -1) {
    cout << checkVisualVictim(camC, message);

  };

  delete robot;
  return 0;
}
