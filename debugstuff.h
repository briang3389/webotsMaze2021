#include <sys/time.h>

#define DEBUGSTUFF

Point centeredTextCoords(const char *s, int x, int y)
{
  int baseline=0;
  Size ts = getTextSize(s, FONT_HERSHEY_PLAIN, 1, 1, &baseline);
  return Point(x-ts.width/2,y+ts.height/2);
}
void debugStuff(bool saveToFile=false) //each box is 30px
{
//todo: call this function whenever state variables are updated
//that didnt change anything

//for some reason when its turning or whatever sometimes, its not updating
  //if(turning)cout<<"turning"<<endl;

  static char tmp[1000];
  const int boxSize=35;
  const int lineWidth=4;
  const int pitSize=22;
  const int robotSize=15;
  const Point textCoords1(10,600);
  Point textCoords2(10,500);
  Point textCoords3(150,600);
  //45-55
  const int startIndex=45;
  const int endIndex=64;
  const int num=endIndex-startIndex+1;
  
  //700 500
  Mat img = Mat( 800, 800, CV_8UC3, Scalar(255,255,255) );
  //putText(img, "45", centeredTextCoords("45",5,100), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0));
  
  for(int i=startIndex,x=boxSize,y=0;i<=endIndex;i++,x+=boxSize)
  {
    int cx=x+boxSize/2,cy=y+boxSize/2;
    sprintf(tmp,"%d",i);
    putText(img, tmp, centeredTextCoords(tmp,cx,cy), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0));
  }
  for(int i=endIndex,x=0,y=boxSize;i>=startIndex;i--,y+=boxSize)
  {
    int cx=x+boxSize/2,cy=y+boxSize/2;
    sprintf(tmp,"%d",i);
    putText(img, tmp, centeredTextCoords(tmp,cx,cy), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0));
  }
  
  pair<int,int> currentCoords=getCoords();
  
  for(int x=startIndex;x<=endIndex;x++)
  {
    for(int y=startIndex;y<=endIndex;y++)
    {
      int xx=boxSize+(x-startIndex)*boxSize;
      int yy=boxSize+(num-1-(y-startIndex))*boxSize;
      int cxx=xx+boxSize/2,cyy=yy+boxSize/2;
      Tile t=board[x][y];
      if(!t.visited)
      {
        rectangle(img,Point(xx,yy),Point(xx+boxSize-1,yy+boxSize-1),Scalar(170,170,170),FILLED);
        goto afterDrawStuff;
      }
      if(!t.open[0]) rectangle(img,Point(xx,yy),Point(xx+boxSize-1,yy+lineWidth-1),Scalar(0,0,0),FILLED);
      if(!t.open[1]) rectangle(img,Point(xx+boxSize-lineWidth,yy),Point(xx+boxSize-1,yy+boxSize-1),Scalar(0,0,0),FILLED);
      if(!t.open[2]) rectangle(img,Point(xx,yy+boxSize-lineWidth),Point(xx+boxSize-1,yy+boxSize-1),Scalar(0,0,0),FILLED);
      if(!t.open[3]) rectangle(img,Point(xx,yy),Point(xx+lineWidth-1,yy+boxSize-1),Scalar(0,0,0),FILLED);
      if(t.isHole) rectangle(img,Point(cxx-pitSize/2,cyy-pitSize/2),Point(cxx+pitSize/2,cyy+pitSize/2),Scalar(0,0,0),FILLED);
      afterDrawStuff:;
      if(x==currentCoords.first && y==currentCoords.second) rectangle(img,Point(cxx-robotSize/2,cyy-robotSize/2),Point(cxx+robotSize/2,cyy+robotSize/2),Scalar(255,0,0),FILLED);
    }
  }
  double angle=getAngle();
  sprintf(tmp,"%d    %.1f",roundAngle(angle),angle);
  putText(img, tmp, textCoords1, FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0));
  const char* staates[7]={turning?"TURNING":"turning", advancing?"ADVANCING":"advancing", stopping?"STOPPING":"stopping", reversing?"REVERSING":"reversing", ending?"ENDING":"ending", skipTurn?"SKIPTURN":"skipturn", visualChecking?"VISUALCHECKING":"visualchecking"};
  //const string staates[7][2]
  for(int i=0;i<7;i++,textCoords2.x+=120,i%4==0?(textCoords2.y+=40,textCoords2.x=10):0)
  {
    
    putText(img, staates[i], textCoords2, FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), isupper(staates[i][0])?2:1);
  }
  
  putText(img,tileTypeStr.c_str(),textCoords3,FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0));
  
  char bruh[100];
  struct timeval tv;
  gettimeofday(&tv, NULL);
  unsigned long long millisecondsSinceEpoch =
  (unsigned long long)(tv.tv_sec) * 1000 +
  (unsigned long long)(tv.tv_usec) / 1000;
  sprintf(bruh,"%llu",millisecondsSinceEpoch);
  putText(img, bruh, Point(10,650), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0));
  imshow("debug",img); //WHY IS THIS DELAYED?? use time(0) to find out. print it to console and put it on the image
  waitKey(10);
  //ok so the img is being created fine but imshow is just garbage
  //nvm you just need waitKey(1)
  //imwrite(string("/home/brian/tmpfs/")+to_string(time(0))+string(".png"), img);
  //cout<<"imshow "<<time(0)<<endl;
  if(saveToFile)
  {
    imwrite(string("/home/brian/Desktop/map.png"), img);
  }
  
  
  if(false)
  {
    cout<<"sleeping"<<endl;
    for(int i = 0; i < 100; i++) robot->step(timeStep);
    cout<<"done sleeping"<<endl;
  }
}