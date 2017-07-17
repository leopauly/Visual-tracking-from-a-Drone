/***************************************************************************************************
 ** Written by : @leopauly | cnlp@leeds.ac.uk   
 ** Modified by : Shan Luo | S.Luo@leeds.ac.uk                                                                 
 ** Description : Program for visual tracking of objects from a drone                            
 ** The code was tested on Jetson TX1 running on Ubuntu16.04 with the attached ZED camera
 ***************************************************************************************************/

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>

//bilal's : start
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <cstring>
//bilal's : end

//Leo's start
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

#define BUFSIZE 1024

using namespace cv;
using namespace std;

//Leo's end

using namespace sl;

cv::Mat slMat2cvMat(sl::Mat& input);

int threshold_value_black_max, threshold_value_black_min,threshold_value_white_min, threshold_value_white_max, threshold_value_blue_max,threshold_value_blue_min,threshold_value_red_max,threshold_value_red_min;
int threshold_max=255;

vector<RotatedRect> minRect(1);
Point pt_right= Point(0,0);
Point pt_left= Point(0,0);
vector < vector<Point> > contours;
vector < vector<Point> > contours_new;
float perimeter_idx, area_idx,roundness;
int id[100];
double gsd= 0.021505 ; //.028672: gsd for altitude =20, resaolution= 10800p  in meter     //.021505: gsd for altitude =15, resaolution= 10800p  in meter  //  .0075:gsd for altitude =7, resaolution= 720p  in meter
float frame_h=1080;  // resolution of the frame-hight
float frame_w=3840;
float resize_ratio=.2;
float resize_h=540;// resolution of the frame-hight; after resizing  //  resize_h=frame_h*resize_ratio
float resize_w=1920;// resolution of the frame-width; after resizing

Point gsd_p= Point(.0075,.0075);
float tx_truck[3][3]={0};


float tx_obs[1][4]={0};
int tracking_flags[3]={0}; // 0: if being tracked; 1: if tracking is gone;
//shan
cv::Mat video_frame;

cv::Mat frame, gray,mask, binary, binary_white, binary_red_yellow, binary_green, binary_white_not, binary_red_yellow_not,binary_black, binary_red, binary_blue,binary_2, binary_1,rgb, bgr[3], contours_image, rgb_in, binary_yellow[2],area_image;

double area;
float area_min, area_max;
int threshold_value_area_max,threshold_value_area_min;
int threshold_max_area=1000;
int black_flag,blue_flag,red_flag=1;


float red_x_prev, red_y_prev , red_x_next,red_y_next;
float blue_x_prev, blue_y_prev , blue_x_next,blue_y_next;
float black_x_prev, black_y_prev , black_x_next, black_y_next;




int counter_t=0;
int flagger_red=1;
int flagger_blue=1;
int flagger_black=1;

//Bilal's code : start
void DataGeneration(char *buf)
{
    //Obstacle ID: X : Y
    int nb_obs=1;
    float Obstacles_Location[nb_obs][4];
    int nb_trucks=3;
    float Truck_Location[nb_trucks][2];

    string msg="Obstacles";


    /*Initialize all values to zero
    for (int i=0; i<nb_obs;i++){
        Obstacles_Location[i][0]=0;
        Obstacles_Location[i][1]=0;
    }

    //Initialize all values to zero
    for (int i=0; i<nb_trucks;i++){
        Truck_Location[i][0]=0;
        Truck_Location[i][1]=0;
    }


    //for testing
    
    Obstacles_Location[0][0]=2;
    Obstacles_Location[0][1]=4;

    Obstacles_Location[1][0]=3;
    Obstacles_Location[1][1]=5;
    

    Truck_Location[0][0]=1;
    Truck_Location[0][1]=21.5;

    Truck_Location[1][0]=6;
    Truck_Location[1][1]=7;

    //end for testing

   */

    for (int i=0; i<nb_obs; i++){
        //if (Obstacles_Location[i][0]>0)
        {   //cout<<i<<endl;
            msg=msg+"&"+to_string(tx_obs[i][0])+";";
            msg=msg+to_string(tx_obs[i][1])+";";
            msg=msg+to_string(tx_obs[i][2])+";";
            msg=msg+to_string(tx_obs[i][3]);
        }
    }

    msg=msg+"||Trucks";

    for (int i=0; i<nb_trucks; i++){
        //if (Truck_Location[i][0]>0)
        {
            //msg=msg+"&"+to_string(tx_truck[i][1])+";";
            //msg=msg+to_string(tx_truck[i][2]);
            msg=msg+"&"+to_string(tx_truck[i][0]);
            msg=msg+":"+to_string(tx_truck[i][1])+";";
            msg=msg+to_string(tx_truck[i][2]);
            //msg=msg+"f"+to_string(tracking_flags[i])+";";
        }
    }
    
    msg=msg+"||Flags";

    for (int i=0; i<nb_trucks; i++){
        //if (Truck_Location[i][0]>0)
        {
            msg=msg+"&"+to_string(tracking_flags[i]);
            msg=msg+";"+to_string(0);
        }
    }

   cout<<'&'<<tx_truck[0][0]<<':'<<tx_truck[0][1]<<';'<<tx_truck[0][2]<<endl;
   cout<<'&'<<tx_truck[1][0]<<':'<<tx_truck[1][1]<<';'<<tx_truck[1][2]<<endl;
   cout<<'&'<<tx_truck[2][0]<<':'<<tx_truck[2][1]<<';'<<tx_truck[2][2]<<endl;
   cout<<'f'<< tracking_flags[0]<<';'<< tracking_flags[1]<<';'<< tracking_flags[2]<<endl;
   cout<<msg<<endl;
    strcpy(buf,msg.c_str());
}

//Bilal's code : end

void area_function(int,void*)
{
area_min= threshold_value_area_min;
area_max= threshold_value_area_max;
}

//shan
/*
void binariser_black(int,void*)
{
   inRange(gray,threshold_value_black_min,threshold_value_black_max,binary_black);
   imshow("binary_black", binary_black);   
}
*/
//shan

void binariser_white(int,void*)
{  
   split(rgb,bgr);
   inRange(bgr[2],threshold_value_white_min+180,threshold_value_white_max,binary_yellow[0]);
   inRange(bgr[1],threshold_value_white_min+180,threshold_value_white_max,binary_yellow[1]);
   cv::Mat src;
   //addWeighted(bgr[2],1,bgr[1],1,0,src);
   //inRange(src, cv::Scalar(threshold_value_white_min, threshold_value_white_min), Scalar(threshold_value_white_max, threshold_value_white_max), binary_white);
   
   //shan
   
   bitwise_and(binary_yellow[0],binary_yellow[1],binary_white);
   //addWeighted(binary_yellow[0],1,binary_yellow[1],1,0,binary_white); 
   //imshow("binary_yellow", binary_white);
   //shan
}
void binariser_red(int,void*)
{  split(rgb,bgr);
   //threshold(bgr[2],binary_red,threshold_value_red,threshold_max,0);
   inRange(bgr[2],threshold_value_red_min+180,threshold_value_red_max,binary_red);
   inRange(bgr[1],threshold_value_red_min+180,threshold_value_red_max,binary_green);
   
   //bitwise_and(binary_red,binary_green,binary_red_yellow);
   //bitwise_not(binary_red_yellow,binary_red_yellow_not);
   //imshow("yellowmask",binary_red_yellow_not);
   //bitwise_and(binary_red,binary_red_yellow_not, binary_red);
   cv::Mat binary_red_res;
   bitwise_xor(binary_red,binary_green, binary_red_res);
   bitwise_and(binary_red,binary_red_res, binary_red);
   
   //imshow("binary_red", binary_red);//shan
}
void binariser_blue(int,void*)
{  split(rgb,bgr);
   //threshold(bgr[0],binary_blue,threshold_value_blue,threshold_max,0);
   inRange(bgr[0],threshold_value_blue_min+180,threshold_value_blue_max,binary_blue);//shan
   //imshow("binary_blue", binary_blue);//shan
}

void contour_finder(cv::Mat binary_f,Scalar identity_f, int id_t,int counter_tf)
{           
            Point pt= Point(resize_w/2,resize_h);
            //circle(contours_image, pt,4, Scalar (0,0,0), CV_FILLED, 2,0); // to check the 0,0 coordinate //shan
            
            findContours (binary_f,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);            
            vector<Moments> mu(contours.size());
            vector<Point2f> mc(contours.size());
            vector< double> areas(contours.size());
            for (int i=0; i<contours.size();i++) 
            {mu[i]=moments(contours[i],false);}
            for (int i=0; i<contours.size(); i++)
            { mc[i]= Point2f (mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);}
    
            
            int j=0;
            for(int i=0; i<contours.size(); i++)
              {
               area=contourArea(contours[i],false);
               if (area>area_min && area<area_max)
                 { //cout<<area<<endl;
                   id[j]=i;
                   j++;
                 }
               }
                 //Scalar color(0,0,255);
            
            
             int idx;
             for (int m=0;m<j;m++)
             { idx=id[m];
               approxPolyDP(contours[idx],contours[idx],1,true);
               
               //drawContours(area_image,contours,id[m],Scalar(0,0,0),2);//, CV_FILLED); //shan
          
               imshow("area",area_image);
               
               perimeter_idx=arcLength(contours[idx], true);
               area_idx=contourArea(contours[idx],false);
               roundness=(perimeter_idx*perimeter_idx)/(4*3.14159*area_idx);
                            
               if (counter_tf>20)             
               {
                if(roundness>.5 && roundness<3.5)
		//if(roundness>.1 && roundness<10) //shan
                {       
   
                       int d=0;  
                       float diff_x,diff_y;

                       if(id_t==2)
                       {   
                         if(flagger_red==1)
                         {  red_x_next=int(mc[idx].x);
                            red_y_next=int(mc[idx].y);
                            flagger_red=2;
                         }
                        else 
                         { 
                           red_x_next=int(mc[idx].x);
                           red_y_next=int(mc[idx].y);
                           
                           diff_x=int(red_x_prev-red_x_next);
                           diff_y=int(red_y_prev-red_y_next);
                           
                           //vector<Point> point1=int([red_x_prev,red_y_prev]);
                           //line(contours_image,point1,(red_x_next,red_y_next),Scalar(0,0,255))   ;  
                           
                           d=sqrt(diff_x*diff_x+diff_y*diff_y);
                         }
                        }
                        
                        if(id_t==1)
                       {   
                         if(flagger_black==1)
                         {  black_x_next=int(mc[idx].x);
                            black_y_next=int(mc[idx].y);
                            flagger_black=2;
                         }
                        else 
                         { 
                           black_x_next=int(mc[idx].x);
                           black_y_next=int(mc[idx].y);
                           
                           diff_x=int(black_x_prev-black_x_next);
                           diff_y=int(black_y_prev-black_y_next);
                           
                           d=sqrt(diff_x*diff_x+diff_y*diff_y);
                         }
                        }
                     
                       if(id_t==3)
                       {   
                         if(flagger_blue==1)
                         {  blue_x_next=int(mc[idx].x);
                            blue_y_next=int(mc[idx].y);
                            flagger_blue=2;
                         }
                        else 
                         { 
                           blue_x_next=int(mc[idx].x);
                           blue_y_next=int(mc[idx].y);
                           
                           diff_x=int(blue_x_prev-blue_x_next);
                           diff_y=int(blue_y_prev-blue_y_next);
                           
                           d=sqrt(diff_x*diff_x+diff_y*diff_y);
                         }
                        }



                                              
                       cout<<"distance: "<<d<<endl;
                       if(d<2000)
		       //if(d<2000)//shan
                       {
                         if(id_t==2)
                         { tx_truck[1][0]=id_t;
                           tx_truck[1][1]=mc[idx].x*gsd; tx_truck[1][2]=mc[idx].y*gsd; 
                           //cout<<'&'<<tx_truck[0][0]<<':'<<tx_truck[0][1]<<';'<<tx_truck[0][2]<<endl;
                           red_x_prev=red_x_next;                          
                           red_y_prev=red_y_next;
                           red_flag=0;
                          }
                         if(id_t==1)
                         { tx_truck[0][0]=id_t;
                           tx_truck[0][1]=mc[idx].x*gsd; tx_truck[0][2]=mc[idx].y*gsd; 
                           //cout<<'&'<<tx_truck[1][0]<<':'<<tx_truck[1][1]<<';'<<tx_truck[1][2]<<endl;
                           black_x_prev=black_x_next;                          
                           black_y_prev=black_y_next;
                           black_flag=0;
                         }
                         if(id_t==3)
                         {
                          tx_truck[2][0]=id_t;
                          tx_truck[2][1]=mc[idx].x*gsd; tx_truck[2][2]=mc[idx].y*gsd; 
                          //cout<<'&'<<tx_truck[2][0]<<':'<<tx_truck[2][1]<<';'<<tx_truck[2][2]<<endl;
                          blue_x_prev=blue_x_next;                          
                          blue_y_prev=blue_y_next;
                          blue_flag=0;
                          }
                       string truck_id,id_c;
                       id_c=to_string(id_t);
                       truck_id="Truck"+id_c; 
                       //drawContours(contours_image,contours,id[m],identity_f,2);//, CV_FILLED);
                       //circle(contours_image, mc[idx],1, Scalar (0,0,0), CV_FILLED, 1,0); //shan
                       
                       if(id_t==4)
                       { {putText(contours_image,"Obstacle1",mc[idx], 1, 0.9, Scalar (0,0,0), 1);
                          //Point pt= Point(resize_w/2,resize_h);
                          circle(contours_image, mc[idx],50, Scalar (0,0,0), 1, 2,0);			
			}}//shan
                       else
                       {putText(contours_image,truck_id,mc[idx], 1, 0.9, Scalar (0,0,0), 1);}//shan
                       //cout<<"center of "<<truck_id<<':'<<mc[idx]<<endl;
                       
                       if (id_t==4)

                          {  
                             minRect[1] = minAreaRect(cv::Mat(contours[idx]));
                             cout<<minRect[1].center.x<<endl;
                             float left_x, left_y,right_x,right_y;
             	             left_x = minRect[1].center.x - ((minRect[1].size.width/2)*6);
                             left_y = minRect[1].center.y - ((minRect[1].size.height/2)*6);
                             right_x = minRect[1].center.x + ((minRect[1].size.width/2)*6);
                             right_y = minRect[1].center.y + ((minRect[1].size.height/2)*6);
                             pt_left.x=left_x;
                             pt_left.y=left_y;
                             //circle(contours_image, pt_left,3, Scalar (0,0,255), CV_FILLED, 2,0);//shan                       

                             pt_right.x=right_x;
                             pt_right.y=right_y;
                             //circle(contours_image, pt_right,3, Scalar (0,0,0), CV_FILLED, 2,0);//shan 
                             
                             if(left_x>0 && left_x<resize_w/2)
                             {tx_obs[0][0]=left_x*gsd;}
                             
                             if(left_y>0 && left_y<resize_h) 
                             {tx_obs[0][1]=left_y*gsd;}

                             if(right_x>0 && right_x<resize_w/2)
                             {tx_obs[0][2]=right_x*gsd;}

                             if(right_y>0 && right_y<resize_h)
                             tx_obs[0][3]=right_y*gsd;

      
                             //cout<<'/'<<id_t<<','<<contours[idx]<<'/'<<endl; //prints all the points of a contour of obstacle
                             
                             float tx_obs[contours[idx].size()][2];
                                tx_obs[0][0]= id_t;
                                tx_obs[0][1]= id_t;
                                cout<<'&'<<tx_obs[0][0]<<','<<tx_obs[0][1]<<endl;

                             for(int c=1;c<contours[idx].size();c++)
                                {tx_obs[c][0]=contours[idx][c].x*gsd ;
                                 tx_obs[c][1]=contours[idx][c].y*gsd;
                                 cout<<tx_obs[c][0]<<','<<tx_obs[c][1]<<endl;
                                }
                                cout<<'/'<<endl;

                          }
                       else
                          { 
                            
                            
                            //float tx_frame[2][2];
                            //tx_frame[1][0]=id_t; tx_frame[0][0]=id_t; 
                            //tx_frame[1][0]=frame_w*gsd,tx_frame[1][1]=frame_h*gsd; 
                            
                            
                                                                

                          }

                       //cout<<"roundness of marker on the above truck:"<<endl;
                       // cout<<roundness<<endl;
                       //cout<<"area of marker on the above truck:"<<endl;
                       //cout<<area<<endl;
                       }
                  }
               } 
             
                   /* vector<RotatedRect> minRect(contours.size());
                   minRect[1] = minAreaRect(contours[idx]);  
             
                   float width, height;
             	   width = minRect[1].center.x - (minRect[1].size.width/2);
                   height = minRect[1].center.y - (minRect[1].size.height/2);
                   float rectangleness;
                   rectangleness=width/height;
                   cout<< rectangleness<<endl;
                   if(rectangleness>1 && rectangleness<4)
                   {
                       cout<<"rectanglenes of rectangle marker:"<<endl;
                       cout<< rectangleness<<endl;
                       cout<<"area of rectangle marker:"<<endl;
                       cout<<area_idx<<endl;
                       
                       //drawContours(contours_image,contours,id[m],identity_f,2);//, CV_FILLED);
                       circle(contours_image, mc[idx],2, identity_f, CV_FILLED, 1,0);
                       
                       cout<<"center of rectangle truck"<<endl;
                       cout<<mc[idx]<<endl;
                   } */

            
            } 


}

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD1080;
    init_params.depth_mode = DEPTH_MODE_NONE;
    init_params.coordinate_units = sl::UNIT_METER;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        return 1;

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD; // Use STANDARD sensing mode

    // Create sl and cv Mat to get ZED left image and depth image
    // Best way of sharing sl::Mat and cv::Mat :
    // Create a sl::Mat and then construct a cv::Mat using the ptr to sl::Mat data.
    Resolution image_size = zed.getResolution();
	sl::Mat image_zed(image_size, sl::MAT_TYPE_8U_C4); // Create a sl::Mat to handle Left image
        cv::Mat image_ocv = slMat2cvMat(image_zed);

    // Create OpenCV images to display (lower resolution to fit the screen)
    //cv::Size displaySize(1920, 1080);
    //cv::Mat image_ocv_display(displaySize, CV_8UC4);
    cv::Mat image_ocv_display;
    // Jetson only. Execute the calling thread on 2nd core
    Camera::sticktoCPUCore(2);

    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {

        // Grab and display image
        if (zed.grab(runtime_parameters) == SUCCESS) {
            zed.retrieveImage(image_zed, VIEW_LEFT); // Retrieve the left image

            // Resize and display with OpenCV
            //cv::resize(image_ocv, image_ocv_display, displaySize);
            //image_ocv_display=image_ocv;
	    //shan
            //std::cout << "shan" << endl;
  	    //VideoCapture cap("./DJI_0027_CUT.mov"); // open video object
	    VideoCapture cap("./DJI_0015.MOV"); // open video object
            if(!cap.isOpened())  // check if we successfully opened a video object
              //return -1;
              {std::cout<<"no"<<endl;
               std::cout<<"no"<<cap.isOpened()<<endl;}

           for(;;)
           {
            cap >> video_frame; // get a new frame from camera
            image_ocv_display=video_frame;
            //imshow("Image", image_ocv_display);
            // tracking :start (cnlp@leeds.ac.uk)
            tx_truck[0][0]=1;
            tx_truck[1][0]=2;
            tx_truck[2][0]=3;
            resize(image_ocv_display,image_ocv_display,cv::Size(),resize_ratio,resize_ratio);
            rgb_in=image_ocv_display ;
            rgb=rgb_in;
            contours_image=rgb;  
            area_image=rgb;  
            cvtColor(rgb, gray, COLOR_BGR2GRAY);
            
            createTrackbar("area_min", "area", &threshold_value_area_min, threshold_max_area, area_function);
            area_function(0,0);
            createTrackbar("area_max", "area", &threshold_value_area_max, threshold_max_area, area_function);
            area_function(0,0);
            imshow("area",area_image);
            
	    //shan
            /*
            createTrackbar("binary_bar_black_max", "binary_black", &threshold_value_black_max, threshold_max, binariser_black);
            binariser_black(0,0);
            createTrackbar("binary_bar_black_min", "binary_black", &threshold_value_black_min, threshold_max, binariser_black);
            binariser_black(0,0);
      
            Scalar identity_b (255,255,255);
            cv::Mat copy_b=binary_black;
            contour_finder(copy_b,identity_b,1, counter_t);
	    */
	    //shan
            
   	    //shan
            //createTrackbar("binary_bar_yellow_max", "binary_yellow", &threshold_value_white_max, threshold_max, binariser_white);
            //binariser_white(0,0);
	    threshold_value_white_max=255;
            createTrackbar("yel_min+180", "area", &threshold_value_white_min, threshold_max-180, binariser_white);//shan
            binariser_white(0,0);
         
            Scalar identity_w (0,255,0);
            cv::Mat copy_w=binary_white;
            contour_finder(copy_w,identity_w,4,counter_t);
               
            //createTrackbar("binary_bar_red_max", "binary_red", &threshold_value_red_max, threshold_max, binariser_red);
            //binariser_red(0,0);
	    threshold_value_red_max=255;
            createTrackbar("red_min+180", "area", &threshold_value_red_min, threshold_max-180, binariser_red);//shan
            binariser_red(0,0);

           Scalar identity_r (255,0,0);
           cv::Mat copy_r=binary_red;
           contour_finder(copy_r,identity_r,2, counter_t);
   
            //createTrackbar("binary_bar_blue_max", "binary_blue", &threshold_value_blue_max, threshold_max, binariser_blue);
            //binariser_blue(0,0);
	    threshold_value_blue_max=255;
            createTrackbar("blu_min+180", "area", &threshold_value_blue_min, threshold_max-180, binariser_blue);//shan
            binariser_blue(0,0);
      
            Scalar identity_blue (0,0,255);
            cv::Mat copy_blue=binary_blue;
            contour_finder(copy_blue,identity_blue,3, counter_t);
            //shan
           
            cout<<counter_t<<endl; 
            counter_t=counter_t+1;
            //imshow("tracking",contours_image);//shan
            key = cv::waitKey(5);
         
           tracking_flags[0]=black_flag;
           black_flag=1;
           tracking_flags[1]=red_flag;
           red_flag=1;
           tracking_flags[2]=blue_flag;
           blue_flag=1;

           //bilal's code: start
int sockfd, portno, n;
    int serverlen;
    struct sockaddr_in serveraddr;
    struct hostent *server;
    char *hostname;
    char buf[BUFSIZE];

    DataGeneration(buf);
    
    /* check command line arguments */
    hostname = "192.168.1.100"; //"127.0.0.1";
    portno = 15000;

    /* socket: create the socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    //if (sockfd < 0) 
    //    error("ERROR opening socket");

    /* gethostbyname: get the server's DNS entry */
    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host as %s\n", hostname);
        exit(0);
    }

    /* build the server's Internet address */
    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
	  (char *)&serveraddr.sin_addr.s_addr, server->h_length);
    serveraddr.sin_port = htons(portno);

    /* send the message to the server */
    serverlen = sizeof(serveraddr);
    n = sendto(sockfd, buf, strlen(buf), 0, (struct sockaddr *) &serveraddr, serverlen);
    //if (n < 0) 
    //  error("ERROR in sendto");
    

           
// bilal's code : end

           // tracking: end (cnlp@leeds.ac.uk)
            key = cv::waitKey(10);
}
        }
    }
    
    zed.close();
    return 0;
}

cv::Mat slMat2cvMat(sl::Mat& input)
{

	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType())
	{
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}
