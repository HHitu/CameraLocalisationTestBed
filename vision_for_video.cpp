#include <boost/bind.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include<math.h>
#include <vector>
//#include "/home/harleen/snp2/devel/include/snpsir/data.h"
#include <sstream>
//#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>
#include <stdio.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/imgproc/imgproc_c.h>
#include<opencv2/imgproc/types_c.h>
#include<iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#define RAD2DEG(rad) (((rad) * 180)/M_PI)
#define DEG2RAD(deg) (((deg) * M_PI)/180)

using namespace std;
using namespace cv;
float x, y, z;
        cv::Mat R1, R2, R3, R4;
        cv::Mat t;
        cv::Mat matE;
Mat currFrame, prevFrame;
vector<Mat> rvec2,tvec2,normals;
Mat rot;// trans;
cv::Mat trans = cv::Mat::zeros(3, 1, CV_64F);
float fX,fY,fZ;
Mat f;
double rotx, roty, rotz;
double pxp,pyp,pzp,rxp,ryp,rzp,rwp;
double px,py,pz;
vector<KeyPoint> kp;

class perspective_transformer {
private:
Mat im, im_transformed, im_perspective_transformed;
Mat M;

public:
perspective_transformer();
void estimate_perspective();
void show_diff();
};
 
perspective_transformer::perspective_transformer() {
im = prevFrame.clone();//imread("/home/harleen/snp2/src/snpsir/proper_data/r5.jpg");//
im_transformed = currFrame.clone();//imread("/home/harleen/snp2/src/snpsir/proper_data/r6.jpg");//currFrame.clone();//
}
void perspective_transformer::estimate_perspective() {
// Match ORB features to point correspondences between the images
vector<KeyPoint> kp, t_kp;
Mat desc, t_desc, im_g, t_im_g;
vector<Point2f> points;
vector<Point2f> points_transformed;

 /*
cvtColor(im, im_g, CV_BGR2GRAY);
cvtColor(im_transformed, t_im_g, CV_BGR2GRAY);
 
OrbFeatureDetector featureDetector;
OrbDescriptorExtractor featureExtractor;
 
featureDetector.detect(im_g, kp);
featureDetector.detect(t_im_g, t_kp);
 
featureExtractor.compute(im_g, kp, desc);
featureExtractor.compute(t_im_g, t_kp, t_desc);
 
flann::Index flannIndex(desc, flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
Mat match_idx(t_desc.rows, 2, CV_32SC1), match_dist(t_desc.rows, 2, CV_32FC1);
flannIndex.knnSearch(t_desc, match_idx, match_dist, 2, flann::SearchParams());

vector<DMatch> good_matches;
for(int i = 0; i < match_dist.rows; i++) {
if(match_dist.at<float>(i, 0) < 0.3 * match_dist.at<float>(i, 1)) {
DMatch dm(i, match_idx.at<int>(i, 0), match_dist.at<float>(i, 0));
good_matches.push_back(dm);
//points.push_back((kp[dm.trainIdx]).pt);
//points_transformed.push_back((t_kp[dm.queryIdx]).pt);
}}*/
vector<Point2f> im_p1,imp1p;
//find corners in the chessboard image
bool pattern_found=0;
//while (pattern_found==0)
{pattern_found = findChessboardCorners(im, Size(9, 6), im_p1,
CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+ CALIB_CB_FAST_CHECK);
}
if(pattern_found) {
cout<<"found1"<<endl;
//object_points.push_back(ob_p);
Mat gray;
cvtColor(im, gray, CV_BGR2GRAY);
cornerSubPix(gray, im_p1, Size(9, 6), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS +
CV_TERMCRIT_ITER, 30, 0.1));
for(int i = 0; i < 14; i++) {
points.push_back(im_p1[i]);}
imp1p=im_p1;}
else
{im_p1=imp1p;
for(int i = 0; i < 14; i++) {
points.push_back(im_p1[i]);}}

/*Mat im_show = im.clone();
drawChessboardCorners(im_show, Size(9, 6), im_p, true);
imshow("Chessboard corners", im_show);
while(char(waitKey(1)) != ' ') {}*/

vector<Point2f> im_p2,imp2p;
//pattern_found=0;
//while (pattern_found==0)
{pattern_found = findChessboardCorners(im_transformed, Size(9, 6), im_p2,
CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+ CALIB_CB_FAST_CHECK);}
if(pattern_found) {
cout<<"found2"<<endl;
//object_points.push_back(ob_p);
Mat gray;
cvtColor(im_transformed, gray, CV_BGR2GRAY);
cornerSubPix(gray, im_p2, Size(9, 6), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS +
CV_TERMCRIT_ITER, 30, 0.1));
for(int i = 0; i < 14; i++) {
points_transformed.push_back(im_p2[i]);}
imp2p=im_p2;}
else
{im_p2=imp2p;
for(int i = 0; i < 14; i++) {
points_transformed.push_back(im_p2[i]);}}


Mat im_show = im_transformed.clone();
drawChessboardCorners(im_show, Size(9, 6), im_p2, true);
imshow("Chessboard corners", im_show);
waitKey(10);
//while(char(waitKey(1)) != ' ') {}
Mat M = findHomography(points, points_transformed, RANSAC, 2);
cout << "Estimated Perspective transform = " << M << endl;
 
// Apply estimated perspecive trasnform
//warpPerspective(im, im_perspective_transformed, M, im.size());
//namedWindow("Estimated Perspective transform");
//namedWindow("Difference");

//imshow("Estimated Perspective transform", im_perspective_transformed);
//imshow("Difference", im_transformed - im_perspective_transformed);
//Mat im_show;
//drawMatches(im_transformed, t_kp, im, kp, good_matches, im_show);
//imshow("ORB matches", im_show);

cv::Mat dist(5,1,cv::DataType<double>::type);
dist.at<double>(0)= -0.004468;
dist.at<double>(1)=-0.120621;
dist.at<double>(2)=-0.012367;
dist.at<double>(3)=-0.006686;
dist.at<double>(4)= 0.00;
cv::Mat cammat(3,3,cv::DataType<double>::type);
cv::Mat res(3,3,cv::DataType<double>::type);
cv::Mat rest(3,3,cv::DataType<double>::type);
cammat.at<double>(0,0)=794.298570; 
cammat.at<double>(0,1)=0.00;
cammat.at<double>(0,2)=300.8439;
cammat.at<double>(1,0)=0.00;
cammat.at<double>(1,1)=805.0919;
cammat.at<double>(1,2)=196.4613;
cammat.at<double>(2,0)=0.00;
cammat.at<double>(2,1)=0.00;
cammat.at<double>(2,2)=1.00;
   cv::Mat matK = cv::Mat::zeros(3, 3, CV_64F);
        matK.at<double>(0, 0) = 794.2985;
        matK.at<double>(1, 1) = 805.0919;
        matK.at<double>(0, 2) = 300.8439;
        matK.at<double>(1, 2) = 196.4613;
        matK.at<double>(2, 2) = 1;

std::vector<cv::Mat> Rs, Ts;
  cv::decomposeHomographyMat(M,
                             cammat,
                             Rs, Ts,
                             cv::noArray());

  std::cout << "-------------------------------------------\n";
  std::cout << "Estimated decomposition:\n\n";
  std::cout << "rvec = " << std::endl;

  for (int i=0; i< Rs.size(); i++) {
    cv::Mat1d rvec_n;
    cv::Rodrigues(Rs[i], rvec_n);
    std::cout << rvec_n*180/CV_PI << std::endl << std::endl;
  }

 // std::cout << std::endl;

  std::cout << "t = " << std::endl;
   for (int i=0; i< Ts.size(); i++){
    std::cout << Ts[i] << std::endl << std::endl;
  }

Mat M_2 = findHomography(points_transformed, points, RANSAC, 2);
std::vector<cv::Mat> Rs_t, Ts_t;
  cv::decomposeHomographyMat(M_2,
                             cammat,
                             Rs_t, Ts_t,
                             cv::noArray());
 std::cout << "rvec2 = " << std::endl;
for (int i=0; i< Rs_t.size(); i++) {
    cv::Mat1d rvec_n_t;
    cv::Rodrigues(Rs_t[i], rvec_n_t);
    std::cout << rvec_n_t*180/CV_PI << std::endl << std::endl;
  }

vector<double> dr;
for (int i=0; i< Rs.size(); i++) {
for (int j=0; j< Rs_t.size(); j++) {
cv::Mat1d rvec_n_t;
cv::Mat1d rvec_n;
 cv::Rodrigues(Rs[i], rvec_n);
cv::Rodrigues(Rs_t[j], rvec_n_t);
Mat1d d=abs(rvec_n)-abs(rvec_n_t);
double diff=d.at<double>(0)+d.at<double>(1)+d.at<double>(2);
dr.push_back(diff);
cout <<"diff" <<i << j<< " " <<diff<<endl;
}}

vector<double> dt;
for (int i=0; i< Ts.size(); i++) {
for (int j=0; j< Ts_t.size(); j++) {
Mat1d d=abs(Ts[i])-abs(Ts_t[j]);
double diff=d.at<double>(0)+d.at<double>(1)+d.at<double>(2);
dt.push_back(diff);
cout <<"diff" <<i << j<< " " <<diff<<endl;
}}

int ir=0;
int it1=0;
int it2=0;
double mint=dt[0];
for (int i=0; i< dt.size(); i++){
if( dt[i]<=mint )
{mint = dt[i];
it1=i;
it2=it1+i;
}}
double minr=dr[0];
for (int i=0; i< dr.size(); i++){
if( abs(dr[i])<=minr )
{minr = abs(dr[i]);
ir=i;
}}
int index_r=int(ir/Rs.size());
int index_t1=int(it1/Ts.size());
int index_t2=int((it2-it1)/Ts.size());
int ddar,ddat;
/*if (index_r!=index_t1 && index_r!=index_t2)
{cout <<"waheguru" <<endl;
ddar=0;
ddat=0;
cout << index_r << " " << index_t1 << " " << index_t2 << endl;}
else*/{
ddar=index_r;
ddat=index_t1;
}
cv::Mat1d rvec;
switch(ddar)
{case 0 :
cv::Rodrigues(Rs[0], rvec);
rotx=rvec.at<double>(0);
roty=rvec.at<double>(1);
rotz=rvec.at<double>(2);
std::cout << "Euler Angles R1 (X,Y,Z): " << RAD2DEG(rotx) << ", " << RAD2DEG(roty) << ", " << RAD2DEG(rotz) << std::endl;
break;
case 1 :
cv::Rodrigues(Rs[1], rvec);
rotx=rvec.at<double>(0);
roty=rvec.at<double>(1);
rotz=rvec.at<double>(2);
std::cout << "Euler Angles R2 (X,Y,Z): " << RAD2DEG(rotx) << ", " <<RAD2DEG(roty) << ", " <<RAD2DEG(rotz) << std::endl;
break;
case 2 :
cv::Rodrigues(Rs[2], rvec);
rotx=rvec.at<double>(0);
roty=rvec.at<double>(1);
rotz=rvec.at<double>(2);
std::cout << "Euler Angles R3 (X,Y,Z): " <<RAD2DEG(rotx) << ", " << RAD2DEG(roty) << ", " <<RAD2DEG(rotz) << std::endl;
break;
case 3 :
cv::Rodrigues(Rs[3], rvec);
rotx=rvec.at<double>(0);
roty=rvec.at<double>(1);
rotz=rvec.at<double>(2);
std::cout << "Euler Angles R4 (X,Y,Z): " << RAD2DEG(rotx) << ", " <<RAD2DEG(roty) << ", " << RAD2DEG(rotz) << std::endl;
break;
default:
std::cout << "values " << std::endl;
}
std::cout << ddar << std::endl;

switch(ddat)
{case 0 :
px=Ts[0].at<double>(0);
py=Ts[0].at<double>(1);
pz=Ts[0].at<double>(2);
std::cout << "Trans : " << px << ", " << py << ", " << pz << std::endl;
break;
case 1 :
px=Ts[1].at<double>(0);
py=Ts[1].at<double>(1);
pz=Ts[1].at<double>(2);
std::cout << "Trans : " << px << ", " << py << ", " << pz << std::endl;
break;
case 2 :
px=Ts[2].at<double>(0);
py=Ts[2].at<double>(1);
pz=Ts[2].at<double>(2);
std::cout << "Trans : " << px << ", " << py << ", " << pz << std::endl;
break;
case 3 :
px=Ts[3].at<double>(0);
py=Ts[3].at<double>(1);
pz=Ts[3].at<double>(2);
std::cout << "Trans : " << px << ", " << py << ", " << pz << std::endl;
break;
default:
std::cout << "values " << std::endl;
}
std::cout << ddat << std::endl;

Mat mask;
Mat Ess=findEssentialMat(points, points_transformed, 800, Point2d(300.8439, 196.4613), RANSAC, 0.999, 1.0, mask);

//recoverPose(Ess, points, points_transformed, rot, trans, 800, Point2d(300.8439, 196.4613), mask);

//rotx=atan2(rot.at<double>(2,1),rot.at<double>(2,2));
//roty=atan2(-rot.at<double>(2,0),sqrt(rot.at<double>(2,1)*rot.at<double>(2,1)+rot.at<double>(2,2)*rot.at<double>(2,2)));
//rotz=atan2(rot.at<double>(1,0),rot.at<double>(0,0));

ofstream myfile, myfile2;
myfile.open ("/home/harleen/Desktop/rot.txt",ios::app);
myfile2.open ("/home/harleen/Desktop/trans.txt",ios::app);
myfile << rot<<"\n";
myfile2 << trans<<"\n";
myfile.close();myfile2.close();
}
 
void perspective_transformer::show_diff() {
namedWindow("Difference", WINDOW_AUTOSIZE);
Mat diff=im_transformed - im_perspective_transformed;
imshow("Difference", diff);
waitKey(15);
}

void OdomCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
   pxp=msg->pose[0].position.x;
   pyp=msg->pose[0].position.y;
   pzp=msg->pose[0].position.z;
   rxp=msg->pose[0].orientation.x;
   ryp=msg->pose[0].orientation.y;
   rzp=msg->pose[0].orientation.z;
rwp=msg->pose[0].orientation.w;
}

int main(int argc, char** argv)
{VideoCapture cap(1);//VideoCapture cap("/home/harleen/Pictures/book.wmv");
ros::init(argc, argv, "talker");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, OdomCallback);
ros::Publisher chatter_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);
 
int f=0;
//check if the file was opened properly
if(!cap.isOpened())
{
cout << "Capture could not be opened successfully" << endl;
return -1;
}

namedWindow("Video");
 
// Play the video in a loop till it ends
while(char(waitKey(1)) != 'q' && cap.isOpened())
{//waitKey(50);
cap >> currFrame;

if (f==0)
{prevFrame=currFrame.clone();}
// Check if the video is over
if(currFrame.empty())
{
cout << "Video over" << endl;
break;
}
perspective_transformer a;
//waitKey(10);
a.estimate_perspective();
//a.show_diff();
imshow("Video", currFrame);
prevFrame  = currFrame.clone();
f=f+1;
ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, OdomCallback);
gazebo_msgs::ModelState msg;
    msg.model_name = "camera";
if (abs(trans.at<double>(0))<0.1 && abs(trans.at<double>(1))<0.1 && abs(trans.at<double>(2))<0.1) 
    {msg.pose.position.x = pxp+((trans.at<double>(0)*100))/100;
    msg.pose.position.y = pyp+((trans.at<double>(1)*100))/100;
    msg.pose.position.z = pzp+((trans.at<double>(2)*100))/100;}
else 
{msg.pose.position.x = pxp;
    msg.pose.position.y = pyp;
    msg.pose.position.z = pzp;
}
double wp = sqrt(1.0 + cos(rotx) * cos(roty) + cos(rotx)*cos(rotz) - sin(rotx) * sin(roty) * sin(rotz) + cos(roty)*cos(rotz)) / 2;
double xp = (cos(roty) * sin(rotz) + cos(rotx) * sin(rotz) + sin(rotx) * sin(roty) * cos(rotz)) / (4.0 * wp);
double yp = (sin(rotx) * cos(roty) + sin(rotx) * cos(rotz) + cos(rotx) * sin(roty) * sin(rotz)) / (4.0 * wp);
double zp = (-sin(rotx) * sin(rotz) + cos(rotx) * sin(roty) * cos(rotz) + sin(roty)) /(4.0 * wp);
if ((abs(RAD2DEG(rotx)) <4 ) &&(abs(RAD2DEG(roty)) <4 ) &&(abs(RAD2DEG(rotz)) <4 )) 
{    msg.pose.orientation.x = rxp+(xp*1)/1000;//rotx;
    msg.pose.orientation.y = ryp+(yp*1)/1000;//roty;
    msg.pose.orientation.z = rzp+(zp*1)/1000;//rotz;
    msg.pose.orientation.w = rwp+(wp*1)/1000;
 }
else
{
msg.pose.orientation.x = rxp;//rotx;
    msg.pose.orientation.y = ryp;//roty;
    msg.pose.orientation.z = rzp;//rotz;
    msg.pose.orientation.w = rwp;
}
   msg.twist.linear.x = 0;
    msg.twist.linear.y = 0;
    msg.twist.linear.z = 0;
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = 0;
 chatter_pub.publish(msg);
//pxp=msg.pose.position.x;
//pyp=msg.pose.position.y;
//pzp=msg.pose.position.z;*/
//return 0;
}
 
return 0;
}
