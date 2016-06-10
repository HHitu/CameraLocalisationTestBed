#include <boost/bind.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include<math.h>
#include <vector>
#include "/home/harleen/snp2/devel/include/snpsir/data.h"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
Mat R1, R2, R3, R4;
Mat t;
Mat matE;
int stop1;
int stop2;
Mat currFrame, prevFrame;
vector<Mat> rvec2,tvec2,normals;
Mat rot;
Mat trans = cv::Mat::zeros(3, 1, CV_64F);
float fX,fY,fZ;
double rotx, roty, rotz;
double pxp,pyp,pzp,rxp,ryp,rzp,rwp;
double px,py,pz;
vector<KeyPoint> kp;
bool pattern_found2=0;
int one=0;
int two=0;
bool pattern_found=0;
int f;
vector<Point2f> imp2p;
vector<Point2f> imp1p;

class perspective_transformer {
private:
Mat im, im_transformed, im_perspective_transformed;
Mat M;

public:
perspective_transformer();
void estimate_perspective();
};
 
perspective_transformer::perspective_transformer() {
im = prevFrame.clone();
im_transformed = currFrame.clone();
}

void perspective_transformer::estimate_perspective() {
// Match point correspondences between the images
vector<KeyPoint> kp, t_kp;
Mat desc, t_desc, im_g, t_im_g;
vector<Point2f> points;
vector<Point2f> points_transformed;
vector<Point2f> im_p2;
vector<Point2f> im_p1;

//uncomment this section and comment the following section to use ORB features for feature correspondences and find camera movement
//this gave many false correspondences in the images esp. in cluttered scenes so better use RANSAC with ORB
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
points.push_back((kp[dm.trainIdx]).pt);
points_transformed.push_back((t_kp[dm.queryIdx]).pt);
}}*/

//this looks for a checkerboard pattern in the scene and uses its corners for correspondences hence better results but less utility
//find corners in the chessboard image
{pattern_found = findChessboardCorners(im, Size(9, 6), im_p1,
CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+ CALIB_CB_FAST_CHECK);
}
waitKey(5);

if(pattern_found) {
cout<<"found1"<<endl;
one=1;
Mat gray;
cvtColor(im, gray, CV_BGR2GRAY);
cornerSubPix(gray, im_p1, Size(9, 6), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS +
CV_TERMCRIT_ITER, 30, 0.1));
for(int i = 0; i < 14; i++) {
points.push_back(im_p1[i]);}
imp1p=im_p1;
waitKey(3);}
else if (f!=0)
{im_p1=imp1p;
for(int i = 0; i < 14; i++) {
points.push_back(im_p1[i]);
}
waitKey(3);
}

{pattern_found2 = findChessboardCorners(im_transformed, Size(9, 6), im_p2,
CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+ CALIB_CB_FAST_CHECK);}
waitKey(5);
if(pattern_found2) {
cout<<"found2"<<endl;
two=1;
Mat gray;
cvtColor(im_transformed, gray, CV_BGR2GRAY);
cornerSubPix(gray, im_p2, Size(9, 6), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS +
CV_TERMCRIT_ITER, 30, 0.1));
for(int i = 0; i < 14; i++) {
points_transformed.push_back(im_p2[i]);}
imp2p=im_p2;
waitKey(5);}
else if (f!=0)
{im_p2=imp2p;
for(int i = 0; i < 14; i++) {
points_transformed.push_back(im_p2[i]);
}
waitKey(3);
}

if (f!=0){
Mat im_show = im_transformed.clone();
drawChessboardCorners(im_show, Size(9, 6), im_p2, true);
imshow("Chessboard corners", im_show);
waitKey(5);
Mat M = findHomography(points, points_transformed, RANSAC, 2);
if (M.empty()==1)
stop1=1;
else stop1=0;
 
Mat dist(5,1,cv::DataType<double>::type);
dist.at<double>(0)= -0.004468;   //camera distortion matrix
dist.at<double>(1)=-0.120621;
dist.at<double>(2)=-0.012367;
dist.at<double>(3)=-0.006686;
dist.at<double>(4)= 0.00;
Mat cammat(3,3,cv::DataType<double>::type);
cammat.at<double>(0,0)=794.298570;      //camera intrinsic matrix
cammat.at<double>(0,1)=0.00;
cammat.at<double>(0,2)=300.8439;
cammat.at<double>(1,0)=0.00;
cammat.at<double>(1,1)=805.0919;
cammat.at<double>(1,2)=196.4613;
cammat.at<double>(2,0)=0.00;
cammat.at<double>(2,1)=0.00;
cammat.at<double>(2,2)=1.00;

vector<Mat> Rs, Ts;
Mat res(3,3,cv::DataType<double>::type);
Mat rest(3,3,cv::DataType<double>::type);
if (stop1==0){
cout << "Estimated Perspective transform = " << M << endl;

decomposeHomographyMat(M, cammat, Rs, Ts, cv::noArray());

  for (int i=0; i< Rs.size(); i++) {
    Mat1d rvec_n;
    Rodrigues(Rs[i], rvec_n);
   // cout << rvec_n*180/CV_PI << endl << endl;
  }

  cout << "t = " << endl;
   for (int i=0; i< Ts.size(); i++){
  //  cout << Ts[i] << endl << endl;
  }
}

Mat M_2 = findHomography(points_transformed, points, RANSAC, 2);
if (M_2.empty()==1)
stop2=1;
else stop2=0;
std::vector<cv::Mat> Rs_t, Ts_t;
 if (stop2==0) {

decomposeHomographyMat(M_2, cammat, Rs_t, Ts_t, cv::noArray());
 //cout << "rvec2 = " << endl;
for (int i=0; i< Rs_t.size(); i++) {
    Mat1d rvec_n_t;
    Rodrigues(Rs_t[i], rvec_n_t);
    cout << rvec_n_t*180/CV_PI << endl << endl;
  }
}

if ((stop1==0) && (stop2==0))           
{
vector<double> dr;
for (int i=0; i< Rs.size(); i++) {
for (int j=0; j< Rs_t.size(); j++) {
  Mat1d rvec_n_t;
  Mat1d rvec_n;
  Rodrigues(Rs[i], rvec_n);
  Rodrigues(Rs_t[j], rvec_n_t);
Mat1d d=abs(rvec_n)-abs(rvec_n_t);
double diff=d.at<double>(0)+d.at<double>(1)+d.at<double>(2);
dr.push_back(diff);
//cout <<"diff" <<i << j<< " " <<diff<<endl;
}}

vector<double> dt;
for (int i=0; i< Ts.size(); i++) {
for (int j=0; j< Ts_t.size(); j++) {
Mat1d d=abs(Ts[i])-abs(Ts_t[j]);
double diff=d.at<double>(0)+d.at<double>(1)+d.at<double>(2);
dt.push_back(diff);
//cout <<"diff" <<i << j<< " " <<diff<<endl;
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
{
ddar=index_r;
ddat=index_t1;
}
Mat1d rvec;

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
std::cout << "issue_rot " << std::endl;
}

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
cout << "issue_trans " << endl;
}

//uncomment this section if you want to use the OpenCv function to find the Essential Matrix and recover pose using the ORB point correspondences
//however this was not giving correct results. Also if your camera has fx different than fy then there is no input argument to specify them in the function therefore, one may
//compute the fundamental matrix using findFundamentalMat OpenCv function and use it and the camera matrix to compute the essential matrix and find out the rotation and translations using 
//SVD and find the correct rotation and translation out of the 4 options using the reprojection constraint (code in vision_for_images.cpp)
/*Mat mask;
Mat Ess=findEssentialMat(points, points_transformed, 800, Point2d(300.8439, 196.4613), RANSAC, 0.999, 1.0, mask);
recoverPose(Ess, points, points_transformed, rot, trans, 800, Point2d(300.8439, 196.4613), mask);

rotx=atan2(rot.at<double>(2,1),rot.at<double>(2,2));
roty=atan2(-rot.at<double>(2,0),sqrt(rot.at<double>(2,1)*rot.at<double>(2,1)+rot.at<double>(2,2)*rot.at<double>(2,2)));
rotz=atan2(rot.at<double>(1,0),rot.at<double>(0,0));
*/

//writing data to files
ofstream myfile, myfile2;
myfile.open ("/home/harleen/Desktop/rot.txt",ios::app);
myfile2.open ("/home/harleen/Desktop/trans.txt",ios::app);
myfile << rot<<"\n";
myfile2 << trans<<"\n";
myfile.close();myfile2.close();
}}
}
 
void OdomCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
/*   pxp=msg->pose[0].position.x;
   pyp=msg->pose[0].position.y;
   pzp=msg->pose[0].position.z;
   rxp=msg->pose[0].orientation.x;
   ryp=msg->pose[0].orientation.y;
   rzp=msg->pose[0].orientation.z;
rwp=msg->pose[0].orientation.w;
*/
}

int main(int argc, char** argv)
{VideoCapture cap(0);
ros::init(argc, argv, "talker");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, OdomCallback);
ros::Publisher chatter_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);
pxp=0;
pyp=-1;
pzp=1.05;

f=0;
//check if the file was opened properly
if(!cap.isOpened())
{
cout << "Capture could not be opened successfully" << endl;
return -1;
}

namedWindow("Video");
 
// Play the video in a loop till it ends
while(char(waitKey(5)) != 'q' && cap.isOpened())
{
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
a.estimate_perspective();

imshow("Video", currFrame);
cout << "f= " << f << endl; 
prevFrame  = currFrame.clone();
if ((one==1) && (two==1))
f=1;
if ((f>0) && (stop1==0) && (stop2==0))
{
ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, OdomCallback);
gazebo_msgs::ModelState msg;
    msg.model_name = "camera";
    {msg.pose.position.x = pxp+((trans.at<double>(0)*100))/1;
    msg.pose.position.y = pyp+((trans.at<double>(1)*100))/1;
    msg.pose.position.z = pzp+((trans.at<double>(2)*100))/1;}
double wp = sqrt(1.0 + cos(rotx) * cos(roty) + cos(rotx)*cos(rotz) - sin(rotx) * sin(roty) * sin(rotz) + cos(roty)*cos(rotz)) / 2;
double xp = (cos(roty) * sin(rotz) + cos(rotx) * sin(rotz) + sin(rotx) * sin(roty) * cos(rotz)) / (4.0 * wp);
double yp = (sin(rotx) * cos(roty) + sin(rotx) * cos(rotz) + cos(rotx) * sin(roty) * sin(rotz)) / (4.0 * wp);
double zp = (-sin(rotx) * sin(rotz) + cos(rotx) * sin(roty) * cos(rotz) + sin(roty)) /(4.0 * wp);
{    msg.pose.orientation.x = rxp+(xp*1)/1;//rotx;
    msg.pose.orientation.y = ryp+(yp*1)/1;//roty;
    msg.pose.orientation.z = rzp+(zp*1)/1;//rotz;
    msg.pose.orientation.w = (wp*1)/1;
 }
   msg.twist.linear.x = 0;
    msg.twist.linear.y = 0;
    msg.twist.linear.z = 0;
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = 0;
 chatter_pub.publish(msg);
pxp=msg.pose.position.x;
pyp=msg.pose.position.y;
pzp=msg.pose.position.z;
rxp=msg.pose.orientation.x;
ryp=msg.pose.orientation.y;
rzp=msg.pose.orientation.z;
rwp=msg.pose.orientation.w;
cout << "done publishing" <<endl;
}
 }
return 0;
}
