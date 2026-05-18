#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>

// 参考网址：https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include<opencv2/core/eigen.hpp>
//EIgen SVD libnary, may help you solve SVD （计算奇异值分解的）
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
using namespace std;
using namespace cv;
using namespace aruco;
using namespace Eigen;//for JacobiSVD


//global varialbles for aruco detector（一些全局变量的定义）
aruco::CameraParameters CamParam;//camera的参数（直接从文件中读取）

MarkerDetector MDetector;//检测出marker
vector<Marker> Markers;

float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat K, D;//定义相机的内参矩阵于畸变参数
 
//计算误差
double error_x=0;
double error_y=0;
double error_z=0;
double error_roll=0;
double error_pitch=0;
double error_yaw=0;
double rmse_x, rmse_y, rmse_z, rmse_roll, rmse_pitch, rmse_yaw;
double rmse_position, rmse_orientation;
int frame=0;
 
// test function, can be used to verify your estimation
//用于计算重投影误差（用于验证）
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        //输出得是：世界坐标得xyz；没有失真得xy；p为重投影得结果；而R和t是计算出来得
        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
               pts_3[i].x, pts_3[i].y, pts_3[i].z,
               un_pts_2[i].x, un_pts_2[i].y,
               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
    }
    puts("calculateReprojectionError ends");
}
 
// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);//通过opencv算出pnp（直接调用函数）
    //pts_3---特征点的世界坐标
    //pts_2---特征点在图像中的像素坐标
    //K--相机的内参矩阵
    //D--相机的畸变参数
    //rvec---输出的旋转向量
    //t---输出的平移向量
 
    cv::Rodrigues(rvec, r);//将旋转向量（1*3）转换为相对应的旋转矩阵（3*3）
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
    Quaterniond Q_ref;//定义四元素
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";//rviz中选的frame ID应该为world
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);
 
    // version 2, your work（相当于自己实现pnp算法）
    Matrix3d R;
    Vector3d T;
    R.setIdentity();//设置为单位矩阵
    T.setZero();//设置为全0
    vector<cv::Point2f> un_pts_2;
    uint num_points=pts_2.size();//定义观测到的特征点的数目（不带符号的int）
    cv::undistortPoints(pts_2, un_pts_2, K, D);//通过相机的参数矩阵及失真参数来对图像的点进行矫正；opencv中使用undistortPoints函数校正特征点（对于针孔相机）
    
    // Retrieve K from cv::Mat to eigen
    Matrix3d K_Matrix3d;
    for (uint i=0;i<3;i++){
        for (uint j=0;j<3;j++){
            K_Matrix3d(i,j)=K.at<double>(i,j);
        }
    }
 
    for(uint i=0; i < num_points;i++){
        Vector3d point;
        point<<un_pts_2[i].x, un_pts_2[i].y,1;
        Vector3d image_point = K_Matrix3d * point;
        un_pts_2[i].x = image_point(0);
        un_pts_2[i].y = image_point(1);
    }
    
    ROS_INFO("write your code here!");
    //...*******************
    //...
   
    MatrixXd obs_matrix (2*num_points,9);//创建观测矩阵。跟ppt27页的公式一个道理，当为一个观测点时，其观测矩阵为2*9
    //对观测矩阵进行初始化
    for (uint i=0;i<num_points;i++){
        VectorXd row1(9);//创建一个含有9个元素的向量
        //放入元素
        row1<<pts_3[i].x,pts_3[i].y,1,
        0,0,0,
        -pts_3[i].x*un_pts_2[i].x,-pts_3[i].y*un_pts_2[i].x,-un_pts_2[i].x;
        VectorXd row2(9);
        row2<<0,0,0,
        pts_3[i].x,pts_3[i].y,1,
        -pts_3[i].x*un_pts_2[i].y,-pts_3[i].y*un_pts_2[i].y,-un_pts_2[i].y;
        //将这两个向量放于观测矩阵中
        obs_matrix.row(2*i)  = row1;
        obs_matrix.row(2*i+1)= row2;
    }
 
    //然后，通过SVD解决Ax=0的问题。求出x即为所需要的H矩阵，再通过K（-1）H=（R T）来求解R与T。
    //对于Ax=0的问题，做法都是使用SVD分解，直接取V矩阵的最后一列作为方程的解。
    //eigen库如何通过svd方法取奇异矩阵的广义逆矩阵
    // JacobiSVD<MatrixXf> svd(obs_matrix, ComputeThinU | ComputeThinV);//关于其中得参数可参考：https://blog.csdn.net/xu_fengyu/article/details/103996945
    JacobiSVD<MatrixXd> svd(obs_matrix, ComputeThinU | ComputeFullV);// if only 4 points are provided, we need FullV
    //U = svd.matrixU();
    // V = svd.matrixV();
    // A = svd.singularValues(); （A为对角线元素，奇异值）
    MatrixXd V_matrix=svd.matrixV();//获取到得V矩阵
    //obs_matrix是一个（2*num_points）*9的矩阵，应该获得U为（2*num_points）*（2*num_points）；V为9*9
    // VectorXd H_matrix_vector=V_matrix.col(-1);//报错
    VectorXd H_matrix_vector=V_matrix.col(8);
 
    //构建H矩阵，将上面获得的向量H_matrix_vector放到矩阵中。
    Matrix3d H;//Matrix3d表示元素类型为double大小为3*3的矩阵变量
    for (uint i=0;i<3;i++){
        for (uint j=0;j<3;j++){
            H(i,j)=H_matrix_vector(i*3+j);
        }
    }
 
    // Matrix3d K_H=K.inverse()*H;//K是opencv中得Mat格式而H是Matrix3d，两者数据不一致？
    //（‘class cv::Mat’ has no member named ‘inverse’）
    //故此需要数据格式转换。
    // Matrix3d K_Matrix3d;
    // for (int i=0;i<3;i++){
    //     for (int j=0;j<3;j++){
    //         K_Matrix3d(i,j)=K.at<double>(i,j);
    //     }
    // }
    Matrix3d K_H=K_Matrix3d.inverse()*H;
    Vector3d h1,h2,h3;//PPT 29 and 30
    h1=K_H.col(0);
    h2=K_H.col(1);
    h3=K_H.col(2);
    Matrix3d R__;
    R__.col(0)=h1;
    R__.col(1)=h2;
    R__.col(2)=h1.cross(h2);
    // JacobiSVD<MatrixXf> svd1(R__, ComputeThinU | ComputeThinV);//关于其中得参数可参考：https://blog.csdn.net/xu_fengyu/article/details/103996945
    JacobiSVD<MatrixXd> svd1(R__, ComputeFullU | ComputeFullV);
    MatrixXd R__V=svd1.matrixV();
    MatrixXd R__U = svd1.matrixU();
    R=R__U*(R__V.transpose());
    T=h3/h1.norm();
    if (T(2) < 0){
        T = -T;
        R.col(0) = R.col(0) * -1;
        R.col(1) = R.col(1) * -1;
    }
    cout<<"R: " << R <<endl;
    cout<<"R_ref: " << R_ref <<endl;
    cout<<"T: " << T <<endl;
    cout<<"T_ref: " << t <<endl;
 
    //测量重投影误差
    // cv::Mat cv_R_ref,cv_T_ref,cv_R,cv_T;
    // eigen2cv(R, cv_R);
    // eigen2cv(T, cv_T);
    // cv_T_ref=t;
    // // eigen2cv(t, cv_T_ref);
    // eigen2cv(R_ref, cv_R_ref);
    // ROS_INFO("重投影误差!");
    // calculateReprojectionError(pts_3, pts_2, cv_R, cv_T);
    // ROS_INFO("重投影误差_reference!");
    // calculateReprojectionError(pts_3, pts_2, cv_R_ref, cv_T_ref);
 
 
    //...********************
    Quaterniond Q_yourwork;
    Q_yourwork = R;
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "world";
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    pub_odom_yourwork.publish(odom_yourwork);
    frame++;
 
    
    //转换成欧拉角
    Eigen::Vector3d eulerAngle_ref=Q_ref.matrix().eulerAngles(2,1,0);
    Eigen::Vector3d eulerAngle_yourwork=Q_yourwork.matrix().eulerAngles(2,1,0);
 
    ///计算误差
    error_x=odom_yourwork.pose.pose.position.x-odom_ref.pose.pose.position.x;
    error_y=odom_yourwork.pose.pose.position.y-odom_ref.pose.pose.position.y;
    error_z=odom_yourwork.pose.pose.position.z-odom_ref.pose.pose.position.z;
    rmse_position=sqrt((pow(rmse_position,2)*(frame-1)+pow(error_x,2)+pow(error_y,2)+pow(error_z,2))/frame);
    
    error_roll=eulerAngle_ref(0)-eulerAngle_yourwork(0);
    error_pitch=eulerAngle_ref(1)-eulerAngle_yourwork(1);
    error_yaw=eulerAngle_ref(2)-eulerAngle_yourwork(2);
    rmse_orientation=sqrt((pow(rmse_orientation,2)*(frame-1)+pow(error_roll,2)+pow(error_pitch,2)+pow(error_yaw,2))/frame);
 
    ROS_INFO("RMSE_position, RMSE_orientation: \n %f, %f",
             rmse_position, rmse_orientation);
}
 
cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}
 
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    //在opencv中图像采用Mat矩阵的形式存储
    //而ROS的图像则有自己的图像消息格式。
    //cv_bridge正是将两者联系在一起
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);//订阅的img_msg是ROS的消息
    MDetector.detect(bridge_ptr->image, Markers);//运用MarkerDetector类的函数来检测Marker
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);//将计算时间显示出来
 
    vector<int> pts_id;//存储ID
    vector<cv::Point3f> pts_3;//存储maker在世界坐标系下的位置
    vector<cv::Point2f> pts_2;//存储对应marker在图像坐标系下的位置
    for (unsigned int i = 0; i < Markers.size(); i++)  //unsigned无符号；Markers为检测出来的marker的数目
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);//获取所检测的第i个marker的ID
 
        char str[100];
        sprintf(str, "%d", idx);//发送格式化输出到 str 所指向的字符串
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }
 
        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));//在世界坐标系下的位置
            pts_2.push_back(Markers[i][j]);//在图像坐标系上的位置
        }
    }
 
    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);//在process这个函数中，进行相机姿态的估计并将发布
 
    cv::imshow("in", bridge_ptr->image);//通过指针将视频显示出来
    cv::waitKey(10);
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");
 
    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);//订阅录取的视频，并且回调函数img_callback
    //发布两个位姿
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);//自己算出来的
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);//参考值
    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);//直接从文件中读取相机的基本参数
    TheBoardConfig.readFromFile(board_config);
    //直接从文件中读取，获取了所有Marker的ID及其对应的角点的位置；通过二进制代码来识别ID与4个角点；而角点的位置是记录下来的world frame的位置（xyz）
 
    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    //直接从文件中读出相机的内存矩阵与畸变参数（后面要重点关注这是如何获取的，毕竟相机的参数model对最终的performance有很大的影响）
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;
 
    //init window for visualization
    cv::namedWindow("in", 1);
 
    ros::spin();
}
 
 
//整理记录于：
// https://blog.csdn.net/gwplovekimi/article/details/115245558?spm=1001.2014.3001.5501