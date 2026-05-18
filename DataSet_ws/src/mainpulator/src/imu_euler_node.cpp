//给的欧拉角是有问题的
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <stdlib.h>
#include<iostream>
#include <vector>
#include <random>
#include <algorithm>
#include "geometry_msgs/Point.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>

#define PI acos(-1)
#define time 0.01;
using namespace std;
using namespace Eigen;
double R,P,Y;
double camera_R,camera_P,camera_Y;
double X_estimatepose,Y_estimatepose,Z_estimatepose;
double smooth_X_estimatepose,smooth_Y_estimatepose,smooth_Z_estimatepose;

double X_estipose_last,Y_estipose_last,Z_estipose_last;
bool pose_estimate_flag =false;
bool pose_flag =false;
double R_initial, P_initial, Y_initial;
bool  initial_flag=false;
void euler_angles_Callback(const geometry_msgs::Vector3 & msg);
void imu_msg_Callback(const sensor_msgs::Imu& imu_raw_msg);

void camera_EstimatePose_msg_Callback(const sensor_msgs::JointState& joint_state);

Vector3d Imu_kalman_filtering(double Angular_Velocity_x, double Angular_Velocity_y, double Angular_Velocity_z,double Linelar_Acceleration_x, double Linelar_Acceleration_y, double Linelar_Acceleration_z);


//粒子滤波相关
// 定义粒子结构体
struct Particle {
    double x; // 粒子的x坐标
    //double y; // 粒子的y坐标
    double weight; // 粒子的权重
};

class ParticleFilter {
private:
    std::vector<Particle> particles; // 粒子集合
    std::default_random_engine generator; // 随机数生成器
    std::normal_distribution<double> normal_dist; // 正态分布辅助生成器
    double stdDev; // 测量误差的标准差
    geometry_msgs::Point estimatedPosition; // 估计的位置

public:
    ParticleFilter(int num_particles, double std_dev)
    : stdDev(std_dev), generator(std::random_device{}()), normal_dist(0.0, stdDev) {
        for (int i = 0; i < num_particles; ++i) {
            Particle p;
            p.x = normal_dist(generator); // 随机初始化粒子位置
            //p.y = normal_dist(generator);
            p.weight = 1.0 / num_particles; // 均匀初始权重
            particles.push_back(p);
        }
    }

    // 2计算估计位置
    void computeEstimatedPosition() {
        double sumX = 0.0, sumY = 0.0;
        for (const auto& particle : particles) {
            sumX += particle.x * particle.weight;
            //sumY += particle.y * particle.weight;
        }
        estimatedPosition.x = sumX;
        //estimatedPosition.y = sumY;
    }

//3重采样
void resampleParticles() {
    std::vector<Particle> newParticles(particles.size());
    std::vector<double> cumulativeWeights;
    double sum = 0.0;

    // 计算累积权重
    for (const auto& particle : particles) {
        sum += particle.weight;
        cumulativeWeights.push_back(sum);
    }

    // 重采样过程
    for (int i = 0; i < particles.size(); ++i) {
        // 生成[0,1]区间的随机数，用于在累积权重数组中选择粒子
        double u = std::uniform_real_distribution<double>(0.0, 1.0)(generator);
        // 根据随机数u和累积权重找到对应的粒子
        auto it = std::lower_bound(cumulativeWeights.begin(), cumulativeWeights.end(), u);
        int index = it - cumulativeWeights.begin();
        Particle selectedParticle = particles[index];

        // 以估计位置为中心，生成新粒子的位置
        // 假设生成的粒子位置符合以estimatedPosition为中心的正态分布
        newParticles[i].x = normal_dist(generator, std::normal_distribution<double>::param_type(estimatedPosition.x, stdDev));
        //newParticles[i].y = normal_dist(generator, std::normal_distribution<double>::param_type(estimatedPosition.y, stdDev));
        newParticles[i].weight = selectedParticle.weight;
    }
    particles.swap(newParticles); // 用新粒子替换旧粒子
}
//1
     void update(double measuredX1, double measuredY1, double measuredX2, double measuredY2) {
        //double stdDev = 0.5; // 假设标准差为0.5，可以根据实际情况调整
        double weightSum = 0.0;

        // 更新每个粒子的权重
        for (auto& particle : particles) {
            double distance1 = std::sqrt(std::pow(measuredX1 - particle.x, 2) );
            double distance2 = std::sqrt(std::pow(measuredX2 - particle.x, 2) );
            particle.weight = std::exp(-(1.6*distance1 + 0.4*distance2) / (2 * stdDev * stdDev));
            //double distance1 = std::sqrt(std::pow(measuredX1 - particle.x, 2) + std::pow(measuredY1 - particle.y, 2));
            //double distance2 = std::sqrt(std::pow(measuredX2 - particle.x, 2) + std::pow(measuredY2 - particle.y, 2));
            //particle.weight = std::exp(-(distance1 + distance2) / (2 * stdDev * stdDev));
            weightSum += particle.weight;
        }

        // 归一化权重
        for (auto& particle : particles) {
            particle.weight /= weightSum;
        }
    }
    // 4获取估计位置
    geometry_msgs::Point getEstimatedPosition() const {
        return estimatedPosition;
    }
};



double theta1=0.0;
double theta2=0.0;
double theta3=0.0;
double theta4=0.0;
double d1 = 129.5;
double a2 = 424.0;
double d2 = -119.5;
double d3 = 119.5;
double d4 = 340;
//double d4 = 268;

//----以下是imu数据预处理变量，加速度计和线速度计
//double dt=0.01;
double Ki=0.05;
double Kp=0.5;
double Angular_Velocity_x=0.001;
double Angular_Velocity_y=0.001;
double Angular_Velocity_z=0.001;
double Linelar_Acceleration_x=0.001;
double Linelar_Acceleration_y=0.001;
double Linelar_Acceleration_z=0.001;

double q0_=0.0;
double q1_=0.0;
double q2_=0.0;
double q3_=0.0;

double pitch;
double roll;
double yaw;
//----
double q0_etoq=0.0;
double q1_etoq=0.0;
double q2_etoq=0.0;
double q3_etoq=0.0;

//kalmanfiltering
Vector3d rpy_kalman;
Matrix3d Q_=0.002*Matrix3d::Identity();
Matrix3d R_=0.2*Matrix3d::Identity();
Matrix3d A_=Matrix3d::Identity();
Matrix3d B_=Matrix3d::Identity();
Matrix3d H_=Matrix3d::Identity();


Vector3d X_bar_k_1=Vector3d::Zero();
Vector3d u_k=Vector3d::Zero();
Vector3d X_bar_k__=Vector3d::Zero();
Vector3d X_bar_k=Vector3d::Zero();
Vector3d Z_k=Vector3d::Zero();

Matrix3d P_k_1=Matrix3d::Zero();
Matrix3d P_k__=Matrix3d::Zero();
Matrix3d P_k=Matrix3d::Zero();
Matrix3d K_k=Matrix3d::Zero();
Matrix3d I=Matrix3d::Identity();
///////////////////////////////////////////////////////////
MatrixXd  T (4,4);
MatrixXd  T_shake (4,4);
MatrixXd  T0_inv (4,4);
MatrixXd  T_upd (4,4);
MatrixXd  T1_upd (4,4);
MatrixXd  T2_upd (4,4);
MatrixXd  T3_upd (4,4);
MatrixXd  T4_upd (4,4);
MatrixXd  T_upd_test (4,4);
MatrixXd  T0 (4,4);
MatrixXd  T1 (4,4);
MatrixXd  T2 (4,4);
MatrixXd  T3 (4,4);
MatrixXd  T4 (4,4);

Vector3d theta_compensation;
Vector3d smooth_theta_compensation;
Vector3d Smooth_theta_shake;
VectorXd  Shake_Compensation(double R,double P,double Y,double X_,double Y_,double Z_);
VectorXd  Shake_Compensation(double R,double P,double Y,double X_,double Y_,double Z_)
{
    Vector3d theta_shake;
	double theta1_upd;
	double theta2_upd;
	double theta3_upd;
	double theta_m;
	double theta_n;
	double q1 = R;
	double q2 = P;
	double q3 = Y;
	double x1 = X_;
	double y1 = Y_;
	double z1 = Z_;

	T1 << cos(theta1), 0 ,sin(theta1), 0 ,
				sin(theta1), 0 ,-1.0*cos(theta1), 0 ,
				0, 1 , 0 , d1,
				0, 0 , 0 , 1;

	T2 << cos(theta2), -1.0*sin(theta2) ,0, a2*cos(theta2) ,
				sin(theta2), cos(theta2) , 0 , a2*sin(theta2),
				0, 0 , 1 , d2,
				0, 0 , 0 , 1;

	T3 << cos(theta3), 0 ,sin(theta3), 0 ,
				sin(theta3), 0 ,-1.0*cos(theta3), 0 ,
				0, 1 , 0 , d3,
				0, 0 , 0 , 1;

	T4 << cos(theta4), -1.0*sin(theta4) , 0 , 0 ,
				sin(theta4), cos(theta4) , 0 , 0 ,
				0, 0 , 1 , d4,
				0, 0 , 0 , 1;
	
	T=T1*T2*T3*T4;

	T0<< cos(q2)*cos(q3) , -1.0*cos(q2)*sin(q3) , sin(q2) , x1 ,
				cos(q3)*sin(q1)*sin(q2)+cos(q1)*sin(q3) , cos(q1)*cos(q3)-sin(q1)*sin(q2)*sin(q3) , -1.0*cos(q2)*sin(q1) , y1 ,
				-1.0*cos(q1)*cos(q3)*sin(q2)+sin(q1)*sin(q3) , cos(q3)*sin(q1)+cos(q1)*sin(q2)*sin(q3) , cos(q1)*cos(q2), z1 ,
				0 , 0 , 0 , 1;
				
	T_shake=T0*T1*T2*T3*T4;
	//ROS_INFO("T_SHAKE= %lf,%lf,%lf",T_shake(0,3), T_shake(1,3), T_shake(2,3));

	T0_inv=T0.inverse();
	//ROS_INFO("T0_inv= %lf,%lf,%lf",T0_inv(0,3), T0_inv(1,3), T0_inv(2,3));

	T_upd=T0_inv*T;
	//ROS_INFO("T_upd= %lf,%lf,%lf",T_upd(0,3), T_upd(1,3), T_upd(2,3));


	theta1_upd=atan2(T_upd(1,3),T_upd(0,3));
	theta_m=atan2(T_upd(2,3)-d1,sqrt(T_upd(1,3)*T_upd(1,3)+T_upd(0,3)*T_upd(0,3)));
	theta_n=acos(   (a2*a2 + T_upd(1,3)*T_upd(1,3)+ T_upd(0,3)*T_upd(0,3)  + (T_upd(2,3)-d1)*(T_upd(2,3)-d1) - d4*d4) /  (2.0*a2*sqrt(  (T_upd(1,3)*T_upd(1,3)) + (T_upd(0,3)*T_upd(0,3)) +  ((T_upd(2,3)-d1)*(T_upd(2,3)-d1))  )  )     );

	theta2_upd=theta_m+theta_n;
	theta3_upd=-0.5*PI+acos( (d4*d4+a2*a2-1.0*(T_upd(1,3)*T_upd(1,3))-1.0*(T_upd(0,3)*T_upd(0,3))-1.0*((T_upd(2,3)-d1)*(T_upd(2,3)-d1)) )/(2.0*a2*d4));

	//ROS_INFO("theta123mn_upd= %lf,%lf,%lf,%lf,%lf",theta1_upd, theta_m, theta_n,theta2_upd,theta3_upd);

	T1_upd<< cos(theta1_upd), 0 ,sin(theta1_upd), 0 ,
				sin(theta1_upd), 0 ,-1.0*cos(theta1_upd), 0 ,
				0, 1 , 0 , d1,
				0, 0 , 0 , 1;
	T2_upd << cos(theta2_upd), -1.0*sin(theta2_upd) ,0, a2*cos(theta2_upd) ,
				sin(theta2_upd), cos(theta2_upd) , 0 , a2*sin(theta2_upd),
				0, 0 , 1 , d2,
				0, 0 , 0 , 1;
	T3_upd << cos(theta3_upd), 0 ,sin(theta3_upd), 0 ,
				sin(theta3_upd), 0 ,-1.0*cos(theta3_upd), 0 ,
				0, 1 , 0 , d3,
				0, 0 , 0 , 1;
	T4_upd << cos(theta4), -1.0*sin(theta4) , 0 , 0 ,
				sin(theta4), cos(theta4) , 0 , 0 ,
				0, 0 , 1 , d4,
				0, 0 , 0 , 1;

	T_upd_test=T0*T1_upd*T2_upd*T3_upd*T4_upd;

	//ROS_INFO("RPY&position= %lf,%lf,%lf,%lf,%lf,%lf",q1,q2,q3,x1,y1, z1);
		
	theta_shake(0)=theta1_upd;
	theta_shake(1)=theta2_upd;
	theta_shake(2)=theta3_upd;

	//ROS_INFO("theta_shake= %lf,%lf,%lf",theta_shake(0), theta_shake(1), theta_shake(2));
	return  theta_shake;

	}

//卡尔曼融合 两路数据 solvepnp的RVEC转rpy，以及imu获得的rpy；
class KalmanFilter {
private:
    Vector3d x; // 状态向量
    Matrix3d P; // 状态协方差矩阵
    Matrix3d Q; // 过程噪声协方差矩阵
    Matrix3d R; // 测量噪声协方差矩阵
    Matrix3d A; // 状态转移矩阵
    Matrix3d H; // 测量矩阵
    Vector3d x0; // 状态向量
	Vector3d x1; // 状态向量
	Vector3d x2; // 状态向量
public:
    KalmanFilter() {
        x.setZero(); // 初始化状态向量
        P.setIdentity(); // 初始化状态协方差矩阵
        Q.setIdentity(); // 初始化过程噪声协方差矩阵
        R.setIdentity(); // 初始化测量噪声协方差矩阵
	    A.setIdentity(); 
        H.setIdentity(); 	
		x0.setZero(); 
		x1.setZero(); 
		x2.setZero(); 
    }

    void predict() {
        x = A * x; // 预测状态
        P = A * P * A.transpose() + Q; // 预测状态协方差
    }

    void correct(const Vector3d& z, const Matrix3d& R_meas) {
        Matrix3d K = P * H.transpose() * (H * P * H.transpose() + R_meas).inverse(); // 计算卡尔曼增益
        x = x + K * (z - H * x); // 更新状态
		x0=x1;
		x1=x2;
		x2=x;
		//ROS_INFO("xx= %lf,%lf,%lf",x0(0),x1(0),x2(0));
        P = (MatrixXd::Identity(3, 3) - K * H) * P; // 更新状态协方差
    }

    void setA(const Matrix3d& A_mat) { A = A_mat; } // 设置状态转移矩阵 A
    void setH(const Matrix3d& H_mat) { H = H_mat; } // 设置测量矩阵 H
    void setQ(const Matrix3d& Q_mat) { Q = Q_mat; } // 设置过程噪声协方差矩阵 Q
    void setR(const Matrix3d& R_mat) { R = R_mat; } // 设置测量噪声协方差矩阵 R
    Vector3d getState() const { return x; } // 获取状态向量 x
    Matrix3d getP() const { return P; } // 获取p
};

KalmanFilter kf_imu;
KalmanFilter kf_camera;
Matrix3d Q_imu, R_imu, Q_camera, R_camera;



void runKalmanFilter(Vector3d& imu_measurement,Vector3d& camera_measurement, Vector3d& fused_state) 
{
    Vector3d z_imu, z_camera;
	z_imu=imu_measurement;
	z_camera=camera_measurement;
	kf_imu.predict();
    kf_imu.correct(z_imu, R_imu);
    kf_camera.predict();
    kf_camera.correct(z_camera, R_camera);

    Matrix3d P_imu = kf_imu.getP();
    Matrix3d P_camera = kf_camera.getP();
	Matrix3d P_fused = (P_imu.inverse() + P_camera.inverse()).inverse();
    Vector3d x_fused = P_fused * (P_imu.inverse() * kf_imu.getState() + P_camera.inverse() * kf_camera.getState());
	fused_state=x_fused;
	//ROS_INFO("`````PPPP= %lf,%lf,%lf",P_imu(0,0), P_camera(0,0), P_fused(0,0));
}

    Vector3d imu_m(0.1, 0.2, 0.3);
    Vector3d camera_m(0.1, 0.1, 0.1);
    Vector3d result3d(0, 0, 0);
	Vector3d i_count(1,1, 1);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Imu_Euler_Node");
	ros::NodeHandle n;
	ros::Publisher Imu_Info_pub = n.advertise<sensor_msgs::JointState>("Imu_Info_", 100);

	ros::NodeHandle Imu_Euler;
    ros::Subscriber Imu_Euler_Sub=Imu_Euler.subscribe("/euler_angles",10,euler_angles_Callback);

	ros::NodeHandle Imu_Msg;
    ros::Subscriber Imu_Msg_Sub=Imu_Msg.subscribe("/imu",10,imu_msg_Callback);

	ros::NodeHandle camera_EstimatePose_Msg;
    ros::Subscriber camera_Msg_Sub=camera_EstimatePose_Msg.subscribe("/camera_EstimatePose",10,camera_EstimatePose_msg_Callback);


	ros::Rate loop_rate(60);
	H_(2,2)=0.0;
	B_=B_*time;

    Q_imu << 0.1, 0, 0,
             0, 0.1, 0,
             0, 0, 0.1;
    R_imu << 0.05, 0, 0,
             0, 0.05, 0,
             0, 0, 0.05;
    Q_camera << 0.1, 0, 0,
                0, 0.1, 0,
                0, 0, 0.1;
    R_camera << 0.5, 0, 0,
                0, 0.5, 0,
                0, 0, 0.5;
    kf_imu.setQ(Q_imu);
    kf_imu.setR(R_imu);
    kf_camera.setQ(Q_camera);
    kf_camera.setR(R_camera);

	Smooth_theta_shake(0)=0;
	Smooth_theta_shake(1)=0;
	Smooth_theta_shake(2)=0;
	smooth_X_estimatepose=0;
	smooth_Y_estimatepose=0;
	smooth_Z_estimatepose=0;


	//double accu_error=0;
	//double R_accu=0;
	//double integral_error=0;
	//double error=0;
	//double R_corret=0;



    int num_particles = 5000;
    ParticleFilter filter(num_particles,0.4);
	geometry_msgs::Point estimate;
    while (ros::ok)
    {

 		sensor_msgs::JointState Imu;
        Imu.header.stamp = ros::Time::now();
        Imu.position.resize(8);
        Imu.velocity.resize(8);
        Imu.effort.resize(16);
        Imu.position[0] = R;
        Imu.position[1] = P;
        Imu.position[2] = Y;
		Imu.position[3] = 0;
		Imu.position[4] = 0;
		Imu.position[5] = 0;
		Imu.position[6] = 0;	

		Imu.velocity[0]=camera_R;
		Imu.velocity[1]=imu_m(0);
		Imu.velocity[2]=camera_P;
		Imu.velocity[3]=imu_m(1);
		Imu.velocity[4]=camera_Y;
		Imu.velocity[5]=imu_m(2);		
		Imu.velocity[6]=result3d(0);

		Imu.effort[0]=(R/180)*PI;
		Imu.effort[1]=-(P/180)*PI;
		Imu.effort[2]=-(Y/180)*PI;
		Imu.effort[3]=X_estimatepose;
		Imu.effort[4]=Y_estimatepose;
		Imu.effort[5]=Z_estimatepose;
		Imu.effort[6]=theta_compensation(0);
		Imu.effort[7]=theta_compensation(1);
		Imu.effort[8]=theta_compensation(2);
		Imu.effort[9]=Smooth_theta_shake(0);
		Imu.effort[10]=Smooth_theta_shake(1);
		Imu.effort[11]=Smooth_theta_shake(2);
		Imu.effort[12]=smooth_X_estimatepose;
		Imu.effort[13]=smooth_Y_estimatepose;
		Imu.effort[14]=smooth_Z_estimatepose;
		Imu.effort[15]=pose_flag;

		//Imu.effort[28]=R_corret;
		//Imu.effort[29]=R;
		//Imu.effort[30]=camera_R;
		//Imu.effort[31]=estimate.x;
        Imu_Info_pub.publish(Imu);
		ROS_INFO("theta_x_= %lf,%lf,%lf,%lf,%lf,%lf",Imu.effort[0], Imu.effort[1], Imu.effort[2],Imu.effort[3],Imu.effort[4],Imu.effort[5]);
        
		
		double measuredX1 = R;
        double measuredY1 = P;
        double measuredX2 = camera_R;
        double measuredY2 = camera_P;


		//accu_error=accu_error+0.005;
		//R_accu=R+accu_error;
		//error=camera_R-R_accu;
		//integral_error=integral_error+error;
		//R_corret=R_accu+0.0005*integral_error;


		//filter.update(measuredX1,0 ,measuredX2,0);
		//filter.computeEstimatedPosition();
		//filter.resampleParticles();
		//estimate=filter.getEstimatedPosition();


		//ROS_INFO("particleFilter_estimated_pose= %lf,%lf,%lf",measuredX1,measuredX2,estimate.x);
    	// 更新粒子权重
    	//updateWeights(particles, measuredX1, measuredY1, measuredX2, measuredY2, stdDev);
        // 重新采样粒子群
        //particles = resampleParticles(particles);
        // 估计位置
        //double estimatedX, estimatedY;
        //estimatePosition(particles, estimatedX, estimatedY);
		//ROS_INFO("particleFilter_estimated_pose= %lf,%lf,%lf,%lf",R,camera_R,estimatedX,estimatedY);



		theta_compensation=Shake_Compensation((R/180)*PI,-(P/180)*PI,-(Y/180)*PI,X_estimatepose,Y_estimatepose,Z_estimatepose);
		
		smooth_X_estimatepose=0.9*smooth_X_estimatepose+0.1*X_estimatepose;
		smooth_Y_estimatepose=0.9*smooth_Y_estimatepose+0.1*Y_estimatepose;
		smooth_Z_estimatepose=0.9*smooth_Z_estimatepose+0.1*Z_estimatepose;

		smooth_theta_compensation=Shake_Compensation((R/180)*PI,-(P/180)*PI,-(Y/180)*PI,smooth_X_estimatepose,smooth_Y_estimatepose,smooth_Z_estimatepose);
		Smooth_theta_shake(0)=0.9*Smooth_theta_shake(0)+0.1*smooth_theta_compensation(0);
		Smooth_theta_shake(1)=0.9*Smooth_theta_shake(1)+0.1*smooth_theta_compensation(1);
		Smooth_theta_shake(2)=0.9*Smooth_theta_shake(2)+0.1*smooth_theta_compensation(2);

		
		
		//i_count(0)++;
		//i_count(1)++;
		//i_count(2)++;
		imu_m(0)=R;
		imu_m(1)=P;
		imu_m(2)=Y;
		camera_m(0)=camera_R;
		camera_m(1)=camera_P;
		camera_m(2)=camera_Y;
		runKalmanFilter(imu_m,camera_m,result3d);
		//ROS_INFO("runKalmanFilter= %lf,%lf,%lf",result3d(0), result3d(1), result3d(2));

		ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}

void euler_angles_Callback(const  geometry_msgs::Vector3 & msg)
{
	if(initial_flag==false)
	{
		Y_initial=msg.z;
		initial_flag=true;
	}

    R = msg.x;
    P = msg.y;
    Y = msg.z-Y_initial; 
	if(Y<0)
	{
		Y=Y+2*PI;
	}
	else if(Y>2*PI)
	{
		Y=Y-2*PI;
	}

	if(Y>PI)
	{
		Y=Y-2*PI;
	}
	R=(R/PI)*180;
	P=(P/PI)*180;
	Y=(Y/PI)*180;
	//ROS_INFO("RPY= %lf,%lf,%lf",R, P, Y);
	q0_etoq=cos((msg.x)/2)*cos((msg.y)/2)*cos((msg.z)/2)+sin((msg.x)/2)*sin((msg.y)/2)*sin((msg.z)/2);
	q1_etoq=sin((msg.x)/2)*cos((msg.y)/2)*cos((msg.z)/2)-cos((msg.x)/2)*sin((msg.y)/2)*sin((msg.z)/2);
	q2_etoq=cos((msg.x)/2)*sin((msg.y)/2)*cos((msg.z)/2)+sin((msg.x)/2)*cos((msg.y)/2)*sin((msg.z)/2);
	q3_etoq=cos((msg.x)/2)*cos((msg.y)/2)*sin((msg.z)/2)-sin((msg.x)/2)*sin((msg.y)/2)*cos((msg.z)/2);
	//ROS_INFO("wxyz= %lf,%lf,%lf,%lf",q0_etoq, q1_etoq, q2_etoq,q3_etoq);
}

void imu_msg_Callback(const sensor_msgs::Imu& imu_raw_msg)
{
	double pitch,roll,yaw;
	Angular_Velocity_x=imu_raw_msg.angular_velocity.x;
	Angular_Velocity_y=imu_raw_msg.angular_velocity.y;
	Angular_Velocity_z=imu_raw_msg.angular_velocity.z;
	Linelar_Acceleration_x=imu_raw_msg.linear_acceleration.x;
	Linelar_Acceleration_y=imu_raw_msg.linear_acceleration.y;
	Linelar_Acceleration_z=imu_raw_msg.linear_acceleration.z;
	q1_=imu_raw_msg.orientation.x;
	q2_=imu_raw_msg.orientation.y;
	q3_=imu_raw_msg.orientation.z;
	q0_=imu_raw_msg.orientation.w;
	//pitch=asin(-2.0*q1_*q3_+2.0*q0_*q2_);
	//roll=atan2((2.0*q2_*q3_+2.0*q0_*q1_),(-2.0*q1_*q1_-2*q2_*q2_+1.0));
	//yaw=atan2((2*(q1_*q2_+q0_*q3_)),(q0_*q0_+q1_*q1_-q2_*q2_-q3_*q3_));
	//ROS_INFO("4to rpy= %lf,%lf,%lf",roll, pitch,yaw);
}
void camera_EstimatePose_msg_Callback(const sensor_msgs::JointState& joint_state)
{
	pose_flag=false;
	//相机直接返回的rpy
	camera_R=-joint_state.effort[0];
	camera_P=joint_state.effort[1];
	camera_Y=-joint_state.effort[2];
	X_estimatepose=10*joint_state.effort[3]*0.925;
	Y_estimatepose=-10*joint_state.effort[4];
	Z_estimatepose=10*joint_state.effort[5];

	if(pose_estimate_flag==false)
	{
		X_estipose_last=10*joint_state.effort[3]*0.925;
		Y_estipose_last=-10*joint_state.effort[4];
		Z_estipose_last=10*joint_state.effort[5];
		pose_estimate_flag=true;
	}

	if(abs(X_estimatepose-X_estipose_last)>100||abs(Y_estimatepose-Y_estipose_last)>100||abs(Z_estimatepose-Z_estipose_last)>100)
	{
		X_estimatepose=X_estipose_last;
		Y_estimatepose=Y_estipose_last;
		Z_estimatepose=Z_estipose_last;
		pose_flag=true;
	}

		X_estipose_last=X_estimatepose;
		Y_estipose_last=Y_estimatepose;
		Z_estipose_last=Z_estimatepose;
	

	if(camera_Y<0&&camera_Y>=-180)
	{
	camera_Y=camera_Y+180;
	}
	else if(camera_Y>0&&camera_Y<=180)
	{
	camera_Y=camera_Y-180;
	}
	else{
	}
}



Vector3d Imu_kalman_filtering(double Angular_Velocity_x, double Angular_Velocity_y, double Angular_Velocity_z,double Linelar_Acceleration_x, double Linelar_Acceleration_y, double Linelar_Acceleration_z)
{

	//计算xy轴上角速度
double droll_dt;
double dpitch_dt;
double dyaw_dt;
double acc_roll;
double acc_pitch;
//double acc_yaw;

droll_dt=Angular_Velocity_x+(sin(X_bar_k(1))*sin(X_bar_k(0))/cos(X_bar_k(1)))*Angular_Velocity_y+(sin(X_bar_k(1))*cos(X_bar_k(0))/cos(X_bar_k(1)))*Angular_Velocity_z;
dpitch_dt=cos(X_bar_k(0))*Angular_Velocity_y-sin(X_bar_k(0))*Angular_Velocity_z;
dyaw_dt=(sin(X_bar_k(0))/cos(X_bar_k(0)))*Angular_Velocity_y+(cos(X_bar_k(0))/cos(X_bar_k(1)))*Angular_Velocity_z;

u_k(0)=droll_dt;
u_k(1)=dpitch_dt;
u_k(2)=dyaw_dt;
//第一步计算状态外推方程
X_bar_k__=A_*X_bar_k_1+B_*u_k;
//第二步计算 协方差外推方程
P_k__=A_*P_k_1*A_.transpose()+Q_;
//第三步计算 卡尔曼增益
K_k=P_k__*H_.transpose()*(H_*P_k__*H_.transpose()+R_).inverse();
//第四步计算，更新协方差矩阵
P_k=(I-K_k*H_)*P_k__;

//观测量计算
acc_roll=atan2(Linelar_Acceleration_y,Linelar_Acceleration_z);
acc_pitch=-1.0*atan2(Linelar_Acceleration_x,sqrt(Linelar_Acceleration_y*Linelar_Acceleration_y+Linelar_Acceleration_z*Linelar_Acceleration_z));
//观测矩阵赋值
Z_k(0)=acc_roll;
Z_k(1)=acc_pitch;
Z_k(2)=0;
//第五步计算 更新状态量
X_bar_k=X_bar_k__+K_k*(Z_k-H_*X_bar_k__);
//更新历史值，为下次循环作准备
P_k_1=P_k;
X_bar_k_1=X_bar_k;

return {X_bar_k(0),X_bar_k(1),X_bar_k(2)};
}

