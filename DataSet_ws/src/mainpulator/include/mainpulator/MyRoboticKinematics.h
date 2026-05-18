#ifndef MYROBOTICKINEMATICS_H
#define MYROBOTICKINEMATICS_H

#include <iostream>
#include <vector>
#include <math.h>
#include <cfloat>
#include <Eigen/Eigen>

double myNorm(std::vector<double> data);
double det(std::vector<std::vector<double>> mat, int excludeRow = -1, int excludeCol = -1);
//PoseMatrix
class PoseMatrix
{
private:
public:
    double elements[4][4]; //���λ�˾�������
    PoseMatrix();
    PoseMatrix(double elements[4][4]);
    PoseMatrix(double x, double y, double z, double roll, double pitch, double yaw);
    ~PoseMatrix();
    PoseMatrix IdentityMat();                   //����Ϊ��λ��
    void AssignElements(double elements[4][4]); //��ֵ
    bool IsValid();                             //�жϾ���Ԫ���Ƿ�Ϸ�
    void Display();                             //չʾλ�˾���
    std::vector<std::vector<double>> ToVec();
    std::vector<double> getPosition();
    std::vector<std::vector<double>> getOrientation(); //ŷ����
    // std::vector<double> getRvec();                     //so(3)
    std::vector<double> getPose(); //λ��+ŷ����
    // std::vector<double> getRTvec();                    //se(3)
    void EliminateSmallValue(); //��<1e-5��ֵ��Ϊ0����ֹ�����Ǻ����������̫��
    operator std::vector<std::vector<double>>();
};
PoseMatrix operator+(PoseMatrix A, PoseMatrix B); //����ӷ�������û�õ���
PoseMatrix operator-(PoseMatrix A, PoseMatrix B); //�������������û�õ���
PoseMatrix operator*(PoseMatrix A, PoseMatrix B); //����˷�
PoseMatrix operator*(double a, PoseMatrix B);     //����-����˷�
PoseMatrix operator*(PoseMatrix A, double b);
std::ostream &operator<<(std::ostream &co, PoseMatrix A); //�������
PoseMatrix inv(PoseMatrix A);
double det(PoseMatrix A, int excludeRow = -1, int excludeCol = -1);
PoseMatrix TransMat(double x, double y, double z);
PoseMatrix RotMat(double r, double p, double y);
PoseMatrix RotMat(std::vector<double> rvec);

/*RobotLink��*/
//��װ�˻�е�۹ؽڵ�D-H�����Լ���������
class RobotLink
{
private:
public:
    double q;                                  //D-H����q���ؽ�ת��
    double a;                                  //D-H����a�����˳���
    double d;                                  //D-H����d��������ת�᷽���ƫ��
    double alpha;                              //D-H����alpha������һ���ؽ�֮���Ťת��
    bool canRotate;                            //��ʾ�ùؽ��Ƿ����ת��������ؽڲ���ת����ʵ�ʵ�4���ؽڿ���ת����
    double upperLimit;                         //�ؽ�ת������
    double lowerLimit;                         //�ؽ�ת������
    PoseMatrix CalcPoseMat(double jointAngle); //����ָ��ת��ʱ���ùؽڵ�λ�˱任����
    PoseMatrix CalcPoseMat();                  //����Ĭ��ת���µ�λ�˱任����
    RobotLink();
    ~RobotLink();
    void setLinkParam(double q, double a, double d, double alpha);      //��D-H�������ùؽ�
    void setLinkLimit(double lower = -DBL_MAX, double upper = DBL_MAX); //���øùؽڵ�ת���Ƕ�������
};

/*RoboticKinematics��*/
//�ṩ��ؽڻ�е�۵�ʵ�����Լ��˶�ѧ����㷨�ӿ�
//enum class IK_TYPE { POSITION, TOWARD };
enum class enum_RobotType
{
    ELECTRIC_PREDFINED,
    ELECTRIC_WITH_CAM,
    HYDRUALIC_PREDFINED,
    NON_PREDFINED
};
class RoboticKinematics
{
private:
    std::vector<RobotLink> linkSerial; //��˳��洢��е�۵ĸ����ؽڣ�RobotLink��
    //Ŀǰʹ�õĻ�е��D-H������������Ϊq,a,d,alpha�����ϵ���Ϊ��ͬ�ؽڣ�
    std::vector<std::vector<double>> const defaultDHParams =
        {{0, 0, 129.5, 90},    //1�ؽڣ�����d=122.5��1�ؽڵ�2�ؽڵ����˳���122.5mm�����Ը���ʵ��ģ�͵ĸĶ�
         {0, 414, 119.5, 180}, //2�ؽڣ�����a=414��2�ؽڵ�3�ؽڵ����˳���414mm�����Ը���ʵ��ģ�͵ĸĶ�
         {0, 264, 119.5, 90},  //3�ؽڣ����Ը���ʵ��ģ�͵ĸĶ�
         {-90, 0, 0, 0},       //����ؽ���Ϊq!=0
         {0.001, 0, 0, -90},   //4�ؽڣ�����d=264��4�ؽڵ�ĩ�˳���254mm,10��15�ռӳ�10mm�����Ը���ʵ��ģ�͵ĸĶ�
         {0, 0, 0, 0}};
    std::vector<double> const defaultLinkLengths = {defaultDHParams[0][2], defaultDHParams[1][1], defaultDHParams[2][1]};
    //std::vector<std::vector<double>> const defaultThetaLimit = { {-180,180},{-180,180},{-180,180},{-180,180} };//Ŀǰ��1~4�ؽڽǶ������ޣ��ɸĶ���
    std::vector<std::vector<double>> const defaultThetaLimit = {{-DBL_MAX, DBL_MAX}, {-DBL_MAX, DBL_MAX}, {-DBL_MAX, DBL_MAX}, {-DBL_MAX, DBL_MAX}}; //Ŀǰ��1~4�ؽڽǶ������ޣ��ɸĶ���
    //double handCoordinateResetMatVal[4][4] = { {0,0,1,0},{-1,0,0,0},{0,-1,0,0},{0,0,0,1} };
    //PoseMatrix handCoordinateResetMat = inv(PoseMatrix(handCoordinateResetMatVal));
    double handCoordinateResetMatVal[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    PoseMatrix handCoordinateResetMat = PoseMatrix(handCoordinateResetMatVal);
    double baseCoordinateMatVal[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    PoseMatrix baseCoordinateMat = PoseMatrix(baseCoordinateMatVal);

public:
    RoboticKinematics();
    ~RoboticKinematics();
    void AddLink(RobotLink link);                                                                                                                                                  //Ϊ��е�۶�����������
    void AddLink(double a, double d, double alpha);                                                                                                                                //����D-H������������
    void AddLink(double q, double a, double d, double alpha);                                                                                                                      //����D-H������������
    std::vector<RobotLink> setRobot(std::vector<std::vector<double>> DHParams);                                                                                                    //���û�е�۵�D-H����
    std::vector<RobotLink> setDefaultRobot();                                                                                                                                      //����ΪĬ�ϻ�е��
    std::vector<RobotLink> setPredefinedRobot(enum_RobotType robotType);                                                                                                           //����ΪԤ���е�ۣ�Ŀǰ�е綯1��Һѹ2��
    void setJointAngleLimit(std::vector<std::vector<double>> thetaLimit);                                                                                                          //���û�е�۸��ؽڽǶ�������
    bool IsValidAngles(std::vector<double> jointAngles);                                                                                                                           //�жϹؽڽǶ��Ƿ񳬳���λ
    PoseMatrix ForwardKinematics(std::vector<double> &jointAngles, std::vector<double> &endPose);                                                                                  //�������˶�ѧ�����ݹؽڽǶ�jointAngles�ó�ĩ��λ��endPose��
    PoseMatrix InverseKinematics3DofPosition(std::vector<double> &jointAngles, std::vector<double> &endPosition, std::vector<double> &actual_angles, double relaDepthDesired = 0); //�������˶�ѧ������ĩ��λ��endPosition�ó��ؽڽǶ�jointAngles��
    PoseMatrix InverseKinematics3DofToward(std::vector<double> &jointAngles, std::vector<double> &towardPosition, double q23 = 0);                                                 //�������˶�ѧ�����ݳ����λ��towardPosition�ó��ؽڽǶ�jointAngles��
    //PoseMatrix InverseKinematics_4DOF(std::vector<double>& jointAngles, std::vector<double>& endPosition);//�������˶�ѧ������ĩ��λ��endPosition�ó��ؽڽǶ�jointAngles��
    // double InverseKinematicsGDPosition(std::vector<double> &jointAngles, std::vector<double> &endPosition, std::vector<double> jointAnglesInit); //�����ſ˱Ⱦ����ͨ��3���ɶ����˶�ѧ
    // double InverseKinematicsGNPosition(std::vector<double> &jointAngles, std::vector<double> &endPosition, std::vector<double> jointAnglesInit); //�����ſ˱Ⱦ����ͨ��3���ɶ����˶�ѧ
    // std::vector<std::vector<double>> calcJacobianNumerical(std::vector<double> jointAngles);                                                     //�����ſ˱Ⱦ��󣨻���ĩ�����꣡��
};
RoboticKinematics getDefaultRobot();
// RoboticKinematics getPredefinedRobot(enum_RobotType robotTyp);

/*�˶�ѧDemo*/
int demoKinematics();

#endif