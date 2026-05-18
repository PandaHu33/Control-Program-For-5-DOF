#include "mainpulator/MyRoboticKinematics.h"

using namespace std;

double myNorm(std::vector<double> data)
{
    double ans = 0.0;
    for (int i = 0; i < data.size(); i++)
    {
        ans += data[i] * data[i];
    }
    return sqrt(ans);
}
double det(std::vector<std::vector<double>> mat, int excludeRow, int excludeCol)
{
    std::vector<std::vector<double>> excludedMat;
    int exMatDim = 0;
    for (int i = 0; i < mat.size(); i++)
    {
        if (i == excludeRow)
            continue;
        excludedMat.push_back({});
        for (int j = 0; j < mat.size(); j++)
        {
            if (j == excludeCol)
                continue;
            excludedMat[exMatDim].push_back(mat[i][j]);
        }
        exMatDim++;
    }
    if (exMatDim == 1)
        return excludedMat[0][0];
    double result = 0;
    for (int j = 0; j < exMatDim; j++)
    {
        if (j % 2 == 0)
            result += excludedMat[0][j] * det(excludedMat, 0, j);
        else
            result -= excludedMat[0][j] * det(excludedMat, 0, j);
    }
    return result;
}

/*PoseMatrix��*/
//����double[4][4]�������ͣ�����һϵ�����λ�˾���Ĳ���
PoseMatrix::PoseMatrix()
{
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            this->elements[i][j] = 0;
}
PoseMatrix::PoseMatrix(double elements[4][4])
{
    this->AssignElements(elements);
}
PoseMatrix::PoseMatrix(double x, double y, double z, double roll, double pitch, double yaw)
{
    *this = TransMat(x, y, z) * RotMat(roll, pitch, yaw);
}
PoseMatrix::~PoseMatrix() {}
PoseMatrix PoseMatrix::IdentityMat()
{
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            if (i == j)
                elements[i][j] = 1;
            else
                elements[i][j] = 0;
        }
    return *this;
}
void PoseMatrix::AssignElements(double elements[4][4])
{
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            this->elements[i][j] = elements[i][j];
}
bool PoseMatrix::IsValid()
{
    if (elements[3][3] != 1)
        return false;
    if (elements[3][0] != 0 || elements[3][1] != 0 || elements[3][2] != 0)
        return false;
    double temp = elements[0][0] * elements[0][0] + elements[1][0] * elements[1][0] + elements[2][0] * elements[2][0];
    if (temp < 0.999 || temp > 1.001)
        return false;
    temp = elements[0][1] * elements[0][1] + elements[1][1] * elements[1][1] + elements[2][1] * elements[2][1];
    if (temp < 0.999 || temp > 1.001)
        return false;
    temp = elements[0][2] * elements[0][2] + elements[1][2] * elements[1][2] + elements[2][2] * elements[2][2];
    if (temp < 0.999 || temp > 1.001)
        return false;
    return true;
}
void PoseMatrix::EliminateSmallValue()
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (abs(elements[i][j]) < 0.00001)
                elements[i][j] = 0;
        }
    }
}
PoseMatrix::operator std::vector<std::vector<double>>()
{
    std::vector<std::vector<double>> res = {};
    for (int i = 0; i < 4; i++)
    {
        res.push_back(std::vector<double>());
        for (int j = 0; j < 4; j++)
        {
            res[i].push_back(this->elements[i][j]);
        }
    }
    return res;
}
void PoseMatrix::Display()
{
    std::cout << "PoseMatrix value:" << std::endl;
    std::cout << *this;
}
std::vector<std::vector<double>> PoseMatrix::ToVec()
{
    std::vector<std::vector<double>> mat;
    for (int i = 0; i < 4; i++)
    {
        mat.push_back({});
        for (int j = 0; j < 4; j++)
        {
            mat[i].push_back(elements[i][j]);
        }
    }
    return mat;
}
std::vector<double> PoseMatrix::getPosition()
{
    return {elements[0][3], elements[1][3], elements[2][3]};
}
std::vector<std::vector<double>> PoseMatrix::getOrientation()
{
    double theta_1 = -asin(elements[2][0]);
    double theta_2 = 3.1416 - theta_1;
    double ksi_1 = atan2(elements[2][1] / cos(theta_1), elements[2][2] / cos(theta_1));
    double ksi_2 = atan2(elements[2][1] / cos(theta_2), elements[2][2] / cos(theta_2));
    double phi_1 = atan2(elements[1][0] / cos(theta_1), elements[0][0] / cos(theta_1));
    double phi_2 = atan2(elements[1][0] / cos(theta_2), elements[0][0] / cos(theta_2));
    vector<double> sol_1 = {theta_1, ksi_1, phi_1};
    vector<double> sol_2 = {theta_2, ksi_2, phi_2};
    return {sol_1, sol_2};
}
// std::vector<double> PoseMatrix::getRvec()
// {
//     std::vector<double> res = {};
//     cv::Rodrigues(vectorToMat(truncate(std::vector<std::vector<double>>(*this), {0, 1, 2}, {0, 1, 2})), res);
//     return res;
// }
std::vector<double> PoseMatrix::getPose()
{
    std::vector<double> position, orientation, pose;
    position = this->getPosition();
    orientation = this->getOrientation()[0];
    pose = position;
    pose.insert(pose.end(), orientation.begin(), orientation.end());
    return pose;
}

// std::vector<double> PoseMatrix::getRTvec()
// {
//     std::vector<double> tvec, rvec, rtvec;
//     tvec = this->getPosition();
//     rvec = this->getRvec();
//     rtvec = tvec;
//     rtvec.insert(rtvec.end(), rvec.begin(), rvec.end());
//     return rtvec;
// }

//PoseMatrix����
PoseMatrix operator+(PoseMatrix A, PoseMatrix B)
{
    PoseMatrix C = PoseMatrix();
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            C.elements[i][j] = A.elements[i][j] + B.elements[i][j];
    return C;
}
PoseMatrix operator-(PoseMatrix A, PoseMatrix B)
{
    PoseMatrix C = PoseMatrix();
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            C.elements[i][j] = A.elements[i][j] - B.elements[i][j];
    return C;
}
PoseMatrix operator*(PoseMatrix A, PoseMatrix B)
{
    PoseMatrix C = PoseMatrix();
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                C.elements[i][j] += A.elements[i][k] * B.elements[k][j];
            }
        }
    }
    return C;
}
PoseMatrix operator*(double a, PoseMatrix B)
{
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            B.elements[i][j] *= a;
    return B;
}
PoseMatrix operator*(PoseMatrix A, double b)
{
    return b * A;
}
PoseMatrix inv(PoseMatrix A)
{
    std::vector<std::vector<double>> mat = A.ToVec();
    double matDet = det(mat);
    if (matDet == 0)
        throw;
    PoseMatrix invMat = PoseMatrix();
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if ((i + j) % 2 == 0)
                invMat.elements[j][i] = (det(mat, i, j) / matDet);
            else
                invMat.elements[j][i] = (-det(mat, i, j) / matDet);
        }
    }
    return invMat;
}
double det(PoseMatrix A, int excludeRow, int excludeCol)
{
    std::vector<std::vector<double>> mat = A.ToVec();
    return det(mat, excludeRow, excludeCol);
}
std::ostream &operator<<(std::ostream &co, PoseMatrix A)
{
    co.precision(2);
    co.flags(std::ios::fixed);
    for (int i = 0; i < 4; i++)
    {
        co << "|\t";
        for (int j = 0; j < 4; j++)
        {
            co << A.elements[i][j] << "\t\t";
        }
        co << "|";
        co << std::endl;
    }
    return co;
}

//ƽ�ơ���ת����
PoseMatrix TransMat(double x, double y, double z)
{
    double matVal[4][4] = {{1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}};
    return PoseMatrix(matVal);
}
PoseMatrix RotMat(double r, double p, double y)
{
    double matRr[4][4] = {{1, 0, 0, 0}, {0, cos(r), -sin(r), 0}, {0, sin(r), cos(r), 0}, {0, 0, 0, 1}};
    double matRp[4][4] = {{cos(p), 0, sin(p), 0}, {0, 1, 0, 0}, {-sin(p), 0, cos(p), 0}, {0, 0, 0, 1}};
    double matRy[4][4] = {{cos(y), -sin(y), 0, 0}, {sin(y), cos(y), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    return PoseMatrix(matRy) * PoseMatrix(matRp) * PoseMatrix(matRr);
}
PoseMatrix RotMat(std::vector<double> rvec)
{
    //Vec3d r = Vec3d({ rvec[0],rvec[1],rvec[2] });
    //Mat R;
    //Rodrigues(rvec, R);
    double theta = myNorm(rvec);
    double tmp[4][4] = {{0, -rvec[2], rvec[1], 0}, {rvec[2], 0, -rvec[0], 0}, {-rvec[1], rvec[0], 0, 0}, {0, 0, 0, 0}};
    PoseMatrix A = 1.0 / theta * PoseMatrix(tmp);
    //cout << "theta = " << theta << ", rvec = " << rvec[0] << " " << rvec[1] << " " << rvec[2] << endl;
    //cout << A << endl;
    //cout << PoseMatrix().IdentityMat() + A * sin(theta) + A * A * (1 - cos(theta)) << endl;
    return PoseMatrix().IdentityMat() + A * sin(theta) + A * A * (1 - cos(theta));
}

/*RobotLink��*/
//��װ�˻�е�۹ؽڵ�D-H�����Լ���������
PoseMatrix RobotLink::CalcPoseMat(double jointAngle)
{
    double mat[4][4] = {
        cos(jointAngle), -sin(jointAngle) * cos(alpha), sin(jointAngle) * sin(alpha), a * cos(jointAngle),
        sin(jointAngle), cos(jointAngle) * cos(alpha), -cos(jointAngle) * sin(alpha), a * sin(jointAngle),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1};
    return PoseMatrix(mat);
}
PoseMatrix RobotLink::CalcPoseMat()
{
    return CalcPoseMat(this->q);
}
RobotLink::RobotLink()
{
    q = 0;
    a = 0;
    d = 0;
    alpha = 0;
    canRotate = true;
    upperLimit = 3.1415926535;
    lowerLimit = -3.1415926535;
}
RobotLink::~RobotLink() {}
void RobotLink::setLinkParam(double q, double a, double d, double alpha)
{
    if (q != 0)
    {
        this->canRotate = false;
        setLinkLimit(q, q);
    }
    else
    {
        this->canRotate = true;
        setLinkLimit();
    }
    this->q = q / 180 * 3.1415926535;
    this->a = a;
    this->d = d;
    this->alpha = alpha / 180 * 3.1415926535;
}
void RobotLink::setLinkLimit(double lower, double upper)
{
    this->lowerLimit = lower / 180.0 * 3.1415926535;
    this->upperLimit = upper / 180.0 * 3.1415926535;
}

/*RoboticKinematics��*/
//�ṩ��ؽڻ�е�۵�ʵ�����Լ��˶�ѧ����㷨�ӿ�
RoboticKinematics::RoboticKinematics() {}
RoboticKinematics::~RoboticKinematics() {}
void RoboticKinematics::AddLink(RobotLink link)
{
    linkSerial.push_back(link);
}
void RoboticKinematics::AddLink(double a, double d, double alpha)
{
    RobotLink link = RobotLink();
    link.setLinkParam(0, a, d, alpha);
    linkSerial.push_back(link);
}
void RoboticKinematics::AddLink(double q, double a, double d, double alpha)
{
    RobotLink link = RobotLink();
    link.setLinkParam(q, a, d, alpha);
    linkSerial.push_back(link);
}
std::vector<RobotLink> RoboticKinematics::setRobot(std::vector<std::vector<double>> DHParams)
{
    RobotLink link = RobotLink();
    std::vector<RobotLink> robot;
    for (int iLink = 0; iLink < DHParams.size(); iLink++)
    {
        if (DHParams[iLink].size() == 3)
        {
            link.setLinkParam(0, DHParams[iLink][0], DHParams[iLink][1], DHParams[iLink][2]);
            link.setLinkLimit();
            robot.push_back(link);
        }
        else if (DHParams[iLink].size() == 4)
        {
            link.setLinkParam(DHParams[iLink][0], DHParams[iLink][1], DHParams[iLink][2], DHParams[iLink][3]);
            link.setLinkLimit();
            robot.push_back(link);
        }
        else
        {
            throw;
        }
    }
    linkSerial = robot;
    return robot;
}
std::vector<RobotLink> RoboticKinematics::setDefaultRobot()
{
    std::vector<RobotLink> defaultRobot = setRobot(defaultDHParams);
    setJointAngleLimit(defaultThetaLimit);
    return defaultRobot;
}
std::vector<RobotLink> RoboticKinematics::setPredefinedRobot(enum_RobotType robotType)
{
    switch (robotType)
    {
    case enum_RobotType::ELECTRIC_PREDFINED:
    {
        return setDefaultRobot();
        break;
    }
    case enum_RobotType::ELECTRIC_WITH_CAM:
    {
        //Ŀǰʹ�õĻ�е��D-H������������Ϊq,a,d,alpha�����ϵ���Ϊ��ͬ�ؽڣ�
        std::vector<std::vector<double>> const DHParams =
            {{0, 0, 129.5, 90},    //1�ؽڣ�����d=122.5��1�ؽڵ�2�ؽڵ����˳���122.5mm�����Ը���ʵ��ģ�͵ĸĶ�
             {0, 414, -119.5, 0},  //2�ؽڣ�����a=414��2�ؽڵ�3�ؽڵ����˳���414mm�����Ը���ʵ��ģ�͵ĸĶ�
             {0, 0, 119.5, -90},   //3�ؽڣ����Ը���ʵ��ģ�͵ĸĶ�
             {90, 0, -50, 90},     //����ؽ���Ϊq!=0
             {0, 0, 264 - 60, 0}}; //4�ؽڣ�����d=264��4�ؽڵ�ĩ�˳���254mm,10��15�ռӳ�10mm�����Ը���ʵ��ģ�͵ĸĶ��������װλ�ö�Ӧ����60mm����װ��Ӧa=50;��
        std::vector<double> const defaultLinkLengths = {defaultDHParams[0][2], defaultDHParams[1][1], defaultDHParams[4][2]};
        //std::vector<std::vector<double>> const defaultThetaLimit = { {-180,180},{-180,180},{-180,180},{-180,180} };//Ŀǰ��1~4�ؽڽǶ������ޣ��ɸĶ���
        std::vector<std::vector<double>> const defaultThetaLimit = {{-DBL_MAX, DBL_MAX}, {-DBL_MAX, DBL_MAX}, {-DBL_MAX, DBL_MAX}, {-DBL_MAX, DBL_MAX}}; //Ŀǰ��1~4�ؽڽǶ������ޣ��ɸĶ���
        std::vector<RobotLink> electric_with_cam = setRobot(DHParams);
        setJointAngleLimit(defaultThetaLimit);
        return electric_with_cam;
        break;
    }
    case enum_RobotType::HYDRUALIC_PREDFINED:
    {
        //Ŀǰʹ�õĻ�е��D-H������������Ϊq,a,d,alpha�����ϵ���Ϊ��ͬ�ؽڣ�
        std::vector<std::vector<double>> const hydralicDHParams =
            {{0, 101, 0, 90},
             {0, 605, 0, 0},
             {0, 284.4, 0, 0},
             {0, 484.4, 0, 90}};
        //std::vector<double> const hydralicLinkLengths = { hydralicDHParams[0][1], hydralicDHParams[1][1], hydralicDHParams[2][1], hydralicDHParams[3][1] };
        std::vector<std::vector<double>> const hydralicThetaLimit = {{-180, 180}, {-180, 180}, {-180, 180}, {-180, 180}}; //Ŀǰ��1~4�ؽڽǶ������ޣ��ɸĶ���
        std::vector<RobotLink> hydralicRobot = setRobot(hydralicDHParams);
        //setJointAngleLimit(hydralicThetaLimit);
        return hydralicRobot;
        break;
    }
    default:
    {
        return setDefaultRobot();
        break;
    }
    }
};
void RoboticKinematics::setJointAngleLimit(std::vector<std::vector<double>> thetaLimit)
{
    int iLimit = 0;
    for (int iLink = 0; iLink < linkSerial.size(); iLink++)
    {
        if (!linkSerial[iLink].canRotate)
        {
            continue;
        }
        else
        {
            linkSerial[iLink].setLinkLimit(thetaLimit[iLimit][0], thetaLimit[iLimit][1]);
            iLimit++;
        }
    }
}
bool RoboticKinematics::IsValidAngles(std::vector<double> jointAngles)
{
    int iAngle = 0;
    for (int iLink = 0; iLink < linkSerial.size(); iLink++)
    {
        if (linkSerial[iLink].canRotate)
        {
            if (jointAngles[iAngle] / 180 * 3.1415926535 < linkSerial[iLink].lowerLimit || jointAngles[iAngle] / 180 * 3.1415926535 > linkSerial[iLink].upperLimit)
                return false;
            if (isnan(jointAngles[iAngle]))
                return false;
            iAngle++;
        }
        else
            continue;
    }
    if (iAngle != jointAngles.size())
        return false;
    return true;
}
PoseMatrix RoboticKinematics::ForwardKinematics(std::vector<double> &jointAngles, std::vector<double> &endPose)
{
    if (!IsValidAngles(jointAngles))
    {
        std::cout << "[ERROR] Invalid joint angles are input!!!" << std::endl;
        throw -1;
    }
    PoseMatrix endTransMat = PoseMatrix();
    //endTransMat.IdentityMat();
    endTransMat = baseCoordinateMat;
    int iAngle = 0;
    for (int iLink = 0; iLink < linkSerial.size(); iLink++)
    {
        if (linkSerial[iLink].canRotate)
            endTransMat = endTransMat * linkSerial[iLink].CalcPoseMat(jointAngles[iAngle++] / 180.0 * 3.1415926535);
        else
            endTransMat = endTransMat * linkSerial[iLink].CalcPoseMat();
    }
    //endTransMat.EliminateSmallValue();
    endTransMat = endTransMat * handCoordinateResetMat;
    if (!endTransMat.IsValid())
    {
        std::cout << "[ERROR] Solved pose matrix is invalid!!!" << std::endl;
        throw -1;
    }
    endPose.clear();
    endPose.resize(6);
    endPose[0] = endTransMat.elements[0][3];
    endPose[1] = endTransMat.elements[1][3];
    endPose[2] = endTransMat.elements[2][3];
    endPose[3] = 0 + atan2(endTransMat.elements[2][1], endTransMat.elements[2][2]) * 180.0 / 3.1415926535;
    endPose[4] = 0 + atan2(-endTransMat.elements[2][0], sqrt(endTransMat.elements[2][1] * endTransMat.elements[2][1] + endTransMat.elements[3][3] * endTransMat.elements[3][3])) * 180.0 / 3.1415926535;
    endPose[5] = 0 + atan2(endTransMat.elements[1][0], endTransMat.elements[0][0]) * 180.0 / 3.1415926535;
    if (endPose[3] > 180)
        endPose[3] -= 360;
    if (endPose[4] > 180)
        endPose[4] -= 360;
    if (endPose[5] > 180)
        endPose[5] -= 360;
    return endTransMat;
}
PoseMatrix RoboticKinematics::InverseKinematics3DofPosition(std::vector<double> &jointAngles, std::vector<double> &endPosition, std::vector<double> &actual_angles, double relaDepthDesired)
{
    PoseMatrix poseMatOnBase = inv(baseCoordinateMat) * TransMat(endPosition[0], endPosition[1], endPosition[2]);
    double L1, L2, L3;
    L1 = defaultLinkLengths[0];
    L2 = defaultLinkLengths[1];
    L3 = defaultLinkLengths[2] + relaDepthDesired; //������������������������
    double x, y, z, d;
    x = poseMatOnBase.elements[0][3];
    y = poseMatOnBase.elements[1][3];
    z = poseMatOnBase.elements[2][3] - L1;
    d = sqrt(x * x + y * y);
    jointAngles.clear();
    jointAngles.push_back(0);
    jointAngles.push_back(0);
    jointAngles.push_back(0);
    jointAngles.push_back(0);
    double q1, q2, q3, q4;
    // try
    // {
    //     q1 = atan2(y, x);
    //     q3 = 3.1415926535 - acos((L2 * L2 + L3 * L3 - d * d - z * z) / (2 * L2 * L3));
    //     q2 = asin(L3 / sqrt(z * z + d * d) * sin(q3)) + atan2(z, d);
    //     q4 = -q1 + 3.1415926535 / 2;

    //     //另一个解
    //     // q1 = atan2(y, x);
    //     // q3 = -3.1415926535 + acos((L2 * L2 + L3 * L3 - z * z - d * d) / (2 * L2 * L3));
    //     // q2 = atan2(z, d) - asin((-sin(q3) * L3) / sqrt(z * z + d * d));
    //     // q4 = -q1 + 3.1415926535 / 2;

    //     jointAngles[0] = q1 * 180.0 / 3.1415926535;
    //     jointAngles[1] = q2 * 180.0 / 3.1415926535;
    //     jointAngles[2] = q3 * 180.0 / 3.1415926535;
    //     jointAngles[3] = q4 * 180.0 / 3.1415926535;

    //     if (!IsValidAngles(jointAngles))
    //         throw -1;

    //     //报错改为赋当前实际角度
    //     if (!IsValidAngles(jointAngles))
    //     {
    //         jointAngles[0] = actual_angles[0];
    //         jointAngles[1] = actual_angles[1];
    //         jointAngles[2] = actual_angles[2];
    //         jointAngles[3] = actual_angles[3];
    //     }
    // }
    // catch (...)
    // {
    //     std::cout << "[ERROR] Invalid inverse kinematics input!!!" << std::endl;
    //     throw -1;
    // }
    if (actual_angles[2] > 0.0)
    {
        q1 = atan2(y, x);
        q3 = 3.1415926535 - acos((L2 * L2 + L3 * L3 - d * d - z * z) / (2 * L2 * L3));
        q2 = asin(L3 / sqrt(z * z + d * d) * sin(q3)) + atan2(z, d);
        q4 = -q1 + 3.1415926535 / 2;
    }
    else
    {
        //另一个解
        q1 = atan2(y, x);
        q3 = -3.1415926535 + acos((L2 * L2 + L3 * L3 - z * z - d * d) / (2 * L2 * L3));
        q2 = atan2(z, d) - asin((-sin(q3) * L3) / sqrt(z * z + d * d));
        q4 = -q1 + 3.1415926535 / 2;
    }

    jointAngles[0] = q1 * 180.0 / 3.1415926535;
    jointAngles[1] = q2 * 180.0 / 3.1415926535;
    jointAngles[2] = q3 * 180.0 / 3.1415926535;
    jointAngles[3] = q4 * 180.0 / 3.1415926535;

    //报错改为赋当前实际角度
    if (!IsValidAngles(jointAngles))
    {
        cout<<"IK unsolved!"<<endl;
        cout<<q1<<" "<<q2<<" "<<q3<<" "<<q4<<endl;
        jointAngles[0] = actual_angles[0];
        jointAngles[1] = actual_angles[1];
        jointAngles[2] = actual_angles[2];
        jointAngles[3] = actual_angles[3];
    }

    std::vector<double> endPose;
    return ForwardKinematics(jointAngles, endPose);
}
PoseMatrix RoboticKinematics::InverseKinematics3DofToward(std::vector<double> &jointAngles, std::vector<double> &towardPosition, double q23)
{
    PoseMatrix poseMatOnBase = inv(baseCoordinateMat) * TransMat(towardPosition[0], towardPosition[1], towardPosition[2]);
    double L1, L2, L3;
    L1 = defaultLinkLengths[0];
    L2 = defaultLinkLengths[1];
    L3 = defaultLinkLengths[2];
    double x, y, z, d;
    x = poseMatOnBase.elements[0][3];
    y = poseMatOnBase.elements[1][3];
    z = poseMatOnBase.elements[2][3] - L1;
    d = sqrt(x * x + y * y);
    jointAngles.clear();
    jointAngles.push_back(0);
    jointAngles.push_back(0);
    jointAngles.push_back(0);
    jointAngles.push_back(0);
    double q1, q2, q3, q4;
    try
    {
        q1 = atan2(y, x);
        double temp = (d * tan(q23) - z) / L2 / sqrt(1 + tan(q23) * tan(q23));
        if (temp > 1)
            q3 = 3.1415926535 / 2;
        else if (temp < -1)
            q3 = -3.1415926535 / 2;
        else
            q3 = asin((d * tan(q23) - z) / L2 / sqrt(1 + tan(q23) * tan(q23)));
        q2 = q23 - q3;
        q4 = 0;
        jointAngles[0] = q1 * 180.0 / 3.1415926535;
        jointAngles[1] = q2 * 180.0 / 3.1415926535;
        jointAngles[2] = q3 * 180.0 / 3.1415926535;
        jointAngles[3] = q4 * 180.0 / 3.1415926535;
        if (!IsValidAngles(jointAngles))
            throw -1;
    }
    catch (...)
    {
        std::cout << "[ERROR] Invalid inverse kinematics input!!!" << std::endl;
        throw -2;
    }
    std::vector<double> endPose;
    return ForwardKinematics(jointAngles, endPose);
}

RoboticKinematics getDefaultRobot()
{
    RoboticKinematics rob;
    rob.setDefaultRobot();
    return rob;
}

// RoboticKinematics getPredefinedRobot(enum_RobotType robotTyp)
// {
//     RoboticKinematics rob;
//     rob.setPredefinedRobot(robotTyp);
//     return rob;
// }
