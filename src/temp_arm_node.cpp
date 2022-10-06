#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "temp_arm/Gripper.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <Eigen/Geometry>
#include <pthread.h>

#define D2R M_PI/180.
#define R2D 180./M_PI

#define X_ 0
#define Y_ 1
#define Z_ 2

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef struct _Joystick {
    double x;
    double y;
    double z;
} Joystick_;
Joystick_ joy;

static struct timespec ik_start_time;
static struct timespec ik_end_time;
static struct timespec start_time;
static struct timespec end_time;
static struct timespec next_time;

// Joy control varibles declaration //
int prev_press[5] = {0};
int press[5] = {0};
bool pressed[5] = { false };

int ctrl_mode = 0;
bool goal_cal_once = true;
int phase = 0;
int joint_id = 0;
//double joint_command = 0.0;

enum {
    J1 = 0, J2, J3, J4, J5, J6
};

namespace robot_arm
{
    class drok4_plugin
    {
        // Joint varibles //
        int nDoF = 6;

        // FK & IK varibles //
        Vector3d start_posi = VectorXd::Zero(3);
        Vector3d goal_posi = VectorXd::Zero(3);
        Vector3d present_posi = VectorXd::Zero(3);
        Vector3d command_posi = VectorXd::Zero(3);

        MatrixXd start_rot = MatrixXd::Identity(3, 3);
        MatrixXd goal_rot = MatrixXd::Identity(3, 3);
        MatrixXd present_rot = MatrixXd::Identity(3, 3);
        MatrixXd command_rot = MatrixXd::Identity(3, 3);

        VectorXd q_init = VectorXd::Zero(6);
        VectorXd q_goal = VectorXd::Zero(6);
        VectorXd q_present = VectorXd::Zero(6);
        VectorXd q_command = VectorXd::Zero(6);

        VectorXd q0 = VectorXd::Zero(6);
        MatrixXd C_err = MatrixXd::Identity(3, 3);

        //double targetRadian[6] = { 0.0 };

        typedef struct _AngleAxis {
            Vector3d n;
            double th;
        } AngleAxis_;
        AngleAxis_ a_axis;

        typedef struct _Orientation {
            double roll;
            double pitch;
            double yaw;
        } Orientation_;
        Orientation_ ori;

        // Pthread varibles //

    public :
        // Time varibles //
        double total_t = 10.0;         //[s]
        double dt = 0.030;      //[s]
        double accum_t = 0.0;      //[s]
        int control_cycle = 30;  //[ms]
        //double temp_count = 0.0;

        double targetRadian[6] = { 0.0 };

        //void *P_Function(void *data);
        //void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg);
        //bool CprBtweenTime(double time_1, double time_2);
        //void ResetTimeVar(double accum_t, double total_t);
        void JoystickControl(void);

        AngleAxis_ RotMatToAngleAxis(MatrixXd C);
        MatrixXd AngleAxisToRotMat(AngleAxis_ a_axis);
        VectorXd RotMatToEulerZyx(MatrixXd C);
        MatrixXd EulerZyxToRotMat(double z_rot, double y_rot, double x_rot);
        MatrixXd RpyToRotMat(double roll, double pitch, double yaw);
        VectorXd RotMatToRotVec(MatrixXd C);

        MatrixXd GetTransformI0(void);
        MatrixXd JointToTransform01(VectorXd q);
        MatrixXd JointToTransform12(VectorXd q);
        MatrixXd JointToTransform23(VectorXd q);
        MatrixXd JointToTransform34(VectorXd q);
        MatrixXd JointToTransform45(VectorXd q);
        MatrixXd JointToTransform56(VectorXd q);
        MatrixXd GetTransform6E(void);

        VectorXd JointToPosition(VectorXd q);
        MatrixXd JointToRotMat(VectorXd q);
        MatrixXd JointToPosJac(VectorXd q);
        MatrixXd JointToRotJac(VectorXd q);
        MatrixXd PseudoInverseMat(MatrixXd A, double lambda);
        VectorXd InverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol);

        double Func_1_cos(double init, double final, double t, double T);
        Vector3d Func_1_cos(Vector3d init, Vector3d final, double t, double T);
        double Func_1_cos_yaw(double start, double end, double t, double T);
    };
}

using namespace robot_arm;

//------------------------------------------------------------//
// Transformation functions between orientation expressions
//------------------------------------------------------------//
drok4_plugin::AngleAxis_ drok4_plugin::RotMatToAngleAxis(MatrixXd C)
{
    AngleAxis_ a_axis;
    Vector3d n;
    double th;

    th = acos((C(0, 0) + C(1, 1) + C(2, 2) - 1) / 2);

    if (fabs(th) < 0.001)
    {
        n << 0, 0, 0;
    }
    else
    {
        n << (C(2, 1) - C(1, 2)),\
                (C(0, 2) - C(2, 0)),\
                (C(1, 0) - C(0, 1));
        n = (1 / (2 * sin(th))) * n;
    }

    a_axis.n = n;
    a_axis.th = th;

    return a_axis;
}

MatrixXd drok4_plugin::AngleAxisToRotMat(AngleAxis_ a_axis)
{
    MatrixXd C(3, 3);
    Vector3d n = a_axis.n;
    double th = a_axis.th;

    if (fabs(th) < 0.001) {
        C << MatrixXd::Identity(3, 3);
    }

    double nx = n(0), ny = n(1), nz = n(2);
    double s = sin(th), c = cos(th);

    C << nx*nx*(1-c)+c, nx*ny*(1-c)-nx*s, nx*nz*(1-c)+ny*s,\
            nx*ny*(1-c)+nz*s, ny*ny*(1-c)+c, ny*nz*(1-c)-nz*s,\
            nz*nx*(1-c)-ny*s, ny*nz*(1-c)+nz*s, nz*nz*(1-c)+c;

    return C;
}

VectorXd drok4_plugin::RotMatToEulerZyx(MatrixXd C)
{
    //회전 행렬을 Euler ZYX로 변환
    VectorXd euler_zyx = VectorXd::Zero(3);

    euler_zyx(0) = atan2(C(1, 0), C(0, 0));
    euler_zyx(1) = atan2(-C(2, 0), sqrt(pow(C(2, 1), 2) + pow(C(2, 2), 2)));
    euler_zyx(2) = atan2(C(2, 1), C(2, 2));

    return euler_zyx;
}

MatrixXd drok4_plugin::EulerZyxToRotMat(double z_rot, double y_rot, double x_rot)
{
    MatrixXd Z_rot(3, 3), Y_rot(3, 3), X_rot(3, 3);

    double sz = sin(z_rot), cz = cos(z_rot);
    double sy = sin(y_rot), cy = cos(y_rot);
    double sx = sin(x_rot), cx = cos(x_rot);

    Z_rot << cz, -sz, 0,\
            sz, cz, 0,\
            0, 0, 1;
    Y_rot << cy, 0, sy,\
            0, 1, 0,\
            -sy, 0, cy;
    X_rot << 1, 0, 0,\
            0, cx, -sx,\
            0, sx, cx;

    return Z_rot * Y_rot * X_rot;
}

MatrixXd drok4_plugin::RpyToRotMat(double roll, double pitch, double yaw)
{
    using namespace Eigen;
    cout << "\nroll : " << roll*R2D << "\npitch : " << pitch*R2D << "\nyaw : " << yaw*R2D << endl;
    MatrixXd C(3, 3);

    AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Quaterniond q = rollAngle * pitchAngle * yawAngle;
    //Quaterniond q = yawAngle * pitchAngle * rollAngle;
    C = q.toRotationMatrix();

    return C;
}

VectorXd drok4_plugin::RotMatToRotVec(MatrixXd C)
{
    //Input: a rotation matrix C
    //Output: the rotational vector which describes the rotation C
    VectorXd phi(3), n(3);
    double th;

    th = acos((C(0, 0) + C(1, 1) + C(2, 2) - 1) / 2);
    if(fabs(th) < 0.001)
    {
         n << 0, 0, 0;
    }
    else
    {
        n << (C(2, 1) - C(1, 2)),\
                (C(0, 2) - C(2, 0)),\
                (C(1, 0) - C(0, 1));
        n = (1 / (2 * sin(th))) * n;
    }

    phi = th * n;

    return phi;
}

//------------------------------------------------------------//
// Practice 2. Forward Kinematics (fuction declaration)
// 1, 2, 5, 6번 관절은 음의 축방향으로 회전한다.
//------------------------------------------------------------//
MatrixXd drok4_plugin::GetTransformI0(void)
{
    //Global frame to base link frame
    //htm = homogeneous transformation matrix

    MatrixXd htmI0 = MatrixXd::Identity(4, 4);

    return htmI0;
}

MatrixXd drok4_plugin::JointToTransform01(VectorXd q)
{
    //Base link frame to joint 1 frame
    //q : Generalized coordinates, q = [q1; q2; q3; q4; q5; q6]
    //q(0) : Angle of joint 1

    double q1 = q(0);

    double sq = sin(q1);
    double cq = cos(q1);

    MatrixXd htm01(4, 4);
    htm01 << cq, -sq, 0, 0,\
            sq, cq, 0, 0,\
            0, 0, 1, 0,\
            0, 0, 0, 1;

    return htm01;
}

MatrixXd drok4_plugin::JointToTransform12(VectorXd q)
{
    //q(1) : Angle of joint 2

    double q2 = q(1);

    double sq = sin(q2);
    double cq = cos(q2);

    MatrixXd htm12(4, 4);
    htm12 << cq, 0, sq, 0,\
            0, 1, 0, 0,\
            -sq, 0, cq, 0.161,\
            0, 0, 0, 1;

    return htm12;
}

MatrixXd drok4_plugin::JointToTransform23(VectorXd q)
{
    //q(2) : Angle of joint 3

    double q3 = q(2);

    double sq = sin(q3);
    double cq = cos(q3);

    MatrixXd htm23(4, 4);
    htm23 << cq, 0, sq, -0.605,\
            0, 1, 0, 0,\
            -sq, 0, cq, 0.002,\
            0, 0, 0, 1;

    return htm23;
}

MatrixXd drok4_plugin::JointToTransform34(VectorXd q)
{
    //q(3) : Angle of joint 4

    double q4 = q(3);

    double sq = sin(q4);
    double cq = cos(q4);

    MatrixXd htm34(4, 4);
    htm34 << 1, 0, 0, 0.507,\
            0, cq, -sq, 0,\
            0, sq, cq, 0.095,\
            0, 0, 0, 1;

    return htm34;
}

MatrixXd drok4_plugin::JointToTransform45(VectorXd q)
{
    //q(4) : angle of joint 5

    double q5 = q(4);

    double sq = sin(q5);
    double cq = cos(q5);

    MatrixXd htm45(4, 4);
    htm45 << cq, 0, sq, 0.096,\
            0, 1, 0, 0,\
            -sq, 0, cq, 0,\
            0, 0, 0, 1;

    return htm45;
}

MatrixXd drok4_plugin::JointToTransform56(VectorXd q)
{
    //q(5) : Angle of joint 6

    double q6 = q(5);

    double sq = sin(q6);
    double cq = cos(q6);

    MatrixXd htm56(4, 4);
    htm56 << 1, 0, 0, 0.109,\
            0, cq, -sq, 0,\
            0, sq, cq, -0.002,\
            0, 0, 0, 1;

    return htm56;
}

MatrixXd drok4_plugin::GetTransform6E(void)
{
    //두 좌표계 사이 회전 관계는 없고 x 방향 거리 관계만 존재한다.

    MatrixXd htm6E(4, 4);
    htm6E << 1, 0, 0, 0.15,\
            0, 1, 0, 0,\
            0, 0, 1, 0,\
            0, 0, 0, 1;

    return htm6E;
}

VectorXd drok4_plugin::JointToPosition(VectorXd q)
{
    MatrixXd TI0(4, 4), T01(4, 4), T12(4, 4), T23(4, 4), T34(4, 4), T45(4, 4),\
            T56(4, 4), T6E(4, 4), TIE(4, 4);
    TI0 = GetTransformI0();
    T01 = JointToTransform01(q);
    T12 = JointToTransform12(q);
    T23 = JointToTransform23(q);
    T34 = JointToTransform34(q);
    T45 = JointToTransform45(q);
    T56 = JointToTransform56(q);
    T6E = GetTransform6E();

    TIE = TI0 * T01 * T12 * T23 * T34 * T45 * T56 * T6E;

    Vector3d position;
    position = TIE.block(0, 3, 3, 1);

    //Block ocommentperation extracts a rectangular part of a matrix or array.
    //Block of size (p, q), starting at (i, j)
    //Version constructing a dynamic-size block expression = matrix.block(i, j, p, q)
    //Version constructing a fixed-size block expression = matrix.block<p, q>(i, j)

    return position;
}

MatrixXd drok4_plugin::JointToRotMat(VectorXd q)
{
    MatrixXd TI0(4, 4), T01(4, 4), T12(4, 4), T23(4, 4), T34(4, 4), T45(4, 4),\
            T56(4, 4), T6E(4, 4), TIE(4, 4);
    TI0 = GetTransformI0();
    T01 = JointToTransform01(q);
    T12 = JointToTransform12(q);
    T23 = JointToTransform23(q);
    T34 = JointToTransform34(q);
    T45 = JointToTransform45(q);
    T56 = JointToTransform56(q);
    T6E = GetTransform6E();

    TIE = TI0 * T01 * T12 * T23 * T34 * T45 * T56 * T6E;

    MatrixXd rot_m(3, 3);
    rot_m = TIE.block(0, 0, 3, 3);

    return rot_m;
}

//------------------------------------------------------------//
// Practice 3. Geometric Jacobian (fuction declaration)
//------------------------------------------------------------//
MatrixXd drok4_plugin::JointToPosJac(VectorXd q)
{
    //Input : Vector of generalized coordinates (joint angles)
    //Output : J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3, 6);
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4);
    MatrixXd T_I1(4, 4), T_I2(4, 4), T_I3(4, 4), T_I4(4, 4), T_I5(4, 4), T_I6(4, 4);
    MatrixXd R_I1(3, 3), R_I2(3, 3), R_I3(3, 3), R_I4(3, 3), R_I5(3, 3), R_I6(3, 3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1, n_I_2, n_I_3, n_I_4, n_I_5, n_I_6;
    Vector3d r_I_IE;

    //Compute the relative homogeneous transformation matrices.
    T_I0 = GetTransformI0();
    T_01 = JointToTransform01(q);
    T_12 = JointToTransform12(q);
    T_23 = JointToTransform23(q);
    T_34 = JointToTransform34(q);
    T_45 = JointToTransform45(q);
    T_56 = JointToTransform56(q);
    T_6E = GetTransform6E();

    //Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0 * T_01;
    T_I2 = T_I0 * T_01 * T_12;
    T_I3 = T_I0 * T_01 * T_12 * T_23;
    T_I4 = T_I0 * T_01 * T_12 * T_23 * T_34;
    T_I5 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45;
    T_I6 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

    //Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0, 0, 3, 3);
    R_I2 = T_I2.block(0, 0, 3, 3);
    R_I3 = T_I3.block(0, 0, 3, 3);
    R_I4 = T_I4.block(0, 0, 3, 3);
    R_I5 = T_I5.block(0, 0, 3, 3);
    R_I6 = T_I6.block(0, 0, 3, 3);

    //Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0, 3, 3, 1);
    r_I_I2 = T_I2.block(0, 3, 3, 1);
    r_I_I3 = T_I3.block(0, 3, 3, 1);
    r_I_I4 = T_I4.block(0, 3, 3, 1);
    r_I_I5 = T_I5.block(0, 3, 3, 1);
    r_I_I6 = T_I6.block(0, 3, 3, 1);

    //Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0, 0, 1;
    n_2 << 0, 1, 0;
    n_3 << 0, 1, 0;
    n_4 << 1, 0, 0;
    n_5 << 0, 1, 0;
    n_6 << 1, 0, 0;

    //Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1 * n_1;
    n_I_2 = R_I2 * n_2;
    n_I_3 = R_I3 * n_3;
    n_I_4 = R_I4 * n_4;
    n_I_5 = R_I5 * n_5;
    n_I_6 = R_I6 * n_6;

    //Compute the end-effector position vector.
    r_I_IE = (T_I6 * T_6E).block(0, 3, 3, 1);       //T_I6 * T_6E = TIEs

    //Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE - r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE - r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE - r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE - r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE - r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE - r_I_I6);

    //std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}

MatrixXd drok4_plugin::JointToRotJac(VectorXd q)
{
    //Input : Vector of generalized coordinates (joint angles)
    //Output : J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3, 6);
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4);
    MatrixXd T_I1(4, 4), T_I2(4, 4), T_I3(4, 4), T_I4(4, 4), T_I5(4, 4), T_I6(4, 4);
    MatrixXd R_I1(3, 3), R_I2(3, 3), R_I3(3, 3), R_I4(3, 3), R_I5(3, 3), R_I6(3, 3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;

    //Compute the relative homogeneous transformation matrices.
    T_I0 = GetTransformI0();
    T_01 = JointToTransform01(q);
    T_12 = JointToTransform12(q);
    T_23 = JointToTransform23(q);
    T_34 = JointToTransform34(q);
    T_45 = JointToTransform45(q);
    T_56 = JointToTransform56(q);
    T_6E = GetTransform6E();

    //Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0 * T_01;
    T_I2 = T_I0 * T_01 * T_12;
    T_I3 = T_I0 * T_01 * T_12 * T_23;
    T_I4 = T_I0 * T_01 * T_12 * T_23 * T_34;
    T_I5 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45;
    T_I6 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

    //Extract the rotation matrices from each homogeneous transformation matrix.
    R_I1 = T_I1.block(0, 0, 3, 3);
    R_I2 = T_I2.block(0, 0, 3, 3);
    R_I3 = T_I3.block(0, 0, 3, 3);
    R_I4 = T_I4.block(0, 0, 3, 3);
    R_I5 = T_I5.block(0, 0, 3, 3);
    R_I6 = T_I6.block(0, 0, 3, 3);

    //Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0, 0, 1;
    n_2 << 0, 1, 0;
    n_3 << 0, 1, 0;
    n_4 << 1, 0, 0;
    n_5 << 0, 1, 0;
    n_6 << 1, 0, 0;

    //Compute the translational Jacobian.
    J_R.col(0) << R_I1 * n_1;
    J_R.col(1) << R_I2 * n_2;
    J_R.col(2) << R_I3 * n_3;
    J_R.col(3) << R_I4 * n_4;
    J_R.col(4) << R_I5 * n_5;
    J_R.col(5) << R_I6 * n_6;

    //std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}

//------------------------------------------------------------//
// Practice 4. Pseudo-inverse (fuction declaration)
//------------------------------------------------------------//
MatrixXd drok4_plugin::PseudoInverseMat(MatrixXd A, double lambda)
{
    //Input : Any m-by-n matrix
    //Output : An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    MatrixXd pinvA, I;
    int m = A.rows(), n = A.cols();

    //실인수 행렬 A의 행 m과 열 n의 크기를 비교한다.
    //m > n이고 rank(A) = n인 경우, if()문 내의 left pseudo-inverse 코드를 실행한다.
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A);

    if (m > n && lu_decomp.rank() == n)
    {
        I = MatrixXd::Identity(n, n);
        pinvA = (A.transpose() * A + pow(lambda, 2) * I).inverse() * A.transpose();
    }
    else if (m < n && lu_decomp.rank() == m)
    {
        I = MatrixXd::Identity(m, m);
        pinvA = A.transpose() * (A * A.transpose() + pow(lambda, 2) * I).inverse();
    }
    else pinvA = A.inverse();

    return pinvA;
}

//------------------------------------------------------------//
// Practice 5. Inverse kinematics (fuction declaration)
//------------------------------------------------------------//
VectorXd drok4_plugin::InverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol)
{
    //Input : desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    //Output : joint angles which match desired end-effector position and orientation

    clock_gettime(CLOCK_MONOTONIC, &ik_start_time);         //IK 시작 시각을 ik_start_time 변수에 저장한다.

    double num_it = 0;
    MatrixXd J_P(3, 6), J_R(3, 6), J(6, 6), pinvJ(6, 6), C_err(3, 3), C_IE(3, 3);
    VectorXd q(6), dq(6), dXe(6);
    Vector3d dr, dph;
    double lambda;

    //Set maximum number of iterations
    double max_it = 200;

    //Initialize the solution with the initial guess
    q = q0;
    C_IE = JointToRotMat(q);     //q0로 구한 end-effector의 orientation
    C_err = C_des * C_IE.transpose();

    //Damping factor
    lambda = 0.001;

    //Initialize error
    dr = r_des - JointToPosition(q);
    dph = RotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);

    ////////////////////////////////////////////////
    // Iterative inverse kinematics
    ////////////////////////////////////////////////

    //Iterate until terminating condition
    while (num_it < max_it && dXe.norm() > tol)
    {

        //Compute Inverse Jacobian
        J_P = JointToPosJac(q);
        J_R = JointToRotJac(q);

        J.block(0, 0, 3, 6) = J_P;
        J.block(3, 0, 3, 6) = J_R; // Geometric Jacobian

        //Convert to Geometric Jacobian to Analytic Jacobian
        dq = PseudoInverseMat(J, lambda) * dXe;

        //Update law
        q += 0.5 * dq;

        //Update error
        C_IE = JointToRotMat(q);
        C_err = C_des * C_IE.transpose();

        dr = r_des - JointToPosition(q);
        dph = RotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);

        num_it++;
    }

    clock_gettime(CLOCK_MONOTONIC, &ik_end_time);
    long nano_sec_dt = ik_end_time.tv_nsec - ik_start_time.tv_nsec;

    std::cout << "\niteration : " << num_it << ", value : \n" << q * R2D << std::endl;
    std::cout << "IK dt (us) : " << nano_sec_dt/1000 << std::endl;

    return q;
}

//------------------------------------------------------------//
// Practice 6. 1 - cos trajectory planning
//------------------------------------------------------------//
double drok4_plugin::Func_1_cos(double init, double final, double t, double T)
{
    double des;

    des = init + (final - init) * 0.5 * (1.0 - cos(M_PI * t/T));

    return des;
}

Vector3d drok4_plugin::Func_1_cos(Vector3d init, Vector3d final, double t, double T)
{
    Vector3d des;

    des(0) = init(0) + (final(0) - init(0)) * 0.5 * (1 - cos(M_PI * t/T));
    des(1) = init(1) + (final(1) - init(1)) * 0.5 * (1 - cos(M_PI * t/T));
    des(2) = init(2) + (final(2) - init(2)) * 0.5 * (1 - cos(M_PI * t/T));

    return des;
}

double drok4_plugin::Func_1_cos_yaw(double start, double end, double t, double T)
{
    double yaw;
    double delta = end - start;

    if (fabs(delta) > M_PI && delta >= 0)
        delta = delta - 2 * M_PI;
    else if (fabs(delta) > M_PI && delta < 0)
        delta = delta + 2 * M_PI;

    yaw = start + delta * 0.5 * (1 - cos(M_PI * (t/T)));
    cout << "\nstart = " << start*R2D << ", end = " << end*R2D << endl;
    cout << "delta = " << delta*R2D << endl;

    return yaw;
}

void drok4_plugin::JoystickControl(void)
{
    //--------------------------------------------------//
    // Joystick Control #1
    //--------------------------------------------------//

    int i;
    if (ctrl_mode == 0 && goal_cal_once == true)
    {
        //total_t = 4.0;
        accum_t = 0.0;
        // Home positioning //
        if (phase == 0) {
            goal_posi << 0.7, 0, 0.8;
            goal_rot = MatrixXd::Identity(3, 3);
            q0 << 0, -75, -70, 0, 5, 0;
            q0 *= D2R;
        }
        q_goal = InverseKinematics(goal_posi, goal_rot, q0, 0.001);
        q0 = q_goal;
        goal_cal_once = false;
    }
    else if (ctrl_mode == 1)
    {
        // Position control //
        start_posi = goal_posi;
        start_rot = goal_rot;
        goal_posi(X_) = start_posi(X_) + joy.x;
        goal_posi(Y_) = start_posi(Y_) + joy.y;
        goal_posi(Z_) = start_posi(Z_) + joy.z;
        if (sqrt(pow(goal_posi(X_), 2) + pow(goal_posi(Y_), 2)) < 0.30) {
            // Position inner limint examination, r = 0.35 m //
            goal_posi(X_) = start_posi(X_);
            goal_posi(Y_) = start_posi(Y_);
        }
        command_posi = goal_posi;
        command_rot = goal_rot;
        q_command = InverseKinematics(command_posi, command_rot, q0, 0.001);
        q0 = q_command;
    }
    else if (ctrl_mode == 2)
    {
        // Rotation control //
        start_posi = goal_posi;
        start_rot = goal_rot;
        Vector3d temp_Euler;
        temp_Euler = RotMatToEulerZyx(start_rot);
        ori.roll = temp_Euler(2) + joy.x;
        ori.pitch = temp_Euler(1) + joy.y;
        ori.yaw = temp_Euler(0) + joy.z;
        goal_rot = EulerZyxToRotMat(ori.yaw, ori.pitch, ori.roll);
        //goal_rot = RpyToRotMat(ori.roll, ori.pitch, ori.yaw);
        //C_err = goal_rot * start_rot.transpose();
        //a_axis = RotMatToAngleAxis(C_err);
        command_posi = goal_posi;
        command_rot = goal_rot;
        //command_rot = start_rot * AngleAxisToRotMat(a_axis);
        q_command = InverseKinematics(command_posi, command_rot, q0, 0.001);
        q0 = q_command;
    }
    else if (ctrl_mode == 3)
    {
        if (goal_cal_once == true) {
            q_goal = q_command;
            goal_cal_once = false;
        }
        cout << "Manual joint control : Joint " << joint_id + 1 << endl;
        //q_goal = q_command;
        q_goal(joint_id) += joy.z;
        goal_posi = JointToPosition(q_goal);
        goal_rot = JointToRotMat(q_goal);
        q_command = q_goal;
        q0 = q_command;
    }

    if (ctrl_mode == 0)
    {

        //total_t = 4.0;
        if (accum_t < total_t) {
            targetRadian[J1] = Func_1_cos(q_init(0), q_goal(0), accum_t, total_t);
            targetRadian[J2] = Func_1_cos(q_init(1), q_goal(1), accum_t, total_t);
            targetRadian[J3] = Func_1_cos(q_init(2), q_goal(2), accum_t, total_t);
            targetRadian[J4] = Func_1_cos(q_init(3), q_goal(3), accum_t, total_t);
            targetRadian[J5] = Func_1_cos(q_init(4), q_goal(4), accum_t, total_t);
            targetRadian[J6] = Func_1_cos(q_init(5), q_goal(5), accum_t, total_t);
        }
        else if (accum_t >= total_t) {
            targetRadian[J1] = q_goal(0);
            targetRadian[J2] = q_goal(1);
            targetRadian[J3] = q_goal(2);
            targetRadian[J4] = q_goal(3);
            targetRadian[J5] = q_goal(4);
            targetRadian[J6] = q_goal(5);
            q_init = q_goal;
            q_command = q_goal;
        }
    }
    else
    {
        VectorXd q_exam(6);
        q_exam = q_command - q_present;
        if (fabs(q_exam(J6)) > 10 * D2R || fabs(q_exam(J5)) > 10 * D2R ||
                fabs(q_exam(J4)) > 10 * D2R || fabs(q_exam(J3) > 10 * D2R) ||
                fabs(q_exam(J2)) > 10 * D2R || fabs(q_exam(J1) > 10 * D2R))
        {
            q_command = q_present;
            goal_posi = JointToPosition(q_command);
            goal_rot = JointToRotMat(q_command);
        }
        targetRadian[J1] = q_command(0);
        targetRadian[J2] = q_command(1);
        targetRadian[J3] = q_command(2);
        targetRadian[J4] = q_command(3);
        targetRadian[J5] = q_command(4);
        targetRadian[J6] = q_command(5);
        q_init = q_command;
    }

    if (ctrl_mode == 3)
        cout << "\nq_command = \n" << q_command << endl;

    for (i = 0; i < nDoF; i++) {
        q_present(i) = targetRadian[i];
    }
    present_posi = JointToPosition(q_present);
    present_rot = JointToRotMat(q_present);

    //temp_count += 0.001;
    //cout << "\nTemp count : " << temp_count << endl;

    //cout << "\nEulerZYX = \n" << RotMatToEulerZyx(present_rot) * R2D << endl;
    //cout << "\nq_present = \n" << q_present * R2D << endl;
    //cout << "\nstart_position = \n" << start_posi << endl;
    //cout << "\ngoal_position = \n" << goal_posi << endl;
    cout << "\nTime : " << accum_t << endl;
    cout << "\npresent_position = \n" << present_posi << endl;
     cout << "\njoint = \n" << q_present << endl;
    //cout << "\ncommand_position = \n" << command_posi << endl;
    //memcpy(commandRadian, targetRadian, sizeof (targetRadian));


    accum_t = accum_t + dt;
}

drok4_plugin run;

//------------------------------------------------------------//
// Joystick callback function
//------------------------------------------------------------//
void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    press[0] = msg->buttons[0];
    press[1] = msg->buttons[1];
    press[2] = msg->buttons[2];
    press[3] = msg->buttons[3];
    press[4] = -(int)msg->axes[6];    //left : 1.0, right : -1.0

    for (int i = 0; i < 5; i++)
    {
        if (abs(prev_press[i]) == 1 && prev_press[i] * press[i] == 0) {
            pressed[i] = true;
        }
        if (run.accum_t < run.total_t) pressed[i] = false;
    }

    if (pressed[1] == true)
    {
        //O, Home positioning button
        pressed[3] = false;
        ctrl_mode = 0;      //Home positioning
        run.accum_t = 0.0;
        goal_cal_once = true;
        pressed[1] = false;
    }
    else if (pressed[0] == true)
    {
        //X, Joystick position control button
        pressed[3] = false;
        ctrl_mode = 1;      //E-e position ctrl
        pressed[0] = false;
    }
    else if (pressed[2] == true)
    {
        //Triangle, Joystick rotation control button
        pressed[3] = false;
        ctrl_mode = 2;      //E-e orientation ctrl
        pressed[2] = false;
    }
    else if (pressed[3] == true)
    {
        //Square, Manual joint control button (Joystick)
        ctrl_mode = 3;      //Manual ctrl
        goal_cal_once = true;
        if (pressed[4] == true && press[4] > 0) {
            if (joint_id == 5) joint_id = 0;
            else joint_id++;
            //joint_id = (joint_id == 5) ? (0) : (joint_id++);
            pressed[4] = false;
        }
        else if (pressed[4] == true && press[4] < 0) {
            if (joint_id == 0) joint_id = 5;
            else joint_id--;
            //joint_id = (joint_id == 0) ? (6) : (joint_id--);
            pressed[4] = false;
        }
    }

    if (ctrl_mode == 1)
    {
        joy.x = msg->axes[1] * 0.0005;
        joy.y = msg->axes[0] * -0.0005;
        joy.z = msg->axes[4] * 0.0005;
    }
    else if (ctrl_mode == 2)
    {
        joy.x = msg->axes[1] * 0.05 * D2R;
        joy.y = msg->axes[0] * -0.05 * D2R;
        joy.z = msg->axes[4] * 0.05 * D2R;
    }
    else if (ctrl_mode == 3)
    {
        joy.z = msg->axes[4] * 0.01 * D2R;
    }

    memcpy(prev_press, press, sizeof (prev_press));
    //cout << "\nJoystick x : " << msg->axes[1] << endl;
    //cout << "Joystick y : " << msg->axes[0] << endl;
    //cout << "Joystick z : " << msg->axes[4] << endl;
    //cout << "\npressed = " << pressed[0] << '\t' << pressed[1] << '\t' << pressed[2]
    //     << '\t' << pressed[3] << '\t' << pressed[4] << endl;
    //cout << "\njoint_id = " << joint_id << endl;
}

void *P_Function(void *data)
{
    ROS_INFO("Thread function start");

    pid_t process_id;
    pthread_t thread_id;

    process_id = getpid();
    thread_id = pthread_self();
    //cout << "\nProcess ID : " << process_id << endl;
    //cout << "Thread ID : " << thread_id << endl;

    //char *thread_name = (char *)data;
    //printf("Thread name : %s", thread_name);
    int index = 0;

    //next_time 구조체에 현재의 시간 저장
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (ros::ok())
    {
        ///run.accum_t = run.accum_t + run.dt;
        ///cout << "\nTime : " << run.accum_t << endl;

        //next_time 구조체에 control_cycle(dt)를 더한 t + 1의 시간 저장
        next_time.tv_sec += (next_time.tv_nsec + run.control_cycle * 1000000) / 1000000000;
        next_time.tv_nsec = (next_time.tv_nsec + run.control_cycle * 1000000) % 1000000000;

        run.JoystickControl();
        //cout << "\nTime : " << run.accum_t << endl;

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "temp_arm_node");

    ros::NodeHandle nh;

    //ros::Publisher joint_pub = nh.advertise<temp_arm::Gripper>("command_joint", 1000);
    ros::Subscriber joy_sub = nh.subscribe("joy", 1000, JoyCallback);
    //ros::Subscriber joy_sub;
    //temp_arm::Gripper joint_msg;

    pthread_t pthread;
    int thread_id;
    int status;
    char pth1[] = "joystick_control";

    thread_id = pthread_create(&pthread, NULL, P_Function, (void *)pth1);

    if (thread_id < 0)
    {
        perror("pthread 0(joystick control) create error");
        exit(EXIT_FAILURE);
    }

    ros::spin();
    /*
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        //joint_msg.joint[J1] = run.targetRadian[J1];
        //joint_msg.joint[J2] = run.targetRadian[J2];
        //joint_msg.joint[J3] = run.targetRadian[J3];
        //joint_msg.joint[J4] = run.targetRadian[J4];
        //joint_msg.joint[J5] = run.targetRadian[J5];
        //joint_msg.joint[J6] = run.targetRadian[J6];

        //joint_pub.publish(joint_msg);

        //joy_sub = nh.subscribe("joy", 1000, JoyCallback);
        ros::spinOnce();
        loop_rate.sleep();
    }
    */

    return 0;
}
