#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
// ros/ros.h是一个实用的头文件，它引用了ROS系统中大部分常用的头文件


#define KEYCODE_Q 0x71
#define KEYCODE_E  0x65
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

//#define 	KEYCODE_a   0x61
#define 	KEYCODE_b   0x62
#define 	KEYCODE_c   0x63
//#define 	KEYCODE_d   0x64
//#define 	KEYCODE_e   0x65
#define 	KEYCODE_ESCAPE   0x1B
#define 	KEYCODE_f   0x66
#define 	KEYCODE_g   0x67
#define 	KEYCODE_h   0x68
#define 	KEYCODE_i   0x69
#define 	KEYCODE_j   0x6a
#define 	KEYCODE_k   0x6b
#define 	KEYCODE_l   0x6c
#define 	KEYCODE_m   0x6d
#define 	KEYCODE_n   0x6e
#define 	KEYCODE_o   0x6f
#define 	KEYCODE_p   0x70
//#define 	KEYCODE_q   0x71
#define 	KEYCODE_r   0x72
//#define 	KEYCODE_s   0x73
#define 	KEYCODE_t   0x74
#define 	KEYCODE_u   0x75
#define 	KEYCODE_v   0x76
//#define 	KEYCODE_w   0x77
#define 	KEYCODE_x   0x78
#define 	KEYCODE_y   0x79
#define 	KEYCODE_z   0x7a
class KeyboardTeleopNode {  //声明TeleopTurle类
private:

  geometry_msgs::Twist Cmd_KeyboardTeleop;
  ros::NodeHandle n_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher KeyboardTeleop_pub_;

public:
  KeyboardTeleopNode()
  {
    KeyboardTeleop_pub_ = n_.advertise<geometry_msgs::Twist>("Cmd_KeyboardTeleop", 1);
            //告诉master我们将要在Cmd_KeyboardTeleop(话题名)上发布geometry_msgs/Twist消息类型的消息
        //这样master就会告诉所有订阅了Cmd_KeyboardTeleop话题的节点,将要有数据发布．
        //第二个参数是发布序列的大小,如果我们发布消息的频率太高，
        //缓冲区中的消息在大于1000个的时候就会开始丢弃先前发布的消息
        //NodeHandle::advertise() 返回一个 ros::Publisher 对象,它有两个作用：
        //1) 它有一个 publish() 成员函数可以让你在topic上发布消息；
        //2) 如果消息类型不对,它会拒绝发布。
        ros::NodeHandle n_private("~");
  }
  ~KeyboardTeleopNode() {}

  void keyboardLoop();

   void stopPubKB() {
        Cmd_KeyboardTeleop.linear.x = 0.0;
        Cmd_KeyboardTeleop.linear.y = 0.0;
        Cmd_KeyboardTeleop.linear.z = 0.0;
        Cmd_KeyboardTeleop.angular.x = 0.0;
        Cmd_KeyboardTeleop.angular.y = 0.0;
        Cmd_KeyboardTeleop.angular.z = 0.0;
        KeyboardTeleop_pub_.publish(Cmd_KeyboardTeleop);
    }
};

KeyboardTeleopNode *tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;


int main(int argc, char **argv) {
    //初始化ROS.它允许允许ROS通过命令进行名称重映射．
    //我们可以指定节点的名称，但是这里的名称必须是basename,名称内不能包含/等符号
    ros::init(argc, argv, "tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);

    KeyboardTeleopNode tbk;

    //重新定义一个线程
    boost::thread t = boost::thread(boost::bind(&KeyboardTeleopNode::keyboardLoop, &tbk));


    //ros::spin()在调用后不会再返回，也就是你的主程序到这儿就不往下执行了，
    //而ros::spinOnce()后者在调用后还可以继续执行之后的程序。
    ros::spin();

    //中断线程
    t.interrupt();
    //由于线程中断，所以立即返回
    t.join();
    //速度置零
    tbk.stopPubKB();

    tcsetattr(kfd, TCSANOW, &cooked);

    return (0);
}


void KeyboardTeleopNode::keyboardLoop() {

    char c;
    //double max_tv = walk_vel_;
    //double max_rv = yaw_rate_;
    bool dirty = false;
    int flag_A = 0;
    int flag_B = 0;
    int flag_C = 0;

    // get the console in raw mode  
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);


    puts("Reading from keyboard");
    //puts("Use WASD keys to control the robot");
    //puts("Press Shift to move faster");


    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for (;;) {
        boost::this_thread::interruption_point();
        // get the next event from the keyboard  
        int num;
        if ((num = poll(&ufd, 1, 250)) < 0) {
            perror("poll():");
            return;
        } else if (num > 0) {
            if (read(kfd, &c, 1) < 0) {
                perror("read():");
                return;
            }
        } else {
            if (dirty == true) {
                stopPubKB();
                dirty = false;
            }
            continue;
        }


        switch (c) {

            case KEYCODE_W:
                flag_A=1;
                dirty = true;
                break;

            case KEYCODE_S:
                flag_A=-1;
                dirty = true;
                break;

            case KEYCODE_A:
                flag_B=1;
                dirty = true;
                break;

            case KEYCODE_D:
                flag_B=-1;
                dirty = true;
                break;

            case KEYCODE_Q:
                flag_C=1;
                dirty = true;
                break;

            case KEYCODE_E:
                flag_C=-1;
                dirty = true;
                break;
            default:
                flag_A=0;
                flag_B=0;
                flag_C=0;
                dirty = false;
        }

        Cmd_KeyboardTeleop.linear.x = flag_A*1.0;
        Cmd_KeyboardTeleop.linear.y = flag_B*1.0;
        Cmd_KeyboardTeleop.linear.z = flag_C*1.0;

        Cmd_KeyboardTeleop.angular.x = flag_B*1.0;
        Cmd_KeyboardTeleop.angular.y = flag_B*1.0;
        Cmd_KeyboardTeleop.angular.z = flag_B*1.0;
        KeyboardTeleop_pub_.publish(Cmd_KeyboardTeleop);
        ROS_INFO("KeyBoard_AHEAD= %lf",Cmd_KeyboardTeleop.linear.x);
        ROS_INFO("KeyBoard_RIGHT= %lf",Cmd_KeyboardTeleop.linear.y);
        ROS_INFO("KeyBoard_UP= %lf",Cmd_KeyboardTeleop.linear.z);
    }

}