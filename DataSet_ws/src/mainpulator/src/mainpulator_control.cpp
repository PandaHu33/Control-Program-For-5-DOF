#include "mainpulator/mainpulator_control.h"
#include <cstdint>
using namespace std;
namespace control{
    mainpulator::mainpulator(int id)
    {
        this->angle = 0;
        this->angleinit = 0;
        this->position = 0;
        this->id = id;
        this->electricinit = 0;
    }
    mainpulator::mainpulator(int id,int position)
    {
        this->angle = 0;
        this->angleinit = 0;
        this->position = position;
        this->id = id;
        this->electricinit = 0;
    }
         
    mainpulator::~mainpulator()
    {
    }

    /** 
     * @brief 记录关节初始位置
     * @param frames  CAN总线当前的返回值
     * @return 无
     */
    void mainpulator::Position_init(const struct can_frame frames)
    {
        if(frames.can_dlc==0)
        {
            ROS_INFO("errod");
            return ;
        }
        int  val ;
        for(int i=7;i>=4;i--)
        {
            val = frames.data[i] + (val)*256;
        }


        int position_init_value=this->position;
        //防止二关节出现多圈
        if(val>524288 || val<0)
        {
            this->position=(floor(val/524288))*524288+position_init_value;
        }
    //此处不需要读取当前编码器位置作为绝对位置，前提是用arc控制，且使用了绝对位置初始化关节
    //  if(this->id==1)
        ROS_INFO("joint%d val = %d this->position = %d",this->id,val ,this->position);
    }

/** 
     * @brief 记录关节初始位置
     * @param frames  CAN总线当前的返回值
     * @return 无
     */
    void mainpulator::Position_init_val_for_joint5(const struct can_frame frames)
    {
        if(frames.can_dlc==0)
        {
            ROS_INFO("errod");
            return ;
        }
        int  val ;
        for(int i=7;i>=4;i--)
        {
            val = frames.data[i] + (val)*256;
        }
        this->position=val;
    //  if(this->id==1)
        ROS_INFO("joint%d val = %d this->position = %d",this->id,val ,this->position);
    }



    /** 
     * @brief 机械臂的额定电流 mA
     * @param frames  CAN总线当前的返回值
     * @return 无
     */
    void mainpulator::Electric_init(const struct can_frame frames)
    {
        this->electricinit = 0;
        if(frames.can_dlc==0)
        {
            ROS_INFO("errod");
            return ;
        }
        for(int i=7;i>=4;i--)
        {
            this->electricinit = frames.data[i] + (this->electricinit)*256;
        }
    //  if(this->id==1)
    ROS_INFO("joint%d this->electricinit = %d",this->id,this->electricinit);
    }

    /** 
     * @brief 将角度转化成编码器对应的绝对位置(16进制，4个字节)，存入放入结构体
     * @param exp_angle  角度
     * @return can_msgs::Frame canopen结构体
     */
    can_msgs::Frame mainpulator::set_angle(double exp_angle){
        can_msgs::Frame frame_msgs;
        frame_msgs.id = 0x200+this->id;
        frame_msgs.dlc = 6;
        if(this->id==3)
        {
            frame_msgs.data[0] = 0x2F;
        }
        else
        {
            frame_msgs.data[0] = 0x2F;
        }

        int except_postion = this->position+(exp_angle*524288/(2*M_PI));
     //ROS_INFO("except_postion = %d, position = %d ",except_postion,this->position);
    //  ROS_INFO("exp_angle = %f ",exp_angle);
        for(int i=2;i<6;i++)
        {
            frame_msgs.data[i] = (except_postion&0xFF);
            except_postion = (except_postion>>8);
        }

        return frame_msgs;
    }

    can_msgs::Frame mainpulator::set_angle_for_new_joint(double exp_angle){
        can_msgs::Frame frame_msgs;
        frame_msgs.id = 0x200+this->id;
        frame_msgs.dlc = 6;       
        frame_msgs.data[0] = 0x3F;
        int except_postion = this->position+(exp_angle*524288/(2*M_PI));
        for(int i=2;i<6;i++)
        {
            frame_msgs.data[i] = (except_postion&0xFF);
            except_postion = (except_postion>>8);
        }
        return frame_msgs;
    }

    /** 
     * @brief 将力矩转化成电流
     * @param tol  力矩
     * @return can_msgs::Frame canopen结构体
     */
    can_msgs::Frame mainpulator:: MomentOutput(double tol){
       
        can_msgs::Frame frame_msgs;
        frame_msgs.id = 0x200 + this->id;
        frame_msgs.dlc = 6;
        frame_msgs.data[0] = 0x1F;
        double I = (tol/0.088)/67.5;
        double am = I*1000*1000/(this->electricinit*1.0);
		short out = (short) am;
        //ROS_INFO("am=%lf out = %lf ",am,out);
        if (out>= 1000){
			out = 1000;
		}
		else if (out<=-1000){
			out = -1000; 
		}
        frame_msgs.data[0] = 0x1F;
        frame_msgs.data[2] = out&0xFF;
        frame_msgs.data[3] = (out>>8)&0xFF;
       // ROS_INFO("out =  %d",out);
        return  frame_msgs; 

    }


        /** 
     * @brief 将力矩转化成电流
     * @param tol  力矩
     * @return can_msgs::Frame canopen结构体
     */
    can_msgs::Frame mainpulator:: MomentOutput80(double tol){
       
        can_msgs::Frame frame_msgs;
        frame_msgs.id = 0x200 + this->id;
        frame_msgs.dlc = 6;
        frame_msgs.data[0] = 0x1F;
        double I = (tol/0.101)/67.5;
        double am = I*1000*1000/(this->electricinit*1.0);
		short out = (short) am;
        if (out>= 1000){
			out = 1000;
		}
		else if (out<=-1000){
			out = -1000; 
		}
        frame_msgs.data[0] = 0x1F;
        frame_msgs.data[2] = out&0xFF;
        frame_msgs.data[3] = (out>>8)&0xFF;
       // ROS_INFO("out =  %d",out);
        return  frame_msgs; 

    }


    /** 
     * @brief 编码器对应的绝对位置(16进制，4个字节)转化为角度值 ，存入放入结构体
     * @param position  canopen结构体
     * @return 无
     */
    void mainpulator::current_angle(const can_msgs::Frame& position)
    {
        int val = 0;
        for (int i = position.dlc-1; i >=position.dlc-4; i--)
        {
            val = (val<<8)+position.data[i];
        }


        if(this->id !=5)
        {
            this->angle = double((val - this->position)%524288)*1.0*2*M_PI/524288;
            // this->angle += this->angle<0 ? 2*M_PI : 0;
            // if(this->id == 3){
            //     cout << "q3 decoder value: " << val - this->position << "q3 angle: " << this->angle << endl;
            // }
        }
        else{
            this->angle = double(val-this->position)*1.0*2*M_PI/524288;
        }
        
     // ROS_INFO("current angle = %f",this->angle);
    }


    void mainpulator:: current_velocity(const can_msgs::Frame& velocity){
        int val = 0;

        for (int i = velocity.dlc-1; i >=velocity.dlc-4; i--)
        {
            val = (val<<8)+velocity.data[i];
        }
        this->velocity = double(val)*1.0*2*M_PI/524288;
       /// ROS_INFO("joint_%d  current velocity = %f",this->id, this->velocity);

    }
    void mainpulator:: ActualCurrent(const can_msgs::Frame& electric){
        if (electric.dlc < 2) return;
        const std::int16_t val = static_cast<std::int16_t>(
            static_cast<std::uint16_t>(electric.data[0]) |
            (static_cast<std::uint16_t>(electric.data[1]) << 8));
        string str ;
       /* for(int i=0;i<electric.dlc;i++)
        {
        //    std::cout << std::hex << (electric.data[i] & 0xff) << " ";
        }*/
        //cout<<endl;
        this->electric = static_cast<double>(val) * static_cast<double>(this->electricinit) / 1000.0;
      // ROS_INFO("joint_%d  val =  %d   current electric = %f",this->id,  val,this->electric);
    }

    /** 
     * @brief 编码器对应的绝对位置(16进制，4个字节)转化为角度值 ，存入放入结构体
     * @param   
     * @return
     */
    can_msgs::Frame mainpulator::ret_to_init()
    {
        can_msgs::Frame frame_msg;
        frame_msg.id = 0x200+this->id;
        frame_msg.dlc = 6;
        frame_msg.data[0] = 0x1F;
        int except_postion = this->position;
        for(int i=2;i<6;i++)
        {
            frame_msg.data[i] = (except_postion&0xFF);
            except_postion = (except_postion>>8);
        }
        if(abs(this->angleinit-this->angle)<0.001)
        {
            frame_msg.data[0] = 0x06;
           // ROS_INFO("manpulator stop");
        }
        return frame_msg;
    }

    /** 
     * @brief 编码器对应的绝对位置(16进制，4个字节)转化为角度值 ，存入放入结构体
     * @param positon  编码器绝对位置
     * @return 无
     */
    can_msgs::Frame mainpulator::ret_to_init(int positon)
    {
        can_msgs::Frame frame_msg;
        frame_msg.id = 0x200+this->id;
        frame_msg.dlc = 6;
        frame_msg.data[0] = 0x1F;
        int except_postion = positon;
        
        for(int i=2;i<6;i++)
        {
            frame_msg.data[i] = (except_postion&0xFF);
            except_postion = (except_postion>>8);
        }
    // ROS_INFO("this->position = %d,  positon = %d",this->position,positon);
        //if(abs(this->angle-this->curent_angle)<0.001)
    //  {
        //  frame_msg.data[0] = 0x06;
        //  ROS_INFO("manpulator stop");
    // }
        return frame_msg;
    }

    double arc_control(double expect_q,double expect_dq,double expect_ddq,double q,double dq,double t ,double m,double c,double g){

        static double dthp1 = 0;
        static double dthp2 = 0;
        static double dthp3 = 0;
        static double thp1 = m;
        static double thp2 = c;
        static double thp3 = g;

        double k2 = 10;
        double k1 = 25;
        double fai1 = 40;
        double fai2 = 0;
        double fai3 = 0;    


        double err = q - expect_q;
        double derr = dq - expect_dq;
        double s = derr + k2 * err;
        double us1 = - k1 * s;

        dthp1 = ((expect_ddq-k1*derr)*s)*t;
        dthp2 = ((expect_dq-k1*err)*s)*t;
        dthp3 = s*t;

        double th_min1 = 0.005;
	    double th_max1 = 50;
	    double th_min2 = 2;
	    double th_max2 = 13;
	    double th_min3 = 2;
	    double th_max3 = 10;

        if(th_max1<=thp1&&dthp1>0)
		    dthp1 = 0;
	    if(thp1<=th_min1&&dthp1<0)
		    dthp1 = 0;

	    if(th_max2<=thp2&&dthp2>0)
		    dthp2 = 0;
	    if(thp2<=th_min2&&dthp2<0)
		    dthp2 = 0;
	
	    if(th_max3<=thp3&&dthp3>0)
		    dthp1 = 0;
	    if(thp1<=th_min3&&dthp3<0)
		    dthp1 = 0;


	    thp1 += dthp1*t;
	    thp2 += dthp2*t;
	    thp3 += dthp3*t;
        double ua = -1.0*(fai1 * thp1 * expect_ddq + fai2 * thp2 * expect_dq + fai3 * thp3 );
        double u = us1 + ua  ;
        return u;

    }
}
