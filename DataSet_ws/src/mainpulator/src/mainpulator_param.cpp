#include "mainpulator/mainpulator_param.h"
using namespace std;

namespace param
{
    /** 
     * @brief socket套字节can总线初始化
     * @param 
     * @return 
     */
    int SocketCANInit(){
        struct ifreq ifr;
        struct sockaddr_can addr;
        int sock_fd;
        int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
        /*建立套接字，设置为原始套接字，原始CAN协议 proto = CAN_RAW*/
        if ((sock_fd = socket(family, type, proto)) < 0) {
            perror("socket");
            return -1;
        }
        /*以下是对CAN接口进行初始化，如设置CAN接口名，即当我们用ifconfig命令时显示的名字*/
        strcpy(ifr.ifr_name, "can0");
        ioctl(sock_fd, SIOCGIFINDEX, &ifr);
        printf("ifr_name-in = %s, ifr_name-out = %s, family = %d, type = %d, proto = %d\n",
            ifr.ifr_name, ifr.ifr_name, family, type, proto);
        /*设置CAN协议*/
        addr.can_ifindex = ifr.ifr_ifindex;
        addr.can_family = AF_CAN;
        /*将刚生成的套接字与网络地址进行绑定*/
        if (bind(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("bind");
            return -1;
        }
        printf("can connect sucess\n");
        return sock_fd;
    }
    /** 
     * @brief socket套字节can总线写入
     * @param frames ros_canopen类对象
     * @param sock_fd socket描述字
     * @return 
     */
    void SocketWrite(const struct can_frame frames,const int sock_fd){

        int writenBytes; 
        struct can_frame  frame;
        frame.can_id =  frames.can_id;
        frame.can_dlc = frames.can_dlc;
        string str = "id = "+ to_string(frames.can_id)+"  ";
        /*cout<<"idw  =  ";
        std::cout << std::hex << (frames.can_id & 0xff) << " ";*/
        for(int i=0;i<frame.can_dlc;i++)
        {
            str += to_string(frames.data[i]) + " ";
            frame.data[i] = frames.data[i];
        }
        writenBytes =  write(sock_fd,&frames,sizeof(frames));
        /*for(int i=0;i<frames.can_dlc;i++)
         {
                std::cout << std::hex << (frames.data[i] & 0xff) << " ";
        }
        cout<<endl;*/
        if(writenBytes!=sizeof(frames))
        {
            printf("send error\n");
            exit(0);
        }
      //  ROS_INFO("write %s",str.c_str());

    }
    /** 
     * @brief socket套字节can总线写入
     * @param frames ros_canopen类对象
     * @param sock_fd socket描述字
     * @return 
     */
    void SocketRead(struct can_frame&  frames,const int sock_fd){
        int readBytes;
        struct can_frame  reframes;
        readBytes = read(sock_fd,&reframes,sizeof(reframes));
        if(reframes.can_dlc!=0)
        {
            if(readBytes<0)
            {
                printf("read error\n");
                exit(0);
            }
            else if (reframes.data[0]==0x80)
            {
                printf("read Data error");
                for(int i=0;i<reframes.can_dlc;i++)
                {
                       std::cout << std::hex << (reframes.data[i] & 0xff) << " ";
                }
                cout<<endl;
                exit(0);
            }else
            {
                frames.can_id = reframes.can_id;
                frames.can_dlc = reframes.can_dlc;
                string str = "id = "+ to_string(reframes.can_id)+"  ";
                for(int i=0;i<reframes.can_dlc;i++)
                {
                    frames.data[i] = reframes.data[i];
                    str += to_string(reframes.data[i]) + " ";
                }
                /*cout<<"idr =  ";
                std::cout << std::hex << (frames.can_id & 0xff) << " ";
               for(int i=0;i<reframes.can_dlc;i++)
                {
                       std::cout << std::hex << (reframes.data[i] & 0xff) << " ";
                }
                cout<<endl;*/
              //  ROS_INFO("read %s",str.c_str());
            }
        }

    }

    void PositionInit(control::mainpulator& joint,const int sock_fd){
        struct can_frame  frames,reframes;
        frames.can_id = 0X00;
        frames.can_dlc = 2;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        frames.data[0] = 0X82;
        frames.data[1] = joint.getid();
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //发送远程帧
        frames.can_id = (0x700+joint.getid())|CAN_RTR_FLAG;
        frames.can_dlc =  0;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //清除错误模式
        frames.can_id = 0x600+joint.getid();
        frames.can_dlc = 8;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,clear,sizeof(clear));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置模式，默认力矩模式
        setstates[4] = 0X01;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,setstates,sizeof(setstates));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //回读模式
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,readstate,sizeof(readstate));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        if(reframes.data[4] != 0X01){
            ROS_INFO("state error joint = %d ",joint.getid());
            exit(0);
        }
        //读取位置
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,readPosition,sizeof(readPosition));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        printf("joint = %d ",joint.getid());
        for(int  i=0;i<8;i++)
        {
            printf(" %d  ",reframes.data[i]);
        }
        printf("\n");

        joint.Position_init(reframes);
        //joint.Position_init_val_for_joint5(reframes);
        //查询当前关节的额定电流
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,getA,sizeof(getA));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        joint.Electric_init(reframes);
         //将位置作为目标位置输入
        memset(frames.data,0,sizeof(frames.data));
        memcpy(frames.data,setposition,sizeof(setposition));
        for(int i=4;i<8;i++)
        {
            frames.data[i] = reframes.data[i];
        }
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
   

        if(joint.getid()==5)
        {
            //4E20==2W       7530==3W   EA60==6W
            //设定加速度
            memset(frames.data,0,sizeof(frames.data));
            memset(reframes.data,0,sizeof(reframes.data));
            
            setacceleration[4] = 0x70;//0x10;
            setacceleration[5] = 0x11;//0x27;
            setacceleration[6] = 0x01;
            memcpy(frames.data,setacceleration,sizeof(setacceleration));
            SocketWrite(frames,sock_fd);
            SocketRead(reframes,sock_fd);
            //设定减速度
            memset(frames.data,0,sizeof(frames.data));
            memset(reframes.data,0,sizeof(reframes.data));

            setdeceleration[4] = 0x70;//0x10;
            setdeceleration[5] = 0x11;//0x27;
            setdeceleration[6] = 0x01;
            memcpy(frames.data,setdeceleration,sizeof(setdeceleration));
            SocketWrite(frames,sock_fd);
            SocketRead(reframes,sock_fd);
            //设定速度
            memset(frames.data,0,sizeof(frames.data));
            memset(reframes.data,0,sizeof(reframes.data));
            setVelocity[4] = 0x30;//0x10;
            setVelocity[5] = 0x75;//0x27;
            setVelocity[6] = 0x00;
            memcpy(frames.data,setVelocity,sizeof(setVelocity));
            SocketWrite(frames,sock_fd);
            SocketRead(reframes,sock_fd);
        }
        else{
                if(joint.getid()==3||joint.getid()==2||joint.getid()==1)
                {
                    memset(frames.data,0,sizeof(frames.data));
                    memset(reframes.data,0,sizeof(reframes.data));
                    
                    //setacceleration[4] = 0x10;
                    //setacceleration[5] = 0x27;
                    //setacceleration[4] = 0x20;
                    //setacceleration[5] = 0x4E;
                    //setacceleration[6] = 0x00;
                    setacceleration[4] = 0x70;
                    setacceleration[5] = 0x11;
                    setacceleration[6] = 0x01;

                    //setacceleration[4] = 0x70;
                    //setacceleration[5] = 0x11;
                    //setacceleration[6] = 0x01;
                    memcpy(frames.data,setacceleration,sizeof(setacceleration));
                    SocketWrite(frames,sock_fd);
                    SocketRead(reframes,sock_fd);
                    //设定减速度
                    memset(frames.data,0,sizeof(frames.data));
                    memset(reframes.data,0,sizeof(reframes.data));



                    //setdeceleration[4] = 0x10;
                    //setdeceleration[5] = 0x27;
                    //setdeceleration[4] = 0x20;
                    //setdeceleration[5] = 0x4E;
                    //setacceleration[6] = 0x00;
                    setdeceleration[4] = 0x70;
                    setdeceleration[5] = 0x11;
                    setdeceleration[6] = 0x01;

                    //setdeceleration[4] = 0x70;
                    //setdeceleration[5] = 0x11;
                    //setdeceleration[6] = 0x01;
                    memcpy(frames.data,setdeceleration,sizeof(setdeceleration));
                    SocketWrite(frames,sock_fd);
                    SocketRead(reframes,sock_fd);
                    //设定速度
                    memset(frames.data,0,sizeof(frames.data));
                    memset(reframes.data,0,sizeof(reframes.data));
                    //setVelocity[4] = 0x10;
                    //setVelocity[5] = 0x27;
                    //setVelocity[4] = 0x20;
                    //setVelocity[5] = 0x4E;
                    //setVelocity[6] = 0x00;
                    setVelocity[4] = 0x50;
                    setVelocity[5] = 0xC3;
                    setVelocity[6] = 0x00;
                    memcpy(frames.data,setVelocity,sizeof(setVelocity));
                    SocketWrite(frames,sock_fd);
                    SocketRead(reframes,sock_fd);
                }
                else{
                        memset(frames.data,0,sizeof(frames.data));
                        memset(reframes.data,0,sizeof(reframes.data));
                        
                        //setacceleration[4] = 0x10;
                        //setacceleration[5] = 0x27;
                        setacceleration[4] = 0x50;
                        setacceleration[5] = 0xC3;
                        setacceleration[6] = 0x00;
                        //setacceleration[4] = 0x30;
                        //setacceleration[5] = 0x75;
                        //setacceleration[6] = 0x00;
                        //setacceleration[4] = 0x70;
                        //setacceleration[5] = 0x11;
                        //setacceleration[6] = 0x01;
                        memcpy(frames.data,setacceleration,sizeof(setacceleration));
                        SocketWrite(frames,sock_fd);
                        SocketRead(reframes,sock_fd);
                        //设定减速度
                        memset(frames.data,0,sizeof(frames.data));
                        memset(reframes.data,0,sizeof(reframes.data));



                        //setdeceleration[4] = 0x10;
                        //setdeceleration[5] = 0x27;
                        //setdeceleration[6] = 0x00;
                        setdeceleration[4] = 0x50;
                        setdeceleration[5] = 0xC3;
                        setdeceleration[6] = 0x00;
                        //setdeceleration[4] = 0x30;
                        //setdeceleration[5] = 0x75;
                        //setdeceleration[6] = 0x00;
                        //setacceleration[4] = 0x70;
                        //setacceleration[5] = 0x11;
                        //setacceleration[6] = 0x01;
                        memcpy(frames.data,setdeceleration,sizeof(setdeceleration));
                        SocketWrite(frames,sock_fd);
                        SocketRead(reframes,sock_fd);
                        //设定速度
                        memset(frames.data,0,sizeof(frames.data));
                        memset(reframes.data,0,sizeof(reframes.data));
                        //setVelocity[4] = 0x10;
                        //setVelocity[5] = 0x27;
                        setVelocity[4] = 0x30;
                        setVelocity[5] = 0x75;
                        setVelocity[6] = 0x00;
                        //setVelocity[4] = 0x30;
                        //setVelocity[5] = 0x75;
                        //setVelocity[6] = 0x00;
                        memcpy(frames.data,setVelocity,sizeof(setVelocity));
                        SocketWrite(frames,sock_fd);
                        SocketRead(reframes,sock_fd);

                }
            //设定加速度

        }

        //禁止PDO
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,closePDO,sizeof(closePDO));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设定同步时间
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,setTimes,sizeof(setTimes));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);

        //关闭TPOD1，设置id
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        closeTPDO[1] = 0x00;
        closeTPDO[4] = 0x80+joint.getid();//对应的是电机的id号
        closeTPDO[5] = 0x01;
        memcpy(frames.data,closeTPDO,sizeof(closeTPDO));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);  
        //设置模式
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        setTPDOtype[1] = 0x00;
        memcpy(frames.data,setTPDOtype,sizeof(setTPDOtype));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置禁止时间
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        setPhbTime[1] = 0x00;
        setPhbTime[4] = 0x64;
        memcpy(frames.data,setPhbTime,sizeof(setPhbTime));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //清空映射
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        TPDOmapclear[1] = 0x00;
        memcpy(frames.data,TPDOmapclear,sizeof(TPDOmapclear));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设定映射
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        TPDOmap[1] = 0x00;
        TPDOmap[3] = 0x01;
        TPDOmap[4] = 0x10;
        TPDOmap[6] = 0x40;
        TPDOmap[7] = 0x60;
        memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);

        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        TPDOmap[1] = 0x00;
        TPDOmap[3] = 0x02;
        TPDOmap[4] = 0x20;
        TPDOmap[6] = 0x64;
        TPDOmap[7] = 0x60;
        memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置映射数目

        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        TPDOmapnum[1] = 0x00;
        TPDOmapnum[4] = 0x02;
        memcpy(frames.data,TPDOmapnum,sizeof(TPDOmapnum));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //激活TPDO1
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        openTPOD[1] = 0x00;
        openTPOD[4] = 0x80+joint.getid();
        openTPOD[5] = 0x01;
        memcpy(frames.data,openTPOD,sizeof(openTPOD));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
    //-----------------------------------------------------------------------
    //关闭TPOD2，设置电机对应id
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        closeTPDO[1] = 0x01;
        closeTPDO[4] = 0x80+joint.getid();
        closeTPDO[5] = 0x02;
        memcpy(frames.data,closeTPDO,sizeof(closeTPDO));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);

        //设置模式

        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        setTPDOtype[1] = 0x01;
        memcpy(frames.data,setTPDOtype,sizeof(setTPDOtype));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置禁止时间
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        setPhbTime[1] = 0x01;
        setPhbTime[4] = 0x64;
        memcpy(frames.data,setPhbTime,sizeof(setPhbTime));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //清空映射
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));   
        TPDOmapclear[1] = 0x01;
        memcpy(frames.data,TPDOmapclear,sizeof(TPDOmapclear));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设定映射
        if(joint.getid()==5)
        {
            memset(frames.data,0,sizeof(frames.data));
            memset(reframes.data,0,sizeof(reframes.data));   
            TPDOmap[1] = 0x01;
            TPDOmap[3] = 0x01;
            TPDOmap[4] = 0x10;
            TPDOmap[6] = 0x78;
            TPDOmap[7] = 0x60;
            memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
            SocketWrite(frames,sock_fd);
            SocketRead(reframes,sock_fd);
        }else{

            memset(frames.data,0,sizeof(frames.data));
            memset(reframes.data,0,sizeof(reframes.data));   
            TPDOmap[1] = 0x01;
            TPDOmap[3] = 0x01;
            TPDOmap[4] = 0x10;
            TPDOmap[6] = 0x40;
            TPDOmap[7] = 0x60;
            memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
            SocketWrite(frames,sock_fd);
            SocketRead(reframes,sock_fd);
        }

        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));   
        TPDOmap[1] = 0x01;
        TPDOmap[3] = 0x02;
        TPDOmap[4] = 0x20;
        TPDOmap[6] = 0x6C;
        TPDOmap[7] = 0x60;
        memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置映射数目
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));   
        TPDOmapnum[1] = 0x01;
        TPDOmapnum[4] = 0x02;
        memcpy(frames.data,TPDOmapnum,sizeof(TPDOmapnum));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //激活TPDO2
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));   
        openTPOD[1] = 0x01;
        openTPOD[4] = 0x80+joint.getid();
        openTPOD[5] = 0x02;
        memcpy(frames.data,openTPOD,sizeof(openTPOD));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
    //-----------------------------------------------------------------------
        //关闭RPOD1，设置电机对应id
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));   
        closeRPDO[1] = 0x00;
        closeRPDO[4] = 0x00+joint.getid();
        closeRPDO[5] = 0x02;
        memcpy(frames.data,closeRPDO,sizeof(closeRPDO));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置模式
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));   
        setRPDOtype[1] = 0x00;
        memcpy(frames.data,setRPDOtype,sizeof(setRPDOtype));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //索引清零
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));   
        RPDOmapclear[1] = 0x00;
        memcpy(frames.data,RPDOmapclear,sizeof(RPDOmapclear));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设定映射
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));   
        RPDOmap[1] = 0x00;
        RPDOmap[3] = 0x01;
        RPDOmap[4] = 0x10;
        RPDOmap[6] = 0x40;
        RPDOmap[7] = 0x60;
        memcpy(frames.data,RPDOmap,sizeof(RPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));   
        RPDOmap[1] = 0x00;
        RPDOmap[3] = 0x02;
        RPDOmap[4] = 0x20;
        RPDOmap[6] = 0x7A;
        RPDOmap[7] = 0x60;
        memcpy(frames.data,RPDOmap,sizeof(RPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置映射数目
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));  
        RPDOmapnum[1] = 0x00;
        RPDOmapnum[4] = 0x02;
        memcpy(frames.data,RPDOmapnum,sizeof(RPDOmapnum));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //激活RPDO1
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data)); 
        openRPOD[1] = 0x00;
        openRPOD[4] = 0x00+joint.getid();
        openRPOD[5] = 0x02;
        memcpy(frames.data,openRPOD,sizeof(openRPOD));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);

    }

    void Enable(const int sock_fd,int length,control::mainpulator& joint){
        struct can_frame  frames,reframes;
        frames.can_id = 0x600+joint.getid();
        frames.can_dlc = 8;

        frames.can_id = 0x00;
        frames.can_dlc = 2;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data)); 
        frames.data[0] = 0x01;
        frames.data[1] = joint.getid();
        SocketWrite(frames,sock_fd);

        //发送远程帧
        frames.can_id = (0x700+joint.getid())|CAN_RTR_FLAG;
        frames.can_dlc =  0;
        memset(frames.data,0,sizeof(frames.data));
        SocketWrite(frames,sock_fd);

        frames.can_id = 0x80;
        frames.can_dlc = 0;
        memset(frames.data,0,sizeof(frames.data));
        SocketWrite(frames,sock_fd);

        frames.can_id = 0x200+joint.getid();
        frames.can_dlc = length;
        memset(frames.data,0,sizeof(frames.data));
        frames.data[0] = 0x26;
        SocketWrite(frames,sock_fd);

        frames.can_id = 0x80;
        frames.can_dlc = 0;
        memset(frames.data,0,sizeof(frames.data));
        SocketWrite(frames,sock_fd);

        frames.can_id = 0x200+joint.getid();
        frames.can_dlc = length;
        memset(frames.data,0,sizeof(frames.data));
        frames.data[0] = 0x27;
        SocketWrite(frames,sock_fd);

        frames.can_id = 0x80;
        frames.can_dlc = 0;
        memset(frames.data,0,sizeof(frames.data));
        SocketWrite(frames,sock_fd);

        frames.can_id = 0x200+joint.getid();
        frames.can_dlc = length;
        memset(frames.data,0,sizeof(frames.data));
        frames.data[0] = 0x2F;
        SocketWrite(frames,sock_fd);

        frames.can_id = 0x80;
        frames.can_dlc = 0;
        memset(frames.data,0,sizeof(frames.data));
        SocketWrite(frames,sock_fd);
    
    }

    void MomentInit(control::mainpulator& joint,const int sock_fd){
        int writenBytes;
        struct can_frame  frames,reframes;
        frames.can_id = 0x00;
        frames.can_dlc = 2;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        frames.data[0] = 0x82;
        frames.data[1] = joint.getid();
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //发送远程帧
        frames.can_id = (0x700+joint.getid())|CAN_RTR_FLAG;
        frames.can_dlc =  0;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //清除错误模式
        frames.can_id = 0x600+joint.getid();
        frames.can_dlc = 8;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,clear,sizeof(clear));
        /*for (int i = 0; i < frames.can_dlc; i++)
        {
            printf("%02x ",frames.data[i]);

        }
        printf(" source: ");
        for (int i = 0; i < frames.can_dlc; i++)
        {
            printf("%02x ",clear[i]);

        }*/
        //printf("\n");
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置运动模式   力矩
        setstates[4] = 0X04;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,setstates,sizeof(setstates));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);

        //查询当前模式

        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,readstate,sizeof(readstate));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        if(reframes.data[4] != 0X04){
            for(int i=0;i<reframes.can_dlc;i++)
            {
                    std::cout << std::hex << (reframes.data[i] & 0xff) << " ";
            }
            cout<<endl;
            ROS_INFO("state error joint = %d ",joint.getid());
            exit(0);
        }
    /*	for (int i = 0; i < 8; i++)
        {
            
            printf("%02x ",p.buffer[i]);
        }
        printf("\n");
    */
        //memset(frames.data,0,sizeof(frames.data));
        //memset(reframes.data,0,sizeof(reframes.data));
        //memcpy(frames.data,setA,sizeof(setA));
        //SocketWrite(frames,sock_fd);
        //SocketRead(reframes,sock_fd);
        //禁止PDO
        //查询当前位置
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,readPosition,sizeof(readPosition));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
      printf("joint = %d ",joint.getid());
      for(int  i=0;i<8;i++)
       {
            printf(" %d  ",reframes.data[i]);
        }
        //printf("\n");
        joint.Position_init(reframes);
        //查询当前关节的额定电流
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,getA,sizeof(getA));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        joint.Electric_init(reframes);




        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,closePDO,sizeof(closeTPDO));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置禁止时间
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,setTimes,sizeof(setTimes));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //关闭TPOD1，设置电机对应id
        closeTPDO[1] = 0x00;
        closeTPDO[4] = 0x80+joint.getid();
        closeTPDO[5] = 0x01;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,closeTPDO,sizeof(closeTPDO));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        
        //设置模式
        
        setTPDOtype[1] = 0x00;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,setTPDOtype,sizeof(setTPDOtype));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置禁止时间
        setPhbTime[1] = 0x00;
        setPhbTime[4] = 0x64;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,setPhbTime,sizeof(setPhbTime));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //清空映射
        TPDOmapclear[1] = 0x00;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,TPDOmapclear,sizeof(TPDOmapclear));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设定映射
        TPDOmap[1] = 0x00;
        TPDOmap[3] = 0x01;
        TPDOmap[4] = 0x10;
        TPDOmap[6] = 0x40;
        TPDOmap[7] = 0x60;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);

        TPDOmap[1] = 0x00;
        TPDOmap[3] = 0x02;
        TPDOmap[4] = 0x20;
        TPDOmap[6] = 0x64;
        TPDOmap[7] = 0x60;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置映射数目
        TPDOmapnum[1] = 0x00;
        TPDOmapnum[4] = 0x02;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,TPDOmapnum,sizeof(TPDOmapnum));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //激活TPDO1
        openTPOD[1] = 0x00;
        openTPOD[4] = 0x80+joint.getid();
        openTPOD[5] = 0x01;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,openTPOD,sizeof(openTPOD));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
    /***********************************************************************************/



    //关闭TPOD2，设置电机对应?
        closeTPDO[1] = 0x01;
        closeTPDO[4] = 0x80+joint.getid();
        closeTPDO[5] = 0x02;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,closeTPDO,sizeof(closeTPDO));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置模式

        setTPDOtype[1] = 0x01;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,setTPDOtype,sizeof(setTPDOtype));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置禁止时间
        setPhbTime[1] = 0x01;
        setPhbTime[4] = 0x64;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,setPhbTime,sizeof(setPhbTime));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //清空映射
        TPDOmapclear[1] = 0x01;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,TPDOmapclear,sizeof(TPDOmapclear));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设定映射
        if(joint.getid() == 5){
            memset(frames.data,0,sizeof(frames.data));
            memset(reframes.data,0,sizeof(reframes.data));   
            TPDOmap[1] = 0x01;
            TPDOmap[3] = 0x01;
            TPDOmap[4] = 0x10;
            TPDOmap[6] = 0x78;
            TPDOmap[7] = 0x60;
            memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
            SocketWrite(frames,sock_fd);
            SocketRead(reframes,sock_fd);
        }else{

            memset(frames.data,0,sizeof(frames.data));
            memset(reframes.data,0,sizeof(reframes.data));   
            TPDOmap[1] = 0x01;
            TPDOmap[3] = 0x01;
            TPDOmap[4] = 0x10;
            TPDOmap[6] = 0x40;
            TPDOmap[7] = 0x60;
            memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
            SocketWrite(frames,sock_fd);
            SocketRead(reframes,sock_fd);
        }

        TPDOmap[1] = 0x01;
        TPDOmap[3] = 0x02;
        TPDOmap[4] = 0x20;
        TPDOmap[6] = 0x6C;
        TPDOmap[7] = 0x60;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,TPDOmap,sizeof(TPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置映射数目
        TPDOmapnum[1] = 0x01;
        TPDOmapnum[4] = 0x02;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,TPDOmapnum,sizeof(TPDOmapnum));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //激活TPDO2
        openTPOD[1] = 0x01;
        openTPOD[4] = 0x80+joint.getid();
        openTPOD[5] = 0x02;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,openTPOD,sizeof(openTPOD));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
    /***********************************************************************************/



        //关闭TPOD1，设置电机对应?
        closeRPDO[1] = 0x00;
        closeRPDO[4] = 0x00+joint.getid();
        closeRPDO[5] = 0x02;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,closeRPDO,sizeof(closeRPDO));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);

        //设置模式
        setRPDOtype[1] = 0x00;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,setRPDOtype,sizeof(setRPDOtype));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);


        RPDOmapclear[1] = 0x00;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,RPDOmapclear,sizeof(RPDOmapclear));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设定映射
        RPDOmap[1] = 0x00;
        RPDOmap[3] = 0x01;
        RPDOmap[4] = 0x10;
        RPDOmap[6] = 0x40;
        RPDOmap[7] = 0x60;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,RPDOmap,sizeof(RPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);

        RPDOmap[1] = 0x00;
        RPDOmap[3] = 0x02;
        RPDOmap[4] = 0x10;
        RPDOmap[6] = 0x71;
        RPDOmap[7] = 0x60;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,RPDOmap,sizeof(RPDOmap));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //设置映射数目
        RPDOmapnum[1] = 0x00;
        RPDOmapnum[4] = 0x02;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,RPDOmapnum,sizeof(RPDOmapnum));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
        //激活RPDO1
        openRPOD[1] = 0x00;
        openRPOD[4] = 0x00+joint.getid();
        openRPOD[5] = 0x02;
        memset(frames.data,0,sizeof(frames.data));
        memset(reframes.data,0,sizeof(reframes.data));
        memcpy(frames.data,openRPOD,sizeof(openRPOD));
        SocketWrite(frames,sock_fd);
        SocketRead(reframes,sock_fd);
    }



} // namespace param
