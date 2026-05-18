while (ros::ok)
{
    ROS_INFO("QQQQ=%f,%f,%f", joint1.get_current_angle(), joint2.get_current_angle(), joint3.get_current_angle());
    count_time++;

    can_msgs::Frame frames;
    frames.id = 0x80;
    frames.dlc = 0;
    socketcan_send_pub.publish(frames);

    send_message = joint1.set_angle(expect_q(0, 0));
    socketcan_send_pub.publish(send_message);

    send_message = joint2.set_angle(expect_q(1, 0));
    socketcan_send_pub.publish(send_message);

    send_message = joint3.set_angle_for_new_joint(expect_q(2, 0));
    socketcan_send_pub.publish(send_message);

    send_message = joint3.set_angle(expect_q(2, 0));
    socketcan_send_pub.publish(send_message);

    ros::spinOnce();
    loop_rate.sleep();
}

//以下是ARC控制部分
else if (control_type == "ARC")
{
    {
        //前进命令，加速阶段A
        double t_KB = 0.002;
        if (Velocity_KB < 0.25 && KB_W > 0)
        {
            q_TeleopeKB = q_TeleopeKB + Velocity_KB * t_KB;
            Aceleration_KB = 0.5;
            Velocity_KB = Velocity_KB + Aceleration_KB * t_KB;
        }
        //前进命令，匀速阶段B
        else if (Velocity_KB >= 0.25 && KB_W > 0)
        {
            q_TeleopeKB = q_TeleopeKB + Velocity_KB * t_KB;
            Aceleration_KB = 0.0;
            Velocity_KB = 0.25;
        }
        //前进无命令，匀速阶段C
        else if (Velocity_KB > 0.01 && KB_W == 0)
        {
            q_TeleopeKB = q_TeleopeKB + Velocity_KB * t_KB;
            Aceleration_KB = -0.5;
            Velocity_KB = Velocity_KB + Aceleration_KB * t_KB;
        }

        //后退命令，加速阶段
        else if (Velocity_KB > -0.25 && KB_W < 0)
        {
            q_TeleopeKB = q_TeleopeKB + Velocity_KB * t_KB;
            Aceleration_KB = -0.5;
            Velocity_KB = Velocity_KB + Aceleration_KB * t_KB;
        }
        //后退命令，匀速阶段
        else if (Velocity_KB <= -0.25 && KB_W < 0)
        {
            q_TeleopeKB = q_TeleopeKB + Velocity_KB * t_KB;
            Aceleration_KB = 0.0;
            Velocity_KB = -0.25;
        }
        //前进无命令，匀速阶段
        else if (Velocity_KB < -0.01 && KB_W == 0)
        {
            q_TeleopeKB = q_TeleopeKB + Velocity_KB * t_KB;
            Aceleration_KB = 0.5;
            Velocity_KB = Velocity_KB + Aceleration_KB * t_KB;
        }
        else if (Velocity_KB <= 0.01 && Velocity_KB >= -0.01 && KB_W == 0)
        {
            Aceleration_KB = 0.0;
            Velocity_KB = 0.0;
            q_TeleopeKB = q_TeleopeKB;
        }
        else
        {
            Aceleration_KB = 0.0;
            Velocity_KB = 0.0;
            q_TeleopeKB = q_TeleopeKB;
        }

        //ROS_INFO("QVA  = %lf , %lf , %lf", q_TeleopeKB, Velocity_KB, Aceleration_KB);
    }

    //arc键盘控制 解算
    //expect_q(0.0)=q_TeleopeKB;
    //expect_qd(0.0)=Velocity_KB;
    //expect_qdd(0.0)=Aceleration_KB;

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.position.resize(8);
    joint_state.velocity.resize(8);
    joint_state.effort.resize(8);
    /*
            joint_state.position[0] = joint1.get_current_angle()-expect_q1;
            joint_state.position[1] = joint1.get_current_angle();
            joint_state.position[2] = expect_q1;//joint2.get_current_angle();
            joint_state.position[3] = thp1;//joint3.get_current_angle();
            joint_state.position[4] = thp2;//expect_q1;
            joint_state.position[5] = thp3;
            joint_state.position[6] = thp4;
            joint_state.velocity[0] = u;//expect_q3;
            joint_state.velocity[1] = us1;//expect_q3;
            joint_state.velocity[2] = ua;//expect_q3;
            joint_state.velocity[3] = thp1*(expect_ddq1) ;//expect_q3;
            joint_state.velocity[4] = thp2 * expect_dq1;//joint1.get_current_velocity();
            joint_state.velocity[5]  = atan2(900*expect_dq1,1)*2/(M_PI)*thp3;
            joint_state.velocity[6]  =thp4;
              joint_state.velocity[7]  =joint1.get_current_velocity();
          
          */
    joint_state.position[0] = expect_q(0, 0);
    joint_state.position[1] = q(0, 0);
    joint_state.position[2] = q(1, 0);
    joint_state.position[3] = q(2, 0);
    joint_state.position[4] = expect_qd(0, 0);
    joint_state.position[5] = qd(0, 0);
    joint_state.position[6] = qd(1, 0);
    joint_state.position[7] = qd(2, 0);

    joint_state.velocity[0] = us1(0, 0);
    joint_state.velocity[1] = us1(1, 0);
    joint_state.velocity[2] = us1(2, 0);
    joint_state.velocity[3] = ua(0, 0); //
    joint_state.velocity[4] = ua(1, 0); //
    joint_state.velocity[5] = ua(2, 0); //
    joint_state.velocity[6] = theta(1, 0);
    joint_state.velocity[7] = theta(2, 0);

    joint_state.effort[0] = theta(3, 0);
    joint_state.effort[1] = theta(4, 0);
    joint_state.effort[2] = theta(5, 0);
    joint_state.effort[3] = theta(6, 0);
    joint_state.effort[4] = theta(7, 0);
    joint_state.effort[5] = theta(8, 0);
    joint_state.effort[6] = theta(9, 0);
    joint_state.effort[7] = theta(10, 0);

    //joint_state.effort[0] = theta(0,0);
    //joint_state.effort[1] = theta(1,0);
    //joint_state.effort[2] = theta(8,0);
    //joint_state.effort[3] = theta(9,0);
    //joint_state.effort[4] = theta(10,0);
    //joint_state.effort[5] = tol(0,0);
    //joint_state.effort[6] = tol(1,0);
    //joint_state.effort[7] = tol(2,0);

    //joint_state.position[4] = theta(0,0);//    一关节转动惯量
    //joint_state.position[5] = theta(1,0);///    m1
    //joint_state.position[6] = theta(2,0); //   m2
    //joint_state.position[7] = theta(3,0);;//    关节一粘滞摩擦系数

    //joint_state.velocity[0] = theta(4,0);;//    关节二粘滞摩擦系数
    //joint_state.velocity[1] = theta(5,0);//    关节三粘滞摩擦系数
    //joint_state.velocity[2] = theta(6,0); //    关节一静摩擦
    //joint_state.velocity[3] = theta(7,0);///    关节二静摩擦
    //joint_state.velocity[4] = theta(8,0);///    关节二静摩擦

    /* joint_state.position[6] = us1(0,0); //   m2
            joint_state.position[7] = us1(1,0);;//    关节一粘滞摩擦系数
            joint_state.velocity[0] = us1(2,0);;//    关节二粘滞摩擦系数
            joint_state.velocity[1] = ua(0,0);//    关节三粘滞摩擦系数
            joint_state.velocity[2] = ua(1,0); //    关节一静摩擦
            joint_state.velocity[3] = ua(2,0);///    关节二静摩擦
            joint_state.velocity[4] = theta(8,0);///    关节二静摩擦
*/

    //joint_state.velocity[5] = theta(9,0);//   关节一静摩擦
    //joint_state.velocity[6] = theta(10,0);//   关节一静摩擦
    //joint_state.velocity[7] = joint2.get_current_velocity()-expect_qd(1,0);
    /*joint_state.velocity[5] =  theta(6,0);//   干扰
            
            joint_state.velocity[6] =  theta(7,0);//  干扰

            joint_state.velocity[7] = us1_1;
            
            joint_state.velocity[8] = ua_1;
            */
    //joint_state.effort[0] = us1(1,0);
    //joint_state.effort[1] = ua(1,0);
    //joint_state.effort[2] = u(0,0);
    //joint_state.effort[3] =  u(1,0);
    //joint_state.effort[4] = u(2,0);
    //joint_state.effort[5] = -fai_dot(1,6)*theta(6,0);
    //joint_state.effort[6] = -fai_dot(1,9)*theta(9,0);

    //////////////////////////////////////////////////////////
    //pub
    pub.publish(joint_state);

    //  tol3 = arc_control(expect_q1,expect_dq1,expect_ddq1,joint1.get_current_angle(),joint1.get_current_velocity(),0.02,0.95,0,0);
    // tol = arc(expect_q,expect_qd,expect_qdd,q,qd,0.002);
    tol = arc(expect_q, expect_qd, expect_qdd, q, qd, 0.002);
    if (tol(0, 0) > 30)
    {
        tol(0, 0) = 30;
    }
    else if (tol(0, 0) < -30)
    {
        tol(0, 0) = -30;
    }

    if (tol(1, 0) > 100)
    {
        tol(1, 0) = 100;
    }
    else if (tol(1, 0) < -100)
    {
        tol(1, 0) = -100;
    }

    if (tol(2, 0) > 35)
    {
        tol(2, 0) = 35;
    }
    else if (tol(2, 0) < -35)
    {
        tol(2, 0) = -35;
    }

    //ROS_INFO("tol1 =%lf tol2 = %lf tol3=%lf ",tol(0,0),tol(1,0),tol(2,0));
    can_msgs::Frame frames;
    frames.id = 0x80;
    frames.dlc = 0;
    socketcan_send_pub.publish(frames);

    send_message = joint1.MomentOutput(tol(0, 0));
    socketcan_send_pub.publish(send_message);

    send_message = joint2.MomentOutput(tol(1, 0));
    socketcan_send_pub.publish(send_message);

    send_message = joint3.MomentOutput80(tol(2, 0));
    socketcan_send_pub.publish(send_message);

    if (count_time >= 500)
    {
        if (abs(joint5_catch_TeleOpe - 1.0) < 0.1)
        {
            send_message = joint5.set_angle(5.90);
            socketcan_send_pub.publish(send_message);
        }
        else
        {
            send_message = joint5.set_angle(0.001);
            socketcan_send_pub.publish(send_message);
        }
        //有问题这里
        //ROS_INFO("joint5-current-angle- = %f ", joint5.get_current_angle());
        count_time = 0;
    }
}

else if (control_type == "CAMERA")
{
    //以下pub chatter形式发送,camera可监听,角度角速度
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.position.resize(8);
    joint_state.velocity.resize(8);
    joint_state.effort.resize(8);

    joint_state.position[0] = joint1.get_current_angle();
    joint_state.position[1] = joint2.get_current_angle();
    joint_state.position[2] = joint3.get_current_angle();
    //joint_state.position[3] ;

    joint_state.velocity[0] = joint1.get_current_velocity();
    joint_state.velocity[1] = joint2.get_current_velocity();
    joint_state.velocity[2] = joint3.get_current_velocity();

    //joint_state.effort[0]
    //joint_state.effort[1]
    //joint_state.effort[2]

    pub.publish(joint_state);

    //停止指令,归位
    if (stop_flag.data == true)
    {
        //   send_message = joint1.ret_to_init();
        // socketcan_send_pub.publish(send_message);

        // send_message = joint2.ret_to_init();
        //socketcan_send_pub.publish(send_message);
        can_msgs::Frame frames;
        frames.id = 0x80;
        frames.dlc = 0;
        socketcan_send_pub.publish(frames);
        send_message = joint1.ret_to_init();
        socketcan_send_pub.publish(send_message);
        send_message = joint2.ret_to_init();
        socketcan_send_pub.publish(send_message);
        send_message = joint3.ret_to_init();

        socketcan_send_pub.publish(send_message);

        send_message = joint5.set_angle(0.00);
        // send_message.data[0] = 0x06;
        socketcan_send_pub.publish(send_message);
    }
    else
    {

        //arc没有归位
        /*
                 if(joint5_state == true&&joint5_open==true)
                {
                    //此处已完成抓取，二三关节回到初始位置，是垂直于地面 勾形
                    socketcan_send_pub.publish(frames);
                    send_message = joint1.set_angle(0);
                    socketcan_send_pub.publish(send_message);             
                    send_message = joint2.set_angle(-1.57);
                    socketcan_send_pub.publish(send_message);             
                    send_message = joint3.set_angle(-1.57);
                    socketcan_send_pub.publish(send_message);  
                    send_message = joint5.set_angle(20);
                    send_message.data[0] = 0x06;
                    socketcan_send_pub.publish(send_message);
                    ROS_INFO("stop");       
                }
                */

        //joint5_state  true表示手抓闭合(物体已抓到),是通过赌转电流来判断置位的
        //joint5_open  true是通过joint_state.position[4]标志位判断,置1时候为true,表示可以抓取(或者已经抓取了)

        //归位
        if (joint5_state == true && joint5_homing == true)
        {
            can_msgs::Frame frames;
            frames.id = 0x80;
            frames.dlc = 0;

            //此处需要发送给视觉client某个标志位,令上位机进行规划+归位 0 -1.57 -1.57
            //只需要上发一个标志位给冲哥camera端,那边作规划
            socketcan_send_pub.publish(frames);
            send_message = joint5.set_angle(0.01);
            //send_message.data[0] = 0x06;
            socketcan_send_pub.publish(send_message);
            ROS_INFO("return_to_init_position ");
        }

        //抓取
        if (joint5_open == true && joint5_state == false)
        {
            can_msgs::Frame frames;
            frames.id = 0x80;
            frames.dlc = 0;
            socketcan_send_pub.publish(frames);

            send_message = joint5.set_angle(20);
            socketcan_send_pub.publish(send_message);
            ROS_INFO("catch_operarting");
            ROS_INFO("catch_current %lf", joint5.get_ActualCurrent());
            //判断抓取电流，若大于某个阈值，说明已抓取到，发生堵转
            //5000需要根改 一般给2000
            if (abs(joint5.get_ActualCurrent()) >= 2200)
            {
                ////////yaogai
                joint5_state = true;
            }
        }
        //arc控制
        else
        {
            can_msgs::Frame frames;
            frames.id = 0x80;
            frames.dlc = 0;
            socketcan_send_pub.publish(frames);

            tol = arc(expect_q, expect_qd, expect_qdd, q, qd, 0.002);
            //ROS_INFO("tol3 = %lf",tol3);
            //can_msgs::Frame frames;
            //frames.id = 0x80;
            //frames.dlc = 0;
            //socketcan_send_pub.publish(frames);

            send_message = joint1.MomentOutput(tol(0, 0));
            socketcan_send_pub.publish(send_message);

            send_message = joint2.MomentOutput(tol(1, 0));
            socketcan_send_pub.publish(send_message);

            send_message = joint3.MomentOutput(tol(2, 0));
            socketcan_send_pub.publish(send_message);
        }
    }
}
