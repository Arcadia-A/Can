
#ifndef __CAN_BUS_HPP_
#define __CAN_BUS_HPP_

#include <ros/ros.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <mutex>
#include <unistd.h>


#include <iostream>
#include <vector>
#include <thread>

#include <can/controlcan.h>

#define DEVICE_ID 0x602


//can 0 是右    can1是左


/**
 * @brief CAN总线：初始化插入的CAN设备，根据电机的数量创建对应的接收线程，并对电机发送指定的数据
 * @param [in]  leftValue   左轮设定速度
 * @param [in]  rightValue  右轮设定速度
 * @param [out] Can_RPM_L   左轮速度 串口
 * @param [out] Can_RPM_R   右轮速度 串口
 * @param [out] vel_left    左轮速度 m/s
 * @param [out] vel_right   右轮速度 m/s
 */
class Can_Bus
{
    public:

        Can_Bus():num(0)
        {
            std::cout << "START CAN !" << std::endl;
            
            //CAN初始化
            Usbcan_Open();
            Can_init();

            //电机初始化
            em_init();

            sleep(1);
            //创建电机数据接收线程
            for(int i = 0; i < num ; i++)
            {
                vt.push_back(std::thread(&Can_Bus::receive_func,this , i));
            }
            
            for(int i = 0; i < num ; i++)
            {
                vt[i].join();
            }

        }

        ~Can_Bus()
        {
            
            std::cout << "START CAN CLOSE!" << std::endl;
            
            //停止电机
            Set_Velocity(0,0);
            Motor_Disable();
            
            //清除数据
            vector_clear();

            //关闭CAN
            for(int i = 0; i < num ; i++)
            {
                usleep(100000);
                VCI_ResetCAN(VCI_USBCAN2, i, 0); //复位CAN1通道。
                usleep(100000);                  //延时100ms。
                VCI_ResetCAN(VCI_USBCAN2, i, 1); //复位CAN2通道。
                usleep(100000);                  //延时100ms。
                VCI_CloseDevice(VCI_USBCAN2, i); //关闭设备。
            }

            std::cout << "CAN CLOSE DONE!" << std::endl;
        }

        int num;//插入CAN设备数

        float temperature;//温度
        float electric_current;//电流
        float voltage;//电压
        
        int real_position_L;//左轮位置
        int real_position_R;//右轮位置


        std::vector<int> Can_RPM_L;
        std::vector<int> Can_RPM_R;
        std::vector<double> Vel_L;
        std::vector<double> Vel_R;
        
        std::vector<std::thread> vt;

        

        /**
         * 获取速度指令
         */
        void Get_Velocity()
        {
            VCI_CAN_OBJ velocity[1];
            velocity[0].ID = DEVICE_ID;    velocity[0].SendType = 0;    velocity[0].RemoteFlag = 0;    velocity[0].ExternFlag = 0;
            velocity[0].DataLen = 8;

            //读取电机当前速度
            velocity[0].Data[0] = 0x00;    velocity[0].Data[1] = 0x00;    velocity[0].Data[2] = 0x00;    velocity[0].Data[3] = 0x00;
            velocity[0].Data[4] = 0x00;    velocity[0].Data[5] = 0x00;    velocity[0].Data[6] = 0x00;    velocity[0].Data[7] = 0x00;

            Send_Msg(velocity, CAN1, 1);    Send_Msg(velocity, CAN2, 1);
        }

        //获取线圈温度
        void Get_temperature_1()
        {

        }

        //获取模块温度
        void Get_temperature_2()
        {

        }

        //获取电流
        void Get_electric_current()
        {

        }

        //获取电压
        void Get_voltage()
        {

        }

        //获取扭矩
        void Get_Torque()
        {

        }
        
        /**
         * 设置电机速度
         * @param [in] leftValue  电机左轮速度（串口）
         * @param [in] rightValue 电机右轮速度（串口）  
         */
        void Set_Velocity(double leftValue,double rightValue)
        {
            int S_Left; int S_Right;


            S_Left = -leftValue;
            S_Right = rightValue;

            //转速限制
            S_Left = Rotation_speed_Limit(S_Left);
            S_Right = Rotation_speed_Limit(S_Right);

            VCI_CAN_OBJ speed_R[1];
            VCI_CAN_OBJ speed_L[1];

            speed_R[0].ID = DEVICE_ID;    speed_R[0].SendType = 0;    speed_R[0].RemoteFlag = 0;    speed_R[0].ExternFlag = 0;
            speed_R[0].DataLen = 8;

            speed_L[0].ID = DEVICE_ID;    speed_L[0].SendType = 0;    speed_L[0].RemoteFlag = 0;    speed_L[0].ExternFlag = 0;
            speed_L[0].DataLen = 8;

            //设置右轮目标速度
            speed_R[0].Data[0] = 0x23;    speed_R[0].Data[1] = 0xFF;    speed_R[0].Data[2] = 0x60;    speed_R[0].Data[3] = 0x00;

            speed_R[0].Data[4] = S_Right & 0x000000FF;
            speed_R[0].Data[5] = (S_Right >> 8) & 0x000000FF;
            speed_R[0].Data[6] = (S_Right >> 16) & 0x000000FF;
            speed_R[0].Data[7] = (S_Right >> 24) & 0x000000FF;

            //设置左轮目标速度
            speed_L[0].Data[0] = 0x23;    speed_L[0].Data[1] = 0xFF;    speed_L[0].Data[2] = 0x60;    speed_L[0].Data[3] = 0x00;

            speed_L[0].Data[4] = S_Left & 0x000000FF;
            speed_L[0].Data[5] = (S_Left >> 8) & 0x000000FF;
            speed_L[0].Data[6] = (S_Left >> 16) & 0x000000FF;
            speed_L[0].Data[7] = (S_Left >> 24) & 0x000000FF;

            Send_Msg(speed_R, CAN1, 1);    Send_Msg(speed_L, CAN2, 1);
        }




        
        
    private:
    
        std::mutex m;//发送数据锁


        VCI_BOARD_INFO pInfo[4]; //用来获取设备信息
        /**
         * 打开USB CAN设备
         * @param [out] num CAN设备的插入数量 
         */
        void Usbcan_Open()
        {
            //printf(">>this is hello !\r\n");  //指示程序已运行
            num = VCI_FindUsbDevice2(pInfo); //返回计算机中已插入的USB-CAN适配器的数量。

            if (num == 0)
            {
                ROS_ERROR_STREAM("Unable to open can ");
                std::exit(-1);
            }
            
            printf(">>USBCAN DEVICE NUM:");
            printf("%d", num);
            printf(" PCS");
            printf("\n");

            for (int i = 0; i < num; i++)
            {
                printf("Device:");
                printf("%d", i);
                printf("\n");
                printf(">>velocity VCI_ReadBoardInfo success!\n");

                printf(">>Serial_Num:%c", pInfo[i].str_Serial_Num[0]);   printf("%c", pInfo[i].str_Serial_Num[1]);        
                printf("%c", pInfo[i].str_Serial_Num[2]);        printf("%c", pInfo[i].str_Serial_Num[3]);        
                printf("%c", pInfo[i].str_Serial_Num[4]);        printf("%c", pInfo[i].str_Serial_Num[5]);        
                printf("%c", pInfo[i].str_Serial_Num[6]);        printf("%c", pInfo[i].str_Serial_Num[7]);

                printf("%c", pInfo[i].str_Serial_Num[8]);         printf("%c", pInfo[i].str_Serial_Num[9]);
                printf("%c", pInfo[i].str_Serial_Num[10]);        printf("%c", pInfo[i].str_Serial_Num[11]);
                printf("%c", pInfo[i].str_Serial_Num[12]);        printf("%c", pInfo[i].str_Serial_Num[13]);
                printf("%c", pInfo[i].str_Serial_Num[14]);        printf("%c", pInfo[i].str_Serial_Num[15]);

                printf("%c", pInfo[i].str_Serial_Num[16]);        printf("%c", pInfo[i].str_Serial_Num[17]);
                printf("%c", pInfo[i].str_Serial_Num[18]);        printf("%c", pInfo[i].str_Serial_Num[19]);
                printf("\n");

                printf(">>hw_Type:%c", pInfo[i].str_hw_Type[0]);        printf("%c", pInfo[i].str_hw_Type[1]);
                printf("%c", pInfo[i].str_hw_Type[2]);        printf("%c", pInfo[i].str_hw_Type[3]);
                printf("%c", pInfo[i].str_hw_Type[4]);        printf("%c", pInfo[i].str_hw_Type[5]);
                printf("%c", pInfo[i].str_hw_Type[6]);        printf("%c", pInfo[i].str_hw_Type[7]);

                printf("%c", pInfo[i].str_hw_Type[8]);        printf("%c", pInfo[i].str_hw_Type[9]);
                printf("\n");

                printf(">>Firmware Version:V");
                printf("%x", (pInfo[i].fw_Version & 0xF00) >> 8);
                printf(".");
                printf("%x", (pInfo[i].fw_Version & 0xF0) >> 4);
                printf("%x", pInfo[i].fw_Version & 0xF);
                printf("\n");
            }
            printf(">>\n");
            printf(">>\n");
            printf(">>\n");

            //打开设备 & 初始化数据位
            for(int i = 0; i < num ; i++)
            {
                if (VCI_OpenDevice(VCI_USBCAN2, i, 0) == 1) 
                {
                    std::cout << ">>open deivce " << i << "success!" << std::endl;
                }
                else
                {
                    std::cout << ">>open deivce " << i << "error!" << std::endl;
                    std::exit(-1);
                }

                vector_init();
            }
        }



        /**
         * 初始化CAN 
         */
        void Can_init()
        {
            //初始化参数，严格参数二次开发函数库说明书。
            VCI_INIT_CONFIG config; //结构体定义了初始化CAN的配置
            config.AccCode = 0;
            config.AccMask = 0xFFFFFFFF;
            config.Filter = 1;     //接收所有帧
            config.Timing0 = 0x00; /*波特率500 Kbps  0x00  0x1C*/
            config.Timing1 = 0x1C;
            config.Mode = 0; //正常模式

            for(int i = 0; i < num ; i++)
            {
                if (VCI_InitCAN(VCI_USBCAN2, i, 0, &config) != 1)
                {
                    std::cout << "CAN num" << i << " :Init CAN0 error "<< std::endl;
                    VCI_CloseDevice(VCI_USBCAN2, 0);
                }

                if (VCI_StartCAN(VCI_USBCAN2, i, 0) != 1)
                {
                    std::cout << "CAN num " << i << " :Start CAN0 error "<< std::endl;
                    VCI_CloseDevice(VCI_USBCAN2, 0);
                }

                if (VCI_InitCAN(VCI_USBCAN2, i, 1, &config) != 1)
                {
                    std::cout << "CAN num" << i << " :Init CAN1 error "<< std::endl;
                    VCI_CloseDevice(VCI_USBCAN2, 0);
                }

                if (VCI_StartCAN(VCI_USBCAN2, i, 1) != 1)
                {
                    std::cout << "CAN num " << i << " :Start CAN1 error "<< std::endl;
                    VCI_CloseDevice(VCI_USBCAN2, 0);
                }
            }

        }

        /**
         * 电机初始化指令
         *    
         */
        void em_init()
        {

            Enable();


            sleep(2);
            Velocity_Mode();

            std::cout << "can start!"<< std::endl;
        }

        //电机使能
        void Enable()
        {

        }



        void Velocity_Mode()
        {

        }

        void Motor_Disable()
        {

        } 


        void vector_init()
        {
            Can_RPM_L.push_back(0);
            Can_RPM_R.push_back(0);
            Vel_L.push_back(0.0);
            Vel_R.push_back(0.0);
        }

        void vector_clear()
        {
            //清除数据
            Can_RPM_L.clear();
            Can_RPM_R.clear();
            Vel_L.clear();
            Vel_R.clear();
        }

        
        /**
         * 发送CAN数据 
         */
        void Send_Msg(PVCI_CAN_OBJ pSend, DWORD DeviceType, uint n)
        {
            std::lock_guard<std::mutex> lockGuard(m);
            for(int i = 0; i < num ; i++)
            {
                VCI_Transmit(VCI_USBCAN2, i, DeviceType, pSend, n);
            }
        }

        double maxRotation_speed = 12732; // 转速限制 12732 对应1.0m/s

        double Rotation_speed_Limit(double value)
        {
            if ( abs(value) > maxRotation_speed ) //最大转速为 转
            {
                if( value > 0 )
                {
                    value = maxRotation_speed;
                }
                else
                {
                    value = - maxRotation_speed;
                }
            }
            return value;
        }

        


        double vel_rectify = 0.812;//速度校正值

        /**
         * @brief 获取电机转速，针对负转速进行相应的转换
         * 
         * @param data 串口转速数据
         * \return RPM 实际的转速
         */
        int data_change(int data)
        {
            if (data > 32768)
            {
                data = data - 65536;
                return data;
            }
            else
            {
                return data;
            }
        }

        std::mutex rv[5];//接收数据锁
        /**
         * @brief 速度读取指令
         * 
         * @param id 通道编号
         * @param id_num CAN设备编号
         * @param read 读取的数据
         */
        void read_vel(int id,int id_num, VCI_CAN_OBJ read)
        {
            int Read_RPM;//读取的转速数据
            Read_RPM = read.Data[4] | read.Data[5] << 8;
            if (id == 0)
            {
                std::lock_guard<std::mutex> lockGuard(rv[id_num]);
                Can_RPM_R[id_num] = data_change(Read_RPM);
                Vel_R[id_num] = Can_RPM_R[id_num] * M_PI * 0.168 * 0.1 / 60 * vel_rectify;
                //std::cout << vel_right << std::endl;
            }
            if (id == 1)
            {
                std::lock_guard<std::mutex> lockGuard(rv[id_num]);
                Can_RPM_L[id_num] = -data_change(Read_RPM);
                Vel_L[id_num] = Can_RPM_L[id_num] * M_PI * 0.168 * 0.1 / 60 * vel_rectify;
                //std::cout << Can_RPM_L << std::endl;
            }
        }

        /**
         * 接收线程函数 读取单个CAN设备
         * @param [in]  re_num      CAN设备编号
         * @param [out] Can_RPM_L   左轮速度 串口
         * @param [out] Can_RPM_R   右轮速度 串口
         * @param [out] vel_left    左轮速度 实际
         * @param [out] vel_right   右轮速度 实际
         */
        void receive_func(int re_num) 
        {

            int reclen = 0;
            VCI_CAN_OBJ rec[3000]; //接收缓存，设为3000为佳
            int j;
            
            int ind = 0;//CAN设备通道
            int Read_RPM;//读取的转速数据

            ros::Rate loop_rate(100);
            while (ros::ok())
            {

                Get_Velocity();

                if ((reclen = VCI_Receive(VCI_USBCAN2, re_num, ind, rec, 3000, 100)) > 0) //调用接收函数，如果有数据，进行数据处理显示。
                {

                    for (j = 0; j < reclen; j++)
                    {
                        //转速读取
                        if (rec[j].Data[0] == 0x00 && rec[j].Data[1] == 0x00 && rec[j].Data[2] == 0x00 && rec[j].Data[3] == 0x00)
                        {
                            read_vel(ind,re_num,rec[j]);
                        }
                    }
                }
                ind = !ind; //变换通道号，以便下次读取另一通道，交替读取。
                loop_rate.sleep();

            }

            
            std::cout << "run num " << re_num << "thread exit" << std::endl;

        }

        
        
        
    
};

#endif
