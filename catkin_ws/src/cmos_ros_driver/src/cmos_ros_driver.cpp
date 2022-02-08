#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;
#define M_PI 3.1415926
serial::Serial sp; //创建一个serial对象
int gBuff_Length = 0, nLoopCnt = 0;
uint8_t gBuff[1024];
//=============================================================================================
class _Scan{
  public:
  bool flag;
  int a, b;
  int cnt;
  double angle[1024];
  double range[1024];
};
//=============================================================================================
int connectSerialPort(){
  //设置要打开的串口名称
  sp.setPort("/dev/ttyUSB0");	//Used "dmesg | grep tfixed << setprecision(2)ty" to see port
  //设置串口通信的波特率
  sp.setBaudrate(115200);	//Must be same with master
  //串口设置timeout
  serial::Timeout to = serial::Timeout::simpleTimeout(100); //创建timeout
  sp.setTimeout(to);
 
  try{
    //打开串口
    sp.open();
  }
  catch(serial::IOException& e){
    ROS_ERROR_STREAM("Unable to open port.");
    return -1;
  }
    
  //判断串口是否打开成功
  if(sp.isOpen()){
    ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
  }
  else{
    return -1;
  }
}
//=============================================================================================
void ReadData(_Scan &Scan){
  if(!sp.isOpen()){
    //cout << ">>> Serial Port is not open" << endl;     
    return;
  }

  try{
    //获取缓冲区内的字节数
    size_t n = sp.available();
    if(n!=0)
    {
      uint8_t buffer[1024];
      //读出数据
      n = sp.read(buffer, n);

      int _i = 0, _j = 0;
      int nStart = 0, nEnd = 0;
      memcpy(gBuff + gBuff_Length, buffer, n);
      gBuff_Length += n;

      for(int i=0; i<gBuff_Length-1; i++) //解讀資料
      {
        _i = i;
        nEnd = i;
        //cout << ">>> gBuff[" << dec << i << "] = " << setw(2) << setfill('0') << hex << (gBuff[i] & 0xff) << endl;
        if(nStart>1 && gBuff[i]==0xaa && gBuff[i+1]==0x55) //找到第二組數據包頭後, 開始處理第一組的資料
        {
          if(i < nStart || gBuff_Length<12)
          {
            //cout << ">>> i = " << dec << i << ", gBuff_Length" << dec << gBuff_Length << " ...break" << endl;
            break;
          }
          uint8_t   CT = gBuff[nStart + 0];
          uint8_t  LSN = gBuff[nStart + 1];
          uint16_t FSA = gBuff[nStart + 2] + (gBuff[nStart + 3] << 8);
          uint16_t LSA = gBuff[nStart + 4] + (gBuff[nStart + 5] << 8);
          uint16_t  CS = gBuff[nStart + 6] + (gBuff[nStart + 7] << 8);
          //cout << "CT = "    << setw(2) << setfill('0') << hex << ( CT & 0xff);
          //cout << ", LSN = " << setw(2) << setfill('0') << hex << (LSN & 0xff);
          //cout << ", FSA = " << setw(4) << setfill('0') << hex << (FSA & 0xffff);
          //cout << ", LSA = " << setw(4) << setfill('0') << hex << (LSA & 0xffff);
          //cout << ", CS = "  << setw(4) << setfill('0') << hex << ( CS & 0xffff);

          ////---------------------------------------------------------------------------------
          //if((CT==0x01 || CT==0x79) && LSN==0x01 && Scan.cnt>1) //一圈的起始點
          if((CT&0x01)==0x01 && LSN==0x01 && Scan.cnt>1) //一圈的起始點
          {
            //cout << endl;
            Scan.flag = true;
            //Scan.cnt = 0;
            //memset(Scan.angle, 0, sizeof(Scan.angle) / sizeof(double));
            //memset(Scan.range, 0, sizeof(Scan.range) / sizeof(double));
            nLoopCnt++;
            //cout << ">>> nLoopCnt1 = " << dec << nLoopCnt << endl;

/*
            //------------------------- 重新整理 Scan 的資料排序 -------------------------
            int nMax = 0;
            double dMax = 0.0;
            for(int i=0; i<Scan.cnt; i++){
              if(Scan.angle[i] > dMax){
                nMax = i;
                dMax = Scan.angle[i];
              }
              //cout << "Scan.angle[" << dec << i << "] = " << fixed << setprecision(5) << Scan.angle[i] << endl;
              cout << dec << i << ", " << fixed << Scan.angle[i] << ", " << fixed << Scan.range[i] << ", " << dec << 0 << endl;
            }
            //cout << ">>> nMax = " << dec << nMax << ", dMax = " << fixed << setprecision(5) << dMax << endl;
            cout << "------------------------------- " << dec << Scan.cnt << ", " << 0  << ", " << 0 << endl;

            _Scan Scan2;
            Scan2.flag = false;
            Scan2.cnt = 0;
            memset(Scan2.angle, 0, sizeof(Scan2.angle));
            memset(Scan2.range, 0, sizeof(Scan2.range));

            int nIndex = 0;
            if(nMax < Scan.cnt / 2.0) nIndex = Scan.cnt / 2.0 - nMax;
            else                    nIndex = nMax - Scan.cnt / 2.0;
            int nW = 18.0 / 360.0 * Scan.cnt;
            //cout << ">>> nW = " << dec << nW << endl;
            nIndex += nW;
            memcpy(Scan2.angle + nIndex, Scan.angle, sizeof(Scan.angle) - nIndex * sizeof(double));
            memcpy(Scan2.angle, Scan.angle + (Scan.cnt - nIndex), nIndex * sizeof(double));

            memset(Scan.angle, 0, sizeof(Scan.angle));
            //memset(Scan.range, 0, sizeof(Scan.range));
            memcpy(Scan.angle , Scan2.angle, sizeof(Scan.angle));
            //memcpy(Scan.range , Scan2.range, sizeof(Scan.range));
            //for(int i=0; i<Scan.cnt; i++){
            //  cout << "Scan.angle[" << dec << i << "] = " << fixed << Scan.angle[i] << endl;
            //}
            //cout << ">>> Scan -> Scan2 End" << endl;
*/

            break;
          }

          ////---------------------------------------------------------------------------------
          double dAngle_FSA = (double)(FSA >> 1) / 64.0;
          double dAngle_LSA = (double)(LSA >> 1) / 64.0;
          double dAngle_diff = dAngle_LSA - dAngle_FSA;
          if(dAngle_diff < 0) dAngle_diff += 360.0;
          for(int j=nStart+8; j<i; j+=2)
          {
            _j = j;
            uint16_t Si = gBuff[j] + (gBuff[j + 1] << 8);
            int nIndex = Scan.cnt;
            /*double dAngle = dAngle_FSA;
            if(LSN > 1){
              dAngle = dAngle_diff / (LSN - 1) * ((j - (nStart+8)) / 2) + dAngle_FSA;
              if(dAngle > 360) dAngle -= 360;
            }
            Scan.angle[nIndex] = (180 - dAngle) / 180 * 3.14159;*/
            //cout << "(" << dec << i << ", " << dec << j << ") ";

            double dDitsance = 0.0;
            dDitsance = (gBuff[j] + (gBuff[j + 1] << 8)) / 4.0;
            //cout << ", (" << fixed << setprecision(2) << dAngle << ") " << fixed << dDitsance;
            Scan.range[nIndex] = dDitsance / 1000.0;
            //cout << fixed << (dDitsance / 1000.0) << ", ";

            double dAngCorrect = 0.0;
            if(dDitsance > 0){
              dAngCorrect = atan(21.8 * (155.3 - dDitsance) / (155.3 * dDitsance));
              dAngCorrect = dAngCorrect * 180.0 / M_PI; //to_degrees(...)
            }
            //cout << fixed << dAngCorrect << ", ";

            double dAngle = dAngle_FSA;
            if(LSN > 1){
              dAngle = dAngle_diff / (LSN - 1) * ((j - (nStart+8)) / 2.0) + dAngle_FSA;
            }
            //cout << fixed << dAngle << ", ";
            //angle = math::from_degrees(angle);
            dAngle = (dAngle + dAngCorrect) * M_PI / 180.0;
            //cout << fixed << dAngle << ", ";

            //if (m_Inverted)
            dAngle = 2.0 * M_PI - dAngle;
            //cout << fixed << dAngle << ", ";

            //angle = math::normalize_angle(angle);
            dAngle = fmod(fmod(dAngle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
            if(dAngle > M_PI) dAngle -= 2.0 * M_PI;
            //cout << fixed << dAngle << endl;

            Scan.angle[nIndex] = dAngle;

            Scan.cnt++;
          }
          //cout << endl;
          //cout << "dAngle_FSA = " << fixed << dAngle_FSA << ", dAngle_LSA = " << fixed << dAngle_LSA << ", dAngle_diff = " << fixed << dAngle_diff << endl;
          //cout << "_i = " << dec << _i << ", _j = " << dec << _j << endl;
          //cout << "gBuff_Length = " << dec << gBuff_Length << ", n = " << dec << n << endl;
          //cout << "nStart = " << dec << nStart << ", nEnd = " << dec << nEnd << endl;
          //cout << "Scan.cnt = " << dec << Scan.cnt << endl;
        
          i = -1; //若設i=0, 下次會從i=1開始
          nStart = 0;
          uint8_t _buff[1024];
          gBuff_Length -= nEnd;
          //刪除已解讀的部份, 並將未處理的資料搬到buffer最前面
          memcpy(_buff, gBuff + nEnd, gBuff_Length);
          memset(gBuff, 0, sizeof(gBuff));
          memcpy(gBuff , _buff, gBuff_Length);
          //cout << "-------------------------------------------------------------------------------" << endl;

          ////---------------------------------------------------------------------------------
          if(CT==0x02 && LSN>=0x01) //一圈的結束點
          {
            Scan.flag = true;
            nLoopCnt++;
            //cout << ">>> nLoopCnt2 = " << dec << nLoopCnt << endl;
           break;
          }
        }
        else if(gBuff[i]==0xaa && gBuff[i+1]==0x55) //先找到第一組數據包頭
        {
          nStart = i + 2;
          //cout << ">>> nStart = " << dec << nStart << endl;
          //continue;
        }
      }
    }
    //cout << ">>> gBuff_Length = " << dec << gBuff_Length << endl;
    //for(int i=0; i<gBuff_Length; i++)
    //{
    //  cout << setw(2) << setfill('0') << hex << (gBuff[i] & 0xff) << " ";
    //}
    //cout << "\n===============================================================================" << endl;
  }
  catch(std::exception &e){
    //cout << ">>> exception: " << e.what() << endl;
  }
}
//=============================================================================================
int main(int argc, char **argv) {
  ros::init(argc, argv, "cmos_ros_driver");
  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 1);
  ros::NodeHandle nh_private("~");
  std::string frame_id;


  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

  //-------------------------------------------------------------------------------------------
  connectSerialPort();
  _Scan Scan;
  Scan.flag = false;
  Scan.cnt = 0;
  memset(Scan.angle, 0, sizeof(Scan.angle)); //sizeof(Scan.angle) = 8192, sizeof(double) = 8
  memset(Scan.range, 0, sizeof(Scan.range));
  //-------------------------------------------------------------------------------------------

  ros::Rate r(30);

  while (nh.ok()) {
    Scan.flag = false;
    //if(!sp.isOpen()) connectSerialPort();
    //cout << ">>> sp.isOpen():" << sp.isOpen() << endl;
    ReadData(Scan);
    //cout << ">>> After ReadData" << endl;

    if(Scan.flag){
      //cout << "Scan.cnt = " << dec << Scan.cnt << endl;
      //-------------------------------------------------------------------------------------------
      try{
      sensor_msgs::LaserScan scan_msg;
      sensor_msgs::PointCloud pc_msg;
      ros::Time start_scan_time;
      scan_msg.header.stamp = ros::Time::now();
      scan_msg.header.frame_id = frame_id;
      pc_msg.header = scan_msg.header;
      scan_msg.angle_min = -3.14159;
      scan_msg.angle_max = 3.14159;
      scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min)/(Scan.cnt-1);
      scan_msg.time_increment = 1/3/Scan.cnt;
      scan_msg.range_min = 0.1;
      scan_msg.range_max = 12;

      int size = Scan.cnt;
      /*if(size < 598 || size > 602){
        Scan.flag = false;
        Scan.cnt = 0;
        memset(Scan.angle, 0, sizeof(Scan.angle) / sizeof(double));
        memset(Scan.range, 0, sizeof(Scan.range) / sizeof(double));
        continue;
      }*/
      scan_msg.ranges.resize(size);
      scan_msg.intensities.resize(size);
      pc_msg.channels.resize(1);
      pc_msg.channels[0].name = "intensities";

      //for (int i = 0; i < size; i++) {
      //  int index = std::ceil((Scan.angle[i] - scan_msg.angle_min) / scan_msg.angle_increment);
      //  cout << "i = " << dec << i  << ", Scan.angle[" << dec << i << "] = " << fixed << setprecision(5) <<  Scan.angle[i] << ", scan_msg.angle_min = " << fixed << scan_msg.angle_min << ", scan_msg.angle_increment = " << fixed << scan_msg.angle_increment << ", index = " << dec << index << endl;
      //}
      int nCnt = 0, nCnt2 = 0;
      for (int i = 0; i < size; i++) {
        int index = std::ceil((Scan.angle[i] - scan_msg.angle_min) / scan_msg.angle_increment);

        //if(i==0)
        //  cout << dec << Scan.cnt << ", " << fixed << Scan.angle[i] << ", " << fixed << Scan.range[i] << ", " << dec << index << endl;
        //cout << dec << setw(3) << i << ", " << fixed << Scan.angle[i] << ", " << fixed << Scan.range[i] << ", " << dec << index << endl;

        if (index >= 0 && index < size) {
          if (Scan.range[i] >= scan_msg.range_min) {
            scan_msg.ranges[index] = Scan.range[i];//dRandom;
            //scan_msg.intensities[i] = 1000 + dRandom;
            //cout << "Scan.angle[" << dec << i << "], " << fixed << Scan.angle[i] << ", scan_msg.ranges[" << dec << index << "] , " << fixed << scan_msg.ranges[index] << ", Scan.range[" << dec << i << "] , " << Scan.range[i] << endl;
            nCnt++;
          }
        }

        if(Scan.range[i] >= scan_msg.range_min && Scan.range[i] <= scan_msg.range_max){
          double dAngle = Scan.angle[i];//3.14159 * 2 / 600;
          geometry_msgs::Point32 point;
          point.x = Scan.range[i] * cos(dAngle);
          point.y = Scan.range[i] * sin(dAngle);
          point.z = 0.0;
          pc_msg.points.push_back(point);
          //cout << dec << i << ", " << fixed << Scan.angle[i] << ", " << fixed << point.x << ", " << fixed << point.y << ", " << fixed << Scan.range[i] << endl;
          nCnt2++;
        }
      }
      //cout << "===============================================================================\n" << endl;
      //cout << ", " << nCnt  << ", " << nCnt2 << endl;
      //cout << "=============================== " << dec << Scan.cnt << ", " << nCnt  << ", " << nCnt2 << endl;


      scan_pub.publish(scan_msg);
      pc_pub.publish(pc_msg);
      }
      catch(std::exception &e){
        cout << "exception: " << e.what() << endl;
      }
      //-------------------------------------------------------------------------------------------
      Scan.flag = false;
      Scan.cnt = 0;
      memset(Scan.angle, 0, sizeof(Scan.angle) / sizeof(double));
      memset(Scan.range, 0, sizeof(Scan.range) / sizeof(double));
    }
    else {
      //ROS_ERROR("Failed to get Lidar Data");
    }

    r.sleep();
    ros::spinOnce();
  }

  //关闭串口
  sp.close();

  return 0;
}


