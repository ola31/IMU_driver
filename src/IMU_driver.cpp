#include "IMU_driver/IMU_driver.h"

IMU::IMU(string port, int id, bool is_ext_mode):CAN(port, id, is_ext_mode){

}

void IMU::initialize_imu(void){
  CAN_initialize(_1M,"CAN1");   //bit_rate : 1Mbps, ID : 1, is_ext_mode : true
}

void IMU::imu_req(void){
  BYTE imu_req_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  int i=0,j=0;

  //imu_req_arr[0] = 0x3c;//0x18;//0x3c;
  //imu_req_arr[1] = 53 & 0xff;    //low data
  //imu_req_arr[2] = 53>>8 & 0xff; //high data

  //imu_req_arr[3]=0x01;
  //imu_req_arr[4]=0x00;

  //CAN_write(imu_req_arr);
  imu_req_arr[0] = 0x3c;

  for(i=0;i<=2;i++){

    //i=0 : acc
    //i=1 : gyro
    //i=2 : angle

    imu_req_arr[1] = (51+i) & 0xff;    //low data
    imu_req_arr[2] = (51+i)>>8 & 0xff; //high data

    for(j=1;j<=3;j++){

      //j=1 : x
      //j=2 : y
      //j=3 : z

     imu_req_arr[3] = j;
     CAN_write(imu_req_arr);
    }
  }

}

void IMU::set_sync_req(bool onoff){
  //CAN 동기화 데이터 전송 설정
  BYTE cmd_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  cmd_arr[0] = 0x18;
  cmd_arr[1] = 22 & 0xff;    //low data
  cmd_arr[2] = 22>>8 & 0xff; //high data
  if(onoff == true){
    cmd_arr[4] = 0x07;
  }
  else{
    cmd_arr[4] = 0x00;
  }
  CAN_write(cmd_arr);
}

void IMU::read_sync_data(void){
  struct CAN_data imu_can_data;
  int i=0;
  for(i=0;i<3;i++){
    imu_can_data = CAN_read();
    if(imu_can_data.data[1]==51){
      this->acc_x = G * Byte2Int16(imu_can_data.data[2],imu_can_data.data[3])/1000.0;  //m/s^2
      this->acc_y = G * Byte2Int16(imu_can_data.data[4],imu_can_data.data[5])/1000.0;
      this->acc_z = G * Byte2Int16(imu_can_data.data[6],imu_can_data.data[7])/1000.0;
    }
    else if(imu_can_data.data[1]==52){
      this->gyro_x = DEG_2_RAD(Byte2Int16(imu_can_data.data[2],imu_can_data.data[3])/10.0);  //rad
      this->gyro_y = DEG_2_RAD(Byte2Int16(imu_can_data.data[4],imu_can_data.data[5])/10.0);
      this->gyro_z = DEG_2_RAD(Byte2Int16(imu_can_data.data[6],imu_can_data.data[7])/10.0);
    }
    else if(imu_can_data.data[1]==53){
      this->angle_x = Byte2Int16(imu_can_data.data[2],imu_can_data.data[3])/100.0;  //deg
      this->angle_y = Byte2Int16(imu_can_data.data[4],imu_can_data.data[5])/100.0;
      this->angle_z = Byte2Int16(imu_can_data.data[6],imu_can_data.data[7])/100.0;
    }
    else{
      ROS_WARN("Something is wrong(read_sync_data) :( ");
    }
  }


}

void IMU::imu_read(void){
  struct CAN_data imu_can_data;

  //imu_can_data = CAN_read();

  //this->angle_x = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);

  int i=0;

  for(i=0;i<9;i++){
    imu_can_data = CAN_read();
    if(imu_can_data.data[1]==51){

      if(imu_can_data.data[3]==1) this->acc_x = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==2) this->acc_y = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==3) this->acc_z = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else ROS_WARN("Something is wrong (acc_read)");

    }

    else if(imu_can_data.data[1]==52){

      if(imu_can_data.data[3]==1) this->gyro_x = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==2) this->gyro_y = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==3) this->gyro_z = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else ROS_WARN("Something is wrong (gyro_read)");
    }

    else if(imu_can_data.data[1]==53){

      if(imu_can_data.data[3]==1) this->angle_x = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==2) this->angle_y = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==3) this->angle_z = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else ROS_WARN("Something is wrong (angle_read)");

    }
    else{
      ROS_WARN("Something is wrong??");
    }

  }
  //ROS_INFO("angle x : %f",angle_x);
}



float IMU::Byte2float32(BYTE d4, BYTE d5, BYTE d6, BYTE d7)
{
  float result = 0.0;
  BYTE data[4] = {d4,d5,d6,d7};
  memcpy(&result, &data, sizeof(float));
  return result;
  //return (float)(d4 | d5<<8 | d6<<16 | d7<<24);
}

int IMU::Byte2Int32(BYTE d4, BYTE d5, BYTE d6, BYTE d7)
{
  return (int)((int)d4 | (int)d5<<8 | (int)d6<<16 | (int)d7<<24);
}

short IMU::Byte2Int16(BYTE d1, BYTE d2)
{
  return (short)((short)d1 | (short)d2<<8);
}

