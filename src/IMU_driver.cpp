#include "IMU_driver/IMU_driver.h"

IMU::IMU(string port, int id, bool is_ext_mode):CAN(port, id, is_ext_mode){

}

void IMU::initialize_imu(void){
  CAN_initialize(_250k,"CAN0");   //bit_rate : 1Mbps, ID : 1, is_ext_mode : true
}

void IMU::set_Bitrate_250(void){
  BYTE set_bitrate[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  set_bitrate[0] = 0x18;

  set_bitrate[1] = (12) & 0xff;    //low data
  set_bitrate[2] = (12)>>8 & 0xff; //high data

  set_bitrate[4] = (250) & 0xff;
  set_bitrate[5] = (250)>>8 & 0xff;
  set_bitrate[6] = (250)>>16 & 0xff;
  set_bitrate[7] = (250)>>24 & 0xff;

  CAN_write(set_bitrate);

}

void IMU::cmd(int cmd_num){

  /*
   *
   * 1 : - 센서의 구성 파라미터를 플래시 메모리에 저장 ('cmd=1' 또는 'fw')
   * 2 : - 센서의 구성 파라미터를 공장 출하시 설정값으로 초기화 ('cmd=2' 또는 'fd')
   * 3 : - 가속도와 각속도 센서의 바이어스와 스케일 보정 실시 ('cmd=3' 또는 'cal')
   * 4 : - 자기 센서의 바이어스와 스케일 보정 실시 ('cmd=4' 또는 'cam')
   * 5 : - Euler 각도를 0으로 리셋 (‘cmd=5’ 또는 ‘zro’)
   * 9 : - 캘리브레이션 데이터만 리셋 (‘cmd=9’ 또는 ‘rcd’)
   * 99 : - 센서를 소프트웨어 리셋 함 ('cmd=99' 또는 'rst')
   *
   */

  BYTE cmd_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  cmd_arr[0] = 0x18;

  cmd_arr[1] = (7) & 0xff;    //low data
  cmd_arr[2] = (7)>>8 & 0xff; //high data

  cmd_arr[4] = (cmd_num) & 0xff;
  cmd_arr[5] = (cmd_num)>>8 & 0xff;
  cmd_arr[6] = (cmd_num)>>16 & 0xff;
  cmd_arr[7] = (cmd_num)>>24 & 0xff;

  CAN_write(cmd_arr);
}

void IMU::save_params(void){
  cmd(1);
  /*
   * 센서의 구성 파라미터 값을 변경하면, 변경된 값들은 RAM에 기록되어 센서가 켜져 있는 동안만 유지됩니다.
   * 만일 센서가 재시작 되면 이 값들은 소실되고 플래시 메모리에 저장된 값을 RAM으로 다시 읽어 들이게 됩니다.
   * 그렇기 때문에 구성 파라미터 값을 변경하고 계속 유지되기를 바란다면,
   * 'cmd=1' 명령을 사용하여 변경한 구성 파라미터 값을 플래시 메모리에 저장해야 합니다.
   */
}

void IMU::software_reset(void){
  cmd(99);  //센서를 소프트웨어적으로 리셋 시킵니다. 센서의 전원을 껐다 켜는 것과 같습니다.
}

void IMU::set_sync_tx_cycle(int cycle_num){
  BYTE cycle_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  cycle_arr[0] = 0x18;

  cycle_arr[1] = (24) & 0xff;    //low data
  cycle_arr[2] = (24)>>8 & 0xff; //high data

  cycle_arr[4] = (cycle_num) & 0xff;
  cycle_arr[5] = (cycle_num)>>8 & 0xff;
  cycle_arr[6] = (cycle_num)>>16 & 0xff;
  cycle_arr[7] = (cycle_num)>>24 & 0xff;

  CAN_write(cycle_arr);
}

void IMU::set_id(int id_num){
  BYTE id_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  id_arr[0] = 0x18;

  id_arr[1] = (24) & 0xff;    //low data
  id_arr[2] = (24)>>8 & 0xff; //high data

  id_arr[4] = (id_num) & 0xff;
  id_arr[5] = (id_num)>>8 & 0xff;
  id_arr[6] = (id_num)>>16 & 0xff;
  id_arr[7] = (id_num)>>24 & 0xff;
  CAN_write(id_arr);
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
    cmd_arr[4] = 0x06; //0x07;   //6 : gyro, angle //// 7 : accel, gyro, angle
  }
  else{
    cmd_arr[4] = 0x00;
  }
  CAN_write(cmd_arr);
}

void IMU::read_sync_data(void){
  struct CAN_data imu_can_data;
  int i=0;
  for(i=0;i<2;i++){
    imu_can_data = CAN_read();
    if(imu_can_data.data[1]==51){
      this->acc_x = G * Byte2Int16(imu_can_data.data[2],imu_can_data.data[3])/1000.0;  //m/s^2
      this->acc_y = G * Byte2Int16(imu_can_data.data[4],imu_can_data.data[5])/1000.0;
      this->acc_z = G * Byte2Int16(imu_can_data.data[6],imu_can_data.data[7])/1000.0;
    }
    else if(imu_can_data.data[1]==52){
      this->gyro_x = DEG_2_RAD(Byte2Int16(imu_can_data.data[2],imu_can_data.data[3])/10.0);  //rad/s
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

