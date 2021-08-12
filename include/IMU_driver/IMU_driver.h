#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "can_test/can.h"

#define DEG_2_RAD(value) (3.141592/180)*value
#define G 9.80665


class IMU : public CAN
{
  public:

  float acc_x = 0.0;
  float acc_y = 0.0;
  float acc_z = 0.0;

  float gyro_x = 0.0;
  float gyro_y = 0.0;
  float gyro_z = 0.0;

  float angle_x= 0.0;
  float angle_y = 0.0;
  float angle_z = 0.0;

  IMU(string port, int id, bool is_ext_mode);

  void initialize_imu(void);
  void imu_req(void);
  float Byte2float32(BYTE d4, BYTE d5, BYTE d6, BYTE d7);
  int Byte2Int32(BYTE d4, BYTE d5, BYTE d6, BYTE d7);
  short Byte2Int16(BYTE d1, BYTE d2);
  void set_sync_req(bool onoff);
  void imu_read(void);
  void read_sync_data(void);

  void set_Bitrate_250(void);

  void cmd(int cmd_num);
  void save_params(void);
  void software_reset(void);
  void set_sync_tx_cycle(int cycle_num);

};

#endif // IMU_DRIVER_H
