#include "mavlink/common/mavlink.h"
#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#define BUF_SIZE (1024)

struct Mavlink_Messages 
{
  int sysid;
  int compid;

  // Heartbeat
  mavlink_heartbeat_t heartbeat;
  // System Status
  mavlink_sys_status_t sys_status;
  // Battery Status
  mavlink_battery_status_t battery_status;
  // Global Position
  mavlink_global_position_int_t global_position_int;
};

class Autopilot_Interface
{
  public:
  Autopilot_Interface();
  int system_id;
  int autopilot_id;
  int companion_id;
  //int guided_mode();
  Mavlink_Messages current_messages;
  void read_messages();
  int set_message_interval();
  /*
  int set_stream_data(uint8_t req_message_rate,uint8_t req_stream_id,uint8_t start_stop);
  int arming();
  int disarming();

  */
};

extern Autopilot_Interface api;
