// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
//#include "Arduino.h"
#include "interface.h"


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------

Autopilot_Interface::Autopilot_Interface()
{

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id
	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;
}


/*
// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
*/

//uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
uint8_t data = 0;
int read_check=0;
void Autopilot_Interface::read_messages() 
{ 
  //printf("sup\n");
  mavlink_message_t message;
  mavlink_status_t status;
  int len = uart_read_bytes(UART_NUM_0, &data, 1, 20 / portTICK_RATE_MS);
  //int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
  //printf("len: %i",len);
  if(len>0) 
  {
    read_check=1;
    //uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    //int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, PACKET_READ_TICS);

    //uint8_t c = Serial2.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, data, &message, &status)) 
    {
     
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//Serial.write("<<MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));	
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//Serial.write("<<MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//Serial.write("<<MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
                                        //uart_write_bytes(UART_NUM_1, (const char *)"<<MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n", sizeof("<<MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n"));
					//Serial.write("<<MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                                        //Serial.print("Latitude: ");
                                        //uart_write_bytes(UART_NUM_1, (const char *)"Latitude: ", sizeof("Latitude: "));
                                        //Serial.print(current_messages.global_position_int.lat);
                                        //Serial.print("Longitude: ");
                                        //Serial.println(current_messages.global_position_int.lon);
                                        //Serial.print("Altitude above MSL: ");
                                        //Serial.println(current_messages.global_position_int.alt);
                                        //printf("Altitude above ground: %i\n",current_messages.global_position_int.relative_alt);
                                        //printf((const char*)current_messages.global_position_int.relative_alt);
                                        //uart_write_bytes(UART_NUM_1, (const char *)"Altitude above ground: ", sizeof("Altitude above ground: "));
                                        //Serial.print("Altitude above ground: ");
                                        //uart_write_bytes(UART_NUM_1, (const char *)current_messages.global_position_int.relative_alt, sizeof((const char *)current_messages.global_position_int.relative_alt));
                                        //Serial.println(current_messages.global_position_int.relative_alt);
                                        //Serial.print("Vehicle heading : ");
                                        //Serial.println(current_messages.global_position_int.hdg);
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}
      
      }
        
    }
      
  }
    
  else if(read_check==0)
  {  set_message_interval();
     //read_check=0;
     //printf("value of len:%i\n",len); 
  }  

}



//------------------------------------------------------------------------------
// Commands
// ------------------------------------------------------------------------------

/*
//guided
int Autopilot_Interface::guided_mode()
{

	// Format the data:
	mavlink_set_mode_t set_mode = {0};
	set_mode.target_system = system_id;
	set_mode.custom_mode   = 4;
  set_mode.base_mode     = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	// Encode:
	mavlink_message_t message;
  mavlink_msg_set_mode_encode(system_id, autopilot_id, &message, &set_mode);
		
	char buf[MAVLINK_MAX_PACKET_LEN];
  // Translate message to buffer
  unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
  // Write buffer to serial port, locks port while writing
  Serial2.write((uint8_t*)buf,len);
  Serial2.write("\n"); 


   uart_write_bytes(UART_NUM_1, (const char *)"working", sizeof("working"));
   printf("hello\n");
   return 1;
}
*/

/*
// arming
int Autopilot_Interface::arming()
{
  
  mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = 0;
	com.param1           = 1;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	char buf[MAVLINK_MAX_PACKET_LEN];
  // Translate message to buffer
  unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
  // Write buffer to serial port, locks port while writing
  Serial2.write((uint8_t*)buf,len);
  Serial2.write("\n");
	// Done!
	return len;   
  
}


//disarming
int Autopilot_Interface::disarming()
{
  mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = 0;
	com.param1           = 0; 

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  char buf[MAVLINK_MAX_PACKET_LEN];
  // Translate message to buffer
  unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
  // Write buffer to serial port, locks port while writing
  Serial2.write((uint8_t*)buf,len);
  Serial2.write("\n");
	// Done!
	return len;   

}


// ------------------------------------------------------------------------------
//  Set Stream Data  
// ------------------------------------------------------------------------------
int Autopilot_Interface::set_stream_data(uint8_t req_message_rate,uint8_t req_stream_id,uint8_t start_stop)
{
	// Prepare command 
	mavlink_request_data_stream_t stream;
	stream.target_system    = system_id;
	stream.target_component = autopilot_id;
	stream.req_message_rate = req_message_rate;
  stream.req_stream_id    = req_stream_id;
  stream.start_stop       = start_stop; 
	
	// Encode
	mavlink_message_t message;
	mavlink_msg_request_data_stream_encode(system_id, companion_id, &message, &stream);
                
  char buf[MAVLINK_MAX_PACKET_LEN];
  // Translate message to buffer
  unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
  // Write buffer to serial port, locks port while writing
  Serial2.write((uint8_t*)buf,len);
  Serial2.write("\n");
  
	// Done!
	return len;
}
*/

int Autopilot_Interface::set_message_interval()
{
  mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = 511;
	com.confirmation     = 0;
	com.param1           = 33; 
	com.param2           = 100000;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
                
  char buf[MAVLINK_MAX_PACKET_LEN];
  // Translate message to buffer
  unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
  
  // Write buffer to serial port, locks port while writing
  uart_write_bytes(UART_NUM_0, (const char *)buf, (uint8_t)len);
  //printf("%i\n",(uint8_t)len);
  //printf("\n");
  uart_write_bytes(UART_NUM_0, (const char *)"\n", sizeof("\n"));
  //Serial2.write((uint8_t*)buf,len);
  //Serial2.write("\n");
  
	// Done!
	return len;
}


Autopilot_Interface api=Autopilot_Interface();
