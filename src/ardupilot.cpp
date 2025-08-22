#include "ardupilot.h"

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t get_time_usec() {
    static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp) {
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x = x;
    sp.y = y;
    sp.z = z;

    printf("USER POSITION SETPOINT XYZ = [ %.7f , %.7f , %.7f ] \n", sp.x, sp.y, sp.z);
}

void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp) {
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.vx = vx;
    sp.vy = vy;
    sp.vz = vz;

    printf("USER VELOCITY SETPOINT XYZ = [ %.7f , %.7f , %.7f ] \n", sp.vx, sp.vy, sp.vz);
}

void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp) {

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;

    printf("USER ACCELERATION SETPOINT XYZ = [ %.7f , %.7f , %.7f ] \n", sp.afx, sp.afy, sp.afz);
}

void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp) {
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

    sp.yaw = yaw;

    printf("USER YAW SETPOINT XYZ = [ %.7f ] \n", sp.yaw);
}

void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp) {
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

    sp.yaw_rate = yaw_rate;

    printf("USER YAW_RATE SETPOINT XYZ = [ %.7f ] \n", sp.yaw_rate);
}

Ardupilot_inference::Ardupilot_inference(Generic_Port *port_) :
    write_count(0),
    reading_status(0),
    writing_status(0),
    control_status(0),

    time_to_exit(false),  // flag to signal thread exit

    read_tid(0),
    write_tid(0),

    system_id(0),
    autopilot_id(0),
    companion_id(0),
    port(port_) { 
        real_TimeMessage.sysid = system_id;
        real_TimeMessage.compid = companion_id;
}

Ardupilot_inference::~Ardupilot_inference() {

}

void Ardupilot_inference::update_setpoint(mavlink_set_position_target_local_ned_t setpoint) {
    std::lock_guard<std::mutex> lock(current_setpoint.mutex);
    current_setpoint.data = setpoint;
}

void Ardupilot_inference::read_messages() {
    bool success;
    bool receive_all = false;

    Time_stamps time_stamps;
    // Blocking wait for new data
    while ( !receive_all && !time_to_exit ) {
        // ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
        mavlink_message_t msg;
        success = port->read_message(msg);

        // ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
        if ( success ) {
            // Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
            real_TimeMessage.sysid = msg.sysid;
            real_TimeMessage.compid = msg.compid;

            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT : {
                    mavlink_msg_heartbeat_decode(&msg, &(real_TimeMessage.heartbeat));
                    real_TimeMessage.time_stamps.heartbeat = get_time_usec();
                    time_stamps.heartbeat = real_TimeMessage.time_stamps.heartbeat;
                    break;
                }

                case MAVLINK_MSG_ID_SYS_STATUS : {
                    mavlink_msg_sys_status_decode(&msg, &(real_TimeMessage.sys_status));
                    real_TimeMessage.time_stamps.sys_status = get_time_usec();
                    time_stamps.sys_status = real_TimeMessage.time_stamps.sys_status;
                    break;
                }

                case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&msg, &(real_TimeMessage.battery_status));
					real_TimeMessage.time_stamps.battery_status = get_time_usec();
					time_stamps.battery_status = real_TimeMessage.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&msg, &(real_TimeMessage.radio_status));
					real_TimeMessage.time_stamps.radio_status = get_time_usec();
					time_stamps.radio_status = real_TimeMessage.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&msg, &(real_TimeMessage.local_position_ned));
					real_TimeMessage.time_stamps.local_position_ned = get_time_usec();
					time_stamps.local_position_ned = real_TimeMessage.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&msg, &(real_TimeMessage.global_position_int));
					real_TimeMessage.time_stamps.global_position_int = get_time_usec();
					time_stamps.global_position_int = real_TimeMessage.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&msg, &(real_TimeMessage.position_target_local_ned));
					real_TimeMessage.time_stamps.position_target_local_ned = get_time_usec();
					time_stamps.position_target_local_ned = real_TimeMessage.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&msg, &(real_TimeMessage.position_target_global_int));
					real_TimeMessage.time_stamps.position_target_global_int = get_time_usec();
					time_stamps.position_target_global_int = real_TimeMessage.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&msg, &(real_TimeMessage.highres_imu));
					real_TimeMessage.time_stamps.highres_imu = get_time_usec();
					time_stamps.highres_imu = real_TimeMessage.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&msg, &(real_TimeMessage.attitude));
					real_TimeMessage.time_stamps.attitude = get_time_usec();
					time_stamps.attitude = real_TimeMessage.time_stamps.attitude;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}
            }
        }

        // Check for receipt of all items
        receive_all = time_stamps.heartbeat &&
                    // time_stamps.battery_status &&
                    time_stamps.sys_status;

        // give the write thread time to use the port
        if ( writing_status ) {
            usleep(100);
        }
    }

    return;
}

int Ardupilot_inference::write_message(mavlink_message_t message) {
    // do the write
    int len = port->write_message(message);

    // book keep
    write_count++;

    return len;
}

void Ardupilot_inference::write_setpoint() {
    mavlink_set_position_target_local_ned_t sp;

    std::lock_guard<std::mutex> lock(current_setpoint.mutex);
    sp = current_setpoint.data;

    if (not sp.time_boot_ms) {
        sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
    }
    sp.target_system = system_id;
    sp.target_component = autopilot_id;
}