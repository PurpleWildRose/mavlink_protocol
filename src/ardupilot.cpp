#include "ardupilot.h"
#include "common.h"

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t get_time_usec() {
    static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

/*---------------------------------------------------------------------------------------
 *  @def set_position 
 *      Use mavlink send the three axis position
 * 
 *  @param x 
 *  @param y
 *  @param z
 *  @param sp
 * 
 *  @return None
 * 
 ---------------------------------------------------------------------------------------*/
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp) {
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x = x;
    sp.y = y;
    sp.z = z;

    printf("USER POSITION SETPOINT XYZ = [ %.7f , %.7f , %.7f ] \n", sp.x, sp.y, sp.z);
}

/*---------------------------------------------------------------------------------------
 *  @def set_velocity 
 *      Use mavlink send the three axis velocity
 * 
 *  @param x 
 *  @param y
 *  @param z
 *  @param sp
 * 
 *  @return None
 * 
 ---------------------------------------------------------------------------------------*/
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp) {
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.vx = vx;
    sp.vy = vy;
    sp.vz = vz;

    printf("USER VELOCITY SETPOINT XYZ = [ %.7f , %.7f , %.7f ] \n", sp.vx, sp.vy, sp.vz);
}

/*---------------------------------------------------------------------------------------
 *  @def set_acceleration 
 *      Use mavlink send the three axis acceleration
 * 
 *  @param ax
 *  @param ay
 *  @param az
 *  @param sp
 * 
 *  @return None
 * 
 ---------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------
 *  @def set_yaw 
 *      Use mavlink send the uav [yaw] value
 * 
 *  @param yaw
 *  @param sp
 * 
 *  @return None
 * 
 ---------------------------------------------------------------------------------------*/
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp) {
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

    sp.yaw = yaw;

    printf("USER YAW SETPOINT XYZ = [ %.7f ] \n", sp.yaw);
}

/*---------------------------------------------------------------------------------------
 *  @brief set_yaw_rate
 *      Use mavlink send the uav [yaw_rate] value
 * 
 *  @param yaw_rate
 *  @param sp
 *   
 *  @return None
 * 
 ---------------------------------------------------------------------------------------*/
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp) {
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

    sp.yaw_rate = yaw_rate;

    printf("USER YAW_RATE SETPOINT XYZ = [ %.7f ] \n", sp.yaw_rate);
}

/*---------------------------------------------------------------------------------------
 *  @brief start_ardupilot_write_thread
 *      开启写线程 (pthread)
 * 
 ---------------------------------------------------------------------------------------*/
 void *start_ardupilot_write_pthread(void *args) {
    // takes an autopilot object argument
    Ardupilot_inference *autopilot_inference = (Ardupilot_inference *)args;

    // run the object's read thread
	autopilot_inference->start_write_thread();

    // Done
    return NULL;
 }

/*---------------------------------------------------------------------------------------
 *  @brief start_ardupilot_write_thread
 *      开启读线程 (pthread)
 * 
 ---------------------------------------------------------------------------------------*/
void *start_ardupilot_read_pthread(void *args) {
    // takes an autopilot object argument
    Ardupilot_inference *autopilot_inference = (Ardupilot_inference *)args;

    // run the object's read thread
	autopilot_inference->start_read_thread();

    // Done
    return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------------------------------
 *  @brief Ardupilot_inference (class : Ardupilot_inference)
 *      构造函数实现，用于初始化与 ArduPilot 飞控通信相关的成员变量。
 *      这个构造函数的核心作用是初始化类的状态变量、线程 ID、系统 ID 等参数，并关联一个通信端口（Generic_Port 类型）。
 * 
 *  @param port_;  serial(/dev/XXX)capture data
 * 
 ---------------------------------------------------------------------------------------*/
Ardupilot_inference::Ardupilot_inference(Generic_Port *port_) :
    write_count(0),
    reading_status(0),
    writing_status(0),
    control_status(false),

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

/*---------------------------------------------------------------------------------------
 *  @brief ~Ardupilot_inference(class : Ardupilot_inference)
 *      析构函数: error exit
 * 
 ---------------------------------------------------------------------------------------*/
Ardupilot_inference::~Ardupilot_inference() {

}

/*---------------------------------------------------------------------------------------
 *  @brief update_setpoint(class : Ardupilot_inference)
 *      更新导航点
 * 
 ---------------------------------------------------------------------------------------*/
void Ardupilot_inference::update_setpoint(mavlink_set_position_target_local_ned_t setpoint) {
    std::lock_guard<std::mutex> lock(current_setpoint.mutex);
    current_setpoint.data = setpoint;
}

/*---------------------------------------------------------------------------------------
 *  @brief read_messages(class : Ardupilot_inference)
 *      从Serial串口读取数据
 * 
 ---------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------
 *  @brief write_message(class : Ardupilot_inference)
 *      向Serial串口发送数据
 * 
 *  @param message 用户需要发送的消息 
 * 
 ---------------------------------------------------------------------------------------*/
int Ardupilot_inference::write_message(mavlink_message_t message) {
    // do the write
    int len = port->write_message(message);

    // book keep
    write_count++;

    return len;
}

/*---------------------------------------------------------------------------------------
 *  @brief write_setpoint(class : Ardupilot_inference)
 *      上传航点
 * 
 ---------------------------------------------------------------------------------------*/
void Ardupilot_inference::write_setpoint() {
    // --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------
    mavlink_set_position_target_local_ned_t sp;

    std::lock_guard<std::mutex> lock(current_setpoint.mutex);
    sp = current_setpoint.data;

    if (not sp.time_boot_ms) {
        sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
    }
    sp.target_system = system_id;
    sp.target_component = autopilot_id;

    // --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);

    // --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------
    int len = write_message(message);

    if (len <= 0) {
        fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    }

    return;
}

/*---------------------------------------------------------------------------------------
 *  @brief enable_offboard_control(class : Ardupilot_inference)
 *      开启外部控制权限
 * 
 ---------------------------------------------------------------------------------------*/
void Ardupilot_inference::enable_offboard_control() {
    if (control_status == false) {
        printf("ENABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------
        // Sends the command to go off-board
		int success = toggle_offboard_control( true );

        if (success) {
            control_status = true;
        } else {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
        }

        printf("\n");
    }
}

/*---------------------------------------------------------------------------------------
 *  @brief disable_offboard_control(class : Ardupilot_inference)
 *      关闭外部控制权限
 * 
 ---------------------------------------------------------------------------------------*/
void Ardupilot_inference::disable_offboard_control() {
    if (control_status == true) {
        printf("DISABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------
        int success = toggle_offboard_control( false );

        if (success) {
            control_status = false;
        } else {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
        }

        printf("\n");
    }
}

/*---------------------------------------------------------------------------------------
 *  @brief get_offboard_control(class : Ardupilot_inference)
 *      获取控制权的位置
 *  
 *  @return   false: on-board     true: off-board
 * 
 ---------------------------------------------------------------------------------------*/
 bool Ardupilot_inference::get_offboard_control() {
    return control_status;
 }

/*---------------------------------------------------------------------------------------
 *  @brief toggle_offboard_control(class : Ardupilot_inference)
 *      更换飞控控制权
 * 
 *  @param flag    true/false 
 *  
 *  @return int(len)    执行结果send bytes
 * 
 ---------------------------------------------------------------------------------------*/
int Ardupilot_inference::toggle_offboard_control(bool flag) {
    // Prepare command for off-board mode
    mavlink_command_long_t  com={0};

    com.target_system = system_id;
    com.target_component = autopilot_id;
    com.command = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation = true;
    com.param1 = (float)flag;

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    // Send the message
    int len = port->write_message(message);

    return len;
}

/*---------------------------------------------------------------------------------------
 *  @brief arm_disarm(class : Ardupilot_inference)
 *      无人机的解锁与上锁
 * 
 *  @param flag    true: 解锁       false: 上锁
 *  
 *  @return 
 *      int(len)    执行结果send bytes
 * 
 ---------------------------------------------------------------------------------------*/
int Ardupilot_inference::arm_disarm(bool flag) {
    if (flag) {
        printf("ARM ROTORS\n");
    } else {
        printf("DISARM ROTORS\n");
    }

    // Prepare command for off-board mode
    mavlink_command_long_t com = {0};
    com.target_system = system_id;
    com.target_component = autopilot_id;
    com.command = MAV_CMD_COMPONENT_ARM_DISARM;
    com.confirmation = true;
    com.param1 = (float)flag;
    com.param2 = 21196;

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    // Send the message
    int len = port->write_message(message);

    return len;
}

void Ardupilot_inference::start() {
    int result;

    // --------------------------------------------------------------------------
	//   CHECK PORT
	// --------------------------------------------------------------------------
    if (!port->is_running()) {
        fprintf(stderr,"ERROR: port not open\n");
        throw 1;
    }
    
    // --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------
    printf("START READ THREAD \n");
    result = pthread_create( &read_tid, NULL, &start_ardupilot_read_pthread, this );
	if ( result ) throw result;
    printf("\n");

    
    // --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------
    printf("CHECK FOR MESSAGES\n");
    while ( not real_TimeMessage.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

    printf("Found Pixhawk device.\n");

	// now we know autopilot is sending messages
	printf("\n");
    
    // --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------
    // This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

    // System ID
	if ( not system_id )
	{
		system_id = real_TimeMessage.sysid;
		printf("GOT COPTER SYSTEM ID: %i\n", system_id );
	}

    // Component ID
	if ( not autopilot_id )
	{
		autopilot_id = real_TimeMessage.compid;
		printf("GOT COPTER COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}

    // --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

    while ( not ( real_TimeMessage.time_stamps.local_position_ned &&
				  real_TimeMessage.time_stamps.attitude            )  )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

    // copy initial position ned
	Pixhawk_info local_data = real_TimeMessage;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	printf("\n");

	// we need this before starting the write thread
    // --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_ardupilot_write_pthread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");

	// Done!
	return;
}

/*---------------------------------------------------------------------------------------
 *  @brief read_thread(class : Ardupilot_inference)
 *      读取串口线程
 * 
 ---------------------------------------------------------------------------------------*/
void Ardupilot_inference::read_thread() {
    reading_status = true;

    while (!time_to_exit) {
        read_messages();
        usleep(100000);      // Read batches at 10Hz
    }

    reading_status = false;

    return;
}

void Ardupilot_inference::start_read_thread() {
    if ( !reading_status == false) {
        fprintf(stderr,"read thread already running\n");
		return;
    } else {
        read_thread();
        return;
    }
}

/*---------------------------------------------------------------------------------------
 *  @brief read_thread(class : Ardupilot_inference)
 *      写串口线程
 * 
 ---------------------------------------------------------------------------------------*/
 void Ardupilot_inference::write_thread() {
    // signal startup
    writing_status = 2;

    // prepare an initial setpoint, just stay put
    mavlink_set_position_target_local_ned_t sp;
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
                    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.vx = 0.0;
    sp.vy = 0.0;
    sp.vz = 0.0;
    sp.yaw_rate = 0.0;

    // set position target
    {
        std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		current_setpoint.data = sp;
    }

    // write a message and signal writing
    write_setpoint();
    writing_status = true;

    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe
    while ( !time_to_exit ) {
        usleep(250000);         // Stream at 4Hz
        write_setpoint();
    }

    writing_status = false;
    return;
 }

void Ardupilot_inference::start_write_thread() {
    if ( !writing_status == false) {
        fprintf(stderr,"write thread already running\n");
		return;
    } else {
        write_thread();
        return;
    }
}

/*---------------------------------------------------------------------------------------
 *  @brief read_thread(class : Ardupilot_inference)
 *      线程中断退出
 * 
 ---------------------------------------------------------------------------------------*/
void Ardupilot_inference::stop() {
    // --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

    // signal exit
	time_to_exit = true;

    // wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

    // now the read and write threads are closed
	printf("\n");

	// still need to close the port separately

}

void Ardupilot_inference::handle_quit(int sig) {
    disable_offboard_control();

    try {
        stop();
    } catch (int error) {
        fprintf(stderr,"Warning, could not stop autopilot interface\n");
    }
}


