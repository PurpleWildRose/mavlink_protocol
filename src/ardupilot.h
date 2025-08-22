#ifndef __EXAMPLE_ARDUPILOT_H__
#define __EXAMPLE_ARDUPILOT_H__

#include "port.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>

#include <common/mavlink.h>

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/***************************************** 
 *  * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * 1	x	忽略 X 轴位置指令（若不忽略，无人机将飞向指定 X 位置）
 * 2	y	忽略 Y 轴位置指令
 * 3	z	忽略 Z 轴位置指令（Z 轴在 NED 中向下为正，通常对应高度控制）
 * 4	vx	忽略 X 轴速度指令
 * 5	vy	忽略 Y 轴速度指令
 * 6	vz	忽略 Z 轴速度指令
 * 7	ax	忽略 X 轴加速度 / 力指令
 * 8	ay	忽略 Y 轴加速度 / 力指令
 * 9	az	忽略 Z 轴加速度 / 力指令
 * 10	is force setpoint	特殊标志：若置 1，afx/afy/afz 被解释为力指令；否则为加速度指令
 * 11	yaw	忽略偏航角指令（无人机不控制航向角）
 * 12	yaw rate	忽略偏航角速度指令
 * 13-16	未使用	保留位，暂未定义功能
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 * 
 * 模式互斥：位置、速度、加速度 / 力控制通常是互斥的（同一时间只应激活一种），例如：

    若同时使用位置（bit1-3=0）和速度（bit4-6=0），无人机可能无法正常解析指令（优先级未定义）。
**************************************** */
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF      0x1000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND         0x2000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER       0x3000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE         0x4000

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_stamps {
    uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;
};

// Struct containing information on the MAV we are currently connected to

struct Pixhawk_info {
    int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

	// System Parameters?


	// Time Stamps
	Time_stamps time_stamps;
};

class Ardupilot_inference {

    public:
        Ardupilot_inference(Generic_Port *port_);
        virtual ~Ardupilot_inference();

    public:
        char reading_status;
        char writing_status;
        char control_status;
        uint64_t write_count;

        int system_id;
        int autopilot_id;
        int companion_id;

        Pixhawk_info real_TimeMessage;
        mavlink_set_position_target_local_ned_t initial_position;

        void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
        void read_messages();
        int  write_message(mavlink_message_t message);

        int	 arm_disarm( bool flag );
        void enable_offboard_control();
        void disable_offboard_control();

        void start();
	    void stop();

        void start_read_thread();
	    void start_write_thread(void);

        void handle_quit( int sig );

    private:
        Generic_Port *port;

        bool time_to_exit;

        pthread_t read_tid;
	    pthread_t write_tid;

        struct {
            std::mutex mutex;
            mavlink_set_position_target_local_ned_t data;
        } current_setpoint;
        
        void read_thread();
        void write_thread(void);

        int toggle_offboard_control( bool flag );
        void write_setpoint();
};

#endif // __EXAMPLE_ARDUPILOT_H__