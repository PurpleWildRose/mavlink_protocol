#include "Applicant.h"


Protocol::Protocol() :
    uart_name("/dev/ttyACM0"),
    baudrate(57600),
    udp_ip("127.0.0.1"),
    udp_port(14540),
    use_udp(false),
    autotakeoff(false){

}

Protocol::~Protocol() {

}

void Protocol::parse_argument(int argc, char **argv) {
    // string for command line usage.
    const char *brief = "MAVLink Protocol Communication Tool";
    const char *commandline_usage = "Usage: mavlink_protocol [-d <devicename> -b <baudrate>] [-u <udp_ip> -p <port>] [--a  ] [--t  ].";
    const char *options_help = 
        "Options:\n"
        "  -d <devicename>   UART device name (e.g., /dev/ttyUSB0)\n"
        "  -b <baudrate>     UART baudrate (e.g., 115200)\n"
        "  -u <udp_ip>       UDP server IP address (e.g., 192.168.1.100)\n"
        "  -p <port>         UDP port number (e.g., 14550)\n"
        "  -h                Show this help message\n"
        "  --a               Enable auto-takeoff\n"
        "  --t               Chose udp connection\n";                      

    
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0  || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", brief);
            printf("%s\n", commandline_usage);
            printf("%s\n", options_help);

            exit(EXIT_SUCCESS);
        }

        if (strcmp(argv[i], "-d") == 0  || strcmp(argv[i], "--devicename") == 0) {
            if (argc > i+1) {
                this->uart_name = argv[i++];
            } else {
                fprintf(stderr, "Not found uart data.");
                throw EXIT_FAILURE;
            }
        }

        if (strcmp(argv[i], "-b") == 0  || strcmp(argv[i], "--baudrate") == 0) {
            if (argc > i+1) {
                i++;
                this->baudrate = atoi(argv[i]);
            } else {
                fprintf(stderr, "Not found baudrate data.");
                throw EXIT_FAILURE;
            }
        }

        if (strcmp(argv[i], "-u") == 0  || strcmp(argv[i], "--udp_ip") == 0) {
            if (argc > i+1) {
                i++;
                this->udp_ip = argv[i];
            } else {
                fprintf(stderr, "Not found udp ip data.");
                throw EXIT_FAILURE;
            }
        }

        if (strcmp(argv[i], "-p") == 0  || strcmp(argv[i], "--port") == 0) {
            if (argc > i+1) {
                i++;
                this->udp_port = atoi(argv[i]);
            } else {
                fprintf(stderr, "Not found port data.");
                throw EXIT_FAILURE;
            }
        }

        if (strcmp(argv[i], "--t") == 0 ) {
            use_udp = true;
        }

        if (strcmp(argv[i], "--a") == 0 ) {
            autotakeoff = true;
        }
    }

    // Done
    return;
}

void Protocol::quit_handler(int sig) {
    printf("\n");
    fprintf(stderr, "###################################################################\n");
    fprintf(stdout, "TERMINATING AT USER REQUEST\n");
    fprintf(stderr, "###################################################################\n");
    printf("\n");

    try {
        ardupilot_interface_quit->handle_quit(sig);
    } catch (int error) {
        fprintf(stderr, "[Ardupilot Interface Error] : Abnormal Exit!\n");
    }

    try {
        port->stop();
    } catch(int error) {
        fprintf(stderr, "[Serial Port Error] : Abnormal Exit!\n");
    }

    exit(0);
}

void Protocol::Demo(int argc, char **argv) {
    parse_argument(argc, argv);

    if (use_udp) {
        // port = new 
    } else {
        port = new Serial(this->uart_name, this->baudrate);
    }

    Ardupilot_inference ardupilot_interface(Protocol::port);

    ardupilot_interface_quit = &ardupilot_interface;

    signal(SIGINT, Protocol::quit_handler);

    port->start();
    ardupilot_interface.start();
    
    task(ardupilot_interface, true);

    ardupilot_interface_quit->stop();
    port->stop();

    delete port;

    return;
}

void Protocol::task(Ardupilot_inference &api, bool flag) {
    api.enable_offboard_control();
    usleep(500);

    if (flag) {
        api.arm_disarm(true);
        usleep(500);
    }

    // --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
	printf("SEND OFFBOARD COMMANDS\n");

	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	// autopilot_interface.h provides some helper functions to build the command




	// Example 1 - Fly up by to 2m
	set_position( ip.x ,       // [m]
			 	  ip.y ,       // [m]
				  ip.z - 2.0 , // [m]
				  sp         );

	if(autotakeoff)
	{
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
	}

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 8 seconds, check position
	for (int i=0; i < 8; i++)
	{
		mavlink_local_position_ned_t pos = api.real_TimeMessage.local_position_ned;
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		sleep(1);
	}

    // Example 2 - Set Velocity
	set_velocity( -1.0       , // [m/s]
				  -1.0       , // [m/s]
				   0.0       , // [m/s]
				   sp        );

	// Example 2.1 - Append Yaw Command
	set_yaw( ip.yaw + 90.0/180.0*M_PI, // [rad]
			 sp     );

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 4 seconds, check position
	for (int i=0; i < 4; i++)
	{
		mavlink_local_position_ned_t pos = api.real_TimeMessage.local_position_ned;
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		sleep(1);
	}

    if(autotakeoff)
	{
		// Example 3 - Land using fixed velocity
		set_velocity(  0.0       , // [m/s]
					   0.0       , // [m/s]
					   1.0       , // [m/s]
					   sp        );

		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;

		// SEND THE COMMAND
		api.update_setpoint(sp);
		// NOW pixhawk will try to move

		// Wait for 8 seconds, check position
		for (int i=0; i < 8; i++)
		{
			mavlink_local_position_ned_t pos = api.real_TimeMessage.local_position_ned;
			printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
			sleep(1);
		}

		printf("\n");

		// disarm autopilot
		api.arm_disarm(false);
		usleep(100); // give some time to let it sink in
	}

	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.disable_offboard_control();

	// now pixhawk isn't listening to setpoint commands


	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
	printf("READ SOME MESSAGES \n");

	// copy current messages
	Pixhawk_info messages = api.real_TimeMessage;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	printf("Got message LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	printf("Got message HIGHRES_IMU (spec: https://mavlink.io/en/messages/common.html#HIGHRES_IMU)\n");
	printf("    ap time:     %lu \n", imu.time_usec);
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
	printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
	printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
	printf("    temperature: %f C \n"       , imu.temperature );

	printf("\n");


	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;
}

Generic_Port* Protocol::port = nullptr;
Ardupilot_inference* Protocol::ardupilot_interface_quit = nullptr;
Protocol* Protocol::instance = nullptr;

int main(int argc, char **argv) {
    Protocol Protocol;
    try {
        Protocol.Demo(argc, argv);
    } catch(int error) {
        fprintf(stderr, "[Protocol Error] : Abnormal exit!");
        exit(-1);
    }
}