// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "common.h"

void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate, bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff) {
    
    // string for command line usage
	const char *commandline_usage = "usage: mavlink_control [-d <devicename> -b <baudrate>] [-u <udp_ip> -p <udp_port>] [-a ]";

    for (int i = 0; i < argc; i++) { 
        // Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

        // UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				i++;
				uart_name = argv[i];
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

        // Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				i++;
				baudrate = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

        // UDP ip
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--udp_ip") == 0) {
			if (argc > i + 1) {
				i++;
				udp_ip = argv[i];
				use_udp = true;
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

        // UDP port
		if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
			if (argc > i + 1) {
				i++;
				udp_port = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

        // Autotakeoff
		if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) {
			autotakeoff = true;
		}
    }

    // Done!
	return;
}

/*******************************************
 *  @param argv[2] dev_name / udp ip
 *  @param argv[3] baudrate / port
 ******************************************/
int user_parser(int argc, char **argv) {
    // --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
    #ifdef __APPLE__
        char *uart_name = (char*)"/dev/tty.usbmodem1";
    #else
        char *uart_name = (char*)"/dev/ttyACM0";
    #endif

    int baudrate = 57600;

    bool use_udp = false;
    char *udp_ip = (char*)"127.0.0.1";
    int udp_port = 14540;
    bool autotakeoff = false;

    // do the parse, will throw an int if it fails
    parse_commandline(argc, argv, uart_name, baudrate, use_udp, udp_ip, udp_port, autotakeoff);

    Generic_Port *port_;
    if (use_udp) {
        // UDP
        // port = new  UDP(udp_ip, udp_port);
    } else {
        // Serial
        port_ = new Serial(uart_name, baudrate);
    }

    /*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a position target
	 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
	 * is changed by using the method update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, a signal
	 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
	 * method.  Signal the exit of this mode with disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
    Ardupilot_inference ardupilot_inference(port_);

    
}