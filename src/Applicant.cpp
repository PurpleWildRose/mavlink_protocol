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
    
    ardupilot_interface_quit->stop();
    port->stop();

    delete port;

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