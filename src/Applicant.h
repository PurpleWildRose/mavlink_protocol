#ifndef __EXAMPLE_APPLICANT_H__
#define __EXAMPLE_APPLICANT_H__

#include "common.h"

class Protocol {
    public:
        Protocol();
        virtual ~Protocol();

    public:
        static Protocol* instance;

        void parse_argument(int argc, char **argv);
        static void quit_handler(int sig);
        void Demo(int argc, char **argv);
        void task(Ardupilot_inference &api, bool flag);

    private:
        const char *uart_name;
        int64_t baudrate;

        const char *udp_ip;
        int udp_port;

        bool autotakeoff;
        bool use_udp;

        static Generic_Port *port;
        static Ardupilot_inference *ardupilot_interface_quit;
};

#endif