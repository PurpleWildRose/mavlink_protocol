/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file serial.cpp
 *
 * @brief Serial interface functions
 *
 * Functions for opening, closing, reading and writing via serial ports
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial.h"
#include "logging.h"


// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Serial::Serial(const char *uart_name_ , int baudrate_) {
	initialize_defaults();
	uart_name = uart_name_;
	baudrate  = baudrate_;
}

Serial::Serial() : 
    uart_name("/dev/ttyACM0"),
    baudrate(57600)  {
	initialize_defaults();
}

Serial::~Serial() {
	// destroy mutex
	pthread_mutex_destroy(&lock);
}

void Serial::initialize_defaults() {
	// Initialize attributes
	debug  = false;
	fd     = -1;
	is_open = false;

	// Start mutex
	int result = pthread_mutex_init(&lock, NULL);
	if ( result != 0 )
	{
		printf("\n mutex init failed\n");
		throw 1;
	}
}


// ------------------------------------------------------------------------------
//   Read from Serial
// ------------------------------------------------------------------------------
int Serial::read_message(mavlink_message_t &message) {
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;

	// --------------------------------------------------------------------------
	//   READ FROM PORT
	// --------------------------------------------------------------------------

	// this function locks the port during read
	int result = _read_port(cp);


	// --------------------------------------------------------------------------
	//   PARSE MESSAGE
	// --------------------------------------------------------------------------
	if (result > 0)
	{
		// the parsing
        // MAVLINK_COMM_1：通信通道标识（MAVLink 支持多通道，COMM_1 通常对应串口 1）。
        /*************************
         * 返回值 msgReceived：表示解析结果，通常：

            1：成功解析出一条完整的 MAVLink 消息。
            0：尚未解析出完整消息（需要继续接收更多字符）。
        ************************* */
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

		// check for dropped packets
		if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
		{
			printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			unsigned char v=cp;
			fprintf(stderr,"%02x ", v);
		}
		lastStatus = status;
	}

	// Couldn't read from port
	else
	{
		fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
	}

	// --------------------------------------------------------------------------
	//   DEBUGGING REPORTS
	// --------------------------------------------------------------------------
	if(msgReceived && debug)
	{
		// Report info
		printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

		fprintf(stderr,"Received serial data: ");
		unsigned int i;
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

		// check message is write length
		unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

		// message length error
		if (messageLength > MAVLINK_MAX_PACKET_LEN)
		{
			fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
		}

		// print out the buffer
		else
		{
			for (i=0; i<messageLength; i++)
			{
				unsigned char v=buffer[i];
				fprintf(stderr,"%02x ", v);
			}
			fprintf(stderr,"\n");
		}
	}

	// Done!
	return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int Serial::write_message(const mavlink_message_t &message) {
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to serial port, locks port while writing
	int bytesWritten = _write_port(buf,len);

	return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Open Serial Port
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void Serial::start() {
	// --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------
	printf("OPEN PORT\n");

	fd = _open_port(uart_name);

	// Check success
	if (fd == -1)
	{
		printf("failure, could not open port.\n");
		throw EXIT_FAILURE;
	}

	// --------------------------------------------------------------------------
	//   SETUP PORT
	// --------------------------------------------------------------------------
	bool success = _setup_port(baudrate, 8, 1, false, false);

	// --------------------------------------------------------------------------
	//   CHECK STATUS
	// --------------------------------------------------------------------------
	if (!success)
	{
		printf("failure, could not configure port.\n");
		throw EXIT_FAILURE;
	}
	if (fd <= 0)
	{
		printf("Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		throw EXIT_FAILURE;
	}

	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	lastStatus.packet_rx_drop_count = 0;

	is_open = true;

	printf("\n");

	return;

}


// ------------------------------------------------------------------------------
//   Close Serial Port
// ------------------------------------------------------------------------------
void Serial::stop() {
	printf("CLOSE PORT\n");

	int result = close(fd);

	if ( result )
	{
		fprintf(stderr,"WARNING: Error on port close (%i)\n", result );
	}

	is_open = false;

	printf("\n");

}

// ------------------------------------------------------------------------------
//   Helper Function - Open Serial Port File Descriptor
// ------------------------------------------------------------------------------
// Where the actual port opening happens, returns file descriptor 'fd'
int Serial::_open_port(const char* port) {
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY：不将该设备设置为控制终端（避免程序收到如 Ctrl+C 等信号）
    // O_NDELAY：非阻塞模式（即使设备未准备好也会立即返回，而非阻塞等待）
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	// Check for Errors
	if (fd == -1)
	{
		/* Could not open the port. */
        NN_LOG_ERROR("Could not open serial port");
		return(-1);
	}

	// Finalize
	else
	{
        // 清除文件描述符 fd 的所有状态标志，将其恢复为默认状态。
        // F_SETFL：表示要设置文件状态标志（file status flags）。
		fcntl(fd, F_SETFL, 0);
	}

	// Done!
	return fd;
}

// ------------------------------------------------------------------------------
//   Helper Function - Setup Serial Port
// ------------------------------------------------------------------------------
// Sets configuration, flags, and baud rate
bool Serial::_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control) {
	// Check file descriptor
    // 判断文件描述符 fd 是否指向一个终端设备（包括串口，因为串口在 Unix/Linux 中被视为终端设备）。
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}

	// Read file descritor configuration
    // 获取串口设备的当前配置信息
	struct termios  config;
    // 于读取文件描述符 fd 对应的终端设备（此处为串口）的配置，并将结果存入 config 结构体。
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
    // 通过清除特定标志来禁用一些默认的输入处理功能
    /***********************************
     *  IGNBRK	忽略 Break 信号（线路中断信号）
        BRKINT	当收到 Break 信号时，产生中断（SIGINT）
        ICRNL	将输入中的回车符（CR, '\r'）转换为换行符（NL, '\n'）
        INLCR	将输入中的换行符（NL, '\n'）转换为回车符（CR, '\r'）
        PARMRK	标记奇偶校验错误（在错误字节前插入特殊字符）
        INPCK	启用输入奇偶校验（检查接收数据的奇偶校验位）
        ISTRIP	剥离输入字节的第 8 位（将 8 位数据转换为 7 位）
        IXON	启用 XON/XOFF 软件流控制（接收Ctrl+S暂停传输，Ctrl+Q恢复传输）
    ********************************** */
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
    // 通过清除特定标志来禁用一些默认的输出处理功能
    /***********************************
     *  OCRNL	将输出中的回车符（CR, '\r'）转换为换行符（NL, '\n'）
        ONLCR	将输出中的换行符（NL, '\n'）转换为回车符（CR, '\r'）
        ONLRET	在输出中添加回车符（CR, '\r'）以实现换行
        ONOCR	在输出中添加换行符（NL, '\n'）以实现回车
        OFILL	启用输出填充字符（在输出中插入填充字符）
        OPOST	启用输出处理（对输出进行处理，例如换行符转换）
    ********************************** */
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);
    
    // 禁用输出字符的大写转换功能。
	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

    // 禁用特定的输出处理功能，确保串口数据传输的原始性。
    // ONOEOT 是一个较为罕见的终端输出标志，其作用是在输出中忽略 EOT 字符（EOT 即 Ctrl+D，ASCII 码为 0x04）。
	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
    // 配置串口的本地模式标志（c_lflag），通过清除一系列标志来禁用终端的本地处理功能
    /*********************************
     *  ECHO	禁用输入字符的回显（不在终端上显示输入的字符）
     *  ECHONL	即使禁用了 ECHO，仍回显换行符（\n），清除后换行符也不回显
     *  ICANON	禁用规范模式（canonical mode），启用原始模式（raw mode）。
     *  规范模式下，输入按行缓冲（需回车确认）；原始模式下，输入立即传递，无缓冲
     *  IEXTEN	禁用扩展的输入处理（如某些系统上的 Ctrl+V 等特殊功能）
     *  ISIG	禁用信号生成（如 Ctrl+C 不产生 SIGINT 信号，Ctrl+Z 不产生 SIGTSTP 信号）
    ********************************* */
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
    // 配置串口的控制模式标志（c_cflag），主要作用是设置串口的数据位为 8 位，并禁用奇偶校验
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
    /****************************
        VMIN	VTIME	行为描述
        0	0	read() 立即返回（非阻塞模式），返回当前可用数据量（可能为 0）, （可能无限阻塞）。
        >0	0	read() 阻塞直到收到至少 VMIN 字节（无超时）。
        0	>0	read() 等待 VTIME 时长，返回期间收到的所有数据（即使为 0）。

    example:
        VMIN 值	VTIME 值	读取行为描述
        1	10	最多等待 1 秒，读到 1 个字节就返回（常用）
        0	10	等待 1 秒后返回，返回期间收到的所有数据（可能为 0 字节）
        5	0	无超时，必须读到 5 个字节才返回（可能无限阻塞）
    **************************** */
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
        // 设置串口的输入和输出波特率（这里设置为 57600)
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}

	// Finally, apply the configuration
    // 用于将前面修改好的串口配置（存储在config结构体中）应用到实际的串口设备，并处理可能的配置失败情况。
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}

	// Done!
	return true;
}



// ------------------------------------------------------------------------------
//   Read Port with Lock
//
//    return
//    1：成功读取 1 个字节（正常情况）。
//    0：读取超时或连接关闭（根据前面 VMIN 和 VTIME 配置，可能是超时无数据）。
//    -1：读取失败（如设备错误，可通过 errno 获取具体原因）。
//
// ------------------------------------------------------------------------------
int Serial::_read_port(uint8_t &cp) {

	// Lock
	pthread_mutex_lock(&lock);

	int result = read(fd, &cp, 1);

	// Unlock
	pthread_mutex_unlock(&lock);

	return result;
}


// ------------------------------------------------------------------------------
//   Write Port with Lock
//
//  return:
//  若返回值为 -1，表示写入失败（可通过 errno 获取错误原因）
// ------------------------------------------------------------------------------
int Serial::_write_port(char *buf, unsigned len) {

	// Lock
	pthread_mutex_lock(&lock);

	// Write packet via serial link
    // 向文件描述符 fd（此处为串口）写入数据。
	const int bytesWritten = static_cast<int>(write(fd, buf, len));

	// Wait until all data has been written
    // 阻塞当前进程，直到串口输出缓冲区中的所有数据都已实际发送到硬件（而非仅写入缓冲区）。
	tcdrain(fd);

	// Unlock
	pthread_mutex_unlock(&lock);


	return bytesWritten;
}


