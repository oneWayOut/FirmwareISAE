#include "mavlink_device.hpp"

#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <system_error>

int 
mavlink_device::read_message(mavlink_message_t& msg) 
{
	ssize_t err;
	mavlink_status_t status;
	bool got_complete_message = false;

	// read the complete message
	while (!got_complete_message) {
		if (_read_idx == _end_idx) {
			err = real_read(_buffer, sizeof(_buffer));
			if (err > 0) {
				_read_idx = 0;
				_end_idx = err;
			} else {
				if (errno == EWOULDBLOCK) return 0; 

				std::cerr << "Error while reading message: " << strerror(errno) << std::endl;
				return -1;
			}
		}

		while (!got_complete_message && _read_idx != _end_idx) {
			got_complete_message = mavlink_parse_char(0, _buffer[_read_idx], &msg, &status);
			_read_idx++;
		}
	}
	return 1;
}

int
mavlink_device::write_message(const mavlink_message_t& msg)
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	size_t m_length = mavlink_msg_to_send_buffer(buffer, &msg);
	int err = real_write(buffer, m_length);

	if (err != m_length) { 
		std::cerr << "Failed to forward the complete message ";
		std::cerr << err << " " << m_length << std::endl;
		return -1;
	}

	return 0;
}

mavlink_serial::mavlink_serial(const std::vector<std::string>& args):
	mavlink_device(args),
	_serial_port_name(args[1]),
	_serial_baud_rate(boost::lexical_cast<int>(args[2]))
{
	_fd = open(_serial_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (_fd < 0) 
		throw std::system_error(errno, std::system_category(), "Failed to open " + _serial_port_name);

	// Read file descritor configuration
	struct termios config;
	if(tcgetattr(_fd, &config) < 0)
		throw std::system_error(errno, std::system_category(), 
								"Failed to read configuration of " + _serial_port_name);

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	bool err;
	// Apply baudrate
	switch (_serial_baud_rate)
	{
		case 1200:
			err =  (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0);
			break;
		case 1800:
			err =  (cfsetispeed(&config, B1800) < 0 || cfsetospeed(&config, B1800) < 0);
			break;
		case 9600:
			err =  (cfsetispeed(&config, B9600) < 0 || cfsetospeed(&config, B9600) < 0);
			break;
		case 19200:
			err =  (cfsetispeed(&config, B19200) < 0 || cfsetospeed(&config, B19200) < 0);
			break;
		case 38400:
			err = (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0);
			break;
		case 57600:
			err = (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0);
			break;
		case 115200:
			err =  (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0);
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			err =  (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0);
			break;
		case 921600:
			err =  (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0);
			break;
		default:
			throw std::system_error(EINVAL, std::system_category(), 
									"Invalid baud rate " + boost::lexical_cast<std::string>(_serial_baud_rate));
			break;
	}

	if (err) 
		throw std::system_error(errno, std::system_category(), 
								"Failed to configure " + _serial_port_name + " to desired rate " +
								boost::lexical_cast<std::string>(_serial_baud_rate));

	// Finally, apply the configuration
	if(tcsetattr(_fd, TCSAFLUSH, &config) < 0)
		throw std::system_error(errno, std::system_category(), 
								"Could not configure " + _serial_port_name);
}

int 
mavlink_serial::real_read(char *data, size_t size)
{
	return read(_fd, data, size);
}

int
mavlink_serial::real_write(const uint8_t* data, size_t size)
{
	return write(_fd, data, size);
}

mavlink_serial::~mavlink_serial()
{
	close(_fd);
}

mavlink_udp::mavlink_udp(const std::vector<std::string>& args):
	mavlink_device(args)
{
	_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (_sock < 0) 
		throw std::system_error(errno, std::system_category(), 
									"Failed to create UDP socket");

	_remote_addr.sin_family = AF_INET;
	_remote_addr.sin_addr.s_addr = inet_addr(args[1].c_str());
	_remote_addr.sin_port = htons(boost::lexical_cast<int>(args[2]));

	int flags = fcntl(_sock, F_GETFL, 0);
	fcntl(_sock, F_SETFL, flags | O_NONBLOCK);

	if (args.size() == 4) {
		struct sockaddr_in   local_addr;
		local_addr.sin_family = AF_INET;
		local_addr.sin_addr.s_addr = INADDR_ANY;
		local_addr.sin_port = htons(boost::lexical_cast<int>(args[3]));
		if (bind(_sock, (const struct sockaddr*)& local_addr, sizeof(local_addr)) < 0)
			throw std::system_error(errno, std::system_category(),
					"Failed to bind UDP socket on port " + args[3]);

	}
}

int 
mavlink_udp::real_read(char* buf, size_t size)
{
	return recvfrom(_sock, buf, size, 0, 0, 0);
}

int
mavlink_udp::real_write(const uint8_t* buf, size_t size)
{
	return sendto(_sock, buf, size, 0, (const sockaddr*)&_remote_addr, sizeof(_remote_addr));
}

mavlink_udp::~mavlink_udp()
{
	close(_sock);
}

mavlink_device_t make_mavlink_device(const std::string& s)
{
	std::vector<std::string> args;
	boost::algorithm::split(args, s, boost::is_any_of(":"), boost::token_compress_on);

	if (args[0] == "none")
		return mavlink_device_t(new mavlink_none(args));
	if (args[0] == "udp")
		return mavlink_device_t(new mavlink_udp(args));
	if (args[0] == "serial")
		return mavlink_device_t(new mavlink_serial(args));
	return mavlink_device_t(nullptr);
}
