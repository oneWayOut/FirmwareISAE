#ifndef _MAVLINK_DEVICE_H_
#define _MAVLINK_DEVICE_H_

#include <vector>
#include <memory>

#include <mavlink.h>

#include <netinet/in.h>

class mavlink_device 
{
	private:
		char _buffer[1024];
		size_t _read_idx, _end_idx;

	protected:
		virtual int real_read(char* data, size_t size) = 0;
		virtual int real_write(const uint8_t* data, size_t size) = 0;

	public:
		mavlink_device(const std::vector<std::string>&) :
			_read_idx(0), _end_idx(0){};
		virtual int read_message(mavlink_message_t& msg);
		virtual int write_message(const mavlink_message_t& msg); 
		virtual ~mavlink_device() {};
};

typedef std::unique_ptr<mavlink_device> mavlink_device_t;

class mavlink_none  : public mavlink_device
{
	protected:
		int real_read(char* data, size_t size) override { return 0; }
		int real_write(const uint8_t* data, size_t size) override { return 0; }
	public:
		mavlink_none(const std::vector<std::string>& args) : mavlink_device(args) {};
		int read_message(mavlink_message_t& msg) override { return 0; }
		int write_message(const mavlink_message_t& msg) override { return 0; }
};


class mavlink_serial : public mavlink_device 
{
	private:
		int _fd;
		std::string _serial_port_name;
		int _serial_baud_rate;
	protected:
		int real_read(char* data, size_t size) override; 
		int real_write(const uint8_t* data, size_t size) override;
	public:
		mavlink_serial(const std::vector<std::string>&);
		~mavlink_serial() override;
};

class mavlink_udp : public mavlink_device
{
	private:
		int _sock;
		struct sockaddr_in   _remote_addr;

	protected:
		int real_read(char* data, size_t size) override; 
		int real_write(const uint8_t* data, size_t size) override;
	public:
		mavlink_udp(const std::vector<std::string>&);
		~mavlink_udp() override;
};

mavlink_device_t make_mavlink_device(const std::string& s);

#endif /* _MAVLINK_DEVICE_H_ */
