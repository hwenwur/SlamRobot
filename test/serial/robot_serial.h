#ifndef ROBOT_SERIAL_H
#define ROBOT_SERIAL_H
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <string>

/* 
    Very simple header-only serial library.
    Author: hwenwur
    Date: 2021/01/09

    How to debug: strace -P /dev/ttyUSB0 -ewrite,read -x -p <PID>
 */
namespace robotserial
{
    enum class SerialStatus
    {
        OK,                // 打开成功
        NO_SUCH_FILE,      // 设备文件不存在
        PERMISSION_DENIED, // 权限不足，把当前用户加入 dialout 组，或使用 root 运行
        FAILED,            // 其他错误
        CLOSED,            // 已关闭
        UNSET
    };
    /* 
        convert literal baudrate to termios flag.
        such as when literal_val=9600 return B9600.
     */
    inline unsigned int convert_literal_baud(unsigned int literal_val)
    {
        switch (literal_val)
        {
        case 9600:
            return B9600;
        case 38400:
            return B38400;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 1152000:
            return B1152000;
        default:
            std::cerr << "unsupport baud rate: " << literal_val << "\n";
            return B9600;
        }
    }
    class Serial
    {
    public:
        Serial(const std::string &path, unsigned int baudRate = 9600);
        Serial(const Serial &val) = delete;
        ~Serial();
        int open(bool defaultSetting = false);
        bool isOpen();
        SerialStatus getStatus();
        int setup();
        int close();
        int write(const void *data, size_t count);
        bool writeAll(const void *data, size_t count);
        int read(void *data, size_t count);
        void setBlocking(bool m);

    private:
        std::string path;
        unsigned int baudRate;
        SerialStatus status;
        int fd;
        bool blocking; // 阻塞模式，默认 true
    };

    // end of header file.

    inline Serial::Serial(const std::string &path, unsigned int baudRate) : path(path),
                                                                            status(SerialStatus::UNSET),
                                                                            blocking(true)
    {
        this->baudRate = convert_literal_baud(baudRate);
    }
    inline Serial::~Serial()
    {
        Serial::close();
    }

    /* 
        @defaultSetting: use exists termios setting in OS.
     */
    inline int Serial::open(bool defaultSetting)
    {
        fd = ::open(path.c_str(), O_RDWR);
        if (fd < 0)
        {
            switch (errno)
            {
            case 1:
                status = SerialStatus::PERMISSION_DENIED;
                break;
            case 2:
                status = SerialStatus::NO_SUCH_FILE;
                break;
            default:
                status = SerialStatus::FAILED;
                break;
            }
            std::cerr << "Error: " << strerror(errno) << "\n";
            return 1;
        }
        if (defaultSetting)
        {
            return 0;
        }
        if (setup() == 0)
        {
            status = SerialStatus::OK;
            return 0;
        }
        else
        {
            status = SerialStatus::FAILED;
            return 1;
        }
    }
    inline int Serial::setup()
    {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0)
        {
            std::cerr << "Error: " << strerror(errno)
                      << "\n";
            return 1;
        }

        cfsetispeed(&tty, baudRate);
        cfsetospeed(&tty, baudRate);

        // 以下设置参考： https://man7.org/linux/man-pages/man1/stty.1.html

        tty.c_cc[VTIME] = 1;
        tty.c_cc[VMIN] = blocking ? 1 : 0;

        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag &= ~HUPCL;

        tty.c_iflag |= IGNBRK;
        tty.c_iflag &= ~(BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

        tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ECHOCTL | ECHOKE | ECHOK | ISIG | ICANON | IEXTEN);

        tty.c_oflag &= ~(OPOST | ONLCR);

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            std::cerr << "Error: " << strerror(errno)
                      << "\n";
            return 1;
        }
        return 0;
    }
    inline bool Serial::isOpen()
    {
        return status == SerialStatus::OK;
    }
    inline SerialStatus Serial::getStatus()
    {
        return status;
    }
    inline int Serial::close()
    {
        status = SerialStatus::CLOSED;
        return ::close(fd);
    }
    inline int Serial::write(const void *data, size_t count)
    {
        // 可能在 data 没有完全写入之前就返回，
        // 比如缓冲区满或收到 Ctrl + C 等信号后。
        return ::write(fd, data, count);
    }
    inline int Serial::read(void *data, size_t count)
    {
        return ::read(fd, data, count);
    }
    inline bool Serial::writeAll(const void *data, size_t count)
    {
        int r = Serial::write(data, count);
        if (r < 0)
        {
            std::cerr << "Error: " << strerror(errno) << "\n";
            return false;
        }
        else if (r < count)
        {
            Serial::writeAll((char *)data + r, count - r);
        }
        else
        {
            return true;
        }
    }
    inline void Serial::setBlocking(bool m)
    {
        if (isOpen())
        {
            std::cerr << "You must call setBlocking before open()";
        }
        blocking = m;
    }
} // namespace robotserial

#endif