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
    class Serial
    {
    public:
        Serial(const std::string &path, unsigned int baudRate = B9600);
        ~Serial();
        int open();
        bool isOpen();
        SerialStatus getStatus();
        int setup();
        int close();
        int write(const void *data, size_t count);
        bool writeAll(const void *data, size_t count);
        int read(void *data, size_t count);

    private:
        std::string path;
        unsigned int baudRate;
        SerialStatus status;
        int fd;
    };

    // end of header file.

    Serial::Serial(const std::string &path, unsigned int baudRate) : path(path), baudRate(baudRate), status(SerialStatus::UNSET) {}
    Serial::~Serial()
    {
        Serial::close();
    }
    int Serial::open()
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
    int Serial::setup()
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

        tty.c_cc[VTIME] = 5;
        tty.c_cc[VMIN] = 1;

        tty.c_cflag |= CRTSCTS;
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
    bool Serial::isOpen()
    {
        return status == SerialStatus::OK;
    }
    SerialStatus Serial::getStatus()
    {
        return status;
    }
    int Serial::close()
    {
        status = SerialStatus::CLOSED;
        return ::close(fd);
    }
    int Serial::write(const void *data, size_t count)
    {
        // 可能在 data 没有完全写入之前就返回，
        // 比如缓冲区满或收到 Ctrl + C 等信号后。
        return ::write(fd, data, count);
    }
    int Serial::read(void *data, size_t count)
    {
        return ::read(fd, data, count);
    }
    bool Serial::writeAll(const void *data, size_t count)
    {
        int r = ::write(fd, data, count);
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
} // namespace robotserial

#endif
