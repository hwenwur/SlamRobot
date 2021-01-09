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
 */
namespace robotserial
{
    enum class SerialStatus
    {
        OK,                // 打开成功
        NO_SUCH_FILE,      // 设备文件不存在
        PERMISSION_DENIED, // 权限不足，把当前用户加入 dialout 组，或使用 root 运行
        FAILED,            // 其他错误
        UNSET
    };
    class Serial
    {
    public:
        Serial(const std::string &path, unsigned int baudRate = B9600);
        int open();
        bool isOpen();
        SerialStatus getStatus();
        int setup();
        int close();
        int write(const void *data, size_t count);
        int read(void *data, size_t count);

    private:
        std::string path;
        unsigned int baudRate;
        SerialStatus status;
        int fd;
    };

    Serial::Serial(const std::string &path, unsigned int baudRate) : path(path), baudRate(baudRate), status(SerialStatus::UNSET) {}
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
        tty.c_cc[VTIME] = 5;
        tty.c_cc[VMIN] = 1;

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
        return ::close(fd);
    }
    int Serial::write(const void *data, size_t count)
    {
        return ::write(fd, data, count);
    }
    int Serial::read(void *data, size_t count)
    {
        return ::read(fd, data, count);
    }
} // namespace robotserial
