#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <string>

namespace robotserial
{
    class Serial
    {
    public:
        Serial(const std::string &path, unsigned int baudRate = B9600);
        int open();
        bool isOpen();
        int setup();
        int close();
        int write(const void *data, size_t count);
        int read(void *data, size_t count);

    private:
        std::string path;
        unsigned int baudRate;
        // Current status: (O)K, (E)rror, (U)nset
        char status;
        int fd;
    };

    Serial::Serial(const std::string &path, unsigned int baudRate) : path(path), baudRate(baudRate), status('U') {}
    int Serial::open()
    {
        fd = ::open(path.c_str(), O_RDWR);
        if (fd < 0)
        {
            status = 'E';
            std::cerr << "Error: " << strerror(errno) << "\n";
        }
        if (setup() == 0)
        {
            status = 'O';
        }
        else
        {
            status = 'E';
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
        tty.c_cc[VTIME] = 1;
        tty.c_cc[VMIN] = 0;

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
        return status == 0;
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
