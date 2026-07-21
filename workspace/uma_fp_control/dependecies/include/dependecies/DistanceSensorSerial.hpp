#ifndef DISTANCE_SENSOR_SERIAL_HPP
#define DISTANCE_SENSOR_SERIAL_HPP

#include <string>
#include <stdexcept>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cstdlib>

class DistanceSensorSerial
{
private:
    int serialFd = -1;
    std::string lineBuffer; // 🔥 acumula entre llamadas
    void configureSerial(speed_t baud)
    {
        struct termios tty;
        std::memset(&tty, 0, sizeof(tty));

        if (tcgetattr(serialFd, &tty) != 0)
        {
            throw std::runtime_error("Error getting serial config");
        }

        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);

        // 8N1
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

        // raw mode
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;

        // 🔥 NO BLOQUEO REAL
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serialFd, TCSANOW, &tty) != 0)
        {
            throw std::runtime_error("Error setting serial config");
        }
    }

public:

    DistanceSensorSerial(const std::string& device, speed_t baud = B115200)
    {
        serialFd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

        std::cout << "Opening serial port..." << std::endl;

        if (serialFd < 0)
            throw std::runtime_error("Cannot open serial port");

        configureSerial(baud);

        // limpiar basura inicial Arduino
        tcflush(serialFd, TCIOFLUSH);
        usleep(2000000); // 2s para reset Arduino
    }

    ~DistanceSensorSerial()
    {
        if (serialFd >= 0)
            close(serialFd);
    }

    bool readDistance(double& distance)
    {
        char c;
        // Acumula bytes disponibles ahora mismo en el buffer persistente
        while (true)
        {
            int n = read(serialFd, &c, 1);
            if (n <= 0) break;          // no hay más bytes por ahora
            if (c == '\r') continue;    // ignora \r
            if (c == '\n')
            {
                // línea completa
                std::string line = lineBuffer;
                lineBuffer.clear();
                if (line.empty()) continue; // línea vacía, sigue
                try {
                    distance = std::stod(line);
                    return true;
                } catch (...) {
                    return false;
                }
            }
            lineBuffer += c;
        }
        return false; // no hay línea completa todavía
    }
};

#endif