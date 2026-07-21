#ifndef HEX_FT_UDP_HPP
#define HEX_FT_UDP_HPP

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdint>

#pragma pack(1)

#define PORT_UDP 49152
#define CMD_STOP 0x0000 //Stop sending the output
#define CMD_START 0x0002 //Start sending the output
#define CMD_TARE 0x0042 //Set software bias
#define CMD_SET_FREQ 0x0082 //Set read-out speed

typedef int SOCKET_HANDLE;

struct FTResponse {
    uint32_t HS_sequence;
    uint32_t FT_sequence;
    uint32_t Status;
    int32_t ForceX, ForceY, ForceZ;
    int32_t TorqueX, TorqueY, TorqueZ;
};

struct UDPCommand {
    uint16_t header;
    uint16_t command;
    uint32_t data;
    uint8_t reserved[12] = {0};
};

class FTSensor {
private:
    SOCKET_HANDLE socketHandle;
    struct sockaddr_in addr;
    const double SCALE_FORCE = 10000.0;  // Escala fija para fuerzas (N)
    const double SCALE_TORQUE = 100000.0; // Escala fija para torques (Nm)

    void swapFTResponse(FTResponse& r) {
        r.HS_sequence = ntohl(r.HS_sequence);
        r.FT_sequence = ntohl(r.FT_sequence);
        r.Status = ntohl(r.Status);
        r.ForceX = ntohl(r.ForceX);
        r.ForceY = ntohl(r.ForceY);
        r.ForceZ = ntohl(r.ForceZ);
        r.TorqueX = ntohl(r.TorqueX);
        r.TorqueY = ntohl(r.TorqueY);
        r.TorqueZ = ntohl(r.TorqueZ);
    }

    bool sendCommand(uint16_t command, uint32_t data) {
        UDPCommand cmd = {0};
        cmd.header = ntohs(0x1234);
        cmd.command = ntohs(command);
        cmd.data = ntohl(data);

        if (sendto(socketHandle, (char*)&cmd, sizeof(cmd), 0, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            return false;
        }
        return true;
    }

public:
    FTSensor(const std::string& ip) {
        socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socketHandle < 0) throw std::runtime_error("Failed to create socket");

        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT_UDP);
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        if (addr.sin_addr.s_addr == INADDR_NONE) throw std::runtime_error("Invalid IP address");

        // Configurar frecuencia a 500 Hz (2 ms)
        if (!sendCommand(CMD_SET_FREQ, 2)) {
            close(socketHandle);
            throw std::runtime_error("Failed to set frequency");
        }

        // Iniciar salida
        if (!sendCommand(CMD_START, 0)) {
            close(socketHandle);
            throw std::runtime_error("Failed to start data output");
        }
    }

    ~FTSensor() {
        sendCommand(CMD_STOP, 0);
        close(socketHandle);
    }

    bool readFT(std::vector<double>& forces) {
        FTResponse response;
        socklen_t addrLen = sizeof(addr);

        if (recvfrom(socketHandle, (char*)&response, sizeof(response), 0, (struct sockaddr*)&addr, &addrLen) != sizeof(response)) {
            return false;
        }

        swapFTResponse(response);

        forces.resize(6);
        forces[0] = (response.ForceX < 0x80000000) ? (double)response.ForceX / SCALE_FORCE : -(double)(~response.ForceX) / SCALE_FORCE;
        forces[1] = (response.ForceY < 0x80000000) ? (double)response.ForceY / SCALE_FORCE : -(double)(~response.ForceY) / SCALE_FORCE;
        forces[2] = (response.ForceZ < 0x80000000) ? (double)response.ForceZ / SCALE_FORCE : -(double)(~response.ForceZ) / SCALE_FORCE;
        forces[3] = (response.TorqueX < 0x80000000) ? (double)response.TorqueX / SCALE_TORQUE : -(double)(~response.TorqueX) / SCALE_TORQUE;
        forces[4] = (response.TorqueY < 0x80000000) ? (double)response.TorqueY / SCALE_TORQUE : -(double)(~response.TorqueY) / SCALE_TORQUE;
        forces[5] = (response.TorqueZ < 0x80000000) ? (double)response.TorqueZ / SCALE_TORQUE : -(double)(~response.TorqueZ) / SCALE_TORQUE;

        return true;
    }

    bool tareSensor() {
        bool success = sendCommand(CMD_TARE, 255);
        if (success) {
            std::cout << "Tare command sent successfully\n";
        }
        return success;
    }
};

#endif