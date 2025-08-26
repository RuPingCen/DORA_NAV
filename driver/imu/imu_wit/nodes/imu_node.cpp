#include "serial.h"
#include "wit_c_sdk.h"
#include "REG.h"
#include "imu_msg.h"
#include <dora/node_api.h>
#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#define ACC_UPDATE        0x01
#define GYRO_UPDATE       0x02
#define ANGLE_UPDATE      0x04
#define MAG_UPDATE        0x08
#define TEMP_UPDATE       0x10
#define READ_UPDATE       0x80

static int fd, s_iCurBaud = 230400;
static volatile char s_cDataUpdate = 0;

const int c_uiBaud[] = {2400 , 4800 , 9600 , 19200 , 38400 , 57600 , 115200 , 230400 , 460800 , 921600};

static int AutoScanSensor(const char* dev);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void SensorUartSend(uint8_t *p_data , uint32_t uiSize);
static void Delayms(uint16_t ucMs);

uint64_t timestamp_now() {
    using namespace std::chrono;
    using time_stamp = time_point<std::chrono::system_clock,
                                  std::chrono::microseconds>;
    time_stamp stamp = time_point_cast<microseconds>(system_clock::now());
    return stamp.time_since_epoch().count();
}

void run_once(void *dora_context) {
    unsigned char cBuff[1];

    WitReadReg(AX, 16);
    
    while(serial_read_data(fd, cBuff, 1)) {
        WitSerialDataIn(cBuff[0]);
    }

    if (s_cDataUpdate) {
        float fAcc[3], fGyro[3];
        cslam::ImuMsg imu_msg;
        imu_msg.stamp = timestamp_now();

        for(int i = 0; i < 3; i++) {
            fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
            fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
        }

        if (s_cDataUpdate & ACC_UPDATE) {
            // X、Y 和 Z 轴的加速度值
            imu_msg.linear_acceleration.x = fAcc[0] * 9.80665;
            imu_msg.linear_acceleration.y = fAcc[1] * 9.80665;
            imu_msg.linear_acceleration.z = fAcc[2] * 9.80665;
            s_cDataUpdate &= ~ACC_UPDATE;
        }
        if (s_cDataUpdate & GYRO_UPDATE) {
            // X、Y 和 Z 轴的角速度值
            imu_msg.angular_velocity.x = fGyro[0] * M_PI/180;  //rad/s
            imu_msg.angular_velocity.y = fGyro[1] * M_PI/180;
            imu_msg.angular_velocity.z = fGyro[2] * M_PI/180;
            s_cDataUpdate &= ~GYRO_UPDATE;
        }

        char out_id[] = "imu_msg";
        spdlog::trace("sending imu msg - stamp: {}", imu_msg.stamp);
        int err = dora_send_output(dora_context,
                                   out_id, sizeof(out_id) - 1,
                                   (char *)&imu_msg, sizeof(cslam::ImuMsg));
        if (err) {
            spdlog::error("failed to send output");
        }

        s_cDataUpdate = 0;
    }
}

void run(void *dora_context) {
    while (true) {
        void *event = dora_next_event(dora_context);
        if (event == nullptr) {
            spdlog::error("unexpected end of event");
            return;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input) {
            char *data, *data_id;
            size_t data_len, data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            
            fmt::string_view input_id(data_id, data_id_len);
            spdlog::trace("received input - id: {}", input_id);
            if (input_id == "tick") {
                run_once(dora_context);
            }
        } else if (ty == DoraEventType_Stop) {
            spdlog::info("received stop event");
            free_dora_event(event);
            return;
        } else {
            spdlog::error("received unexpected event: {}", ty);
        }

        free_dora_event(event);
    }
}

int main() {
    spdlog::cfg::load_env_levels();

    const char *dev = getenv("DEV");
    if (dev == nullptr) {
        spdlog::error("DEV not set");
        return -1;
    }

    int fd = serial_open(dev, s_iCurBaud);
    if (fd < 0) {
        spdlog::error("failed to open {}: {}", dev, strerror(errno));
        return fd;
    }

    WitInit(WIT_PROTOCOL_905x_MODBUS, 0xff);
    WitRegisterCallBack(CopeSensorData);
    WitSerialWriteRegister(SensorUartSend);
    int err = AutoScanSensor(dev);
    if (err) {
        return err;
    }

    void *dora_context = init_dora_context_from_env();
    run(dora_context);

    serial_close(fd);
    free_dora_context(dora_context);
    return 0;
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) {
    for (uint32_t i = 0; i < uiRegNum; i++) {
        switch(uiReg) {
            case AZ:
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case GZ:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            case HZ:
                s_cDataUpdate |= MAG_UPDATE;
                break;
            case Yaw:
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            case TEMP905x:
                s_cDataUpdate |= TEMP_UPDATE;
                break;
            default:
                s_cDataUpdate |= READ_UPDATE;
                break;
        }
        uiReg++;
    }
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
    uint32_t uiDelayUs = ((1000000/(s_iCurBaud/10)) * uiSize) + 300;
    serial_write_data(fd, p_data, uiSize);
    usleep(uiDelayUs);
}

static int AutoScanSensor(const char* dev) {
    char cBuff[1];
    for (size_t i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++) {
        serial_close(fd);

        s_iCurBaud = c_uiBaud[i];
        fd = serial_open(dev, s_iCurBaud);
        if (fd < 0) {
            spdlog::error("failed to open {}: {}", dev, strerror(errno));
            return fd;
        }

        int iRetry = 2;
        s_cDataUpdate = 0;
        do {
            WitReadReg(AX, 3);
            Delayms(200);
            while(serial_read_data(fd, cBuff, 1)) {
               WitSerialDataIn(cBuff[0]);
            }
            
            if(s_cDataUpdate != 0) {
                spdlog::info("{} baud find sensor", c_uiBaud[i]);
                return 0;
            }
            
            iRetry--;
        } while(iRetry);
    }

    spdlog::error("can not find sensor, please check your connection");
    return -1;
}

static void Delayms(uint16_t ucMs) {
    usleep(ucMs*1000);
}