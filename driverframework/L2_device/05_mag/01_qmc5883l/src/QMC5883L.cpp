#include "QMC5883L.hpp"

QMC5883L::QMC5883L(int id)
{
    id_ = id;
}

bool QMC5883L::Init()
{
    // port_ = DeviceFactory::ShareInstance()->GetI2CDeviceById(id_, QMC5883L_I2C_SLAVE_ADDRESS);
    uint8_t who_am_i = 0x00;

    if (!ReadWhoAmI(&who_am_i) || who_am_i != 0xFF)
    {
        logger.Info("[Mag]read who am i fail 0x%x", who_am_i);
        return false;
    }
    logger.Info("[Mag]read who am i success 0x%x", who_am_i);

    logger.Info("[Mag]QMC5883L init....");
    if (!port_->WriteRegister(QMC5883L_CONFIG2, 0X80))
    {
        logger.Info("[Mag]QMC5883L init reset fail!!!!!!");
    }
    Timer::SleepMs(100);
    port_->WriteRegister(QMC5883L_RESET, 0X01);

    oversampling = QMC5883L_CONFIG_OS512;
    range = QMC5883L_CONFIG_8GAUSS;
    rate = QMC5883L_CONFIG_100HZ;
    mode = QMC5883L_CONFIG_CONT;
    port_->WriteRegister(QMC5883L_CONFIG, oversampling | range | rate | mode);

    uint8_t read_config = 0;
    port_->ReadRegister(QMC5883L_CONFIG, &read_config);
    logger.Info("[Mag]read config is 0x%x", read_config);

    // if (mag_post_to_ap_socket_ == nullptr)
    // {
    //     char buffer[100];
    //     mag_post_to_ap_socket_ = dspd::Socket::Create();
    //     sprintf(buffer, "ipc:///tmp/app_%d.sock", static_cast<uint8_t>(NotifyMessageType::MagData));
    //     mag_post_to_ap_socket_->Bind(buffer);
    // }
    return true;
}

bool QMC5883L::ReadWhoAmI(uint8_t *who_am_i)
{
    return port_->ReadRegister(0x0D, who_am_i);
}

bool QMC5883L::Read(MagData &data)
{
    data.id = id_;
    uint8_t mag_buffer[6];
    data.timestamp_ms = Timer::Now();
    Timer::SleepMs(6);

    port_->ReadRegisters(QMC5883L_X_LSB, mag_buffer, 6);

    data.value_x = LittleEndian::Int16FromLittleEndian(mag_buffer);
    data.value_y = LittleEndian::Int16FromLittleEndian(mag_buffer + 2);
    data.value_z = LittleEndian::Int16FromLittleEndian(mag_buffer + 4);
    // logger.Info("[Mag]port return is %d", port_return);
    mag_post_to_ap_socket_->Write((uint8_t *)&data, sizeof(MagData));
    return true;
}

bool QMC5883L::WaitForReady(int max_wait_time_ms)
{
    uint8_t status;
    int gap_time_ms = 1;
    int try_times = max_wait_time_ms / gap_time_ms;
    int i = 0;
    for (i = 0; i < try_times; i++)
    {
        Timer::SleepMs(1);
        if (!port_->ReadRegister(QMC5883L_STATUS, &status))
        {
            logger.Error("QMC5883L read status fail");
            return false;
        }
        if (status & 0x01)
        {
            break;
        }
    }
    if (i >= try_times)
    {
        return false;
    }

    return true;
}

void QMC5883L::SoftReset()
{
    if (!port_->WriteRegister(QMC5883L_RESET, 0X01))
    {
        logger.Info("[Mag]QMC5883L reset fail");
        // return false;
    }
    if (!port_->WriteRegister(QMC5883L_CONFIG, oversampling | range | rate | mode))
    {
        logger.Info("[Mag]QMC5883L config fail");
        // return false;
    }
}

bool QMC5883L::Deinit()
{
    // port_ = DeviceFactory::ShareInstance()->GetI2CDeviceById(id_, QMC5883L_I2C_SLAVE_ADDRESS);
    uint8_t who_am_i = 0x00;
    if (!ReadWhoAmI(&who_am_i) || who_am_i != 0xFF)
    {
        logger.Info("[Mag Deinit]read who am i fail 0x%x", who_am_i);
        return false;
    }
    port_->WriteRegister(QMC5883L_CONFIG, 0x00);  // Enter Standby Mode
    return true;
}