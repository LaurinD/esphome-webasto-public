#pragma once
#include "esphome.h"

class Webasto : public Component, public UARTDevice {
public:
    Webasto(ESP32ArduinoUARTComponent *parent);

    void VentOn(uint8_t t_on_mins);
    void VentOn();
    void HeatOn(uint8_t t_on_mins);
    void HeatOn();
    void Off();
    void KeepAlive();
    void get_state_50_03();
    void get_state_50_04();
    void get_state_50_05();
    void get_state_50_06();
    void get_state_50_07();

    void setup() override;
    void loop() override;

private:
    ESP32ArduinoUARTComponent *_uart_comp;
    void SendBreak();
    uint8_t checksum(uint8_t *buf, uint8_t len);
    bool tx_msg2(uint8_t* dat, uint8_t len);
    bool rx_msg2(uint8_t* dat, uint8_t len);
    void itob(uint8_t x, char *buf);

    uint8_t keep_alive_cmd = 0;
    unsigned long keep_alive_time = 0;
    unsigned long last_ok_rx;

    const uint8_t WBUS_CLIENT_ADDR = 0x0F;
    const uint8_t WBUS_HOST_ADDR   = 0x04;
    const uint8_t WBUS_CMD_OFF     = 0x10;
    const uint8_t WBUS_CMD_ON_PH   = 0x21;
    const uint8_t WBUS_CMD_ON_VENT = 0x22;
    const uint8_t WBUS_CMD_CHK     = 0x44;

public:
    struct state_50_03_t { bool heat_request, vent_request, bit3, bit4, combustion_fan, glowplug, fuel_pump, nozzle_heating; } state_50_03;
    struct state_50_04_t { float glowplug, fuel_pump, combustion_fan; } state_50_04;
    struct state_50_05_t { float temperature, voltage, glowplug_resistance; } state_50_05;
    struct state_50_06_t { float working_hours, operating_hours; uint16_t start_counter; } state_50_06;
    struct state_50_07_t { uint8_t op_state; } state_50_07;
};
