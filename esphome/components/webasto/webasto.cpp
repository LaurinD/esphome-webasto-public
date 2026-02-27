#include "webasto.h"

static const char *const TAG = "Webasto";

Webasto::Webasto(ESP32ArduinoUARTComponent *parent) : UARTDevice(parent), _uart_comp(parent) {
    last_ok_rx = millis() - 30000;
}

// ----- Hilfsfunktionen -----
void Webasto::itob(uint8_t x, char *buf) {
    unsigned char *ptr = (unsigned char *)&x;
    int pos = 0;
    for (int i = sizeof(uint8_t)-1; i>=0; i--)
        for (int j = CHAR_BIT-1; j>=0; j--)
            buf[pos++] = '0' + !!(ptr[i] & 1U << j);
    buf[pos]='\0';
}

void Webasto::SendBreak() {
    unsigned long now = millis();
    if(now - last_ok_rx >= 30000) {
        ESP_LOGD(TAG, "SendBreak");
        _uart_comp->flush();
        uint8_t waste;
        while(_uart_comp->available() > 0) _uart_comp->read_byte(&waste);

        HardwareSerial *_hw_serial = _uart_comp->get_hw_serial();
        _hw_serial->updateBaudRate(300);
        _uart_comp->write_byte(0b10000000);
        _uart_comp->flush();

        unsigned long rxstr = millis();
        while((millis()-rxstr)<50){
            if(_uart_comp->available()){ _uart_comp->read_byte(&waste); break; }
            else delay(1);
        }

        _hw_serial->updateBaudRate(2400);
    } else {
        ESP_LOGD(TAG, "SendBreak not needed, last good rx before: %lu ms", now - last_ok_rx);
    }
}

uint8_t Webasto::checksum(uint8_t *buf, uint8_t len) {
    uint8_t chk = 0;
    for(; len!=0; len--) chk ^= *buf++;
    return chk;
}

bool Webasto::tx_msg2(uint8_t* dat, uint8_t len) {
    ESP_LOGD(TAG,"tx_msg start");
    uint8_t txbuf[40];
    txbuf[0] = ((WBUS_CLIENT_ADDR<<4)|WBUS_HOST_ADDR);
    txbuf[1] = len+1;
    uint8_t txcnt=2;
    for(uint8_t i=0;i<len;) txbuf[txcnt++]=dat[i++];
    uint8_t chks=0;
    for(uint8_t i=0;i<txcnt;i++) chks^=txbuf[i];
    txbuf[txcnt++]=chks;

    char logbuf[130]; logbuf[0]='\0';
    for(uint8_t i=0;i<txcnt;i++) sprintf(logbuf,"%s %02X",logbuf,txbuf[i]);
    ESP_LOGD(TAG,"TX: %s",logbuf);

    uint8_t waste;
    while(_uart_comp->available()>0) _uart_comp->read_byte(&waste);
    _uart_comp->write_array(&txbuf[0], txcnt);
    _uart_comp->flush();

    uint8_t rxbuf[40]; uint8_t rxcnt=0; unsigned long rxstr=millis();
    while(rxcnt<txcnt && (millis()-rxstr)<100){
        if(_uart_comp->available()) rxbuf[rxcnt++]=_uart_comp->read_byte(&rxbuf[rxcnt]);
        else delay(1);
    }

    logbuf[0]='\0';
    for(uint8_t i=0;i<rxcnt;i++) sprintf(logbuf,"%s %02X",logbuf,rxbuf[i]);
    ESP_LOGD(TAG,"RX: %s",logbuf);

    bool ok = txcnt==rxcnt;
    for(uint8_t i=0;ok && i<txcnt;i++) ok=txbuf[i]==rxbuf[i];
    if(ok) ESP_LOGD(TAG,"tx_msg done: ok"); else ESP_LOGD(TAG,"tx_msg done: error");
    return ok;
}

bool Webasto::rx_msg2(uint8_t* dat, uint8_t len) {
    ESP_LOGD(TAG,"rx_msg start");
    uint8_t rx_len=len+3;
    uint8_t rxbuf[40]; uint8_t rxcnt=0;
    unsigned long rxstr=millis();
    long delta=0;
    while(rxcnt<rx_len && delta<200){
        if(_uart_comp->available()) rxbuf[rxcnt++]=_uart_comp->read_byte(&rxbuf[rxcnt]);
        else delay(1);
        delta=millis()-rxstr;
    }
    bool ok_time = delta<200;
    bool ok_rxcnt = rxcnt==rx_len;
    uint8_t chks=0; for(uint8_t i=0;i<rxcnt-1;i++) chks^=rxbuf[i];
    bool ok_chks = rxbuf[rxcnt-1]==chks;
    bool ok_addr = rxbuf[0]==((WBUS_HOST_ADDR<<4)|WBUS_CLIENT_ADDR);
    bool ok_len = rxbuf[1]==rxcnt-2;

    bool ok = ok_time && ok_rxcnt && ok_addr && ok_len && ok_chks;
    if(ok) memcpy(&dat[0], &rxbuf[2], len);
    if(ok) last_ok_rx=millis();
    ESP_LOGD(TAG,"rx_msg done: %s", ok?"ok":"error");
    return ok;
}

// ----- Befehle -----
void Webasto::VentOn(uint8_t t_on_mins) {
    ESP_LOGD(TAG,"Send VentOn");
    uint8_t tx_dat[] = {WBUS_CMD_ON_VENT, t_on_mins};
    uint8_t rx_dat[sizeof(tx_dat)];
    for(uint8_t i=0;i<3;i++){
        SendBreak();
        if(!tx_msg2(tx_dat,sizeof(tx_dat))){ ESP_LOGE(TAG,"VentOn !tx_ok"); continue; }
        if(!rx_msg2(rx_dat,sizeof(rx_dat))){ ESP_LOGE(TAG,"VentOn !rx_ok"); continue; }
        if((tx_dat[0]|0x80)!=rx_dat[0]){ ESP_LOGE(TAG,"VentOn !cmd_ok"); continue; }
        if(tx_dat[1]!=rx_dat[1]){ ESP_LOGE(TAG,"VentOn !subcmd_ok"); continue; }
        keep_alive_cmd=WBUS_CMD_ON_VENT;
        keep_alive_time=(unsigned long)t_on_mins*60*1000;
        break;
    }
}
void Webasto::VentOn(){ VentOn(1); }

void Webasto::HeatOn(uint8_t t_on_mins) {
    ESP_LOGD(TAG,"Send HeatOn");
    uint8_t tx_dat[] = {WBUS_CMD_ON_PH, t_on_mins};
    uint8_t rx_dat[sizeof(tx_dat)];
    for(uint8_t i=0;i<3;i++){
        SendBreak();
        if(!tx_msg2(tx_dat,sizeof(tx_dat))){ ESP_LOGE(TAG,"HeatOn !tx_ok"); continue; }
        if(!rx_msg2(rx_dat,sizeof(rx_dat))){ ESP_LOGE(TAG,"HeatOn !rx_ok"); continue; }
        if((tx_dat[0]|0x80)!=rx_dat[0]){ ESP_LOGE(TAG,"HeatOn !cmd_ok"); continue; }
        if(tx_dat[1]!=rx_dat[1]){ ESP_LOGE(TAG,"HeatOn !subcmd_ok"); continue; }
        keep_alive_cmd=WBUS_CMD_ON_PH;
        keep_alive_time=(unsigned long)t_on_mins*60*1000;
        break;
    }
}
void Webasto::HeatOn(){ HeatOn(1); }

void Webasto::Off() {
    ESP_LOGD(TAG,"Send Off");
    uint8_t tx_dat[]={WBUS_CMD_OFF};
    uint8_t rx_dat[sizeof(tx_dat)];
    for(uint8_t i=0;i<3;i++){
        SendBreak();
        if(!tx_msg2(tx_dat,sizeof(tx_dat))){ ESP_LOGE(TAG,"Off !tx_ok"); continue; }
        if(!rx_msg2(rx_dat,sizeof(rx_dat))){ ESP_LOGE(TAG,"Off !rx_ok"); continue; }
        if((tx_dat[0]|0x80)!=rx_dat[0]){ ESP_LOGE(TAG,"Off !cmd_ok"); continue; }
        keep_alive_cmd=0;
        keep_alive_time=0;
        break;
    }
}

void Webasto::KeepAlive() {
    const unsigned long periode=10000;
    unsigned long now=millis();
    static unsigned long last=now-periode;
    if(now-last>=periode){
        last+=periode;
        if(keep_alive_cmd>0 && keep_alive_time>0){
            ESP_LOGD(TAG,"Send KeepAlive");
            uint8_t tx_dat[]={WBUS_CMD_CHK, keep_alive_cmd, 0};
            uint8_t rx_dat[2];
            for(uint8_t i=0;i<3;i++){
                SendBreak();
                if(!tx_msg2(tx_dat,sizeof(tx_dat))){ ESP_LOGE(TAG,"KeepAlive !tx_ok"); continue; }
                if(!rx_msg2(rx_dat,sizeof(rx_dat))){ ESP_LOGE(TAG,"KeepAlive !rx_ok"); continue; }
                if((tx_dat[0]|0x80)!=rx_dat[0]){ ESP_LOGE(TAG,"KeepAlive !cmd_ok"); continue; }
                if(keep_alive_time>periode) keep_alive_time-=periode;
                else keep_alive_time=0;
                break;
            }
        }
        if(keep_alive_cmd>0 && keep_alive_time<30000){
            ESP_LOGD(TAG,"Send ReNew");
            if(keep_alive_cmd==WBUS_CMD_ON_VENT) VentOn(1);
            if(keep_alive_cmd==WBUS_CMD_ON_PH) HeatOn(1);
        }
    }
}

// ----- State lesen -----
// ----- State lesen -----
void Webasto::get_state_50_03() {
    uint8_t dat[1]={0x50};
    uint8_t rx[1];
    SendBreak();
    if(!tx_msg2(dat,1)){ ESP_LOGE(TAG,"get_state_50_03 !tx"); return; }
    if(!rx_msg2(rx,1)){ ESP_LOGE(TAG,"get_state_50_03 !rx"); return; }

    state_50_03.heat_request    = rx[0] & 0x01;
    state_50_03.vent_request    = rx[0] & 0x02;
    state_50_03.bit3            = rx[0] & 0x04;
    state_50_03.bit4            = rx[0] & 0x08;
    state_50_03.combustion_fan  = rx[0] & 0x10;
    state_50_03.glowplug        = rx[0] & 0x20;
    state_50_03.fuel_pump       = rx[0] & 0x40;
    state_50_03.nozzle_heating  = rx[0] & 0x80;
}

void Webasto::get_state_50_04() {
    uint8_t dat[1]={0x50};
    uint8_t rx[3];
    SendBreak();
    if(!tx_msg2(dat,1)){ ESP_LOGE(TAG,"get_state_50_04 !tx"); return; }
    if(!rx_msg2(rx,3)){ ESP_LOGE(TAG,"get_state_50_04 !rx"); return; }

    state_50_04.glowplug       = rx[0];
    state_50_04.fuel_pump      = rx[1];
    state_50_04.combustion_fan = rx[2];
}

void Webasto::get_state_50_05() {
    uint8_t dat[1]={0x50};
    uint8_t rx[6];
    SendBreak();
    if(!tx_msg2(dat,1)){ ESP_LOGE(TAG,"get_state_50_05 !tx"); return; }
    if(!rx_msg2(rx,6)){ ESP_LOGE(TAG,"get_state_50_05 !rx"); return; }

    state_50_05.temperature      = ((rx[0]<<8)|rx[1]) * 0.1;
    state_50_05.voltage          = ((rx[2]<<8)|rx[3]) * 0.01;
    state_50_05.glowplug_resistance = ((rx[4]<<8)|rx[5]) * 0.1;
}

void Webasto::get_state_50_06() {
    uint8_t dat[1]={0x50};
    uint8_t rx[5];
    SendBreak();
    if(!tx_msg2(dat,1)){ ESP_LOGE(TAG,"get_state_50_06 !tx"); return; }
    if(!rx_msg2(rx,5)){ ESP_LOGE(TAG,"get_state_50_06 !rx"); return; }

    state_50_06.working_hours   = (rx[0]<<8 | rx[1]) * 0.1;
    state_50_06.operating_hours = (rx[2]<<8 | rx[3]) * 0.1;
    state_50_06.start_counter   = rx[4];
}

void Webasto::get_state_50_07() {
    uint8_t dat[1]={0x50};
    uint8_t rx[1];
    SendBreak();
    if(!tx_msg2(dat,1)){ ESP_LOGE(TAG,"get_state_50_07 !tx"); return; }
    if(!rx_msg2(rx,1)){ ESP_LOGE(TAG,"get_state_50_07 !rx"); return; }

    state_50_07.op_state = rx[0];
}

void Webasto::setup() {}

void Webasto::loop() {
    static uint8_t state=0;
    switch(state++){
        case 0: KeepAlive(); break;
        case 1: get_state_50_03(); break;
        case 2: get_state_50_04(); break;
        case 3: get_state_50_05(); break;
        case 4: get_state_50_06(); break;
        case 5: get_state_50_07(); break;
        default: state=0; break;
    }

    char logbuf[130]; logbuf[0]='\0';
    while(_uart_comp->available()){
        uint8_t rx;
        _uart_comp->read_byte(&rx);
        sprintf(logbuf,"%s %02X",logbuf,rx);
    }
    if(logbuf[0]!='\0') ESP_LOGD(TAG,"%010ld, RX: %s",millis(),logbuf);
}
