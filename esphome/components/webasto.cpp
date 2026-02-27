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
void Webasto::VentOn(uint8_t t_on_mins){ /* Code aus Header übernehmen */ }
void Webasto::VentOn(){ VentOn(1); }
void Webasto::HeatOn(uint8_t t_on_mins){ /* Code aus Header übernehmen */ }
void Webasto::HeatOn(){ HeatOn(1); }
void Webasto::Off(){ /* Code aus Header übernehmen */ }
void Webasto::KeepAlive(){ /* Code aus Header übernehmen */ }

// ----- State lesen -----
void Webasto::get_state_50_03(){ /* Code aus Header übernehmen */ }
void Webasto::get_state_50_04(){ /* Code aus Header übernehmen */ }
void Webasto::get_state_50_05(){ /* Code aus Header übernehmen */ }
void Webasto::get_state_50_06(){ /* Code aus Header übernehmen */ }
void Webasto::get_state_50_07(){ /* Code aus Header übernehmen */ }

void Webasto::setup(){}
void Webasto::loop(){ /* Code aus Header übernehmen */ }
