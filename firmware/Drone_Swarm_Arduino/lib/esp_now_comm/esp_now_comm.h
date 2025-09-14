/*
 * esp_now_comm.h
 *
 *  Created on: May 29, 2025
 *      Author: Ti Manh
 */
#pragma once

#include <WiFi.h>
#include <esp_now.h>

class EspNowComm {
public:
    // Khởi tạo đối tượng
    bool init();

    // Truyền dữ liệu (struct data) tới địa chỉ mac
    esp_now_peer_info_t peerInfo;

    template<typename T>
    bool send_data(const uint8_t* mac, const T& data){
        return esp_now_send(mac, reinterpret_cast<const uint8_t*>(&data), sizeof(T)) == ESP_OK;
    }
    // Thêm thiết bị vào mạng Esp-Now
    bool add_peer(const uint8_t* mac, uint8_t channel_ = 0, bool encrypt_ = false);
    // Hàm trả về khi gửi tín hiệu thành công
    void on_data_sent(void (*callback)(const uint8_t *mac_addr, esp_now_send_status_t status));
    // Hàm trả về khi nhận tín hiệu thành công
    void on_data_receive(void (*callback)(const uint8_t * mac, const uint8_t *incomingData, int len));
private:

};