/*
 * esp_now_comm.h
 *
 *  Created on: May 29, 2025
 *      Author: Ti Manh
 */

#ifndef INC_ESPNOWCOMM_H_
#define INC_ESPNOWCOMM_H_

#include <WiFi.h>
#include <esp_now.h>


class EspNowComm {
public:

    bool init();

    esp_now_peer_info_t peerInfo;

    template<typename T>
    bool send_data(const uint8_t* mac, const T& data){
        return esp_now_send(mac, reinterpret_cast<const uint8_t*>(&data), sizeof(T)) == ESP_OK;
    }

    bool add_peer(const uint8_t* mac, uint8_t channel_ = 0, bool encrypt_ = false);

private:

};


#endif /* INC_ESPNOWCOMM_H_ */