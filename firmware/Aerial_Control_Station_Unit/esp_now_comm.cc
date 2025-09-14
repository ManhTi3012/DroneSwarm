/*
 * esp_now_comm.cc
 *
 *  Created on: May 29, 2025
 *      Author: Ti Manh
 */

#include "esp_now_comm.h"

bool EspNowComm::init(){
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }

    return true;
}   

bool EspNowComm::add_peer(const uint8_t* mac, uint8_t channel_, bool encrypt_) {
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = channel_;  // TODO: Add getter if you store these in Peer
    peerInfo.encrypt = encrypt_;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return false;
    }

    return true;
}