/* 
 * File:   wifi_manager.h
 * Author: telea
 *
 * Created on February 10, 2026, 9:03 PM
 */

#ifndef WIFI_MANAGER_H
#define	WIFI_MANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>

#define WIFI_SSID_MAX_LEN 32
#define WIFI_PASS_MAX_LEN 64

// WiFi ????????? ?????
typedef struct {
    uint8_t connected;      // 0 = disconnected, 1 = connected
    uint8_t ip_ready;       // DHCP ???????
    uint8_t time_synced;    // NTP ?????????
    uint8_t error;          // ?????? WiFi
    uint8_t provisioning;   // SoftAP ???????
} wifi_status_t;

// WiFi ???????
typedef enum {
    WIFI_CMD_CONNECT,
    WIFI_CMD_DISCONNECT,
    WIFI_CMD_START_PROVISION,
    WIFI_CMD_STOP_PROVISION
} wifi_cmd_type_t;

typedef struct {
    wifi_cmd_type_t type;
    char ssid[WIFI_SSID_MAX_LEN+1];
    char pass[WIFI_PASS_MAX_LEN+1];
} wifi_cmd_t;

// ---------------- API ----------------

// ????????????? WiFi Manager
// statusCb - callback ?? ?????? WiFi
// mode     - WIFI_SOFT_AP ??? WIFI_CLIENT
void wifiManager_init(void (*statusCb)(uint8_t), uint8_t mode);

// ???????? ?????? WiFi (connect/disconnect/provision)
void wifiManager_sendCommand(wifi_cmd_type_t type, const char *ssid, const char *pass);

// ????????? ???????? ??????? WiFi
void wifiManager_getStatus(wifi_status_t *status);



#ifdef	__cplusplus
}
#endif

#endif	/* WIFI_MANAGER_H */

