#include "wifi_manager.h"
#include "winc/m2m/m2m_types.h"
#include "cloud/wifi_service.h"
#include "led.h"
#include <string.h>

// ---------------- Internal ----------------
static QueueHandle_t wifiQueue;
static SemaphoreHandle_t wifiStatusMutex;
static wifi_status_t wifiStatus;

// ??????? callback ??? ???????? ??????
static void (*wifiStatusCallback)(uint8_t) = NULL;
extern void (*callback_funcPtr)(uint8_t);

// ???? SoftAP provisioning
static bool responseFromProvisionConnect = false;

// ---------------- WiFi Driver Task ----------------
static void wifiDriverTask(void *arg)
{
    /*
    TickType_t lastWake = xTaskGetTickCount();
    wifi_cmd_t cmd;

    for(;;)
    {
        // ????????? ?????? ?? ???????
        while(xQueueReceive(wifiQueue, &cmd, 0))
        {
            switch(cmd.type)
            {
                case WIFI_CMD_CONNECT:
                    wifi_connectToAp(NEW_CREDENTIALS);
                    break;

                case WIFI_CMD_DISCONNECT:
                    wifi_disconnectFromAp();
                    break;

                case WIFI_CMD_START_PROVISION:
                    enable_provision_ap();
                    break;

                case WIFI_CMD_STOP_PROVISION:
                    // TODO: ??? ????????????? ???????? ??????
                    break;
            }
        }

        // Poll WINC
        wifiHandlerTask(NULL);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(50));
    }
    */
}

// ---------------- SoftAP Task ----------------
static void softApTask(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();

    for(;;)
    {
        xSemaphoreTake(wifiStatusMutex, portMAX_DELAY);
        bool provActive = responseFromProvisionConnect || wifiStatus.provisioning;
        xSemaphoreGive(wifiStatusMutex);

        if(!provActive)
        {
            softApConnectTask(NULL);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    }
}

// ---------------- NTP Task ----------------
static void ntpTask(void *arg)
{
    /*
    TickType_t lastWake = xTaskGetTickCount();

    for(;;)
    {
        xSemaphoreTake(wifiStatusMutex, portMAX_DELAY);
        bool connected = wifiStatus.connected;
        xSemaphoreGive(wifiStatusMutex);

        if(connected)
        {
            ntpTimeFetchTask(NULL);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    }
    */
}

// ---------------- Callback Adapter ----------------
static void wifiCallbackAdapter(uint8_t msgType, const void *pMsg)
{
    wifiCallback(msgType, pMsg); // ?????? callback

    xSemaphoreTake(wifiStatusMutex, portMAX_DELAY);

    switch(msgType)
    {
        case M2M_WIFI_RESP_CON_STATE_CHANGED:
        {
            tstrM2mWifiStateChanged *st = (tstrM2mWifiStateChanged*)pMsg;
            if(st->u8CurrState == M2M_WIFI_CONNECTED)
            {
                wifiStatus.connected = 1;

                // LED GREEN: Connected
                ledParameterGreen.onTime = LED_BLINK;
                ledParameterGreen.offTime = LED_BLINK;
                LED_control(&ledParameterGreen);
            }
            else
            {
                wifiStatus.connected = 0;
                wifiStatus.ip_ready = 0;

                // LED RED: Disconnected
                ledParameterRed.onTime = SOLID_OFF;
                ledParameterRed.offTime = SOLID_ON;
                LED_control(&ledParameterRed);
            }

            if(wifiStatusCallback)
                wifiStatusCallback(st->u8CurrState);
            break;
        }

        case M2M_WIFI_REQ_DHCP_CONF:
            wifiStatus.ip_ready = 1;

            // LED BLUE: DHCP ready
            ledParameterBlue.onTime = LED_1_SEC_ON;
            ledParameterBlue.offTime = LED_1_SEC_ON;
            LED_control(&ledParameterBlue);
            break;

        case M2M_WIFI_RESP_GET_SYS_TIME:
            wifiStatus.time_synced = 1;
            break;

        case M2M_WIFI_RESP_PROVISION_INFO:
            wifiStatus.provisioning = 1;
            responseFromProvisionConnect = true;
            break;

        default: break;
    }

    xSemaphoreGive(wifiStatusMutex);
}

// ---------------- API ----------------
void wifiManager_init(void (*statusCb)(uint8_t), uint8_t mode)
{
    wifiStatusCallback = statusCb;

    wifiQueue = xQueueCreate(8, sizeof(wifi_cmd_t));
    wifiStatusMutex = xSemaphoreCreateMutex();

    memset(&wifiStatus, 0, sizeof(wifiStatus));

    // ????????????? ??????? WiFi ???????
    callback_funcPtr = wifiStatusCallback;
    wifi_init(statusCb, mode);

    callback_funcPtr = wifiCallbackAdapter;
    wifi_reinit();

    // ??????? FreeRTOS ??????
    xTaskCreate(wifiDriverTask, "WiFiDriver", 1400, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(ntpTask, "NTPTask", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(softApTask, "SoftAPTask", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
}

void wifiManager_sendCommand(wifi_cmd_type_t type, const char *ssid, const char *pass)
{
    wifi_cmd_t cmd = {0};
    cmd.type = type;
    if(ssid) strncpy(cmd.ssid, ssid, WIFI_SSID_MAX_LEN);
    if(pass) strncpy(cmd.pass, pass, WIFI_PASS_MAX_LEN);
    xQueueSend(wifiQueue, &cmd, portMAX_DELAY);
}

void wifiManager_getStatus(wifi_status_t *status)
{
    if(!status) return;
    xSemaphoreTake(wifiStatusMutex, portMAX_DELAY);
    memcpy(status, &wifiStatus, sizeof(wifi_status_t));
    xSemaphoreGive(wifiStatusMutex);
}

