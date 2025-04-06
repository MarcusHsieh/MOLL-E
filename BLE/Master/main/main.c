/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "common.h"
#include "gap.h"
// for data collection and equation work
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Library function declarations */
void ble_store_config_init(void);

/* Private function declarations */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *param);

int originCount = 0;
int beacon1Count = 0;
int beacon2Count = 0;

double originSum = 0.0;
double beacon1Sum = 0.0;
double beacon2Sum = 0.0;

FILE *file; 

typedef struct {
    double x;
    double y;
} Point;

Point origin = {0, 0};
Point beacon1 = {1.14, 5.66};
Point beacon2 = {-2.01, 4.978};
Point loc;

Point trilaterate(Point p1, double r1, Point p2, double r2, Point p3, double r3) {
    // Translate system so p1 is at origin
    double exx = (p2.x - p1.x);
    double exy = (p2.y - p1.y);
    double d = sqrt(exx*exx + exy*exy);
    exx /= d;
    exy /= d;

    double p3p1x = p3.x - p1.x;
    double p3p1y = p3.y - p1.y;

    double i = exx * p3p1x + exy * p3p1y;

    double eyx = p3p1x - i * exx;
    double eyy = p3p1y - i * exy;
    double j = sqrt(eyx*eyx + eyy*eyy);
    eyx /= j;
    eyy /= j;

    double x = (r1*r1 - r2*r2 + d*d) / (2*d);
    double y = (r1*r1 - r3*r3 + i*i + j*j - 2*i*x) / (2*j);

    Point result;
    result.x = p1.x + x * exx + y * eyx;
    result.y = p1.y + x * exy + y * eyy;
    return result;
}

// array of allowed mac addresses (beacons)
static const char *allowed_macs[] = {
    "36:22:72:12:CF:A4", // Origin
    "93:4E:B6:2D:E6:B4", // Beacon1
    "8A:24:11:BF:71:3C" // Beacon 2
};

#define NUM_ALLOWED_MACS (sizeof(allowed_macs) / sizeof(allowed_macs[0]))

// helper function for string conversion
char* my_ble_addr_to_str(const ble_addr_t *addr, char *buf, size_t buf_len) {
    if (buf_len < 18) { // "xx:xx:xx:xx:xx:xx" plus null terminator
        return NULL;
    }
    snprintf(buf, buf_len, "%02x:%02x:%02x:%02x:%02x:%02x",
             addr->val[0], addr->val[1], addr->val[2],
             addr->val[3], addr->val[4], addr->val[5]);
    return buf;
}



// Helper function to compare MAC addresses (case-insensitive)
static int is_allowed_mac(const ble_addr_t *addr) {
    char addr_str[18] = {0};
    my_ble_addr_to_str(addr, addr_str, sizeof(addr_str));
    for (int i = 0; i < NUM_ALLOWED_MACS; i++) {
        if (strcasecmp(addr_str, allowed_macs[i]) == 0) {
            return 1; // Allowed
        }
    }
    return 0; // Not allowed
}

double rssi_to_distance(int rssi) {
    double distance = pow(10.0, (-70.8 - rssi) / ( 32 ));
    return distance;
}

// This function sets up and starts scanning

static int ble_gap_event_handler(struct ble_gap_event *event, void *arg) {
    char addr_str[18] = {0};
    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            my_ble_addr_to_str(&event->disc.addr, addr_str, sizeof(addr_str));
            if (is_allowed_mac(&event->disc.addr)) {
                ESP_LOGI("SCAN", "Allowed device discovered: %s, RSSI: %d", addr_str, event->disc.rssi);
                // Process this device further if needed.
                // Check ADDR's and put into corresponding queue
                // Origin
                if (strcmp(addr_str, "36:22:72:12:cf:a4") == 0 && originCount < 10) {
                    originSum += rssi_to_distance(event->disc.rssi);
                    originCount++;
                }
                // Beacon1
                else if (strcmp(addr_str, "93:4e:b6:2d:e6:b4") == 0 && beacon1Count < 10) {
                    beacon1Sum += rssi_to_distance(event->disc.rssi);
                    beacon1Count++;
                }
                // Beacon2
                else if (strcmp(addr_str, "8a:24:11:bf:71:3c") == 0 && beacon2Count < 10) {
                    beacon2Sum += rssi_to_distance(event->disc.rssi);
                    beacon2Count++;
                }
                // At this point all beacon counts are 1-
                else {
                    originSum = originSum / 10;
                    ESP_LOGE(TAG, "origin average: %.2f", originSum);
                    beacon1Sum = beacon1Sum / 10;
                    ESP_LOGE(TAG, "beacon1 average %.2f", beacon1Sum);
                    beacon2Sum = beacon2Sum / 10;
                    ESP_LOGE(TAG, "beacon2 average %.2f", beacon2Sum);

                    originCount = 0;
                    beacon1Count = 0;
                    beacon2Count = 0;

                    // Trilateration
                    loc = trilaterate(origin, originSum, beacon1, beacon1Sum, beacon2, beacon2Sum);
                    ESP_LOGE(TAG, "estimated location ( %.2f, %.2f )", loc.x, loc.y);
                }
            }
            break;
        default:
            break;
    }
    return 0;
}

// BLE GAP event callback (in C)
static void ble_app_scan(void) {
    struct ble_gap_disc_params disc_params;
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.passive = 0;            // Active mode, which actively sends out requests
    disc_params.filter_duplicates = 0;  // Does not filter duplicate advertisements
    disc_params.itvl = 0x0010;          // Scan interval (20 ms, matches beacon rate)
    disc_params.window = 0x0010;        // Scan window (must be <= interval)
    disc_params.filter_policy = BLE_HCI_SCAN_FILT_NO_WL;
    
    // Use a public address type (adjust if necessary)
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting scan; rc=%d", rc);
    }
}

static void host_task(void *param) {
    nimble_port_run();
    vTaskDelete(NULL);
}

/* Private functions */
/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
    ble_app_scan();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

void print_mac_address(void) {
    uint8_t mac[6];
    esp_err_t ret = esp_efuse_mac_get_default(mac);
    if (ret != ESP_OK) {
        ESP_LOGE("MAC_PRINT", "Failed to get MAC address, err=0x%x", ret);
        return;
    }
    // Print the MAC address in the standard format
    printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


void app_main(void) {
    // print_mac_address();

    /* Local variables */
    int rc = 0;
    esp_err_t ret = ESP_OK;
    // file = fopen("main/data.csv", "a"); // Open file for writing

    // if (file == NULL) {
    //     perror("Unable to open file!");
    // }

    // fprintf(file, "Distance,RSSI\n");


    /* NVS flash initialization */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }

    /* NimBLE host stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }

    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
        return;
    }

    // Configure the BLE host; set the sync callback to start scanning once sync is complete.
    ble_hs_cfg.sync_cb = ble_app_scan;

    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    /* Start NimBLE host task thread and return */
    xTaskCreate(nimble_host_task, "NimBLE Host", 4*1024, NULL, 5, NULL);
    return;
}
