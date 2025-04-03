#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "lwip/inet.h"
#include "lwip/netif.h"

#include "netif/ppp/pppos.h"
#include "driver/uart.h"
#include "hal/uart_hal.h"
#include "driver/gpio.h"
#include "freertos/event_groups.h"

#include "lwip/etharp.h"
#include "lwip/lwip_napt.h"
#include "netif/etharp.h"
#include "lwip/prot/iana.h"
#include "sys/socket.h"
#include "netdb.h"

#define UART_PORT_NUM           UART_NUM_1
#define UART_BAUD_RATE          921600
//#define UART_BAUD_RATE          115200
#define UART_TX_PIN             (7)
#define UART_RX_PIN             (6)
#define UART_RX_BUFFER_SIZE     (1024 * 2)
#define UART_TX_BUFFER_SIZE     (1024 * 30)
#define UART_QUEUE_SIZE         100

#define RESET_PIN_NUM GPIO_NUM_10
#define RESERVE_PIN_NUM GPIO_NUM_20

#define TCP_SERVER_PORT (23)

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "ESP-NET-TEST" //CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      "12345678" //CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  3 //CONFIG_ESP_MAXIMUM_RETRY

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK

/*
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif
*/


/**
 * @brief Indicates that the file descriptor represents an invalid (uninitialized or closed) socket
 *
 * Used in the TCP server structure `sock[]` which holds list of active clients we serve.
 */
#define INVALID_SOCK (-1)

/**
 * @brief Time in ms to yield to all tasks when a non-blocking socket would block
 *
 * Non-blocking socket operations are typically executed in a separate task validating
 * the socket status. Whenever the socket returns `EAGAIN` (idle status, i.e. would block)
 * we have to yield to all tasks to prevent lower priority tasks from starving.
 */
#define YIELD_TO_ALL_MS 50

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define PPP_HOST_IP           LWIP_MAKEU32(192, 168, 4, 202)

static const char *TAG = "APP";

static int s_retry_num = 0;

static ppp_pcb *ppp;
static struct netif pppos_netif;
static QueueHandle_t uart_queue;

struct netif* find_netif_from_esp_netif(esp_netif_t *esp_netif) {
    // Get the MAC address of the esp_netif
    uint8_t esp_mac_addr[6];
    esp_err_t ret = esp_netif_get_mac(esp_netif, esp_mac_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address from esp_netif: %d", ret);
        return NULL;
    }

    // Iterate over the netif_list to find the matching netif structure
    struct netif *netif = netif_list;
    while (netif != NULL) {
        if (memcmp(netif->hwaddr, esp_mac_addr, sizeof(esp_mac_addr)) == 0) {
            return netif;
        }
        netif = netif->next;
    }
    return NULL;
}

static void wifi_sta_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static int wifi_init = 0;

void wifi_init_sta(void)
{
    if (wifi_init)
    {
	return;
    }

    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_sta_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_sta_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
	     * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_LOGI(TAG, "wifi_init_sta finished.");
    wifi_init = 1;
    return;
    ESP_ERROR_CHECK(esp_wifi_start() );


    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by wifi_sta_event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *)malloc(UART_RX_BUFFER_SIZE);

    while (true) {
        if (xQueueReceive(uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, UART_RX_BUFFER_SIZE);
            switch (event.type) {
                case UART_DATA:
                    uart_read_bytes(UART_PORT_NUM, dtmp, event.size, portMAX_DELAY);
                    pppos_input_tcpip(ppp, dtmp, event.size);
                    break;
                default:
                    ESP_LOGI(TAG, "uart event type: %d - %d", event.type, uxQueueSpacesAvailable( uart_queue ) );
                    break;
            }
        }
    }

    free(dtmp);
    vTaskDelete(NULL);
}

static void
ppp_link_status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{
    struct netif *pppif = ppp_netif(pcb);
    LWIP_UNUSED_ARG(ctx);

    switch(err_code) {
    case PPPERR_NONE:
        {

	/*
        ip4_addr_t   	nm;
        ip4addr_aton("255.255.255.128", &nm);
        netif_set_netmask(pppif, &nm);
        ESP_LOGI(TAG, "PPP itf netmask set to %s", ip4addr_ntoa(&nm));
	*/
	const ip4_addr_t *our_ip; 

        ESP_LOGI(TAG, "PPP link OK");
        our_ip = netif_ip4_addr(pppif);
        ip_napt_enable(our_ip->addr, 1);
        fprintf(stderr, "ppp itf enable nat done\n\r");

        fprintf(stderr, "ppp_link_status_cb: PPPERR_NONE\n\r");
#if LWIP_IPV4
        fprintf(stderr, "   our_ip4addr = %s\n\r", ip4addr_ntoa(netif_ip4_addr(pppif)));
        fprintf(stderr, "   his_ipaddr  = %s\n\r", ip4addr_ntoa(netif_ip4_gw(pppif)));
        fprintf(stderr, "   netmask     = %s\n\r", ip4addr_ntoa(netif_ip4_netmask(pppif)));
#endif /* LWIP_IPV4 */
        }

        break;

    case PPPERR_PARAM:
        ESP_LOGE(TAG, "PPPERR_PARAM: Invalid parameter");
        break;
    case PPPERR_OPEN:
        ESP_LOGE(TAG, "PPPERR_OPEN: Unable to open PPP session");
        break;
    case PPPERR_DEVICE:
        ESP_LOGE(TAG, "PPPERR_DEVICE: Invalid I/O device for PPP");
        break;
    case PPPERR_ALLOC:
        ESP_LOGE(TAG, "PPPERR_ALLOC: Unable to allocate resources");
        break;
    case PPPERR_USER:
        ESP_LOGE(TAG, "PPPERR_USER: User interrupt");
        break;
    case PPPERR_CONNECT:
        ESP_LOGE(TAG, "PPPERR_CONNECT: Connection lost");
        break;
    case PPPERR_AUTHFAIL:
        ESP_LOGE(TAG, "PPPERR_AUTHFAIL: Failed authentication challenge");
        break;
    case PPPERR_PROTOCOL:
        ESP_LOGE(TAG, "PPPERR_PROTOCOL: Failed to meet protocol");
        break;
    case PPPERR_PEERDEAD:
        ESP_LOGE(TAG, "PPPERR_PEERDEAD: Connection timeout");
        break;
    case PPPERR_IDLETIMEOUT:
        ESP_LOGE(TAG, "PPPERR_IDLETIMEOUT: Idle Timeout");
        break;
    case PPPERR_CONNECTTIME:
        ESP_LOGE(TAG, "PPPERR_CONNECTTIME");
        break;
    case PPPERR_LOOPBACK:
        ESP_LOGE(TAG, "PPPERR_LOOPBACK");
        break;
    default:
        ESP_LOGE(TAG, "Unknown Error");
        break;
    }
}

static u32_t
ppp_output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
    LWIP_UNUSED_ARG(pcb);
    LWIP_UNUSED_ARG(ctx);
    return uart_write_bytes(UART_PORT_NUM, (const char *) data, len);
}

static void setup_uart()
{
    // Configure a UART interrupt threshold and timeout
    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
        .rxfifo_full_thresh = 100,
        .rx_timeout_thresh = 10,
    };

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT_NUM, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, UART_QUEUE_SIZE, &uart_queue, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    
    // Enable UART RX FIFO full threshold and timeout interrupts
    ESP_ERROR_CHECK(uart_intr_config(UART_PORT_NUM, &uart_intr));
    ESP_ERROR_CHECK(uart_enable_rx_intr(UART_PORT_NUM));

    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void setup_ppp()
{
    ppp = pppos_create(&pppos_netif, ppp_output_cb, ppp_link_status_cb, NULL);
    if (!ppp) {
        ESP_LOGE(TAG, "PPPOS example: Could not create PPP control interface");
        return;
    }
  
    ppp_set_default(ppp);
    ppp_set_silent(ppp, 1);
    ppp_connect(ppp, 0);

    /*
    	ip4_addr_t   	nm;
    ip4addr_aton("255.255.255.128", &nm);
    netif_set_netmask(&pppos_netif, &nm);
    ESP_LOGI(TAG, "PPP itf netmask set to %s", ip4addr_ntoa(&nm));
    */
}

/**
 * @brief Utility to log socket errors
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket number
 * @param[in] err Socket errno
 * @param[in] message Message to print
 */
static void log_socket_error(const char *tag, const int sock, const int err, const char *message)
{
    ESP_LOGE(tag, "[sock=%d]: %s\n"
                  "error=%d: %s", sock, message, err, strerror(err));
}

/**
 * @brief Tries to receive data from specified sockets in a non-blocking way,
 *        i.e. returns immediately if no data.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket for reception
 * @param[out] data Data pointer to write the received data
 * @param[in] max_len Maximum size of the allocated space for receiving data
 * @return
 *          >0 : Size of received data
 *          =0 : No data available
 *          -1 : Error occurred during socket read operation
 *          -2 : Socket is not connected, to distinguish between an actual socket error and active disconnection
 */
static int try_receive(const char *tag, const int sock, char * data, size_t max_len)
{
    int len = recv(sock, data, max_len, 0);
    if (len < 0) {
        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;   // Not an error
        }
        if (errno == ENOTCONN) {
            ESP_LOGW(tag, "[sock=%d]: Connection closed", sock);
            return -2;  // Socket has been disconnected
        }
        log_socket_error(tag, sock, errno, "Error occurred during receiving");
        return -1;
    }

    return len;
}

/**
 * @brief Sends the specified data to the socket. This function blocks until all bytes got sent.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket to write data
 * @param[in] data Data to be written
 * @param[in] len Length of the data
 * @return
 *          >0 : Size the written data
 *          -1 : Error occurred during socket write operation
 */
static int socket_send(const char *tag, const int sock, const char * data, const size_t len)
{
    int to_write = len;
    while (to_write > 0) {
        int written = send(sock, data + (len - to_write), to_write, 0);
        if (written < 0 && errno != EINPROGRESS && errno != EAGAIN && errno != EWOULDBLOCK) {
            log_socket_error(tag, sock, errno, "Error occurred during sending");
            return -1;
        }
        to_write -= written;
    }
    return len;
}

static inline char* get_clients_address(struct sockaddr_storage *source_addr)
{
    static char address_str[128];
    char *res = NULL;
    // Convert ip address to string
    if (source_addr->ss_family == PF_INET) {
        res = inet_ntoa_r(((struct sockaddr_in *)source_addr)->sin_addr, address_str, sizeof(address_str) - 1);
    }
    if (!res) {
        address_str[0] = '\0'; // Returns empty string if conversion didn't succeed
    }
    return address_str;
}

static void tcp_server_task(void *pvParameters)
{
    static char rx_buffer[128];
    static const char *TAG = "nonblocking-socket-server";
    SemaphoreHandle_t *server_ready = pvParameters;
    struct sockaddr_storage dest_addr;
    int listen_sock = INVALID_SOCK;
    int cur_sock = INVALID_SOCK;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(TCP_SERVER_PORT);

    // Creating a listener socket
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if (listen_sock < 0) {
        log_socket_error(TAG, listen_sock, errno, "Unable to create socket");
        goto error;
    }
    ESP_LOGI(TAG, "Listener socket created");

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Marking the socket as non-blocking
    int flags = fcntl(listen_sock, F_GETFL);
    if (fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK) == -1) {
        log_socket_error(TAG, listen_sock, errno, "Unable to set socket non blocking");
        goto error;
    }
    ESP_LOGI(TAG, "Socket marked as non blocking");

    // Binding socket to the given address
    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        log_socket_error(TAG, listen_sock, errno, "Socket unable to bind");
        goto error;
    }
    ESP_LOGI(TAG, "Socket bound on INANY:%d", TCP_SERVER_PORT);

    // Set queue (backlog) of pending connections to one (can be more)
    err = listen(listen_sock, 1);
    if (err != 0) {
        log_socket_error(TAG, listen_sock, errno, "Error occurred during listen");
        goto error;
    }
    ESP_LOGI(TAG, "Socket listening");
    xSemaphoreGive(*server_ready);

    // Main loop for accepting new connections and serving all connected clients
    while (1) {
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int new_sock = INVALID_SOCK;

        // Try to accept a new connections
        new_sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

        if (new_sock < 0) {
            if (errno == EWOULDBLOCK) { // The listener socket did not accepts any connection
                                        // continue to serve open connections and try to accept again upon the next iteration
                ESP_LOGV(TAG, "No pending connections...");
            } else {
                log_socket_error(TAG, listen_sock, errno, "Error when accepting connection");
                goto error;
            }
        } else {
            if (cur_sock != INVALID_SOCK) {
                shutdown(cur_sock, 0);
                close(cur_sock);
                cur_sock = INVALID_SOCK;
            }
            cur_sock = new_sock;
            // We have a new client connected -> print it's address
            ESP_LOGI(TAG, "[sock=%d]: Connection accepted from IP:%s", cur_sock, get_clients_address(&source_addr));

            // ...and set the client's socket non-blocking
            flags = fcntl(cur_sock, F_GETFL);
            if (fcntl(cur_sock, F_SETFL, flags | O_NONBLOCK) == -1) {
                log_socket_error(TAG, cur_sock, errno, "Unable to set socket non blocking");
                goto error;
            }
            ESP_LOGI(TAG, "[sock=%d]: Socket marked as non blocking", cur_sock);
        }

        // We serve all the connected clients in this loop
        if (cur_sock != INVALID_SOCK) {

            // This is an open socket -> try to serve it
            int len = try_receive(TAG, cur_sock, rx_buffer, sizeof(rx_buffer));
            if (len < 0) {
                // Error occurred within this client's socket -> close and mark invalid
                ESP_LOGI(TAG, "[sock=%d]: try_receive() returned %d -> closing the socket", cur_sock, len);
                close(cur_sock);
                cur_sock = INVALID_SOCK;
            } else if (len > 0) {
                // Received some data -> echo back
                ESP_LOGI(TAG, "[sock=%d]: Received %.*s", cur_sock, len, rx_buffer);

                len = socket_send(TAG, cur_sock, rx_buffer, len);
                if (len < 0) {
                    // Error occurred on write to this socket -> close it and mark invalid
                    ESP_LOGI(TAG, "[sock=%d]: socket_send() returned %d -> closing the socket", cur_sock, len);
                    close(cur_sock);
                    cur_sock = INVALID_SOCK;
                } else {
                    // Successfully echoed to this socket
                    ESP_LOGI(TAG, "[sock=%d]: Written %.*s", cur_sock, len, rx_buffer);
                }
                char tail = rx_buffer[len - 1];
		if (tail == '\n' || tail == '\r') {
                    rx_buffer[len - 1] = '\n';
                }
                if (len > 2) {
                    char tail2 = rx_buffer[len - 2];
		    if (tail2 == '\n' || tail2 == '\r') {
                        rx_buffer[len - 2] = '\n';
                    }
                }
                //goto skip_cmd;
                rx_buffer[sizeof(rx_buffer) - 1] = '\0';
                 
                char *cmd = strtok((char *)rx_buffer, " ");
                if (cmd != NULL && 0 == strcmp(cmd, "connect")) {
                    char *ssid_str = strtok(NULL, " ");
                    if (ssid_str != NULL) {
                        char *pass_str = strtok(NULL, "\n");
                        if (pass_str != NULL) {
                            wifi_init_sta();
                            ESP_LOGI(TAG, "wifi connect: %s : %s", ssid_str, pass_str);
                            wifi_config_t wifi_config = {0};
                            strncpy((char *)&wifi_config.sta.ssid[0], ssid_str, sizeof(wifi_config.sta.ssid));
                            strncpy((char *)&wifi_config.sta.password[0], pass_str, sizeof(wifi_config.sta.password)); 
                            wifi_config.sta.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;
                            wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
                            ESP_ERROR_CHECK(esp_wifi_stop() );
                            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
                            ESP_ERROR_CHECK(esp_wifi_start() );
                        }
                    }
                } else if (cmd != NULL && 0 == strcmp(cmd, "exit")) {
                    shutdown(cur_sock, 0);
                    close(cur_sock);
                    cur_sock = INVALID_SOCK;
                }
//skip_cmd:
            }

        } // one client's socket

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(YIELD_TO_ALL_MS));
    }

error:
    if (listen_sock != INVALID_SOCK) {
        close(listen_sock);
    }

    if (cur_sock != INVALID_SOCK) {
        close(cur_sock);
    }

    vTaskDelete(NULL);
}

static void reset_pin_event_task(void *pvParameters) 
{
    unsigned long cnt = 0;
    while(1) {
        if (0 == gpio_get_level(RESET_PIN_NUM)) {
            cnt++;
        } else {
            cnt = 0;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);

        if (cnt > 20) {
            ESP_LOGI(TAG, "reset by pin");
            esp_restart();
        }
    }
    vTaskDelete(NULL);
}

void setup_reset_pin()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RESET_PIN_NUM);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RESERVE_PIN_NUM) ;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    xTaskCreate(reset_pin_event_task, "reset_pin_event_task", 1024, NULL, 12, NULL);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_LOGI(TAG, "ESP_START_UART_PPP");

    setup_uart();
    setup_ppp();
    setup_reset_pin();
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    ESP_LOGI(TAG, "ESP_START_TCP_SERVER");

    SemaphoreHandle_t server_ready = xSemaphoreCreateBinary();
    assert(server_ready);
    xTaskCreate(tcp_server_task, "tcp_server", 4096, &server_ready, 5, NULL);
    xSemaphoreTake(server_ready, portMAX_DELAY);
    vSemaphoreDelete(server_ready);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    //wifi_init_softap();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //wifi_init_sta();
}
