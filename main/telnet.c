/* telnet.c */
#include <string.h>
#include <sys/param.h> /* For MIN/MAX */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h" /* For esp_netif_init, if not done elsewhere */
#include "esp_event.h" /* For esp_event_loop_create_default, if not done elsewhere */

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

/* Added for ESP Console integration */
#include "esp_console.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h> /* Required for va_list, va_copy, etc. */

/* Telnet Command Definitions */
#define TELNET_IAC 255  /* Interpret As Command */
#define TELNET_DONT 254 /* Don't perform option */
#define TELNET_DO 253   /* Do perform option */
#define TELNET_WONT 252 /* Won't perform option */
#define TELNET_WILL 251 /* Will perform option */
#define TELNET_SB 250   /* Subnegotiation Begin */
#define TELNET_SE 240   /* Subnegotiation End */

/* Telnet Options */
#define TELNET_OPT_BINARY 0       /* Binary Transmission */
#define TELNET_OPT_ECHO 1         /* Echo */
#define TELNET_OPT_RECONNECTION 2 /* Reconnection */
#define TELNET_OPT_SGA 3          /* Suppress Go Ahead */
#define TELNET_OPT_TTYPE 24       /* Terminal Type */
#define TELNET_OPT_NAWS 31        /* Negotiate About Window Size */
#define TELNET_OPT_LINEMODE 34    /* Linemode */

/* Static variable for the current Telnet client socket, -1 if none. */
static int s_telnet_client_sock = -1;
/* Static variable to store the original vprintf log handler. */
static vprintf_like_t s_original_vprintf_handler = NULL;

#define TELNET_PORT 23
#define TELNET_KEEPALIVE_IDLE 5     /* Keepalive idle time (seconds) */
#define TELNET_KEEPALIVE_INTERVAL 5 /* Keepalive interval time (seconds) */
#define TELNET_KEEPALIVE_COUNT 3    /* Keepalive packet retry count */
#define TELNET_RX_BUFFER_SIZE 128
#define TELNET_TASK_STACK_SIZE 6188
#define TELNET_TASK_PRIORITY 5
#define TELNET_MAX_CONNECTIONS 1 /* Max simultaneous connections (listen backlog) */

static const char *TAG_TELNET = "telnet_server";

/*
 * Custom vprintf implementation that redirects log output to the Telnet socket
 * in addition to calling the original vprintf handler.
 */
static int telnet_vprintf_redirect(const char *format, va_list args)
{
    int ret = 0;

    /* Call the original vprintf handler to output to UART/default log. */

    /* If a Telnet client is connected, send the log to them. */
    if (s_telnet_client_sock == -1)
    {
        return s_original_vprintf_handler(format, args);
    }

    char temp_buffer[256]; /* Buffer for the formatted log line. */
    int len = vsnprintf(temp_buffer, sizeof(temp_buffer), format, args);

    if (len > 0)
    {
        /* Send character by character, converting \n to \r\n for Telnet. */
        for (int i = 0; i < len && i < sizeof(temp_buffer) - 1; ++i)
        {
            if (temp_buffer[i] == '\n')
            {
                if (send(s_telnet_client_sock, "\r\n", 2, 0) < 0)
                {
                    /* Error sending, client might have disconnected abruptly. */
                    /* No easy way to fully handle this here without more state. */
                    break;
                }
            }
            else
            {
                if (send(s_telnet_client_sock, &temp_buffer[i], 1, 0) < 0)
                {
                    break;
                }
            }
        }
    }

    return ret;
}

/* Helper to send Telnet command sequences */
static void send_telnet_iac(const int sock, unsigned char command, unsigned char option)
{
    unsigned char seq[3];
    seq[0] = TELNET_IAC;
    seq[1] = command;
    seq[2] = option;
    if (send(sock, seq, 3, 0) < 0)
    {
        ESP_LOGE(TAG_TELNET, "Error sending IAC command %u %u: errno %d", command, option, errno);
    }
}

/*
 * Handles a single Telnet client connection.
 * This function will run indefinitely for the duration of the connection,
 * passing input to the ESP console and sending output back.
 * Implements basic character-by-character handling and server-side echo.
 */
static void handle_telnet_client_connection(const int sock)
{
    char line_buffer[TELNET_RX_BUFFER_SIZE];
    int line_len = 0;
    char input_char_buf[1]; /* Buffer for single character recv */

    const char *welcome_msg = "Welcome to ESP32 Telnet Console!\r\nType 'help' for a list of commands.\r\n";
    const char *prompt = "> ";
    char *output_buffer = NULL;
    size_t output_buffer_size = 0;
    FILE *temp_stdout = NULL;
    FILE *original_stdout = stdout; /* Store original stdout */

    ESP_LOGI(TAG_TELNET, "New client connection, attempting to set character mode.");
    s_telnet_client_sock = sock; /* Set the active telnet socket for logging */

    /* Negotiate Telnet options: Server WILL ECHO, Server WILL SGA */
    /* This tells the client that the server will handle echoing and suppress go-ahead prompts */
    send_telnet_iac(sock, TELNET_WILL, TELNET_OPT_ECHO);
    send_telnet_iac(sock, TELNET_WILL, TELNET_OPT_SGA);
    /* Optionally, ask client to DO SGA and DO ECHO if we want to confirm client state, */
    /* but for simplicity, we assume client will adapt or server-side handling is sufficient. */
    /* send_telnet_iac(sock, TELNET_DO, TELNET_OPT_SGA); */
    /* send_telnet_iac(sock, TELNET_DO, TELNET_OPT_ECHO); // If we want client to also echo, usually not with server echo */

    /* Send welcome message */
    if (send(sock, welcome_msg, strlen(welcome_msg), 0) < 0)
    {
        ESP_LOGE(TAG_TELNET, "Error sending welcome message: errno %d", errno);
        goto close_socket_cleanup;
    }

    /* Send initial prompt */
    if (send(sock, prompt, strlen(prompt), 0) < 0)
    {
        ESP_LOGE(TAG_TELNET, "Error sending initial prompt: errno %d", errno);
        goto close_socket_cleanup;
    }

    memset(line_buffer, 0, sizeof(line_buffer));
    line_len = 0;

    do
    {
        int len_recv = recv(sock, input_char_buf, 1, 0);

        if (len_recv < 0)
        {
            ESP_LOGE(TAG_TELNET, "Error occurred during receiving: errno %d", errno);
            break; /* Break loop on receive error */
        }
        else if (len_recv == 0)
        {
            ESP_LOGI(TAG_TELNET, "Connection closed by client");
            break; /* Break loop if client closes connection */
        }

        unsigned char c = input_char_buf[0];

        if (c == TELNET_IAC)
        {
            unsigned char iac_cmd_buf[2];
            int iac_len = recv(sock, iac_cmd_buf, 2, 0);
            if (iac_len == 2)
            {
                unsigned char iac_command = iac_cmd_buf[0];
                unsigned char iac_option = iac_cmd_buf[1];
                ESP_LOGD(TAG_TELNET, "Received IAC %u %u", iac_command, iac_option);
                /* Basic handling: if client says DONT ECHO, we should stop server-side echo. */
                /* This part can be expanded to a full Telnet option state machine. */
                /* For now, we mostly ignore client's responses to keep it simple, */
                /* assuming our initial WILLs are accepted. */
                if (iac_command == TELNET_DO && iac_option == TELNET_OPT_TTYPE)
                {
                    /* Client wants to send terminal type, respond with WILL TTYPE then SB ... SE */
                    /* This is more advanced, for now just acknowledge by sending WONT */
                    send_telnet_iac(sock, TELNET_WONT, TELNET_OPT_TTYPE);
                }
            }
            else
            {
                ESP_LOGE(TAG_TELNET, "Error receiving IAC sequence or connection closed.");
                break;
            }
            continue; /* Processed IAC, get next char */
        }

        /* Handle actual characters */
        if (c == '\r')
        { /* Carriage return */
            /* Typically followed by \n (from client) or client sends \r as Enter */
            line_buffer[line_len] = 0; /* Null-terminate */
            if (send(sock, "\r\n", 2, 0) < 0)
            {
                break;
            } /* Echo CR LF */

            ESP_LOGI(TAG_TELNET, "Received command: '%s'", line_buffer);

            if (strcmp(line_buffer, "exit") == 0)
            {
                ESP_LOGI(TAG_TELNET, "Client requested exit");
                if (send(sock, "Goodbye!\r\n", strlen("Goodbye!\r\n"), 0) < 0)
                { /* Ignore error on exit */
                }
                break;
            }
            else if (line_len > 0)
            {
                temp_stdout = open_memstream(&output_buffer, &output_buffer_size);
                if (!temp_stdout)
                {
                    ESP_LOGE(TAG_TELNET, "Failed to open memstream");
                    if (send(sock, "Error: Internal server error (memstream)\r\n", strlen("Error: Internal server error (memstream)\r\n"), 0) < 0)
                    {
                        break;
                    }
                    /* No continue here, will go to prompt sending */
                }
                else
                {
                    stdout = temp_stdout;
                    int cmd_ret_code;
                    esp_err_t exec_ret = esp_console_run(line_buffer, &cmd_ret_code);
                    stdout = original_stdout;
                    fflush(temp_stdout);
                    fclose(temp_stdout);
                    temp_stdout = NULL;

                    if (exec_ret == ESP_ERR_NOT_FOUND)
                    {
                        if (send(sock, "Error: Command not found\r\n", strlen("Error: Command not found\r\n"), 0) < 0)
                        {
                            break;
                        }
                    }
                    else if (exec_ret == ESP_ERR_INVALID_ARG)
                    {
                        if (send(sock, "Error: Invalid arguments\r\n", strlen("Error: Invalid arguments\r\n"), 0) < 0)
                        {
                            break;
                        }
                    }
                    else if (exec_ret != ESP_OK)
                    {
                        char err_msg[80];
                        snprintf(err_msg, sizeof(err_msg), "Error: Command failed (err %d)\r\n", exec_ret);
                        if (send(sock, err_msg, strlen(err_msg), 0) < 0)
                        {
                            break;
                        }
                    }

                    if (output_buffer && output_buffer_size > 0)
                    {
                        if (send(sock, output_buffer, output_buffer_size, 0) < 0)
                        {
                            break;
                        }
                        /* Ensure CRLF if not present, common for console output */
                        if (output_buffer_size > 0 && output_buffer[output_buffer_size - 1] != '\n')
                        {
                            if (send(sock, "\r\n", 2, 0) < 0)
                            {
                                break;
                            }
                        }
                    }
                    else if (exec_ret == ESP_OK && cmd_ret_code == ESP_OK)
                    {
                        /* If command was successful but produced no output, still send a CRLF for neatness */
                        if (send(sock, "\r\n", 2, 0) < 0)
                        {
                            break;
                        }
                    }
                    if (output_buffer)
                    {
                        free(output_buffer);
                        output_buffer = NULL;
                        output_buffer_size = 0;
                    }
                }
            }
            else
            { /* Empty line submitted */
                /* send(sock, "\r\n", 2, 0); // Already sent CR LF above */
            }

            /* Reset line buffer and send prompt for next command */
            memset(line_buffer, 0, sizeof(line_buffer));
            line_len = 0;
            if (send(sock, prompt, strlen(prompt), 0) < 0)
            {
                break;
            }
        }
        else if (c == '\n')
        { /* Line feed */
            /* Typically ignored if \r was just processed. Some clients might send \n alone. */
            /* For simplicity, we primarily act on \r. If \r handling also sends prompt, this might double prompt. */
            /* If line_buffer is not empty and last char was not \r, could treat as enter. */
            /* Current logic: \r is the main EOL trigger. */
            continue;
        }
        else if (c == '\b' || c == '\x7f')
        { /* Backspace or Delete */
            if (line_len > 0)
            {
                line_len--;
                line_buffer[line_len] = 0; /* Effectively delete char */
                /* Echo backspace, space, backspace to erase on client terminal */
                if (send(sock, "\b \b", 3, 0) < 0)
                {
                    break;
                }
            }
        }
        else if (c >= 32 && c < 127)
        { /* Printable ASCII characters */
            if (line_len < (sizeof(line_buffer) - 1))
            {
                line_buffer[line_len++] = c;
                line_buffer[line_len] = 0; /* Keep null-terminated */
                /* Echo character back to client */
                if (send(sock, &c, 1, 0) < 0)
                {
                    break;
                }
            }
        }
        else
        {
            /* Other control characters or non-ASCII, log or ignore */
            ESP_LOGD(TAG_TELNET, "Received unhandled char: 0x%02X", c);
        }

    } while (1); /* Loop indefinitely until break */

close_socket_cleanup:
    ESP_LOGI(TAG_TELNET, "Shutting down client socket and closing connection");
    s_telnet_client_sock = -1; /* Clear the active telnet socket for logging */

    /* Ensure stdout is restored if loop broken unexpectedly while redirected */
    if (stdout != original_stdout)
    {
        stdout = original_stdout;
    }
    /* Clean up resources if they were allocated and not freed due to an error exit */
    if (temp_stdout)
    {
        fclose(temp_stdout);
    }
    if (output_buffer)
    {
        free(output_buffer);
    }

    shutdown(sock, SHUT_RDWR); /* SHUT_RDWR to signal no more send/receive */
    close(sock);
}

/*
 * Main Telnet server task.
 * Creates a listening socket and accepts incoming connections.
 */
static void telnet_server_main_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = AF_INET; /* For IPv4 */
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_storage dest_addr; /* Use sockaddr_storage for IPv4/IPv6 compatibility */

    /* Prepare the destination address structure */
    if (addr_family == AF_INET)
    {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY); /* Listen on all interfaces */
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(TELNET_PORT);
    }
    else
    {
        ESP_LOGE(TAG_TELNET, "Unsupported address family: %d", addr_family);
        vTaskDelete(NULL);
        return;
    }

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG_TELNET, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_TELNET, "Socket created successfully");

    /* Set SO_REUSEADDR to allow binding to the same address/port if socket was recently closed */
    int opt = 1;
    if (setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        ESP_LOGW(TAG_TELNET, "setsockopt(SO_REUSEADDR) failed: errno %d", errno);
        /* Not fatal, but can be an issue on quick restarts */
    }

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG_TELNET, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP_SOCKET;
    }
    ESP_LOGI(TAG_TELNET, "Socket bound to port %d", TELNET_PORT);

    /* Set our custom vprintf handler. Store the original one. */
    /* This should ideally be done only once if the task persists.*/
    if (s_original_vprintf_handler == NULL)
    {
        s_original_vprintf_handler = esp_log_set_vprintf(telnet_vprintf_redirect);
        ESP_LOGI(TAG_TELNET, "Telnet vprintf redirector installed.");
    }

    err = listen(listen_sock, TELNET_MAX_CONNECTIONS);
    if (err != 0)
    {
        ESP_LOGE(TAG_TELNET, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP_SOCKET;
    }

    ESP_LOGI(TAG_TELNET, "Telnet server listening on port %d", TELNET_PORT);

    while (1)
    {
        ESP_LOGI(TAG_TELNET, "Waiting for a new connection...");

        struct sockaddr_storage source_addr; /* Used to store client address */
        socklen_t addr_len = sizeof(source_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

        if (client_sock < 0)
        {
            ESP_LOGE(TAG_TELNET, "Unable to accept connection: errno %d", errno);
            /* Consider adding a delay here or checking for specific errors if accept fails continuously */
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            {
                /* Non-fatal errors, continue listening */
                continue;
            }
            break; /* For other errors, break the loop and clean up */
        }

        /* Connection accepted */
        if (source_addr.ss_family == PF_INET)
        { /* PF_INET is the same as AF_INET */
            struct sockaddr_in *s = (struct sockaddr_in *)&source_addr;
            inet_ntoa_r(s->sin_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG_TELNET, "Connection accepted from: %s:%u", addr_str, ntohs(s->sin_port));
        }

        /* Configure TCP keepalive */
        int keepAlive = 1;
        int keepIdle = TELNET_KEEPALIVE_IDLE;
        int keepInterval = TELNET_KEEPALIVE_INTERVAL;
        int keepCount = TELNET_KEEPALIVE_COUNT;
        setsockopt(client_sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

        /*
         * Handle the client connection.
         * For simplicity, this example handles one client at a time.
         * For concurrent clients, a new task should be created for each connection.
         */
        handle_telnet_client_connection(client_sock);
        /* After handle_telnet_client_connection returns, the client socket is closed by it. */
    }

CLEAN_UP_SOCKET:
    ESP_LOGI(TAG_TELNET, "Closing listen socket.");
    close(listen_sock);
    vTaskDelete(NULL); /* Delete this task */
}

/*
 * Public function to initialize and start the Telnet server.
 * This should be called once from your application's main initialization sequence.
 * Ensure network (Wi-Fi/Ethernet) is initialized before calling this.
 *
 *
 */
void telnet_start(void) /* Renamed to avoid conflict with potential framework start_ functions */
{
    /*
     * It's assumed that network interface (Wi-Fi or Ethernet) has been initialized
     * and the device is connected to the network before this function is called.
     */
    BaseType_t xReturned = xTaskCreate(telnet_server_main_task, /* Task function */
                                       "telnet_srv_task",       /* Name of task */
                                       TELNET_TASK_STACK_SIZE,  /* Stack size of task */
                                       NULL,                    /* Parameter of the task */
                                       TELNET_TASK_PRIORITY,    /* Priority of the task */
                                       NULL);                   /* Task handle (NULL if not needed) */

    if (xReturned == pdPASS)
    {
        ESP_LOGI(TAG_TELNET, "Telnet server task created successfully.");
    }
    else
    {
        ESP_LOGE(TAG_TELNET, "Failed to create Telnet server task.");
        /* Optionally, add more robust error handling here */
    }
}
