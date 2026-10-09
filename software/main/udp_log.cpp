#include "udp_log.hpp"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#if CONFIG_QUTEE_LOG_UDP_PORT > 0

// Holds the logs until WiFi is up (about 7 KB from boot to the menu), then feeds the sender task
static const size_t BUFFER_SIZE = 16384;
static StreamBufferHandle_t s_buffer = nullptr;
static SemaphoreHandle_t s_write_lock = nullptr; // a stream buffer allows only one writer at a time
static vprintf_like_t s_usb_vprintf = nullptr;

static int log_vprintf(const char* fmt, va_list args)
{
  va_list copy;
  va_copy(copy, args);
  int ret = s_usb_vprintf(fmt, args); // USB output unchanged
  char line[256];
  int n = vsnprintf(line, sizeof(line), fmt, copy);
  va_end(copy);
  if(n <= 0)
    return ret;
  if(n >= (int) sizeof(line))
    n = sizeof(line) - 1;
  // micro-ROS's WiFi code logs the WiFi password: never send that over the network
  if(strstr(line, "password") != nullptr)
    return ret;
  // Never block a task for logging: drop the line if the buffer is busy or full
  if(xSemaphoreTake(s_write_lock, 0) == pdTRUE){
    xStreamBufferSend(s_buffer, line, n, 0);
    xSemaphoreGive(s_write_lock);
  }
  return ret;
}

static void sender_task(void*)
{
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  struct sockaddr_in dest = {};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(CONFIG_QUTEE_LOG_UDP_PORT);
  inet_pton(AF_INET, CONFIG_MICRO_ROS_AGENT_IP, &dest.sin_addr);
  char chunk[1024];
  while(true){
    size_t n = xStreamBufferReceive(s_buffer, chunk, sizeof(chunk), portMAX_DELAY);
    if(n > 0)
      sendto(sock, chunk, n, 0, (struct sockaddr*) &dest, sizeof(dest));
  }
}

void udp_log_init()
{
  s_buffer = xStreamBufferCreate(BUFFER_SIZE, 1);
  s_write_lock = xSemaphoreCreateMutex();
  if(s_buffer && s_write_lock)
    s_usb_vprintf = esp_log_set_vprintf(log_vprintf);
}

void udp_log_start()
{
  if(!s_usb_vprintf)
    return;
  xTaskCreate(sender_task, "udp_log", 3072, nullptr, 2, nullptr);
  ESP_LOGI("UDP log", "Sending logs to " CONFIG_MICRO_ROS_AGENT_IP ":%d", CONFIG_QUTEE_LOG_UDP_PORT);
}

#else

void udp_log_init() {}
void udp_log_start() {}

#endif
