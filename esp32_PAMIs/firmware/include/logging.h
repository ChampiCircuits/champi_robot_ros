#ifndef FIRMWARE_LOGGING_H
#define FIRMWARE_LOGGING_H

#include <Arduino.h>

#ifndef FIRMWARE_ENABLE_LOGS
#define FIRMWARE_ENABLE_LOGS 0
#endif

#if FIRMWARE_ENABLE_LOGS

#define LOG_DEBUG(logger, fmt, ...) \
{Serial.printf("[%s] [DEBUG] " fmt "\n", logger, ##__VA_ARGS__);}

#define LOG_DEBUG_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {Serial.printf("[%s] [DEBUG] " fmt "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#define LOG_INFO(logger, fmt, ...) \
{Serial.printf("[%s] [INFO] " fmt "\n", logger, ##__VA_ARGS__);}

#define LOG_WARN(logger, fmt, ...) \
{Serial.printf("[%s] [WARN] " fmt "\n", logger, ##__VA_ARGS__);}

#define LOG_ERROR(logger, fmt, ...) \
{Serial.printf("[%s] [ERROR] " fmt "\n", logger, ##__VA_ARGS__);}

#define LOG_INFO_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {Serial.printf("[%s] [INFO] " fmt "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#define LOG_WARN_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {Serial.printf("[%s] [WARN] " fmt "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#define LOG_ERROR_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {Serial.printf("[%s] [ERROR] " fmt "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#else

#define LOG_DEBUG(logger, fmt, ...) {}

#define LOG_DEBUG_THROTTLE(logger, N, fmt, ...) {}

#define LOG_INFO(logger, fmt, ...) {}

#define LOG_WARN(logger, fmt, ...) {}

#define LOG_ERROR(logger, fmt, ...) {}

#define LOG_INFO_THROTTLE(logger, N, fmt, ...) {}

#define LOG_WARN_THROTTLE(logger, N, fmt, ...) {}

#define LOG_ERROR_THROTTLE(logger, N, fmt, ...) {}

#endif // FIRMWARE_ENABLE_LOGS

#endif //FIRMWARE_LOGGING_H