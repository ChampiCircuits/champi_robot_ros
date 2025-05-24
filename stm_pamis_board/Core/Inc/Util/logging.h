#ifndef INC_UTIL_LOGGING_H_
#define INC_UTIL_LOGGING_H_

#include "stdio.h"
#include "usart.h"
#include "Config/Config.h"

#if ENABLE_LOG_DEBUG

#define LOG_DEBUG(logger, fmt, ...) \
{printf("[%s] [DEBUG] " fmt "\n", logger, ##__VA_ARGS__);}

#define LOG_DEBUG_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {printf("[%s] [DEBUG] " fmt "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#else

#define LOG_DEBUG(logger, fmt, ...) {}

#define LOG_DEBUG_THROTTLE(logger, fmt, ...) {}

#endif // ENABLE_LOG_DEBUG


#define LOG_INFO(logger, fmt, ...) \
{printf("[%s] [INFO] " fmt "\n", logger, ##__VA_ARGS__);}

#define LOG_WARN(logger, fmt, ...) \
{printf("[%s] [WARN] " fmt "\n", logger, ##__VA_ARGS__);}

#define LOG_ERROR(logger, fmt, ...) \
{printf("[%s] [ERROR] " fmt "\n", logger, ##__VA_ARGS__);}

#define LOG_INFO_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {printf("[%s] [INFO] " fmt "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#define LOG_WARN_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {printf("[%s] [WARN] " fmt "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#define LOG_ERROR_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {printf("[%s] [ERROR] " fmt "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#endif /* INC_UTIL_LOGGING_H_ */
