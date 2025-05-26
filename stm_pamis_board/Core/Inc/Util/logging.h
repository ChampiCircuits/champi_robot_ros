#ifndef INC_UTIL_LOGGING_H_
#define INC_UTIL_LOGGING_H_

#include "stdio.h"
#include "usart.h"
#include "Config/Config.h"

// ANSI color codes
#define ESC "\x1B"
#define COLOR_RESET   ESC "[0m"
#define COLOR_DEBUG   ESC "[36m"
#define COLOR_INFO    ESC "[32m"
#define COLOR_WARN    ESC "[33m"
#define COLOR_ERROR   ESC "[31m"

#if ENABLE_LOG_DEBUG

#define LOG_DEBUG(logger, fmt, ...) \
{printf(COLOR_DEBUG "[%s] [DEBUG] " fmt COLOR_RESET "\n", logger, ##__VA_ARGS__);}

#define LOG_DEBUG_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {printf(COLOR_DEBUG "[%s] [DEBUG] " fmt COLOR_RESET "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#else

#define LOG_DEBUG(logger, fmt, ...) {}

#define LOG_DEBUG_THROTTLE(logger, N, fmt, ...) {}

#endif // ENABLE_LOG_DEBUG

#define LOG_INFO(logger, fmt, ...) \
{printf(COLOR_INFO "[%s] [INFO] " fmt COLOR_RESET "\n", logger, ##__VA_ARGS__);}

#define LOG_WARN(logger, fmt, ...) \
{printf(COLOR_WARN "[%s] [WARN] " fmt COLOR_RESET "\n", logger, ##__VA_ARGS__);}

#define LOG_ERROR(logger, fmt, ...) \
{printf(COLOR_ERROR "[%s] [ERROR] " fmt COLOR_RESET "\n", logger, ##__VA_ARGS__);}

#define LOG_INFO_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {printf(COLOR_INFO "[%s] [INFO] " fmt COLOR_RESET "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#define LOG_WARN_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {printf(COLOR_WARN "[%s] [WARN] " fmt COLOR_RESET "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#define LOG_ERROR_THROTTLE(logger, N, fmt, ...) \
{static int n = 0; \
if (n==0) {printf(COLOR_ERROR "[%s] [ERROR] " fmt COLOR_RESET "\n", logger, ##__VA_ARGS__);} \
n += 1; n = n%N;}

#endif /* INC_UTIL_LOGGING_H_ */
