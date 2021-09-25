// Copyright (C) 2018-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

/*
 * Add logging capabilities over simple printf.
 * Allows 5 different logging levels:
 *
 * MVLOG_DEBUG = 0
 * MVLOG_INFO = 1
 * MVLOG_WARN = 2
 * MVLOG_ERROR = 3
 * MVLOG_FATAL = 4
 * Before including header, a unit name can be set, otherwise defaults to global. eg:
 *
 * #define MVLOG_UNIT_NAME unitname
 * #include <mvLog.h>
 * Setting log level through debugger can be done in the following way:
 * mset mvLogLevel_unitname 2
 * Will set log level to warnings and above
 */
#ifndef MVLOG_H__
#define MVLOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif


typedef enum mvLog_t{
    MVLOG_DEBUG = 0,
    MVLOG_INFO,
    MVLOG_WARN,
    MVLOG_ERROR,
    MVLOG_FATAL,
    MVLOG_LAST,
} mvLog_t;

// Windows-only
#if (defined (WINNT) || defined(_WIN32) || defined(_WIN64) )
#define __attribute__(x)
#define FUNCATTR_WEAK static
#else
#define FUNCATTR_WEAK
#endif

#define _MVLOGLEVEL(UNIT_NAME)  mvLogLevel_ ## UNIT_NAME
#define  MVLOGLEVEL(UNIT_NAME) _MVLOGLEVEL(UNIT_NAME)

#define MVLOG_STR(x) _MVLOG_STR(x)
#define _MVLOG_STR(x)  #x

#ifndef MVLOG_UNIT_NAME
#define MVLOG_UNIT_NAME global
#else
FUNCATTR_WEAK mvLog_t __attribute__ ((weak)) MVLOGLEVEL(MVLOG_UNIT_NAME) = MVLOG_LAST;
#endif


#ifndef MVLOG_MAXIMUM_THREAD_NAME_SIZE
#define MVLOG_MAXIMUM_THREAD_NAME_SIZE 16
#endif

#define UNIT_NAME_STR MVLOG_STR(MVLOG_UNIT_NAME)


extern mvLog_t MVLOGLEVEL(global);
extern mvLog_t MVLOGLEVEL(default);

int __attribute__ ((unused)) logprintf(mvLog_t curLogLvl, mvLog_t lvl, const char * func, const int line, const char * format, ...);

#define mvLog(lvl, format, ...)                              \
    logprintf(MVLOGLEVEL(MVLOG_UNIT_NAME), lvl, __func__, __LINE__, format, ##__VA_ARGS__)

// Set log level for the current unit. Note that the level must be smaller than the global default
inline void mvLogLevelSet(mvLog_t lvl){
    if(lvl < MVLOG_LAST){
        MVLOGLEVEL(MVLOG_UNIT_NAME) = lvl;
    }
}

// Set the global log level. Can be used to prevent modules from hiding messages (enable all of them with a single change)
// This should be an application setting, not a per module one
inline void mvLogDefaultLevelSet(mvLog_t lvl){
    if(lvl <= MVLOG_LAST){
        MVLOGLEVEL(default) = lvl;
    }
}


#ifdef __cplusplus
}
#endif

#endif


