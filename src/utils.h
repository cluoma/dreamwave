/**
 * utils.h
 *
 * Collection of utility functions and macros
 *
 * Copyright (c) 2025 Colin Luoma
 */

#ifndef UTILS_H
#define UTILS_H

#ifdef DEBUG
#define DEBUG_PRINT(fmt, args...) \
        printf("DEBUG: %s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, ## args)
#else
#define DEBUG_PRINT(fmt, ...) ((void)0)
#endif

#endif //UTILS_H
