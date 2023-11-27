/**
 * @file test.c
 * @author Bronya19c
 * @brief 
 * @version 0.1
 * @date 2023-01-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <math.h>
#include "udf.h"

DEFINE_ON_DEMAND(hello_world) {
    Message("hello world!");
}