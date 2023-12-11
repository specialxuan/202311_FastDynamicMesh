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
#include "udf.h" // note: move udf.h to last

DEFINE_ON_DEMAND(Preprocess)
{
#if !RP_NODE
    Message("hello world!");
#endif

#if !RP_HOST
    Message("\n ---         This is Node          ---\n");
#endif
}