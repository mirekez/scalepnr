#include "debug.h"

int debug_level = 0;
uint8_t debug_module[65536/8] = {};

void stop_if_needed()
{
    *(char*)0 = 0;
}
