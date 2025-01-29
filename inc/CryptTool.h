#ifndef CRYPTTOOL_H
#define CRYPTTOOL_H

#include "pxt.h"

void get_rf_payload(const uint8_t *addr, const uint8_t addrLength, const uint8_t *data, const uint8_t dataLength, const uint8_t ctxValue, uint8_t *rfPayload);

#endif