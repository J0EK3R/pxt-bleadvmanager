#include "CryptTool.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static uint8_t invert_8(const uint8_t value) 
{
    uint8_t result = 0;
    for (int index = 0; index < 8; ++index) {
        if ((value & (1 << index)) != 0) {
            result |= (1 << (7 - index));
        }
    }
    return result;
}

static uint16_t invert_16(const uint16_t value) 
{
    uint16_t result = 0;
    for (int index = 0; index < 16; ++index) {
        if ((value & (1 << index)) != 0) {
            result |= (1 << (15 - index));
        }
    }
    return result;
}

static uint16_t check_crc16(const uint8_t* array1, const uint8_t array1Length, const uint8_t* array2, const uint8_t array2Length) 
{
    uint16_t result = 0xffff;
    for (uint8_t index = 0; index < array1Length; ++index) {
        result ^= static_cast<uint16_t>(array1[array1Length - 1 - index] << 8);

        for (uint8_t local_24 = 0; local_24 < 8; ++local_24) {
            if ((result & 0x8000) == 0) {
                result <<= 1;
            } else {
                result = (result << 1) ^ 0x1021;
            }
        }
    }

    for (uint8_t index = 0; index < array2Length; ++index) {
        uint8_t cVar1 = invert_8(array2[index]);

        result ^= static_cast<uint16_t>(cVar1 << 8);

        for (uint8_t local_2c = 0; local_2c < 8; ++local_2c) {
            if ((result & 0x8000) == 0) {
                result <<= 1;
            } else {
                result = (result << 1) ^ 0x1021;
            }
        }
    }

    uint16_t result_inverse = invert_16(result);
    return static_cast<uint16_t>(result_inverse ^ 0xffff);
}

static void whitening_init(const uint8_t val, uint8_t ctx[7]) {
    ctx[0] = 1;
    ctx[1] = (val >> 5) & 1;
    ctx[2] = (val >> 4) & 1;
    ctx[3] = (val >> 3) & 1;
    ctx[4] = (val >> 2) & 1;
    ctx[5] = (val >> 1) & 1;
    ctx[6] = val & 1;
}

static uint8_t whitening_output(uint8_t ctx[7]) {
    uint8_t value_3 = ctx[3];
    uint8_t value_6 = ctx[6];
    ctx[3] = ctx[2];
    ctx[2] = ctx[1];
    ctx[1] = ctx[0];
    ctx[0] = ctx[6];
    ctx[6] = ctx[5];
    ctx[5] = ctx[4];
    ctx[4] = value_3 ^ value_6;
    return ctx[0];
}

static void whitening_encode(uint8_t data[], const uint8_t dataStartIndex, const uint8_t len, uint8_t ctx[7]) {
    for (uint8_t index = 0; index < len; ++index) {
        uint8_t currentByte = data[dataStartIndex + index];
        uint16_t currentResult = 0;
        for (uint8_t bitIndex = 0; bitIndex < 8; ++bitIndex) {
            uint8_t uVar2 = whitening_output(ctx);
            currentResult = (uVar2 ^ ((currentByte >> bitIndex) & 0x0001)) << bitIndex | currentResult;
        }
        data[dataStartIndex + index] = static_cast<uint8_t>(currentResult);
    }
}

void get_rf_payload(const uint8_t *addr, const uint8_t addrLength, const uint8_t *data, const uint8_t dataLength, const uint8_t ctxValue, uint8_t *rfPayload) {

    uint8_t data_offset = 0x12;     // 0x12 (18)
    uint8_t inverse_offset = 0x0f;  // 0x0f (15)

    uint8_t result_data_size = data_offset + addrLength + dataLength + 2;
    uint8_t resultbuf[result_data_size];

    resultbuf[15] = 0x71; // 0x71 (113)
    resultbuf[16] = 0x0f; // 0x0f (15)
    resultbuf[17] = 0x55; // 0x55 (85)

    // copy addrArray reverse into resultbuf with offset 18
    for (uint8_t index = 0; index < addrLength; index++)
    {
        //resultbuf[data_offset + addrLength - index - 1] = addr[index];
        resultbuf[index + data_offset] = addr[(addrLength - index) - 1];
    }

    // copy dataArray into resultbuf with offset 18 + addrLength
    for (uint8_t index = 0; index < dataLength; index++)
    {
        resultbuf[data_offset + addrLength + index] = data[index];
    }

    // crypt Bytes from position 15 to 22
    for (uint8_t index = inverse_offset; index < (addrLength + data_offset); index++)
    {
        resultbuf[index] = invert_8(resultbuf[index]);
    }

    // calc checksum und copy to array
    uint16_t checksum = check_crc16(addr, addrLength, data, dataLength);
    resultbuf[result_data_size - 2] = static_cast<uint8_t>(checksum & 0xff);
    resultbuf[result_data_size - 1] = static_cast<uint8_t>((checksum >> 8) & 0xff);

    // uint8_t[] ctx_0x3F = new uint8_t[7]; // int local_58[8];
    uint8_t ctx_0x3F[7];
    whitening_init(0x3f, ctx_0x3F); // 0x3f (63) -> ctx_0x3F = [1111111]
    whitening_encode(resultbuf, 0x12, addrLength + dataLength + 2, ctx_0x3F);

    // uint8_t ctx = new uint8_t[7];
    uint8_t ctx[7];
    whitening_init(ctxValue, ctx); // ctxValue= 0x25 (37) -> ctx = [1101110]
    whitening_encode(resultbuf, 0, result_data_size, ctx);

    // // resulting advertisment array has a length of constant 24 bytes
    // rfPayload = new uint8_t[24];

    uint8_t lengthResultArray = addrLength + dataLength + 5;

    // if (lengthResultArray > rfPayload.Length)
    // {
    //     return;
    // }

    // // Buffer.BlockCopy(resultbuf, 15, rfPayload, 0, lengthResultArray);
    memcpy(&rfPayload[7], &resultbuf[15], sizeof(uint8_t) * lengthResultArray);

    // fill rest of array
    for (uint8_t index = lengthResultArray; index < 24; index++)
    {
        rfPayload[index+ 7] = index + 1;
    }
}
