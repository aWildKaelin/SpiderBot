/*
Copyright (C) 2026 aWildKaelin

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _RING_CRSF_PARSER_H_
#define _RING_CRSF_PARSER_H_
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdio.h>

#include "ring_buffer.h"
#include "crsf_packets.h"


typedef enum crsf_states{
    WAIT_SYNC, READ_LENGTH, AWAIT_PACKET, CHECK_CRC, PARSE
}crsf_states;


typedef struct crsf_parser crsf_parser;

typedef struct crsf_parser{
    uint16_t channels[16];
    uint32_t lastChannelsRcv;
    crsf_link_statistics statistics;

    void (*rc_channels_callback)(const struct crsf_parser* parser);
    void (*link_statistics_callback)(const struct crsf_parser* parser);
    void (*failsafe_callback)(const struct crsf_parser* parser);

    uart_inst_t *uart;
    crsf_states state;

    uint8_t rawPacket[62];
    uint8_t cursor;

    bool failsafe;

    uint8_t packetLength;

    ringBuffer buffer;
} crsf_parser;



// https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#crc
uint8_t crc8(const uint8_t * ptr, uint8_t len);


void CRSFParser_init(crsf_parser *parser, uart_inst_t *uart);
void CRSFParser_destroy(crsf_parser *parser);

void CRSFParser_setChannelsCallback(struct crsf_parser *parser, void (*callback)(const struct crsf_parser *parser));
void CRSFParser_setStatisticsCallback(struct crsf_parser *parser, void (*callback)(const struct crsf_parser *parser));
void CRSFParser_setFailsafeCallback(struct crsf_parser *parser, void (*callback)(const struct crsf_parser *parser));

void CRSFParser_setFailsafe();

void CRSFParser_update(crsf_parser *parser);

bool CRSFParser_isConnected(crsf_parser *parser);

#endif

#ifdef __cplusplus
}
#endif
