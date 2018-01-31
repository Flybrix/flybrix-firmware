/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "cobs.h"

size_t cobsEncode(uint8_t* dst_ptr, const uint8_t* src_begin, const uint8_t* src_end) {
    uint8_t* dst_counter{dst_ptr++};
    uint8_t* dst_begin{dst_ptr};
    *dst_counter = 0;
    while (src_begin != src_end) {
        if (*dst_counter == 0xFE) {
            *dst_counter = 0xFF;
            dst_counter = dst_ptr++;
            *dst_counter = 0;
        }

        uint8_t val{*src_begin++};
        ++*dst_counter;

        if (val) {
            *dst_ptr++ = val;
        } else {
            dst_counter = dst_ptr++;
            *dst_counter = 0;
        }
    }
    return dst_ptr - dst_begin + 1;
}

std::size_t cobsDecode(uint8_t* dst_ptr, const uint8_t* src_ptr) {
    uint8_t* dst_start{dst_ptr};
    std::size_t leftover_length{0};
    bool append_zero = false;

    while (*src_ptr) {
        if (!leftover_length) {
            if (append_zero)
                *dst_ptr++ = 0;
            leftover_length = *src_ptr++ - 1;
            append_zero = leftover_length < 0xFE;
        } else {
            --leftover_length;
            *dst_ptr++ = *src_ptr++;
        }
    }

    return leftover_length ? 0 : dst_ptr - dst_start;
}
