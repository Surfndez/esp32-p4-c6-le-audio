#pragma once

#include <stdint.h>

void aes_ref_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);
/* Same signature as cis_platform_ops_t.aes128. */
void aes_ref_platform(void *ctx, const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);
