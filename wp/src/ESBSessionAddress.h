/*
 * ESBSessionAddress.h
 *
 * Copyright PathTrack Limited 2026
 *
 * Shared by the tag and the wireless programmer, the two copies must stay identical.
 */

#ifndef ESBSESSIONADDRESS_H_
#define ESBSESSIONADDRESS_H_

#include <stdint.h>

/* Pipe 0 base address used after PAIR_ACK, so other tag/programmer pairs on the channel neither receive nor ESB-ACK
 * this session's packets. Both sides know the tag's channel hopping seed and both IDs by then. The seed alone only
 * has 128 values, so the IDs make the address effectively unique per pair. The prefix stays as it is. */
static inline void ESBSession_baseAddress(const uint8_t pairingAddr[4], uint16_t seed, uint32_t baseID,
                                          uint32_t tagID, uint8_t addr[4]) {
    uint32_t hash = seed ^ (baseID * 0x9E3779B1u) ^ (tagID * 0x85EBCA77u);
    uint8_t matchesPairing = 1;

    /* MurmurHash3 finaliser, so similar IDs and seeds still give unrelated addresses */
    hash ^= hash >> 16;
    hash *= 0x85EBCA6Bu;
    hash ^= hash >> 13;
    hash *= 0xC2B2AE35u;
    hash ^= hash >> 16;

    for(uint8_t i = 0; i < 4; i++) {
        uint8_t byte = (uint8_t)(hash >> (8 * i));

        /* 0x55 and 0xAA look like the preamble, and 0x00 and 0xFF have too few level shifts, all of which Nordic
         * warn raise the packet error rate */
        if((byte == 0x00) || (byte == 0xFF) || (byte == 0x55) || (byte == 0xAA)) {
            byte ^= 0x0F;
        }
        addr[i] = byte;
        if(byte != pairingAddr[i]) {
            matchesPairing = 0;
        }
    }

    if(matchesPairing) {
        addr[0] ^= 0x0F;
    }
}

#endif /* ESBSESSIONADDRESS_H_ */
