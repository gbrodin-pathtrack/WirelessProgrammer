#include <stdint.h>

typedef struct serialPacketHeader_t{
    uint16_t u16header;
    uint16_t u16messageType;
    uint16_t u16length;
}serialPacketHeader_t;

/************************* Bridge -> PC ***************************/
#define SERIAL_TX_MAX_PAYLOAD 1024

#define SERIAL_TX_PAIR_REQUEST  0x0080
#define SERIAL_TX_PAIR_REQUEST_LEN 1
typedef struct serialPairRequest_t{
    serialPacketHeader_t header;
    uint8_t u8devType;
} serialPairRequest_t;

#define SERIAL_TX_FW_INFO  0x0081
#define SERIAL_TX_FW_INFO_LEN 6
typedef struct serialFWInfo_t{
    serialPacketHeader_t header;
    uint8_t u8protocolVer;
    uint8_t u8padding;
    uint32_t u32gitRev;
} serialFWInfo_t;

#define SERIAL_TX_KEEP_ALIVE  0x0084
#define SERIAL_TX_KEEP_ALIVE_LEN 0
typedef struct serialKeepAlive_t{
    serialPacketHeader_t header;
}serialKeepAlive_t;

/* TEMPORARY */
#define SERIAL_TX_TAG_DATA 0x008F
typedef struct serialTagData_t{
    serialPacketHeader_t header;
    uint8_t payload[SERIAL_TX_MAX_PAYLOAD];
}serialTagData_t;

#define SERIAL_TX_HEADER 0x7470

typedef union serialTXMessage_t{
    serialPacketHeader_t header;
    serialPairRequest_t pairRequest;
    serialFWInfo_t fwInfo;
    serialKeepAlive_t keepAlive;
    serialTagData_t tagData;
    uint8_t blob[SERIAL_TX_MAX_PAYLOAD + sizeof(serialPacketHeader_t) + 2 /* CRC */];
}serialTXMessage_t;

/************************* PC -> Bridge ***************************/

#define SERIAL_RX_MAX_PAYLOAD 1024

#define SERIAL_RX_PAIR_REQUEST_ACK 0x0080
typedef struct serialPairRequestAck_t{
    serialPacketHeader_t header;
} serialPairRequestAck_t;

#define SERIAL_RX_FW_INFO_ACK 0x0003
typedef struct serialFWInfoAck_t{
    serialPacketHeader_t header;
    uint8_t u8status;
} serialFWInfoAck_t;

#define SERIAL_RX_KEEP_ALIVE_ACK 0x0084
typedef struct serialKeepAliveAck_t{
    serialPacketHeader_t header;
} serialKeepAliveAck_t;

/* TEMPORARY */
#define SERIAL_RX_TAG_DATA_ACK 0x008E
typedef struct serialTagDataAck_t{
    serialPacketHeader_t header;
}serialTagDataAck_t;

/* UNUSED FOR NOW */
#define SERIAL_RX_PASS_THROUGH 0x00B0
#define SERIAL_RX_PASS_THROUGH_DISCONNECT 0x00D0
typedef struct serialPassThrough_t{
    serialPacketHeader_t header;
    uint8_t payload[SERIAL_RX_MAX_PAYLOAD];
} serialPassThrough_t;

#define SERIAL_RX_HEADER 0x6870

typedef union serialRXMessage_t{
    serialPacketHeader_t header;
    serialPairRequestAck_t pairRequestAck;
    serialFWInfoAck_t fwInfoAck;
    serialKeepAliveAck_t keepAliveAck;
    serialPassThrough_t passThrough;
    serialTagDataAck_t tagDataAck;
    uint8_t blob[SERIAL_RX_MAX_PAYLOAD + sizeof(serialPacketHeader_t) + 2 /* CRC */];
}serialRXMessage_t;
