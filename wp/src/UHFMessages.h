/*
 * UHFMessages.h
 *
 * Created: 11/08/2026 14:34:50
 *  Author: Will
 */


#ifndef UHFMESSAGES_H_
#define UHFMESSAGES_H_

#include <stdint.h>

/* Pack all these struct/union defs to 1 byte as they have been explicitly aligned with nesting in mind */
#pragma pack(push, 1)

typedef struct ptTimePacked_t {
    uint8_t data[5];
} ptTimePacked_t;

/* ------------------------------------------ Tag <-> BS (32-bit) ----------------------------------------- */

typedef struct picofix32UHFHeader_t {
    uint8_t u8picofixTag;
    uint8_t u8checkA;
    uint8_t u8checkB;
    uint8_t u8messageType;
    uint32_t u32senderID;
    uint32_t u32receiverID;
    uint16_t u16length;
} picofix32UHFHeader_t;

typedef struct picofix32UHF_dataToSend_t {
    uint16_t u16channelHoppingSeed;
    uint16_t u16numPages;
    ptTimePacked_t time;
    uint8_t u8batteryVoltage;
    uint8_t u8battType;
    int8_t i8RSSI; /* Special case: it's not part of the transmission and as such not returned in datLength */
} picofix32UHF_dataToSend_t;

typedef struct picofix32UHF_dataToOffload_t { /* Paired with BS version of picofix32UHF_dataToSend_t */
    uint16_t u16numPages;
    ptTimePacked_t time;
    uint8_t u8batteryVoltage;
    uint8_t u8battType;
    int8_t i8RSSI; /* Special case: it's not part of the transmission and as such not returned in datLength */
} picofix32UHF_dataToOffload_t;

typedef struct picofix32UHF_dataStop_t {
    int8_t i8tagRSSI;
} picofix32UHF_dataStop_t;

typedef struct picofix32UHF_dataTransfer_t {
    uint8_t u8data[512];
} picofix32UHF_dataTransfer_t;

typedef struct picofix32UHF_dataResponseContinue {
    uint8_t u8transferMode;
    uint8_t i8RSSI; /* Special case: We want access to this when calling DATA_STOP, however it's not part of the transmission and as such not returned in datLength */
} picofix32UHF_dataResponseContinue;

typedef struct picofix32UHF_pairInit_t {
    uint16_t u16channelHoppingSeed;
    uint8_t u8batteryVoltage;
} picofix32UHF_pairInit_t;

typedef struct  picofix32UHF_pairAck_t {
    uint8_t u8status;
} picofix32UHF_pairAck_t;

typedef union picofix32UHF_tagTXMessage_t {
    picofix32UHF_dataToSend_t dataToSend;
    picofix32UHF_dataToOffload_t dataToOffload;
    picofix32UHF_dataStop_t dataStop;
    picofix32UHF_dataTransfer_t dataTransfer;
    picofix32UHF_pairInit_t pairInit;
    uint8_t blob[100];
} picofix32UHF_tagTXMessage_t;

typedef union picofix32UHF_tagRXMessage_t {
    picofix32UHF_dataResponseContinue dataResponseContinue;
    picofix32UHF_pairAck_t pairAck;
    uint8_t blob[100];
} picofix32UHF_tagRXMessage_t;

typedef struct picofix32UHFTagTX_t {
    picofix32UHFHeader_t header;
    picofix32UHF_tagTXMessage_t message;
    uint8_t u8overflow[10];
} picofix32UHFTagTX_t;

typedef struct picofix32UHFTagRX_t {
    picofix32UHFHeader_t header;
    picofix32UHF_tagRXMessage_t message;
    uint8_t u8overflow[10];
} picofix32UHFTagRX_t;

#pragma pack(pop)

/* --------------------------------------- Common defines + enums -------------------------------------------- */

#define PICOFIX_32_MESSAGE_SIZE_DATA_TO_SEND				(sizeof(picofix32UHFHeader_t) + sizeof(picofix32UHF_dataToSend_t) - 1) /* Remove i8RSSI from the length since that isn't part of the transmission */
#define PICOFIX_32_MESSAGE_SIZE_DATA_STOP					(sizeof(picofix32UHFHeader_t) + sizeof(picofix32UHF_dataStop_t))
#define PICOFIX_32_MESSAGE_SIZE_DATA_TRANSFER				(sizeof(picofix32UHFHeader_t) + sizeof(picofix32UHF_dataTransfer_t))
#define PICOFIX_32_MESSAGE_SIZE_DATA_RESPONSE_CONTINUE		(sizeof(picofix32UHFHeader_t) + sizeof(picofix32UHF_dataResponseContinue) - 1) /* Remove u8tagRSSI from the length since that isn't part of the transmission */

/* Messages with no content */
#define PICOFIX_32_MESSAGE_SIZE_NOT_AUTHORISED				sizeof(picofix32UHFHeader_t)
#define PICOFIX_32_MESSAGE_SIZE_DATA_RESPONSE_FINAL			sizeof(picofix32UHFHeader_t)
#define PICOFIX_32_MESSAGE_SIZE_CONT_ACK					sizeof(picofix32UHFHeader_t)

#define PICOFIX_32_MESSAGE_SIZE_PAIR_INIT					(sizeof(picofix32UHFHeader_t) + sizeof(picofix32UHF_pairInit_t))
#define PICOFIX_32_MESSAGE_SIZE_CONFIG_ACK					(sizeof(picofix32UHFHeader_t) + sizeof(picofix32UHF_pairAck_t))
#define PICOFIX_32_MESSAGE_SIZE_DATA_TO_OFFLOAD				(sizeof(picofix32UHFHeader_t) + sizeof(picofix32UHF_dataToOffload_t) - 1) /* Remove i8RSSI from the length since that isn't part of the transmission */

#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_CONNECT1				(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughHeader_t) + sizeof(picofixV3_connect1_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_CONNECT2				(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_connect2_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_CONNECT3				(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughHeader_t) + sizeof(picofixV3_connect3_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_CONNECT4				(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_connect4_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_CONNECT_ACK			(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughReceiveHeader_t) + sizeof(picofixV3_connectAck_t))

#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_PING					(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughHeader_t) + sizeof(picofixV3_ping_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_DISABLE_ACK			(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughHeader_t) + sizeof(picofixV3_disable_ack_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_TIME_ACK				(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughHeader_t) + sizeof(picofixV3_time_ack_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_CONFIG_ACK			(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughHeader_t) + sizeof(picofixV3_config_ack_t))

#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_KEEPALIVE				(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughReceiveHeader_t) + sizeof(picofixV3_keepAlive_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_DISABLE_REQUEST		(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughReceiveHeader_t) + sizeof(picofixV3_disableRequest_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_TIME_UPDATE			(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughReceiveHeader_t) + sizeof(picofixV3_time_update_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_CONFIG_UPDATE			(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughReceiveHeader_t) + sizeof(picofixV3_config_update_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_RECONNECT				(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughReceiveHeader_t) + sizeof(picofixV3_reconnect_t))
#define PICOFIX_PCBIN_V3_MESSAGE_SIZE_POINTER_UPDATE		(sizeof(picofix32UHFHeader_t) + sizeof(picofixV3_passThroughReceiveHeader_t) + sizeof(picofixV3_pointer_update_t))

#define PICOFIX_8_MESSAGE_SIZE_CONFIG_REQUEST				(sizeof(picofix8UHFHeader_t) + sizeof(picofix8UHF_configRequest_t) - 1) /* Remove i8RSSI from the length since that isn't part of the transmission */
#define PICOFIX_8_MESSAGE_SIZE_CONFIG_REQUEST_V2			(sizeof(picofix8UHFHeader_t) + sizeof(picofix8UHF_configRequest_v2_t) - 1) /* Remove i8RSSI from the length since that isn't part of the transmission */
#define PICOFIX_8_MESSAGE_SIZE_CONFIG_DELIVERY_RESPONSE		(sizeof(picofix8UHFHeader_t) + sizeof(picofix8UHF_configDeliveryResponse_t))
#define PICOFIX_8_MESSAGE_SIZE_DATA_TO_SEND					(sizeof(picofix8UHFHeader_t) + sizeof(picofix8UHF_dataToSend_t) - 1) /* Remove i8RSSI from the length since that isn't part of the transmission */
#define PICOFIX_8_MESSAGE_SIZE_DATA_STOP					(sizeof(picofix8UHFHeader_t) + sizeof(picofix8UHF_dataStop_t))
#define PICOFIX_8_MESSAGE_SIZE_DATA_TRANSFER				(sizeof(picofix8UHFHeader_t) + sizeof(picofix8UHF_dataTransfer_t))
#define PICOFIX_8_MESSAGE_SIZE_CONFIG_DELIVERY				(sizeof(picofix8UHFHeader_t) + sizeof(picofix8UHF_configDelivery_t))
#define PICOFIX_8_MESSAGE_SIZE_PARAMETER_DELIVERY			(sizeof(picofix8UHFHeader_t) + sizeof(picofix8UHF_parameterDelivery_t))
#define PICOFIX_8_MESSAGE_SIZE_DATA_RESPONSE_CONTINUE		(sizeof(picofix8UHFHeader_t) + sizeof(picofix8UHF_dataResponseContinue))

/* Messages with no content */
#define PICOFIX_8_MESSAGE_SIZE_PARAMETER_DELIVERY_ACK		sizeof(picofix8UHFHeader_t)
#define PICOFIX_8_MESSAGE_SIZE_PARAMETER_DELIVERY_NACK		sizeof(picofix8UHFHeader_t)
#define PICOFIX_8_MESSAGE_SIZE_NOT_AUTHORISED				sizeof(picofix8UHFHeader_t)
#define PICOFIX_8_MESSAGE_SIZE_CONFIG_REQUEST_ACK			sizeof(picofix8UHFHeader_t)
#define PICOFIX_8_MESSAGE_SIZE_DATA_RESPONSE_FINAL			sizeof(picofix8UHFHeader_t)

#define UHF_PICOFIX_8_TAG		0x88
#define UHF_PICOFIX_8_BS		0xBB
#define UHF_PICOFIX_32_TAG		0x83
#define UHF_PICOFIX_32_BS		0xB3

enum {
    /* Tag -> BaseStation : Communication start */
    PICOFIX_CONFIG_REQUEST				= 0x01,
    PICOFIX_DATA_TO_SEND,
    PICOFIX_CONFIG_REQUEST_V2,
    /* Tag -> BaseStation : Config */
    PICOFIX_CONFIG_DELIVERY_RESPONSE	= 0x10,
    PICOFIX_PARAM_DELIVERY_ACK,
    PICOFIX_PARAM_DELIVERY_NACK,
    /* Tag -> BaseStation : Data */
    PICOFIX_NOT_AUTHORISED				= 0x20,
    PICOFIX_DATA_STOP,
    PICOFIX_DATA_TRANSFER,
    /* BaseStation -> Tag : Config */
    PICOFIX_CONFIG_REQUEST_ACK			= 0x80,
    PICOFIX_CONFIG_DELIVERY,
    PICOFIX_PARAM_DELIVERY,
    /* BaseStation -> Tag : Data */
    PICOFIX_DATA_ACK_CONTINUE			= 0x90,
    PICOFIX_DATA_ACK_FINAL,
    PICOFIX_DATA_NACK_CONTINUE,
    PICOFIX_DATA_NACK_FINAL,

    PICOFIX_V3_CONFIG_INIT				= 0xA0,
    PICOFIX_V3_CONFIG_ACK				= 0xA1,
    PICOFIX_V3_DATA_TO_OFFLOAD			= 0xA2,

    PICOFIX_V3_BASE_PASSTHROUGH			= 0xB0,
    PICOFIX_V3_BASE_PASSTHROUGH_CONT,
    PICOFIX_V3_BASE_CONT_ACK,

    PICOFIX_V3_TAG_PASSTHROUGH			= 0xC0,
    PICOFIX_V3_TAG_PASSTHROUGH_CONT,

    PICOFIX_V3_TAG_DISCONNECT			= 0xD0
};

enum {
    /* Tag -> BaseStation : Config */
    PICOFIX_PCBIN_V3_CONNECT1					= 0xA1,
    PICOFIX_PCBIN_V3_CONNECT2					= 0xA2,
    PICOFIX_PCBIN_V3_CONNECT3					= 0xA3,
    PICOFIX_PCBIN_V3_PING						= 0xA4,
    PICOFIX_PCBIN_V3_DISABLE_ACK				= 0xA5,
    PICOFIX_PCBIN_V3_TIME_ACK					= 0xA6,
    PICOFIX_PCBIN_V3_CONFIG_ACK					= 0xA7,
    PICOFIX_PCBIN_V3_CONNECT4					= 0xA8,
    PICOFIX_PCBIN_V3_CONNECT2_FAILED			= 0xAA,
    /* BaseStation -> Tag : Config */
    PICOFIX_PCBIN_V3_CONNECT_ACK				= 0xB3,
    PICOFIX_PCBIN_V3_KEEPALIVE					= 0xB4,
    PICOFIX_PCBIN_V3_DISABLE_REQUEST			= 0xB5,
    PICOFIX_PCBIN_V3_TIME_UPDATE				= 0xB6,
    PICOFIX_PCBIN_V3_CONFIG_UPDATE				= 0xB7,
    PICOFIX_PCBIN_V3_RECONNECT					= 0xB8,
    PICOFIX_PCBIN_V3_POINTER_UPDATE				= 0x8024,
    PICOFIX_PCBIN_V3_EEPROM_UPDATE				= 0x802F,
    PICOFIX_PCBIN_V3_OFFLOAD_DATA				= 0x8070
};

#endif /* UHFMESSAGES_H_ */