/**
 * @file canopen_types.h
 * @brief CANopen CiA 301/401 core types and CanFestival objdictgen compatibility
 *
 * This header defines data types, object access specifiers, NMT state codes,
 * SDO abort codes, and object dictionary data structures compatible with
 * CanFestival objdictgen (gen_cfile.py) and standard CANopen specifications.
 *
 * Compatible with STM32F407, STM32F767, and host test environments.
 */

#ifndef CANOPEN_TYPES_H
#define CANOPEN_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * CanFestival-compatible Basic Data Types
 * ============================================================ */
typedef uint8_t   UNS8;
typedef uint16_t  UNS16;
typedef uint32_t  UNS32;
typedef uint64_t  UNS64;

typedef int8_t    INTEGER8;
typedef int16_t   INTEGER16;
typedef int32_t   INTEGER32;
typedef int64_t   INTEGER64;

typedef float     REAL32;
typedef double    REAL64;

/* ============================================================
 * CiA 301 Data Type Identifiers
 * ============================================================ */
#ifndef boolean
#define boolean         0x01
#endif
#ifndef int8
#define int8            0x02
#endif
#ifndef int16
#define int16           0x03
#endif
#ifndef int32
#define int32           0x04
#endif
#ifndef uint8
#define uint8           0x05
#endif
#ifndef uint16
#define uint16          0x06
#endif
#ifndef uint32
#define uint32          0x07
#endif
#ifndef real32
#define real32          0x08
#endif
#ifndef visible_string
#define visible_string  0x09
#endif
#ifndef octet_string
#define octet_string    0x0A
#endif
#ifndef unicode_string
#define unicode_string  0x0B
#endif
#ifndef time_of_day
#define time_of_day     0x0C
#endif
#ifndef time_diff
#define time_diff       0x0D
#endif
#ifndef domain
#define domain          0x0F
#endif

/* ============================================================
 * Object Access Types (objdictgen compatibility)
 * ============================================================ */
#ifndef ro
#define ro              0x05U   /**< Read-only access */
#endif
#ifndef wo
#define wo              0x03U   /**< Write-only access */
#endif
#ifndef rw
#define rw              0x07U   /**< Read-write access */
#endif
#ifndef const_access
#define const_access    0x01U   /**< Constant value access */
#endif

#define CANOPEN_ACCESS_RO          ro
#define CANOPEN_ACCESS_WO          wo
#define CANOPEN_ACCESS_RW          rw
#define CANOPEN_ACCESS_CONST       const_access

#define CANOPEN_ACCESS_READ_MASK   0x04U  /**< CanFestival bit 2: Read permission */
#define CANOPEN_ACCESS_WRITE_MASK  0x02U  /**< CanFestival bit 1: Write permission */

/* ============================================================
 * Object Dictionary Structures (CanFestival gen_cfile.py compatible)
 * ============================================================ */

/**
 * @brief Subindex definition for a single OD variable
 */
typedef struct {
    UNS8   bAccessType;   /**< Access type: ro, wo, rw, const_access */
    UNS8   bDataType;     /**< CiA 301 data type code */
    UNS32  size;          /**< Size of data object in bytes */
    void  *pObject;       /**< Pointer to the actual data variable */
} subindex;

/**
 * @brief Index table entry grouping subindices
 */
typedef struct {
    subindex *pSubindex;   /**< Array of subindices */
    UNS8      bEndpoints;  /**< Number of subindices (highest subindex + 1) */
    UNS16     index;       /**< 16-bit CANopen object index (e.g. 0x1000) */
} indextable;

/**
 * @brief Object Dictionary container
 */
typedef struct {
    const indextable *entries; /**< Array of indextable entries (must be sorted by index) */
    UNS16             count;   /**< Total number of index entries */
} canopen_od_t;

/* ============================================================
 * NMT State Machine (CiA 301)
 * ============================================================ */
typedef enum {
    NMT_STATE_INITIALISATION    = 0x00,
    NMT_STATE_STOPPED           = 0x04,
    NMT_STATE_OPERATIONAL       = 0x05,
    NMT_STATE_PRE_OPERATIONAL   = 0x7F
} canopen_nmt_state_t;

/** NMT Master Command Specifiers */
#define NMT_CMD_START_NODE              0x01U
#define NMT_CMD_STOP_NODE               0x02U
#define NMT_CMD_ENTER_PRE_OPERATIONAL   0x80U
#define NMT_CMD_RESET_NODE              0x81U
#define NMT_CMD_RESET_COMMUNICATION     0x82U

/* ============================================================
 * SDO Abort Codes (CiA 301)
 * ============================================================ */
#define OD_SUCCESSFUL                           0x00000000UL
#define SDO_ABORT_TOGGLE_BIT_NOT_ALTERNATED     0x05030000UL
#define SDO_ABORT_SDO_PROTOCOL_TIMED_OUT        0x05040000UL
#define SDO_ABORT_CMD_SPECIFIER_INVALID         0x05040001UL
#define SDO_ABORT_INVALID_BLOCK_SIZE            0x05040002UL
#define SDO_ABORT_INVALID_SEQ_NUMBER            0x05040003UL
#define SDO_ABORT_CRC_ERROR                     0x05040004UL
#define SDO_ABORT_OUT_OF_MEMORY                 0x05040005UL
#define SDO_ABORT_UNSUPPORTED_ACCESS            0x06010000UL
#define SDO_ABORT_READ_NOT_ALLOWED              0x06010001UL
#define SDO_ABORT_WRITE_NOT_ALLOWED             0x06010002UL
#define SDO_ABORT_NO_SUCH_OBJECT                0x06020000UL
#define SDO_ABORT_CANNOT_MAP_TO_PDO             0x06040041UL
#define SDO_ABORT_PDO_LENGTH_EXCEEDED           0x06040042UL
#define SDO_ABORT_GENERAL_PARAM_INCOMPATIBLE    0x06040043UL
#define SDO_ABORT_INTERNAL_DEVICE_INCOMPATIBLE  0x06040047UL
#define SDO_ABORT_HARDWARE_ERROR                0x06060000UL
#define SDO_ABORT_DATA_LENGTH_MISMATCH          0x06070010UL
#define SDO_ABORT_DATA_LENGTH_TOO_HIGH          0x06070012UL
#define SDO_ABORT_DATA_LENGTH_TOO_SHORT         0x06070013UL
#define SDO_ABORT_NO_SUCH_SUBINDEX              0x06090011UL
#define SDO_ABORT_VALUE_RANGE_EXCEEDED          0x06090030UL
#define SDO_ABORT_VALUE_TOO_HIGH                0x06090031UL
#define SDO_ABORT_VALUE_TOO_LOW                 0x06090032UL
#define SDO_ABORT_MAX_VALUE_LESS_MIN_VALUE      0x06090036UL
#define SDO_ABORT_GENERAL_ERROR                 0x08000000UL
#define SDO_ABORT_DATA_CANNOT_BE_STORED         0x08000020UL
#define SDO_ABORT_DATA_CANNOT_BE_TRANSFERRED    0x08000022UL

/* CanFestival compatibility aliases */
#define OD_VALUE_RANGE_EXCEEDED                 SDO_ABORT_VALUE_RANGE_EXCEEDED
#define OD_NO_SUCH_OBJECT                       SDO_ABORT_NO_SUCH_OBJECT
#define OD_NO_SUCH_SUBINDEX                     SDO_ABORT_NO_SUCH_SUBINDEX
#define OD_READ_NOT_ALLOWED                     SDO_ABORT_READ_NOT_ALLOWED
#define OD_WRITE_NOT_ALLOWED                    SDO_ABORT_WRITE_NOT_ALLOWED

/* ============================================================
 * Standard CAN Frame & Transmit Function Type
 * ============================================================ */
typedef struct {
    uint32_t cob_id;    /**< 11-bit standard or 29-bit extended CAN ID */
    uint8_t  len;       /**< Data length 0..8 */
    uint8_t  data[8];   /**< Payload */
    uint8_t  rtr;       /**< 1 = Remote Transmission Request, 0 = Data frame */
} canopen_frame_t;

/**
 * @brief CAN frame transmit callback function pointer
 * @param frame Pointer to frame to transmit
 * @return 0 on success, negative error code on failure
 */
typedef int (*canopen_tx_func_t)(const canopen_frame_t *frame);

/* ============================================================
 * Standard Base COB-IDs (CiA 301 Pre-defined Connection Set)
 * ============================================================ */
#define CANOPEN_COB_NMT                 0x000U
#define CANOPEN_COB_SYNC                0x080U
#define CANOPEN_COB_TIME                0x100U
#define CANOPEN_COB_EMCY_BASE           0x080U
#define CANOPEN_COB_TPDO1_BASE          0x180U
#define CANOPEN_COB_RPDO1_BASE          0x200U
#define CANOPEN_COB_TPDO2_BASE          0x280U
#define CANOPEN_COB_RPDO2_BASE          0x300U
#define CANOPEN_COB_TPDO3_BASE          0x380U
#define CANOPEN_COB_RPDO3_BASE          0x400U
#define CANOPEN_COB_TPDO4_BASE          0x480U
#define CANOPEN_COB_RPDO4_BASE          0x500U
#define CANOPEN_COB_TSDO_BASE           0x580U  /**< Server Tx / Client Rx */
#define CANOPEN_COB_RSDO_BASE           0x600U  /**< Client Tx / Server Rx */
#define CANOPEN_COB_HEARTBEAT_BASE      0x700U

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_TYPES_H */
