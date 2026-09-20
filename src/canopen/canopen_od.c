/**
 * @file canopen_od.c
 * @brief CANopen Object Dictionary implementation and default Automation Board OD
 */

#include "canopen/canopen_od.h"
#include <string.h>

/* ============================================================
 * Internal Callback Pointer & Weak Hooks
 * ============================================================ */
static canopen_od_post_write_callback_t s_post_write_cb = NULL;

void canopen_od_set_write_callback(canopen_od_post_write_callback_t cb)
{
    s_post_write_cb = cb;
}

__attribute__((weak)) void canopen_app_sync_inputs(void)
{
    /* Weak default: board-specific application can override */
}

__attribute__((weak)) void canopen_app_sync_outputs(void)
{
    /* Weak default: board-specific application can override */
}

/* ============================================================
 * Object Dictionary Engine Functions
 * ============================================================ */

const indextable *canopen_od_find_index(const canopen_od_t *od, UNS16 index)
{
    if (od == NULL || od->entries == NULL || od->count == 0) {
        return NULL;
    }

    for (UNS16 i = 0; i < od->count; i++) {
        if (od->entries[i].index == index) {
            return &od->entries[i];
        }
    }
    return NULL;
}

subindex *canopen_od_find_subindex(const canopen_od_t *od, UNS16 index, UNS8 bSubindex)
{
    const indextable *entry = canopen_od_find_index(od, index);
    if (entry == NULL || entry->pSubindex == NULL) {
        return NULL;
    }

    if (bSubindex >= entry->bEndpoints) {
        return NULL;
    }

    return &entry->pSubindex[bSubindex];
}

UNS32 canopen_od_read(const canopen_od_t *od, UNS16 index, UNS8 bSubindex,
                      void *pDest, UNS32 *pSize)
{
    if (od == NULL || pDest == NULL || pSize == NULL) {
        return SDO_ABORT_GENERAL_ERROR;
    }

    const indextable *entry = canopen_od_find_index(od, index);
    if (entry == NULL) {
        return SDO_ABORT_NO_SUCH_OBJECT;
    }

    if (bSubindex >= entry->bEndpoints) {
        return SDO_ABORT_NO_SUCH_SUBINDEX;
    }

    subindex *sub = &entry->pSubindex[bSubindex];
    if (sub->pObject == NULL) {
        return SDO_ABORT_NO_SUCH_OBJECT;
    }

    /* Check access rights: read allowed if Read bit is set or CONST */
    if ((sub->bAccessType & CANOPEN_ACCESS_READ_MASK) == 0 &&
        sub->bAccessType != CANOPEN_ACCESS_CONST) {
        return SDO_ABORT_READ_NOT_ALLOWED;
    }

    UNS32 bytes_to_copy = sub->size;
    if (*pSize < bytes_to_copy) {
        bytes_to_copy = *pSize;
    }

    memcpy(pDest, sub->pObject, bytes_to_copy);
    *pSize = sub->size;

    return OD_SUCCESSFUL;
}

UNS32 canopen_od_write(const canopen_od_t *od, UNS16 index, UNS8 bSubindex,
                       const void *pSrc, UNS32 size)
{
    if (od == NULL || pSrc == NULL) {
        return SDO_ABORT_GENERAL_ERROR;
    }

    const indextable *entry = canopen_od_find_index(od, index);
    if (entry == NULL) {
        return SDO_ABORT_NO_SUCH_OBJECT;
    }

    if (bSubindex >= entry->bEndpoints) {
        return SDO_ABORT_NO_SUCH_SUBINDEX;
    }

    subindex *sub = &entry->pSubindex[bSubindex];
    if (sub->pObject == NULL) {
        return SDO_ABORT_NO_SUCH_OBJECT;
    }

    /* Check access rights: write allowed only if Write bit is set */
    if ((sub->bAccessType & CANOPEN_ACCESS_WRITE_MASK) == 0) {
        return SDO_ABORT_WRITE_NOT_ALLOWED;
    }

    /* Size checking */
    if (sub->size > 0 && size > sub->size) {
        return SDO_ABORT_DATA_LENGTH_TOO_HIGH;
    }
    if (sub->size > 0 && size < sub->size) {
        return SDO_ABORT_DATA_LENGTH_TOO_SHORT;
    }

    memcpy(sub->pObject, pSrc, size);

    /* Invoke post-write hook if configured */
    if (s_post_write_cb != NULL) {
        s_post_write_cb(index, bSubindex, pSrc, size);
    }

    /* If writing to digital/analog outputs, notify weak application hook */
    if (index == 0x6200 || index == 0x6411) {
        canopen_app_sync_outputs();
    }

    return OD_SUCCESSFUL;
}

/* ============================================================
 * Default Automation Board Object Dictionary Data
 * ============================================================ */

/* 0x1000: Device Type (CiA 401 Digital & Analog I/O) */
static UNS32 Board_obj1000 = 0x00020191UL;
static subindex Board_Index1000[] = {
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1000 }
};

/* 0x1001: Error Register */
static UNS8 Board_obj1001 = 0x00U;
static subindex Board_Index1001[] = {
    { ro, uint8, sizeof(UNS8), (void*)&Board_obj1001 }
};

/* 0x1008: Manufacturer Device Name */
static char Board_obj1008[] = "STM32 Automation Board";
static subindex Board_Index1008[] = {
    { ro, visible_string, sizeof(Board_obj1008) - 1, (void*)Board_obj1008 }
};

/* 0x1009: Hardware Version */
static char Board_obj1009[] = "v1.0";
static subindex Board_Index1009[] = {
    { ro, visible_string, sizeof(Board_obj1009) - 1, (void*)Board_obj1009 }
};

/* 0x100A: Software Version */
static char Board_obj100A[] = "v1.0";
static subindex Board_Index100A[] = {
    { ro, visible_string, sizeof(Board_obj100A) - 1, (void*)Board_obj100A }
};

/* 0x1017: Producer Heartbeat Time */
static UNS16 Board_obj1017 = 1000U;
static subindex Board_Index1017[] = {
    { rw, uint16, sizeof(UNS16), (void*)&Board_obj1017 }
};

/* 0x1018: Identity Object */
static UNS8  Board_obj1018_highestSubIndex = 4U;
static UNS32 Board_obj1018_Vendor_ID       = 0x00000000UL;
static UNS32 Board_obj1018_Product_Code    = 0x00004070UL;
static UNS32 Board_obj1018_Revision_Number = 0x00010000UL;
static UNS32 Board_obj1018_Serial_Number   = 0x12345678UL;
static subindex Board_Index1018[] = {
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1018_highestSubIndex },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1018_Vendor_ID },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1018_Product_Code },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1018_Revision_Number },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1018_Serial_Number }
};

/* 0x1400: RPDO1 Communication Parameter */
static UNS8  Board_obj1400_highestSubIndex = 2U;
static UNS32 Board_obj1400_COB_ID          = 0x201UL;
static UNS8  Board_obj1400_Transmission_Type = 254U;
static subindex Board_Index1400[] = {
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1400_highestSubIndex },
    { rw, uint32, sizeof(UNS32), (void*)&Board_obj1400_COB_ID },
    { rw, uint8,  sizeof(UNS8),  (void*)&Board_obj1400_Transmission_Type }
};

/* 0x1401: RPDO2 Communication Parameter */
static UNS8  Board_obj1401_highestSubIndex = 2U;
static UNS32 Board_obj1401_COB_ID          = 0x301UL;
static UNS8  Board_obj1401_Transmission_Type = 254U;
static subindex Board_Index1401[] = {
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1401_highestSubIndex },
    { rw, uint32, sizeof(UNS32), (void*)&Board_obj1401_COB_ID },
    { rw, uint8,  sizeof(UNS8),  (void*)&Board_obj1401_Transmission_Type }
};

/* 0x1600: RPDO1 Mapping Parameter (maps 0x6200 sub 1: 8 bits DO) */
static UNS8  Board_obj1600_highestSubIndex = 1U;
static UNS32 Board_obj1600_Map1            = 0x62000108UL;
static subindex Board_Index1600[] = {
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1600_highestSubIndex },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1600_Map1 }
};

/* 0x1601: RPDO2 Mapping Parameter (maps 0x6411 sub 1 & 2: 2x 16 bits AO) */
static UNS8  Board_obj1601_highestSubIndex = 2U;
static UNS32 Board_obj1601_Map1            = 0x64110110UL;
static UNS32 Board_obj1601_Map2            = 0x64110210UL;
static subindex Board_Index1601[] = {
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1601_highestSubIndex },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1601_Map1 },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1601_Map2 }
};

/* 0x1800: TPDO1 Communication Parameter */
static UNS8  Board_obj1800_highestSubIndex = 5U;
static UNS32 Board_obj1800_COB_ID          = 0x181UL;
static UNS8  Board_obj1800_Transmission_Type = 254U;
static UNS16 Board_obj1800_Inhibit_Time    = 100U;
static UNS8  Board_obj1800_Reserved        = 0U;
static UNS16 Board_obj1800_Event_Timer     = 1000U;
static subindex Board_Index1800[] = {
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1800_highestSubIndex },
    { rw, uint32, sizeof(UNS32), (void*)&Board_obj1800_COB_ID },
    { rw, uint8,  sizeof(UNS8),  (void*)&Board_obj1800_Transmission_Type },
    { rw, uint16, sizeof(UNS16), (void*)&Board_obj1800_Inhibit_Time },
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1800_Reserved },
    { rw, uint16, sizeof(UNS16), (void*)&Board_obj1800_Event_Timer }
};

/* 0x1801: TPDO2 Communication Parameter */
static UNS8  Board_obj1801_highestSubIndex = 5U;
static UNS32 Board_obj1801_COB_ID          = 0x281UL;
static UNS8  Board_obj1801_Transmission_Type = 254U;
static UNS16 Board_obj1801_Inhibit_Time    = 100U;
static UNS8  Board_obj1801_Reserved        = 0U;
static UNS16 Board_obj1801_Event_Timer     = 1000U;
static subindex Board_Index1801[] = {
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1801_highestSubIndex },
    { rw, uint32, sizeof(UNS32), (void*)&Board_obj1801_COB_ID },
    { rw, uint8,  sizeof(UNS8),  (void*)&Board_obj1801_Transmission_Type },
    { rw, uint16, sizeof(UNS16), (void*)&Board_obj1801_Inhibit_Time },
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1801_Reserved },
    { rw, uint16, sizeof(UNS16), (void*)&Board_obj1801_Event_Timer }
};

/* 0x1A00: TPDO1 Mapping Parameter (maps 0x6000 sub 1: 8 bits DI) */
static UNS8  Board_obj1A00_highestSubIndex = 1U;
static UNS32 Board_obj1A00_Map1            = 0x60000108UL;
static subindex Board_Index1A00[] = {
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1A00_highestSubIndex },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1A00_Map1 }
};

/* 0x1A01: TPDO2 Mapping Parameter (maps 0x6401 sub 1..4: 4x 16 bits AI) */
static UNS8  Board_obj1A01_highestSubIndex = 4U;
static UNS32 Board_obj1A01_Map1            = 0x64010110UL;
static UNS32 Board_obj1A01_Map2            = 0x64010210UL;
static UNS32 Board_obj1A01_Map3            = 0x64010310UL;
static UNS32 Board_obj1A01_Map4            = 0x64010410UL;
static subindex Board_Index1A01[] = {
    { ro, uint8,  sizeof(UNS8),  (void*)&Board_obj1A01_highestSubIndex },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1A01_Map1 },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1A01_Map2 },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1A01_Map3 },
    { ro, uint32, sizeof(UNS32), (void*)&Board_obj1A01_Map4 }
};

/* 0x6000: CiA 401 Read 8 Digital Inputs */
static UNS8  Board_obj6000_highestSubIndex = 1U;
static UNS8  Board_obj6000_di_state        = 0x00U;
static subindex Board_Index6000[] = {
    { ro, uint8, sizeof(UNS8), (void*)&Board_obj6000_highestSubIndex },
    { ro, uint8, sizeof(UNS8), (void*)&Board_obj6000_di_state }
};

/* 0x6200: CiA 401 Write 8 Digital Outputs */
static UNS8  Board_obj6200_highestSubIndex = 1U;
static UNS8  Board_obj6200_do_state        = 0x00U;
static subindex Board_Index6200[] = {
    { ro, uint8, sizeof(UNS8), (void*)&Board_obj6200_highestSubIndex },
    { rw, uint8, sizeof(UNS8), (void*)&Board_obj6200_do_state }
};

/* 0x6401: CiA 401 Read Analogue Input 16-bit */
static UNS8      Board_obj6401_highestSubIndex = 4U;
static INTEGER16 Board_obj6401_ai[4]           = { 0, 0, 0, 0 };
static subindex Board_Index6401[] = {
    { ro, uint8, sizeof(UNS8),      (void*)&Board_obj6401_highestSubIndex },
    { ro, int16, sizeof(INTEGER16), (void*)&Board_obj6401_ai[0] },
    { ro, int16, sizeof(INTEGER16), (void*)&Board_obj6401_ai[1] },
    { ro, int16, sizeof(INTEGER16), (void*)&Board_obj6401_ai[2] },
    { ro, int16, sizeof(INTEGER16), (void*)&Board_obj6401_ai[3] }
};

/* 0x6411: CiA 401 Write Analogue Output 16-bit */
static UNS8      Board_obj6411_highestSubIndex = 2U;
static INTEGER16 Board_obj6411_ao[2]           = { 0, 0 };
static subindex Board_Index6411[] = {
    { ro, uint8, sizeof(UNS8),      (void*)&Board_obj6411_highestSubIndex },
    { rw, int16, sizeof(INTEGER16), (void*)&Board_obj6411_ao[0] },
    { rw, int16, sizeof(INTEGER16), (void*)&Board_obj6411_ao[1] }
};

/* Master Table for Default Object Dictionary (sorted by index) */
static const indextable Board_objdict[] = {
    { Board_Index1000, sizeof(Board_Index1000)/sizeof(Board_Index1000[0]), 0x1000 },
    { Board_Index1001, sizeof(Board_Index1001)/sizeof(Board_Index1001[0]), 0x1001 },
    { Board_Index1008, sizeof(Board_Index1008)/sizeof(Board_Index1008[0]), 0x1008 },
    { Board_Index1009, sizeof(Board_Index1009)/sizeof(Board_Index1009[0]), 0x1009 },
    { Board_Index100A, sizeof(Board_Index100A)/sizeof(Board_Index100A[0]), 0x100A },
    { Board_Index1017, sizeof(Board_Index1017)/sizeof(Board_Index1017[0]), 0x1017 },
    { Board_Index1018, sizeof(Board_Index1018)/sizeof(Board_Index1018[0]), 0x1018 },
    { Board_Index1400, sizeof(Board_Index1400)/sizeof(Board_Index1400[0]), 0x1400 },
    { Board_Index1401, sizeof(Board_Index1401)/sizeof(Board_Index1401[0]), 0x1401 },
    { Board_Index1600, sizeof(Board_Index1600)/sizeof(Board_Index1600[0]), 0x1600 },
    { Board_Index1601, sizeof(Board_Index1601)/sizeof(Board_Index1601[0]), 0x1601 },
    { Board_Index1800, sizeof(Board_Index1800)/sizeof(Board_Index1800[0]), 0x1800 },
    { Board_Index1801, sizeof(Board_Index1801)/sizeof(Board_Index1801[0]), 0x1801 },
    { Board_Index1A00, sizeof(Board_Index1A00)/sizeof(Board_Index1A00[0]), 0x1A00 },
    { Board_Index1A01, sizeof(Board_Index1A01)/sizeof(Board_Index1A01[0]), 0x1A01 },
    { Board_Index6000, sizeof(Board_Index6000)/sizeof(Board_Index6000[0]), 0x6000 },
    { Board_Index6200, sizeof(Board_Index6200)/sizeof(Board_Index6200[0]), 0x6200 },
    { Board_Index6401, sizeof(Board_Index6401)/sizeof(Board_Index6401[0]), 0x6401 },
    { Board_Index6411, sizeof(Board_Index6411)/sizeof(Board_Index6411[0]), 0x6411 }
};

static const canopen_od_t s_default_od = {
    Board_objdict,
    (UNS16)(sizeof(Board_objdict) / sizeof(Board_objdict[0]))
};

const canopen_od_t *canopen_default_od_get(void)
{
    return &s_default_od;
}

void canopen_default_od_reset(void)
{
    Board_obj6000_di_state = 0x00U;
    Board_obj6200_do_state = 0x00U;
    memset(Board_obj6401_ai, 0, sizeof(Board_obj6401_ai));
    memset(Board_obj6411_ao, 0, sizeof(Board_obj6411_ao));
    Board_obj1001          = 0x00U;
}

void canopen_default_od_update_node_id(UNS8 node_id)
{
    Board_obj1400_COB_ID = CANOPEN_COB_RPDO1_BASE + node_id;
    Board_obj1401_COB_ID = CANOPEN_COB_RPDO2_BASE + node_id;
    Board_obj1800_COB_ID = CANOPEN_COB_TPDO1_BASE + node_id;
    Board_obj1801_COB_ID = CANOPEN_COB_TPDO2_BASE + node_id;
    canopen_default_od_reset();
}

canopen_automation_vars_t canopen_default_od_get_vars(void)
{
    canopen_automation_vars_t vars;
    vars.p_digital_inputs       = &Board_obj6000_di_state;
    vars.p_digital_outputs      = &Board_obj6200_do_state;
    vars.p_analog_inputs        = Board_obj6401_ai;
    vars.p_analog_outputs       = Board_obj6411_ao;
    vars.p_heartbeat_producer_ms= &Board_obj1017;
    vars.p_error_register       = &Board_obj1001;
    return vars;
}
