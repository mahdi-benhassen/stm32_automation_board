#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

typedef uint8_t   u8_t;
typedef int8_t    s8_t;
typedef uint16_t  u16_t;
typedef int16_t   s16_t;
typedef uint32_t  u32_t;
typedef int32_t   s32_t;
typedef uintptr_t mem_ptr_t;

#define U16_F "u"
#define S16_F "d"
#define X16_F "x"
#define U32_F "lu"
#define S32_F "ld"
#define X32_F "lx"
#define SZT_F "u"

#define BYTE_ORDER LITTLE_ENDIAN

#define LWIP_PLATFORM_BYTESWAP 0
#define LWIP_PLATFORM_HTONS(x) ((uint16_t)((((x) & 0xFFU) << 8) | (((x) & 0xFF00U) >> 8)))
#define LWIP_PLATFORM_HTONL(x) ((uint32_t)((((x) & 0xFFU) << 24) | \
    (((x) & 0xFF00U) << 8) | (((x) & 0xFF0000U) >> 8) | (((x) & 0xFF000000U) >> 24)))

#define LWIP_PLATFORM_DIAG(x)  do { } while (0)
#define LWIP_PLATFORM_ASSERT(x) do { } while (0)

#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

#endif /* LWIP_ARCH_CC_H */
