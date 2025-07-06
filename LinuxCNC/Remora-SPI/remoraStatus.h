#ifndef REMORASTATUS_H
#define REMORASTATUS_H

#include <stdint.h>

// Status byte structure:
// Bit 7   = FATAL flag
// Bits 6-4 = ERROR SOURCE
// Bits 3-0 = ERROR CODE

// Error sources (shifted to align with bits 6-4)
#define REMORA_SOURCE_NONE         0x00
#define REMORA_SOURCE_CORE         0x10  // 0x01 << 4
#define REMORA_SOURCE_JSON_CONFIG  0x20  // 0x02 << 4
#define REMORA_SOURCE_MODULE       0x30  // 0x03 << 4
#define REMORA_SOURCE_TMC_DRIVER   0x40  // 0x04 << 4

// Error codes (lower 4 bits)
#define REMORA_CODE_NONE                  0x00

// CORE errors
#define REMORA_CORE_ERROR                 0x01

// JSON_CONFIG errors
#define REMORA_SD_MOUNT_FAILED            0x01
#define REMORA_CONFIG_FILE_OPEN_FAILED    0x02
#define REMORA_CONFIG_FILE_READ_FAILED    0x03
#define REMORA_CONFIG_INVALID_INPUT       0x04
#define REMORA_CONFIG_NO_MEMORY           0x05
#define REMORA_CONFIG_PARSE_FAILED        0x06

// MODULE_LOADER errors
#define REMORA_MODULE_CREATE_FAILED       0x01

// TMC_DRIVER errors
#define REMORA_TMC_DRIVER_ERROR           0x01

#endif
