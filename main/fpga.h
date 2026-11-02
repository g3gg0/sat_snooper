#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define FPGA_PID_FILTER_COUNT 4

typedef struct
{
    bool enabled;
    uint16_t pid;
    uint16_t mask;
} fpga_pid_filter_t;

#define FPGA_FEABITS_WIDTH 16
#define FPGA_FEABITS_DEFAULT (FPGA_MACHXO2_FEABITS_JTAG_PERSIST_DIS | FPGA_MACHXO2_FEABITS_SSP_PERSIST_DIS)

#define FPGA_ADDR_CONF_PRIMARY 0x40
#define FPGA_ADDR_USER_PRIMARY 0x41
#define FPGA_ADDR_USER_SECONDARY 0x42
#define FPGA_ADDR_CONF_RESET 0x43

/* BSDL IDCODEs: https://bsdl.info/list.htm?f=788
 * HE aliases: https://github.com/trabucayre/openFPGALoader/blob/master/src/part.hpp
 * Package and speed grade are not encoded; several variants share device IDs.
 */
typedef enum
{
    FPGA_MACHXO2_ID_256ZE = 0x012B0043,
    FPGA_MACHXO2_ID_640ZE = 0x012B1043,
    FPGA_MACHXO2_ID_1200ZE = 0x012B2043,
    FPGA_MACHXO2_ID_2000ZE = 0x012B3043,
    FPGA_MACHXO2_ID_4000ZE = 0x012B4043,
    FPGA_MACHXO2_ID_7000ZE = 0x012B5043,
    FPGA_MACHXO2_ID_256HC = 0x012B8043,
    FPGA_MACHXO2_ID_640HC = 0x012B9043,
    FPGA_MACHXO2_ID_1200HC = 0x012BA043,
    FPGA_MACHXO2_ID_2000HC = 0x012BB043,
    FPGA_MACHXO2_ID_4000HC = 0x012BC043,
    FPGA_MACHXO2_ID_7000HC = 0x012BD043,
    FPGA_MACHXO2_ID_640UHC = FPGA_MACHXO2_ID_1200HC,
    FPGA_MACHXO2_ID_1200UHC = FPGA_MACHXO2_ID_2000HC,
    FPGA_MACHXO2_ID_2000UHC = FPGA_MACHXO2_ID_4000HC,
    FPGA_MACHXO2_ID_2000HE = FPGA_MACHXO2_ID_2000ZE,
    FPGA_MACHXO2_ID_4000HE = FPGA_MACHXO2_ID_4000ZE,
    FPGA_MACHXO2_ID_7000HE = FPGA_MACHXO2_ID_7000ZE,
    FPGA_MACHXO2_ID_2000UHE = FPGA_MACHXO2_ID_4000ZE,
} fpga_machxo2_id_t;

typedef enum
{
    FPGA_MACHXO2_FEABITS_SECURE_PWD = (1 << 0),
    FPGA_MACHXO2_FEABITS_DEC_ONLY = (1 << 1),
    FPGA_MACHXO2_FEABITS_PWD_ENABLE = (1 << 2),
    FPGA_MACHXO2_FEABITS_PWD_ENABLE_ALL = (1 << 3),
    FPGA_MACHXO2_FEABITS_MY_ASSP_ENABLE = (1 << 4),
    FPGA_MACHXO2_FEABITS_PRG_PERSIST_DIS = (1 << 5),
    FPGA_MACHXO2_FEABITS_INIT_PERSIST_EN = (1 << 6),
    FPGA_MACHXO2_FEABITS_DONE_PERSIST_EN = (1 << 7),
    FPGA_MACHXO2_FEABITS_JTAG_PERSIST_DIS = (1 << 8),
    FPGA_MACHXO2_FEABITS_SSP_PERSIST_DIS = (1 << 9),
    FPGA_MACHXO2_FEABITS_I2C_PERSIST_DIS = (1 << 10),
    FPGA_MACHXO2_FEABITS_MSPI_PERSIST_EN = (1 << 11),
    FPGA_MACHXO2_FEABITS_BOOTSEL0 = (1 << 12),
    FPGA_MACHXO2_FEABITS_BOOTSEL1 = (1 << 13),
    FPGA_MACHXO2_FEABITS_RES2 = (1 << 14),
    FPGA_MACHXO2_FEABITS_RES1 = (1 << 15)
} fpga_machxo2_feabits;

typedef enum
{
    FPGA_MACHXO2_FEATUREBITS_OFFSET_TRACED_ID_0 = 0,
    FPGA_MACHXO2_FEATUREBITS_OFFSET_I2C_SLAVE_ADDR_0 = 4,
    FPGA_MACHXO2_FEATUREBITS_OFFSET_DB_GOLDEN_ADDR_0 = 12,
    FPGA_MACHXO2_FEATUREBITS_OFFSET_CUSTOM_ID = 28,
} fpga_machxo2_feature_bits;

/* MachXO2 sysCONFIG Programming Commands */
typedef enum
{
    FPGA_CMD_READ_DEVICE_ID = 0xE0,            /* op: 00 00 00, YY characters represent the device-specific ID code */
    FPGA_CMD_ENABLE_CFG_IF_TRANSPARENT = 0x74, /* op: 08 00 00, Enable the Configuration Logic for device programming in transparent mode. Note 1 */
    FPGA_CMD_ENABLE_CFG_IF_OFFLINE = 0xC6,     /* op: 08 00 00, Enable the Configuration Logic for device programming in Offline mode. Note 1 */
    FPGA_CMD_READ_BUSY_FLAG = 0xF0,            /* op: 00 00 00, Bit 7: Busy, Bit 0: Ready */
    FPGA_CMD_READ_STATUS_REGISTER = 0x3C,      /* op: 00 00 00, Bit 12: Busy/Ready, Bit 13: Fail/OK */
    FPGA_CMD_ERASE = 0x0E,                     /* op: 0Y 00 00, Y = Memory space to erase (bitwise OR): Bit 16: SRAM, Bit 17: Feature Row, Bit 18: Config Flash, Bit 19: UFM */
    FPGA_CMD_ERASE_UFM = 0xCB,                 /* op: 00 00 00, Erase the UFM sector only. */
    FPGA_CMD_RESET_CFG_FLASH_ADDRESS = 0x46,   /* op: 00 00 00, Set Page Address pointer to the beginning of the Configuration Flash sector. */
    FPGA_CMD_SET_ADDRESS = 0xB4,               /* op: 00 00 00, Set Page Address pointer. M field defines memory space (0x0: Config Flash, 0x4: UFM). PP PP is 14-bit page address. */
    FPGA_CMD_PROGRAM_PAGE = 0x70,              /* op: 00 00 01, Program one Flash page (Config Flash or UFM). */
    FPGA_CMD_RESET_UFM_ADDRESS = 0x47,         /* op: 00 00 00, Set the Page Address Pointer to the beginning of the UFM sector. */
    FPGA_CMD_PROGRAM_UFM_PAGE = 0xC9,          /* op: 00 00 01, Program one UFM page. */
    FPGA_CMD_PROGRAM_USERCODE = 0xC2,          /* op: 00 00 00, Program the USERCODE. */
    FPGA_CMD_READ_USERCODE = 0xC0,             /* op: 00 00 00, Retrieves the 32-bit USERCODE value. */
    FPGA_CMD_WRITE_FEATURE_ROW = 0xE4,         /* op: 00 00 00, Program the Feature Row bits. */
    FPGA_CMD_READ_FEATURE_ROW = 0xE7,          /* op: 00 00 00, Retrieves the Feature Row bits. */
    FPGA_CMD_WRITE_FEABITS = 0xF8,             /* op: 00 00 00, Program the FEABITS. */
    FPGA_CMD_READ_FEABITS = 0xFB,              /* op: 00 00 00, Retrieves the FEABITS. */
    FPGA_CMD_READ_FLASH = 0x73,                /* op: M0 PP PP, Retrieves PPPP count pages. M field must be set (0x0: I2C, 0x0/0x1: JTAG/SSPI/WB). See Reading Flash Pages section. */
    FPGA_CMD_READ_UFM_FLASH = 0xCA,            /* op: M0 PP PP, Retrieves PPPP count UFM pages. M field must be set (0x0: I2C, 0x0/0x1: JTAG/SSPI/WB). See Reading Flash Pages section. */
    FPGA_CMD_PROGRAM_DONE = 0x5E,              /* op: 00 00 00, Program the DONE status bit enabling SDM. */
    FPGA_CMD_PROGRAM_OTP_FUSES = 0xF9,         /* op: 00 00 00, Makes selected memory space One Time Programmable. See datasheet for bits. */
    FPGA_CMD_READ_OTP_FUSES = 0xFA,            /* op: 00 00 00, Read the state of the One Time Programmable fuses. See datasheet for bits. */
    FPGA_CMD_DISABLE_CFG_IF = 0x26,            /* op: 00 00, Exit Offline or Transparent programming mode. Causes reconfigure when leaving Offline mode. LSC_REFRESH (0x79) recommended for Offline exit. */
    FPGA_CMD_BYPASS = 0xFF,                    /* op: FF FF FF, No Operation and Device Wakeup */
    FPGA_CMD_REFRESH = 0x79,                   /* op: 00 00, Force the MachXO2 to reconfigure (like asserting PROGRAMN). */
    FPGA_CMD_PROGRAM_SECURITY = 0xCE,          /* op: 00 00 00, Program the Security bit (Secures CFG Flash sector). Note 2 */
    FPGA_CMD_PROGRAM_SECURITY_PLUS = 0xCF,     /* op: 00 00 00, Program the Security Plus bit (Secures CFG and UFM Sectors). Note 2 */
    FPGA_CMD_READ_TRACEID = 0x19,              /* op: 00 00 00, Read 64-bit TraceID. */
    FPGA_CMD_LSC_BITSTREAM_BURST = 0x7A,
} fpga_cmd_t;

/* Erase options for FPGA_CMD_ERASE */
#define ERASE_SRAM 0x01
#define ERASE_FLASH 0x02
#define ERASE_FEATURE 0x04
#define ERASE_UFM 0x08
#define ERASE_ALL (ERASE_SRAM | ERASE_FLASH | ERASE_FEATURE | ERASE_UFM)

/* Notes from datasheet:
 * 1. Transmit the command opcode and first two operand bytes when using the I2C port. The final operand byte must not be transmitted.
 * 2. SECURITY and SECURITY PLUS commands are mutually exclusive.
 */

/* MachXO2 Device Status Register */
typedef struct
{
    uint32_t jtag_transparent_mode : 1;   /* 0: Device is in transparent mode. */
    uint32_t config_target_selection : 3; /* [3:1] Config Target Selection: 000=SRAM, 001=E_Fuse_Normal, 010=E_Fuse_SUDO, 011=E_Fuse_Safe, 100=Flash_Normal, 101=Flash_SUDO, 110=Flash_Safe, 111=Flash_UFM */
    uint32_t jtag_active : 1;             /* 4: JTAG state machine is active. */
    uint32_t pwd_protect : 1;             /* 5: Configuration logic is password protected. */
    uint32_t otp : 1;                     /* 6: Device is one-time programmable. */
    uint32_t decrypt_enable : 1;          /* 7: Decrypted bitstreams can be accepted. */
    uint32_t done : 1;                    /* 8: Device configuration is complete and the internal DONE bit is set. */
    uint32_t isc_enable : 1;              /* 9: JTAG instructions are being executed with ISC enabled. */
    uint32_t write_enable : 1;            /* 10: 1=Selected configuration target is write-enabled. */
    uint32_t read_enable : 1;             /* 11: 1=Selected configuration target is read-enabled. */
    uint32_t busy_flag : 1;               /* 12: Configuration logic is busy executing a previous instruction/command. */
    uint32_t fail_flag : 1;               /* 13: Previous instruction/command execution failed. */
    uint32_t fea_otp : 1;                 /* 14: Feature Row is one-time programmable. */
    uint32_t decrypt_only : 1;            /* 15: 1=Only encrypted data can be accepted. */
    uint32_t pwd_enable : 1;              /* 16: Password protection is enabled. */
    uint32_t ufm_otp : 1;                 /* 17: UFM is one-time programmable. */
    uint32_t assp : 1;                    /* 18: ASSP poly fuses or flash cells are programmed. */
    uint32_t sdm_en : 1;                  /* 19: Self-download mode is enabled. */
    uint32_t encrypt_preamble : 1;        /* 20: An encrypt preamble detected from the last bitstream execution. */
    uint32_t std_preamble : 1;            /* 21: A standard preamble detected from the last bitstream execution. */
    uint32_t spim_fail_1 : 1;             /* 22: Primary pattern failed when dual-boot is enabled. */
    uint32_t bse_error_code : 3;          /* [25:23] BSE Error Code: 000=No Error, 001=ID Error, 010=CMD Error, 011=CRC Error, 100=PRMB Error, 101=ABRT Error, 110=OVFL Error, 111=SDM EOF */
    uint32_t execution_error : 1;         /* 26: Previous instruction/command encountered an error. */
    uint32_t id_error : 1;                /* 27: ID code mismatch detected for the verify_id instruction/command. */
    uint32_t invalid_command : 1;         /* 28: Instruction found in the Command field of the frame is invalid. */
    uint32_t sed_error : 1;               /* 29: Soft error detection logic detected an error. */
    uint32_t bypass_mode : 1;             /* 30: Device is in bypass mode. */
    uint32_t flowthrough_mode : 1;        /* 31: Device is in flow-through mode. */
} fpga_statusreg_t;

void fpga_init();
esp_err_t fpga_set_pid_filters(const fpga_pid_filter_t filters[FPGA_PID_FILTER_COUNT]);
void fpga_prog(bool active);
bool fpga_prog_get();
