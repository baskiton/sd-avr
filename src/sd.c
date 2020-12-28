#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <defines.h>
#include <spi.h>
#include <fs.h>
#include <block_dev.h>

typedef struct sd_card_s {
    struct spi_device_s *spi_dev;
    struct block_dev_s *bdev;

    uint8_t type;   // SDSCv1, SDSCv2, SDHC/SDXC
    struct {
        uint8_t
            in_block : 1,       // indicates that the device is in the process of reading a block.
            partial_read : 1;   // partial block read
    } flags;
    uint32_t cur_block; // current reading block
    uint16_t offset;    // current offset
} sd_card_t;

static inline sd_card_t *sd_get_dev(bdev_t *bdev) {
    return (sd_card_t *)blk_get_priv(bdev);
}

static inline void sd_set_dev(bdev_t *bdev, sd_card_t *sd) {
    blk_set_priv(bdev, sd);
}

/* Commands */

enum sd_command {
    CMD0 = 0,
#define GO_IDLE_STATE CMD0
    CMD1,
#define SEND_OP_COND CMD1
    CMD2,   // not in SPI mode
    CMD3,   // not in SPI mode
    CMD4,   // not in SPI mode
    CMD5,   // reserved for I/O Mode
    CMD6,
#define SWITCH_FUNC CMD6
    CMD7,   // not in SPI mode
    CMD8,
#define SEND_IF_COND CMD8
    CMD9,
#define SEND_CSD CMD9
    CMD10,
#define SEND_CID CMD10
    CMD11,  // not in SPI mode
    CMD12,
#define STOP_TRANSMISSION CMD12
    CMD13,
#define SEND_STATUS CMD13
    CMD14,  // reserved
    CMD15,  // not in SPI mode
    CMD16,
#define SET_BLOCKLEN CMD16
    CMD17,
#define READ_SINGLE_BLOCK CMD17
    CMD18,
#define READ_MULTIPLE_BLOCK CMD18
    CMD19,  // reserved
    CMD20,  // not in SPI mode
    CMD21,  // reserved
    CMD22,  // reserved
    CMD23,  // reserved
    CMD24,
#define WRITE_BLOCK CMD24
    CMD25,
#define WRITE_MULTIPLE_BLOCK CMD25
    CMD26,  // not in SPI mode
    CMD27,
#define PROGRAM_CSD CMD27
    CMD28,
#define SET_WRITE_PROT CMD28
    CMD29,
#define CLR_WRITE_PROT CMD29
    CMD30,
#define SEND_WRITE_PROT CMD30
    CMD31,  // reserved
    CMD32,
#define ERASE_WR_BLK_START_ADDR CMD32
    CMD33,
#define ERASE_WR_BLK_END_ADDR CMD33
    CMD34,  // reserved for command system
    CMD35,  // reserved for command system
    CMD36,  // reserved for command system
    CMD37,  // reserved for command system
    CMD38,
#define ERASE CMD38
    CMD39,  // not in SPI mode
    CMD40,  // not in SPI mode
    CMD41,  // reserved
    CMD42,
#define LOCK_UNLOCK CMD42
    CMD43,  // reserved
    CMD44,  // reserved
    CMD45,  // reserved
    CMD46,  // reserved
    CMD47,  // reserved
    CMD48,  // reserved
    CMD49,  // reserved
    CMD50,  // reserved for command system
    CMD51,  // reserved
    CMD52,  // reserved for I/O Mode
    CMD53,  // reserved for I/O Mode
    CMD54,  // reserved for I/O Mode
    CMD55,
#define APP_CMD CMD55
    CMD56,
#define GEN_CMD CMD56
    CMD57,  // reserved for command system
    CMD58,
#define READ_OCR CMD58
    CMD59,
#define CRC_ON_OFF CMD59
    CMD60,  // reserved
    CMD61,  // reserved
    CMD62,  // reserved
    CMD63,  // reserved
};

enum sd_app_command {
    ACMD6 = 6,  // not in SPI mode
    ACMD13 = 13,
#define SD_STATUS ACMD13
    ACMD17 = 17,    // reserved
    ACMD18,     // reserved for SD security apps
    ACMD19,     // reserved
    ACMD20,     // reserved
    ACMD21,     // reserved
    ACMD22,
#define SEND_NUM_WR_BLOCKS ACMD22
    ACMD23,
#define SET_WR_BLK_ERASE_COUNT ACMD23
    ACMD24,     // reserved
    ACMD25,     // reserved for SD security apps
    ACMD26,     // reserved for SD security apps
    ACMD38 = 38,    // reserved for SD security apps
    ACMD39,     // reserved
    ACMD40,     // reserved
    ACMD41,
#define SD_SEND_OP_COND ACMD41
    ACMD42,
#define SET_CLR_CARD_DETECT ACMD42
    ACMD43,     // reserved for SD security apps
    ACMD44,     // reserved for SD security apps
    ACMD45,     // reserved for SD security apps
    ACMD46,     // reserved for SD security apps
    ACMD47,     // reserved for SD security apps
    ACMD48,     // reserved for SD security apps
    ACMD49,     // reserved for SD security apps
    ACMD51 = 51
#define SEND_SCR ACMD51
};

/* Command Token */
struct __attribute__((packed)) cmd_token {
    /*

    |31  29       24|23                            8|7           1|0|
    |*|*|* * * * * *|* * * * * * * * * * * * * * * *|* * * * * * *|*|
    | | |    cmd    |           arguments           |    CRC7     |end|
    | |tran|
    |start|

     */
    uint8_t
        cmd : 6,        // command code
        tran_bit : 1,   // always 1
        start : 1;      // always 0
    uint32_t args;      // command arguments
    uint8_t
        end : 1,        // always 1
        crc : 7;        // CRC7
};

/** Responses */

/* response to every command, except SEND_STATUS commands */
struct __attribute__((packed)) response_r1 {
    union {
        struct {
            uint8_t
                in_idle_state : 1,  // The card is in idle state and running the initializing process
                erase_reset : 1,    // An erase sequence was cleared before executing because an out of erase sequence command was received
                illegal_cmd : 1,    // An illegal command code was detected
                com_crc_err : 1,    // The CRC check of the last command failed
                erase_seq_err : 1,  // An error in the sequence of erase commands occurred
                addr_err : 1,   // A misaligned address that did not match the block length was used in the command
                param_err : 1,  // The command's argument (e.g. address, block length) was outside the allowed range for this card
                zero : 1;   // always zero
        };
        uint8_t u8;
    };
};

/* response to the SEND_STATUS command */
struct __attribute__((packed)) response_r2 {
    uint8_t
        csd_ocerwrite : 1,  // out of range | csd overwrite
        erase_param : 1,    // An invalid selection for erase, sectors or groups
        wp_violation : 1,   // The command tried to write a write-protected block
        card_ecc_failed : 1,    // Card internal ECC was applied but failed to correct the data
        cc_err : 1, // Internal card controller error
        error : 1,  // A general or an unknown error occurred during the operation
        wp_erase_skip : 1,  // Write protect erase skip | lock/unlock command failed
        card_is_locked : 1; // Set when the card is locked by the user. Reset when it is unlocked
};

/* response to the READ_OCR command */
struct __attribute__((packed)) response_r3 {
    /*

    |31|| | |7|6|5|4|3|   20        15|14     10   8|7|6           0|
    |*|*|*|*|*|* *|*|* * * * * * * * *|* * * * * * *|*|* * * * * * *|
    | | | | | |   | |                 |             | |  reserved   |
    | | | | | |   | |    voltages     |   reserved  |LVR|
    | | | | | |   |S18A|
    | | | | | |reserved|
    | | | | |CO2T|
    | | | |reserved|
    | | |UHS-II|
    | |CCS|
    |busy|

     */
    union {
        struct {
            uint8_t
                s18a : 1,   // S18A - Switching to 1.8v Accepted (only for UHS-I card)
                : 2,        // reserved
                co2t : 1,   // CO2T - Over 2TB support (only for SDUC card)
                : 1,        // reserved
                uhs2 : 1,   // UHS-II card status
                ccs : 1,    // Card Capacity Status (CCS)
                busy : 1;   // Card Power Up Status
            union {
                struct {
                    uint8_t
                        v_3v5_3v6 : 1,  // 3.5-3.6V
                        v_3v4_3v5 : 1,  // 3.4-3.5V
                        v_3v3_3v4 : 1,  // 3.3-3.4V
                        v_3v2_3v3 : 1,  // 3.2-3.3V
                        v_3v1_3v2 : 1,  // 3.1-3.2V
                        v_3v0_3v1 : 1,  // 3.0-3.1V
                        v_2v9_3v0 : 1,  // 2.9-3.0V
                        v_2v8_2v9 : 1;  // 2.8-2.9V
                    uint8_t
                        : 7,    // reserved
                        v_2v7_2v8 : 1;  // 2.7-2.8V
                };
                uint16_t voltages;
            };
            uint8_t
                : 7,        // reserved
                lvr : 1;    // Low Voltage Range
        };
        uint32_t ocr_reg;   // OCR register
    };
};

/* response to the SEND_IF_COND command */
struct __attribute__((packed)) response_r7 {
    /*

    |31   28|27                           12|11    8|7             0|
    |* * * *|* * * * * * * * * * * * * * * *|* * * *|* * * * * * * *|
    |cmd_ver|          reserved             |  VHC  | check_pattern |

     */

    uint8_t
        : 4,
        cmd_ver : 4;
    uint8_t : 8;
    uint8_t
        voltage_accepted : 4;
    uint8_t check_pattern;
};

/** Registers */

/* CID - Card IDentification register */
typedef struct __attribute__((packed)) cid_s {
    uint8_t mid;                // Manufacturer ID
    char oid[2];                // OEM/Application ID
    char pnm[5];                // Product name
    unsigned prv_m : 4;         // Product revision n.m
    unsigned prv_n : 4;         // Product revision n.m
    uint32_t psn;               // Product serial number
    unsigned mdt_year_high : 4; // Manufacturing date
    unsigned : 4;               // reserved
    unsigned mdt_month : 4;
    unsigned mdt_year_low :4;
    unsigned always1 : 1;
    unsigned crc : 7;
} cid_t;

#define SD_CSD_V1 0 // for SDSC
#define SD_CSD_V2 1 // for SDHC/SDXH
#define SD_CSD_V3 2 // for SDUC - Not supported with SPI

/* CSD version 1.00 */
typedef struct __attribute__((packed)) csd_v1_s {
    unsigned : 6;                       // reserved 00_0000b
    unsigned csd_ver : 2;               // SD_CSD_V1
    uint8_t taac;                       // XXh
    uint8_t nsac;                       // XXh
    uint8_t tran_speed;                 // 32h or 5Ah
    uint8_t ccc_high;                   // 01x1_1011b
    unsigned read_bl_len : 4;           // Xh
    unsigned ccc_low : 4;               // 0101b
    unsigned c_size_high : 2;           // xxb
    unsigned : 2;                       // reserved 00b
    unsigned dsr_imp : 1;               // xb
    unsigned read_blk_misalign :1;      // xb
    unsigned write_blk_misalign : 1;    // xb
    unsigned read_bl_partial : 1;       // 1b
    uint8_t c_size_mid;                 // XXh
    unsigned vdd_r_curr_max : 3;        // xxxb
    unsigned vdd_r_curr_min : 3;        // xxxb
    unsigned c_size_low :2;             // xxb
    unsigned c_size_mult_high : 2;      // xxb
    unsigned vdd_w_cur_max : 3;         // xxxb
    unsigned vdd_w_curr_min : 3;        // xxxb
    unsigned sector_size_high : 6;      // xx_xxxxb
    unsigned erase_blk_en : 1;          // xb
    unsigned c_size_mult_low : 1;       // xb
    unsigned wp_grp_size : 7;           // xxx_xxxxb
    unsigned sector_size_low : 1;       // xb
    unsigned write_bl_len_high : 2;     // xxb
    unsigned r2w_factor : 3;            // xxxb
    unsigned : 2;                       // reserved 00b
    unsigned wp_grp_enable : 1;         // xb
    unsigned : 5;                       // reserved 0_0000b
    unsigned write_bl_partial : 1;      // xb
    unsigned write_bl_len_low : 2;      // xxb
    unsigned : 2;                       // reserved 00b
    unsigned file_format : 2;           // xxb
    unsigned tmp_write_protect : 1;     // xb
    unsigned perm_write_protect : 1;    // xb
    unsigned copy : 1;                  // xb
    unsigned file_format_grp : 1;       // xb
    unsigned always1 : 1;               // 1b
    unsigned crc : 7;                   // xxx_xxxxb
} csd1_t;

/* CSD version 2.00 */
typedef struct __attribute__((packed)) csd_v2_s {
    unsigned : 6;                       // reserved 00_0000b
    unsigned csd_ver : 2;               // SD_CSD_V2
    uint8_t taac;                       // 0Eh
    uint8_t nsac;                       // 00h
    uint8_t tran_speed;                 // 32h, 5Ah, 0Bh or 2Bh
    uint8_t ccc_high;                   // x1x1_1011b
    unsigned read_bl_len : 4;           // 09h
    unsigned ccc_low : 4;               // 01x1b
    unsigned : 4;                       // reserved 0000b
    unsigned dsr_imp : 1;               // xb
    unsigned read_blk_misalign :1;      // 0b
    unsigned write_blk_misalign : 1;    // 0b
    unsigned read_bl_partial : 1;       // 0b
    unsigned : 2;                       // reserved 00b
    unsigned c_size_high : 6;           // xx_xxxxh
    uint8_t c_size_mid;                 // XXh
    uint8_t c_size_low;                 // XXh
    unsigned sector_size_high : 6;      // 11_1111b
    unsigned erase_blk_en : 1;          // 1b
    unsigned : 1;                       // reserved 0b
    unsigned wp_grp_size : 7;           // 000_0000b
    unsigned sector_size_low : 1;       // 1b
    unsigned write_bl_len_high : 2;     // 10b
    unsigned r2w_factor : 3;            // 010b
    unsigned : 2;                       // reserved 00b
    unsigned wp_grp_enable : 1;         // 0b
    unsigned : 5;                       // reserved 0_0000b
    unsigned write_bl_partial : 1;      // 0b
    unsigned write_bl_len_low : 2;      // 01b
    unsigned : 2;                       // reserved 00b
    unsigned file_format : 2;           // 00b
    unsigned tmp_write_protect : 1;     // xb
    unsigned perm_write_protect : 1;    // xb
    unsigned copy : 1;                  // xb
    unsigned file_format_grp : 1;       // 0b
    unsigned always1 : 1;               // 1b
    unsigned crc : 7;                   // xxx_xxxxb
} csd2_t;

typedef union csd_reg {
    struct {    // common fields
        unsigned : 6;           // reserved 00_0000b
        unsigned csd_ver : 2;   // SD_CSD_V2
    };
    csd1_t v1;
    csd2_t v2;
} csd_t;


/* Data response token */
#define DRT_MASK 0x1F
#define DRT_ACCCEPT 5       // Data accepted
#define DRT_REJECT_CRCE 11  // Data rejected due to a CRC error
#define DRT_REJECT_WRE 13   // Data Rejected due to a Write Error

/* Start Block Tokens and Stop Tran Token */
#define SINGLE_START_BLOCK 0xFE     // For Single Block Read/Write and Multiple Block Read
#define MULTIPLE_START_BLOCK 0xFC   // Start token for Multiple Block Write
#define MULTIPLE_STOP_TRAN 0xFD     // Stop Trensmition for Multiple Block Write

/* Data Error Token
 *  |0|0|0|0|OOR|ECC|CC|ERR|
 */
#define DET_MASK 0xFF
#define DET_ZERO 0xF0
#define DET_ERR 0x01            // General or unknown error
#define DET_CC_ERR 0x02         // Internal Card Controller Error
#define DET_CARD_ECC_FAIL 0x04  // Card internal ECC was applied but failed to correct the data
#define DET_OOR 0x08            // Out of Range command argument

/* States */
#define R1_READY_STATE 0x00
#define R1_IDLE_STATE 0x01  // card in the idle state
#define R1_ILLEGAL_CMD 0x04
#define R1_CRC_ERR 0x08

/* Host Voltages */
#define VHS_3V3 0x01    // 2.7-3.6V
#define VHS_LV 0x02     // reserved for low voltage range

#define CHECK_PATTERN 0xAA

#define SD_TYPE_NONTYPE 0   // None
#define SD_TYPE_SC1 0x01    // Standars Capacity v1
#define SD_TYPE_SC2 0x02    // Standars Capacity v2
#define SD_TYPE_HC 0x03     // High Capacity (HC/XC)
#define SD_TYPE_UC 0x04     // Ultra Capacity (UC) - Not supported with SPI

/* Error Codes */
#define SD_NO_ERR 0x00
#define SD_ERR_BUSY 0x01
#define SD_ERR_STATE 0x02
#define SD_ERR_VHS 0x03
#define SD_ERR_CRC 0x04
#define SD_ERR_IF_SEND 0x05
#define SD_ERR_VNOTSUPPORT 0x06
#define SD_ERR_UNUSABLE 0x07
#define SD_ERR_ILLEGAL_CMD 0x08
#define SD_ERR_BAD_CSD 0x09
#define SD_ERR_READ_R1 0x0A
#define SD_ERR_READ 0x0B
#define SD_ERR_WRITE 0x0D
#define SD_ERR_NOMEM 0x0C

static void sd_print_err_msg(int8_t err) {
    const char *str_P;
    if (err == SD_NO_ERR) {
        printf_P(PSTR("Success\n"));
        return;
    }
    printf_P(PSTR("Error: "));

    switch (err) {
        case SD_ERR_BUSY:
            str_P = PSTR("SD card is not inserted or busy");
            break;
        case SD_ERR_STATE:
            str_P = PSTR("Incorrect state");
            break;
        case SD_ERR_VHS:
            str_P = PSTR("Invalid sended VHS");
            break;
        case SD_ERR_CRC:
            str_P = PSTR("Invalid CRC");
            break;
        case SD_ERR_IF_SEND:
            str_P = PSTR("Interface sending");
            break;
        case SD_ERR_VNOTSUPPORT:
            str_P = PSTR("Host not support a voltage of SD card");
            break;
        case SD_ERR_UNUSABLE:
            str_P = PSTR("SD card with non compatible voltage range");
            break;
        case SD_ERR_ILLEGAL_CMD:
            str_P = PSTR("Illegal command or SD card is not inserted");
            break;
        case SD_ERR_BAD_CSD:
            str_P = PSTR("Bad CSD version");
            break;
        case SD_ERR_READ_R1:
            str_P = PSTR("R1 response");
            break;
        case SD_ERR_READ:
            str_P = PSTR("Card read");
            break;
        case SD_ERR_WRITE:
            str_P = PSTR("Card write");
            break;
        case SD_ERR_NOMEM:
            str_P = PSTR("Out of memory");
            break;
        default:
            str_P = PSTR("Unknown");
            break;
    }
    printf_P(str_P);
    putchar('\n');
}

/*!
 * @param type SD Card type
 */
static uint32_t sd_get_cmd1_acmd41_arg(uint8_t type) {
    uint32_t result = 0;

    if (type != SD_TYPE_SC1)
        result |= ((uint32_t)1 << 30);

    return result;
}

/*!
 * @param mode 0 - check function; 1 - switch function
 * @param cmd_mode Function group for command mode
 * @param acc_mode Function group for access mode
 */
// static uint32_t sd_get_cmd6_arg(bool mode, uint8_t cmd_mode, uint8_t acc_mode) {
//     union {
//         struct {
//             uint32_t
//                 acc_mode : 4,   // function group 1 for access mode
//                 cmd_mode : 4,   // function group 1 for command mode
//                 fg_3 : 4,       // must be 0 or 0xF
//                 fg_4 : 4,       // must be 0 or 0xF
//                 fg_5 : 4,       // must be 0 or 0xF
//                 fg_6 : 4,       // must be 0 or 0xF
//                 reserved : 7,   // must be 0
//                 mode : 1;       // 0 - check function; 1 - switch function
//         };
//         uint32_t u32;
//     } result = {0};
//
//     result.mode = mode;
//     result.cmd_mode = cmd_mode;
//     result.acc_mode = acc_mode;
//
//     return result.u32;
// }

/*!
 * @param vhs Host Controller Voltage
 * @param pattern Check Pattern
 */
static uint32_t sd_get_cmd8_arg(uint8_t vhs, uint8_t pattern) {
    uint32_t result = 0;

    result |= (((vhs & 0x0F) << 8) | pattern);

    return result;
}

/*!
 * @param cnt Number of blocks to be pre-erasing before writing
 */
// static uint32_t sd_get_acmd23_arg(uint32_t cnt) {
//     union {
//         struct {
//             uint32_t cnt : 23;   // number of blocks
//         };
//         uint32_t u32;
//     } result = {0};
//
//     result.cnt = cnt;
//
//     return result.u32;
// }

static int8_t sd_wait_not_busy(void) {
    for (int32_t i = (F_CPU >> 5); i > 0; i--) {
        if (spi_read_8() == 0xFF)
            return SD_NO_ERR;
    }
    return SD_ERR_BUSY;
}

/*!
 * @brief Send command to SD card
 * @param cmd Command to send
 * @param args Arguments for command
 * @return SD_NO_ERR on success
 */
static int8_t sd_cmd_send(uint8_t cmd, uint32_t args) {
    struct cmd_token ct;

    if (sd_wait_not_busy()) {
        return SD_ERR_BUSY;
    }

    ct.start = 0;
    ct.tran_bit = 1;
    ct.cmd = cmd;
    ct.args = bswap_32(args);
    ct.end = 1;

    if (cmd == GO_IDLE_STATE)
        ct.crc = 0x4A;  // correct crc for CMD0 with args 0
    else if (cmd == SEND_IF_COND)
        ct.crc = 0x43;  // correct crc for CMD8 with args 0x1AA
    else
        ct.crc = 0x7F;

    spi_write_buf((void *)&ct, sizeof(ct));

    return SD_NO_ERR;
}

/*!
 * @brief Read R1 response
 */
static uint8_t sd_read_r1(void) {
    struct response_r1 r1;

    for (uint8_t i = 0; i < 25; i++) {
        r1.u8 = spi_read_8();
        if (r1.zero == 0)
            break;
    }

    return r1.u8;
}

/*!
 * @brief Wait and get specified data token
 * @param token Data Token
 * @return SD_NO_ERR on success
 */
static int8_t sd_wait_data_token(uint8_t token) {
    uint8_t rt;

    for (int32_t i = (F_CPU >> 8); i > 0; i--) {
        rt = spi_read_8();
        if (rt == token)
            break;
        if (!(rt & DET_ZERO) && (rt & ~DET_ZERO))
            return SD_ERR_READ; // get error token
        if (rt != 0xFF)
            return SD_ERR_BUSY;
    }

    return SD_NO_ERR;
}

/*!
 * @brief Determine block size and number of blocks.
 * Store this data to private structure.
 */
static int8_t sd_get_card_size(sd_card_t *sd) {
    bdev_t *bdev = sd->bdev;
    struct avr_pin_s *cs = &sd->spi_dev->cs;
    int8_t err;
    csd_t csd;
    uint32_t c_size;

    chip_select(cs);
    err = sd_cmd_send(SEND_CSD, 0);
    if (err != SD_NO_ERR) {
        chip_desel(cs);
        return err;
    }

    if (sd_read_r1() != R1_READY_STATE) {
        // Error state. exit
        chip_desel(cs);
        return SD_ERR_STATE;
    }

    err = sd_wait_data_token(SINGLE_START_BLOCK);
    if (err) {
        chip_desel(cs);
        return err;
    }
    spi_read_buf((void *)&csd, sizeof(csd));
    chip_desel(cs);

    if (csd.csd_ver == SD_CSD_V1) {
        uint8_t c_size_mult;

        c_size = ((csd.v1.c_size_high << 10) |
                  (csd.v1.c_size_mid << 2) |
                  csd.v1.c_size_low);
        c_size_mult = ((csd.v1.c_size_mult_high << 1) |
                       csd.v1.c_size_mult_low);
        c_size = (c_size + 1) << (c_size_mult + csd.v1.read_bl_len - 7);
        bdev->bd_blk_size = 1 << csd.v1.read_bl_len;
    } else if (csd.csd_ver == SD_CSD_V2) {
        c_size = (((uint32_t)csd.v2.c_size_high << 16) |
                  (csd.v2.c_size_mid << 8) |
                  csd.v2.c_size_low);
        c_size = (c_size + 1) << 10;
        bdev->bd_blk_size = 512;
    } else {
        return SD_ERR_BAD_CSD;
    }

    bdev->bd_blk_num = c_size;

    return SD_NO_ERR;
}

/*!
 * @brief Get and print CID register
 */
int8_t sd_get_cid(spi_dev_t *card) {
    struct avr_pin_s *cs = &card->cs;
    int8_t err = SD_NO_ERR;
    cid_t cid;
    char oid[3] = {0};
    char pnm[6] = {0};

    chip_select(cs);
    err = sd_cmd_send(SEND_CID, 0);
    if (err != SD_NO_ERR)
        goto out;

    if (sd_read_r1() != R1_READY_STATE) {
        // Error state. exit
        err = SD_ERR_READ_R1;
        goto out;
    }

    err = sd_wait_data_token(SINGLE_START_BLOCK);
    if (err != SD_NO_ERR)
        goto out;

    spi_read_buf((void *)&cid, sizeof(cid));
    chip_desel(cs);

    memcpy(oid, cid.oid, 2);
    memcpy(pnm, cid.pnm, 5);
    printf_P(PSTR("MID=0x%02X\n"), cid.mid);
    printf_P(PSTR("OEM/PNAME: %s/%s\n"), oid, pnm);
    printf_P(PSTR("rev=%u.%u\n"), cid.prv_n, cid.prv_m);
    printf_P(PSTR("sn=%lu\n"), cid.psn);
    printf_P(PSTR("Manufacturing date: %u/%u\n"), cid.mdt_month, ((cid.mdt_year_high << 4) | cid.mdt_year_low) + 2000);

out:
    chip_desel(cs);
    return err;
}

void sd_get_card_info(spi_dev_t *card) {
    bdev_t *bdev = blk_get_dev(card);
    sd_card_t *sd = sd_get_dev(bdev);

    printf_P(PSTR("Type:       "));
    switch (sd->type) {
        case SD_TYPE_SC1:
            printf_P(PSTR("SDSCv1"));
            break;
        case SD_TYPE_SC2:
            printf_P(PSTR("SDSCv2"));
            break;
        case SD_TYPE_HC:
            printf_P(PSTR("SDHC/SDXC"));
            break;
        default:
            printf_P(PSTR("unknown/unsupported"));
            break;
    }
    putchar('\n');

    printf_P(PSTR("Blocks:     %lu\n"), bdev->bd_blk_num);
    printf_P(PSTR("Block size: %u bytes\n"), bdev->bd_blk_size);    // always 512
    float size_gb = (double)((uint64_t)bdev->bd_blk_num << 9) / 1073741824.0;
    uint16_t rest_mb = (size_gb - (float)((uint16_t)size_gb)) * 1000.0f;
    printf_P(PSTR("Total Size: %u.%03u GiB\n"), (uint16_t)size_gb, rest_mb);
}

/** TODO: sd_read_end 
static int8_t sd_read_end(bdev_t *sd_bdev) {
    sd_card_t *sd = sd_get_dev(sd_bdev);
    int8_t err = SD_NO_ERR;
    uint8_t r1;

    if (sd->flags.in_block) {
        sd->flags.in_block = 0;

        err = sd_cmd_send(STOP_TRANSMISSION, 0);
        if (err != SD_NO_ERR)
            goto out;

        spi_read_8();   // dummy read

        r1 = sd_read_r1();
        if (r1 != R1_READY_STATE)
            err = SD_ERR_READ_R1;
    }
out:
    return err;
}
// */

/*!
 * @brief Read data from SD card
 * @param sd_bdev SD card block device
 * @param block Logical block to be read
 * @param dst Pointer to buffer to store data
 * @param offset Offset from start of block in bytes
 * @param cnt Number of bytes to read; 512 max
 * @return 0 on success
 */
static int8_t sd_read_data(bdev_t *sd_bdev, uint32_t block, uint8_t *dst,
                    uint16_t offset, uint16_t cnt) {
    sd_card_t *sd = sd_get_dev(sd_bdev);
    int8_t err = SD_NO_ERR;

    if (cnt == 0)
        return err;

    if ((offset + cnt) > 512) {
        // EINVAL - invalid arguments
        return 22;
    }

    chip_select(&sd->spi_dev->cs);

    if (!sd->flags.in_block || sd->cur_block != block || sd->offset > offset) {
        sd->cur_block = block;

        if ((sd->type == SD_TYPE_SC1) || (sd->type == SD_TYPE_SC2))
            // SDSC used byte address
            block <<= 9;

        err = sd_cmd_send(READ_SINGLE_BLOCK, block);
        if (err != SD_NO_ERR)
            goto out;

        uint8_t r1 = sd_read_r1();
        if (r1 != R1_READY_STATE) {
            err = SD_ERR_READ_R1;
            goto out;
        }

        err = sd_wait_data_token(SINGLE_START_BLOCK);
        if (err != SD_NO_ERR)
            goto out;

        sd->flags.in_block = 1;
        sd->offset = 0;
    }

    // skip to offset
    for (; sd->offset < offset; sd->offset++)
        spi_read_8();

    // get data
    spi_read_buf(dst, cnt);

    sd->offset += cnt;

    // stop transfer if partial read is disabled or are reads 512 bytes
    if (!sd->flags.partial_read || sd->offset >= 512) {
        sd->flags.in_block = 0;
        // pass rest data and crc
        while (sd->offset++ < 514)
            spi_read_8();
    }

out:
    chip_desel(&sd->spi_dev->cs);

    return err;
}

/*!
 * @brief Read a one 512 bytes block from SD card
 * @param sd_bdev SD card block device
 * @param block Logical block to be read
 * @param dst Pointer to buffer to store data
 * @return 0 on success
 */
static int8_t sd_read_block(bdev_t *sd_bdev, uint32_t block, uint8_t *dst) {
    return sd_read_data(sd_bdev, block, dst, 0, 512);
}

/*!
 * @brief Write a one 512 bytes block to SD card
 * @param sd_bdev SD card block device
 * @param block Logical block to be write
 * @param src Pointer to buffer of data to be write
 * @return 0 on success
 */
static int8_t sd_write_block(bdev_t *sd_bdev, uint32_t block, const uint8_t *src) {
    sd_card_t *sd = sd_get_dev(sd_bdev);
    int8_t err = SD_NO_ERR;
    uint8_t response;

    if ((sd->type == SD_TYPE_SC1) || (sd->type == SD_TYPE_SC2))
        // SDSC used byte address
        block <<= 9;

    chip_select(&sd->spi_dev->cs);

    err = sd_cmd_send(WRITE_BLOCK, block);
    if (err != SD_NO_ERR)
        goto out;

    response = sd_read_r1();
    if (response != R1_READY_STATE) {
        err = SD_ERR_READ_R1;
        goto out;
    }

    // send token
    spi_write(SINGLE_START_BLOCK);
    // send data
    spi_write_buf(src, 512);
    // send crc
    spi_write16(0xFFFF);

    // get response token
    response = spi_read_8() & DRT_MASK;
    if (response != DRT_ACCCEPT) {
        /** TODO: to determine the cause of the error, send CMD13
         * (SEND_STATUS) and receive a R2 */
        err = SD_ERR_WRITE;
        goto out;
    }

    // wait to complete
    if (sd_wait_not_busy())
        err = SD_ERR_BUSY;

out:
    chip_desel(&sd->spi_dev->cs);

    return err;
}

static int8_t sd_request(struct request_s *req) {
    int8_t ret = -1;

    switch (req->cmd_flags) {
        case REQ_READ:
            ret = sd_read_block(req->bdev, req->block, req->buf);
            break;

        case REQ_WRITE:
            ret = sd_write_block(req->bdev, req->block, req->buf);
            break;

        default:
            break;
    }

    return ret;
}

static const struct blk_dev_ops_s sd_bdev_ops PROGMEM = {
    // .open = NULL,
    // .release = NULL,
    .request = sd_request,
};

/*!
 * @brief SD card initialization routine
 * @param card SD card on SPI device
 * @return 0 on success
 */
int8_t sd_init(spi_dev_t *card) {
    uint8_t sreg = SREG;
    int8_t err = SD_ERR_BUSY;
    uint8_t r1;
    bdev_t *bdev;
    sd_card_t *sd;
    struct avr_pin_s *cs = &card->cs;

    cli();
    _delay_ms(1);
    spi_set_speed(25000000);

    printf_P(PSTR("\nInitialize SD card... "));

    // send a minimum of 74 clocks with CS and MOSI is high.
    chip_desel(cs);
    for (uint8_t i = 0; i < 10; i++)
        spi_write(0xFF);

    for (uint8_t try = 0; try < 3; try++) {
        // turn to idle state
        chip_select(cs);
        if (sd_cmd_send(GO_IDLE_STATE, 0) != SD_NO_ERR) {
            chip_desel(cs);
            err = SD_ERR_BUSY;
            continue;   // busy
        }

        // wait for response
        r1 = sd_read_r1();
        chip_desel(cs);
        if (r1 == R1_IDLE_STATE) {
            err = SD_NO_ERR;
            break;
        }
        err = SD_ERR_READ_R1;
    }
    if (err != SD_NO_ERR)
        // SD card is not set or busy
        goto out;

    bdev = malloc(sizeof(bdev_t));
    if (!bdev) {
        // ENOMEM
        err = SD_ERR_NOMEM;
        goto out;
    }

    sd = malloc(sizeof(sd_card_t));
    if (!sd) {
        // ENOMEM
        free(bdev);
        err = SD_ERR_NOMEM;
        goto out;
    }
    blk_set_dev(card, bdev);
    sd_set_dev(bdev, sd);
    bdev->blk_ops = &sd_bdev_ops;

    sd->spi_dev = card;
    sd->bdev = bdev;
    sd->flags.in_block = sd->flags.partial_read = 0;
    sd->cur_block = 0;
    sd->offset = 0;

    // send host controller voltage and check pattern
    chip_select(cs);
    err = sd_cmd_send(SEND_IF_COND, sd_get_cmd8_arg(VHS_3V3, CHECK_PATTERN));
    if (err != SD_NO_ERR) {
        // SD card is not set or busy
        chip_desel(cs);
        goto error;
    }

    r1 = sd_read_r1();

    if (r1 == R1_IDLE_STATE) {
        struct response_r7 r7;

        spi_read_buf((void *)&r7, sizeof(r7));
        chip_desel(cs);

        if ((r7.voltage_accepted != VHS_3V3) ||
            (r7.check_pattern != CHECK_PATTERN)) {
            // Error state. Exit
            err = SD_ERR_VHS;
            goto error;
        }
        // ver 2.00 or later SD Memory Card
        sd->type = SD_TYPE_SC2;
    } else {
        chip_desel(cs);
        if (r1 & R1_ILLEGAL_CMD) {
            // ver 1.X SD Memory Card (e.g. SDSC) or not SD Memory Card
            sd->type = SD_TYPE_SC1;
        } else if (r1 & R1_CRC_ERR) {
            // CRC error
            err = SD_ERR_CRC;
            goto error;
        } else {
            // Error state. Exit
            err = SD_ERR_IF_SEND;
            goto error;
        }
    }

    // check voltage range
    chip_select(cs);
    err = sd_cmd_send(READ_OCR, 0);
    if (err != SD_NO_ERR) {
        // SD card is not set or busy
        chip_desel(cs);
        goto error;
    }

    r1 = sd_read_r1();

    if (r1 == R1_IDLE_STATE) {
        struct response_r3 r3;

        spi_read_buf((void *)&r3, sizeof(r3));
        chip_desel(cs);

        // check 3.3v support
        if (!(r3.v_3v1_3v2 || r3.v_3v2_3v3)) {
            err = SD_ERR_VNOTSUPPORT;
            goto error;
        }
    } else {
        chip_desel(cs);
        err = SD_ERR_UNUSABLE;
        goto error;
    }

    // Start of initialization
    while (true) {
        uint8_t flag = false;
        // prepare to send app cmd
        while (!flag) {
            chip_select(cs);
            err = sd_cmd_send(APP_CMD, 0);
            if (err != SD_NO_ERR) {
                // SD card is not set or busy
                chip_desel(cs);
                goto error;
            }

            r1 = sd_read_r1();
            chip_desel(cs);

            if (r1 == R1_IDLE_STATE)
                break;
        }

        // init
        chip_select(cs);
        err = sd_cmd_send(flag ? SEND_OP_COND : SD_SEND_OP_COND,
                          sd_get_cmd1_acmd41_arg(sd->type));
        if (err != SD_NO_ERR) {
            // SD card is not set or busy
            chip_desel(cs);
            goto error;
        }

        r1 = sd_read_r1();
        chip_desel(cs);

        if (r1 & R1_ILLEGAL_CMD) {
            if (sd->type == SD_TYPE_SC1) {
                if (!flag) {
                    flag = true;
                    continue;
                }
            }
            // Error State. Exit
            err = SD_ERR_ILLEGAL_CMD;
            goto error;
        } else if (r1 == R1_READY_STATE) {
            // Initialization is complete
            break;
        } else if (r1 != R1_IDLE_STATE) {
            // Error State. Exit
            err = SD_ERR_STATE;
            goto error;
        }
        // else card is still initializing. retry.
    }

    if (sd->type != SD_TYPE_SC1) {
        // check CCS (Card Capacity Status)
        chip_select(cs);
        err = sd_cmd_send(READ_OCR, 0);
        if (err != SD_NO_ERR) {
            // SD card is not set or busy
            chip_desel(cs);
            goto error;
        }

        r1 = sd_read_r1();

        if (r1 == R1_READY_STATE) {
            struct response_r3 r3;

            spi_read_buf((void *)&r3, sizeof(r3));
            chip_desel(cs);

            if (!r3.busy) {
                // card still in init state. this error?
                err = SD_ERR_STATE;
                goto error;
            }
            if (r3.ccs) {
                // is SDHC/SDXC
                sd->type = SD_TYPE_HC;
                // init success
            } else {
                // is SDSCv2
                sd->type = SD_TYPE_SC2;
                chip_select(cs);
                err = sd_cmd_send(SET_BLOCKLEN, 512);
                if (err != SD_NO_ERR) {
                    // SD card is not set or busy
                    chip_desel(cs);
                    goto error;
                }

                r1 = sd_read_r1();
                chip_desel(cs);
                if (r1 != R1_READY_STATE) {
                    // error state. exit
                    err = SD_ERR_READ_R1;
                    goto error;
                }
                // init success
            }
        } else {
            chip_desel(cs);
            err = SD_ERR_READ_R1;
            goto error;
        }
    }

    // Set block length to 512 bytes
    if ((sd->type == SD_TYPE_SC1) ||
        (sd->type == SD_TYPE_SC2)) {
        chip_select(cs);
        err = sd_cmd_send(SET_BLOCKLEN, 512);
        if (err != SD_NO_ERR) {
            // SD card is not set or busy
            chip_desel(cs);
            goto error;
        }

        r1 = sd_read_r1();
        chip_desel(cs);
        if (r1 != R1_READY_STATE) {
            // Error state. exit
            err = SD_ERR_READ_R1;
            goto error;
        }
        // success
    }

    err = sd_get_card_size(sd);
    if (err)
        goto error;

out:
    chip_desel(cs);
    SREG = sreg;

    sd_print_err_msg(err);

    return err;
error:
    sd_set_dev(bdev, NULL);
    free(bdev);
    sd->bdev = NULL;
    free(sd);
    blk_set_dev(card, NULL);
    goto out;
}
