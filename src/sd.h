#ifndef SD_H
#define SD_H

#include <stdint.h>

#include <spi.h>

int8_t sd_init(spi_dev_t *card);
int8_t sd_read_data(spi_dev_t *card, uint32_t block, uint8_t *dst,
                    uint16_t offset, uint16_t cnt);
int8_t sd_read_block(spi_dev_t *card, uint32_t block, uint8_t *dst);
int8_t sd_write_block(spi_dev_t *card, uint32_t block, const uint8_t *src);

int8_t sd_get_cid(spi_dev_t *card);
void sd_get_card_info(spi_dev_t *card);

#endif  /* !SD_H */
