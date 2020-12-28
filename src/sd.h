#ifndef SD_H
#define SD_H

#include <stdint.h>

#include <spi.h>

int8_t sd_init(spi_dev_t *card);

int8_t sd_get_cid(spi_dev_t *card);
void sd_get_card_info(spi_dev_t *card);

#endif  /* !SD_H */
