/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief This file defines the private data structures for spi flash driver for is25wp016d
 */

#ifndef ZEPHYR_DRIVERS_FLASH_SPI_FLASH_IS25WP016D_H_
#define ZEPHYR_DRIVERS_FLASH_SPI_FLASH_IS25WP016D_H_

#define IS25WP016D_MAX_ID_LEN	3

#define IS25WP016D_RDID_VALUE  (0x009d7015)
#define IS25WP016D_WIP_BIT         (0x1 << 0)
#define IS25WP016D_WEL_BIT         (0x1 << 1)

#define IS25WP016D_SRWD_BIT        (0x1 << 7)
#define IS25WP016D_SR_BP_OFFSET    (2)
#define IS25WP016D_CMD_RDSR        0x05
#define IS25WP016D_CMD_READ        0x03
#define IS25WP016D_CMD_WREN        0x06
#define IS25WP016D_CMD_WRDI        0x04
#define IS25WP016D_CMD_PP          0x02
#define IS25WP016D_SECTOR_SIZE     (0x1000)
#define IS25WP016D_BLOCK32K_SIZE   (0x8000)
#define IS25WP016D_BLOCK_SIZE      (0x10000)
#define IS25WP016D_CMD_SE          0xD7
#define IS25WP016D_CMD_BE32K       0x52
#define IS25WP016D_CMD_BE          0xD8
#define IS25WP016D_CMD_CE          0xC7
#define IS25WP016D_SECTOR_MASK     (0xFFF)
#define IS25WP016D_CMD_RDID        0x9F



#endif /* ZEPHYR_DRIVERS_FLASH_SPI_FLASH_IS25WP016D_H_ */
