/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <flash.h>
#include <spi.h>
#include <init.h>
#include <string.h>
#include "spi_flash_is25wp016d.h"
#include "flash_priv.h"

struct is25wp016d_device_data {
	struct device *spi;
#if defined(CONFIG_SPI_FLASH_IS25WP016D_GPIO_SPI_CS)
	struct spi_cs_control cs_ctrl;
#endif /* CONFIG_SPI_FLASH_IS25WP016D_GPIO_SPI_CS */
	struct spi_config spi_cfg;
#if defined(CONFIG_MULTITHREADING)
	struct k_sem my_sem;
#endif /* CONFIG_MULTITHREADING */
};

#if defined(CONFIG_MULTITHREADING)
#define SYNC_INIT() k_sem_init( \
		&((struct spi_flashis25_device_data *)dev->driver_data)->my_sem, 1, UINT_MAX)
#define SYNC_LOCK() k_sem_take(&driver_data->my_sem, K_FOREVER)
#define SYNC_UNLOCK() k_sem_give(&driver_data->my_sem)
#else
#define SYNC_INIT()
#define SYNC_LOCK()
#define SYNC_UNLOCK()
#endif

static int is25wp016d_access(const struct device *const dev,
			     u8_t opcode,bool address_check,off_t offset_addr,
			     void *data, size_t length,bool isWrite)
{
	struct is25wp016d_device_data *const driver_data = dev->driver_data;

	u8_t my_buf[4] = {
		opcode,
		(addr & 0xFF0000) >> 16,
		(addr & 0xFF00) >> 8,
		(addr & 0xFF),
	};

	struct spi_buf spi_buf[2] = {
		{
			.buf = my_buf,
			.len = (is_addressed) ? 4 : 1,
		},
		{
			.buf = data,
			.len = length
		}
	};
	const struct spi_buf_set tx_set = {
		.buffers = spi_buf,
		.count = (length) ? 2 : 1
	};

	const struct spi_buf_set rx_set = {
		.buffers = spi_buf,
		.count = 2
	};

	if (is_write) {
		return spi_write(driver_data->spi,
			&driver_data->spi_cfg, &tx_set);
	}

	return spi_transceive(driver_data->spi,
		&driver_data->spi_cfg, &tx_set, &rx_set);
}


static inline int is25wp016d_read_id(struct device *dev)
{
	struct is25wp016d_device_data *const driver_data = dev->driver_data;
	u32_t data_id;
	u8_t my_buf[IS25WP016D_MAX_ID_LEN];

	if (is25wp016d_access(driver_data, IS25WP016D_CMD_RDID,
				false, 0, my_buf, IS25WP016D_MAX_ID_LEN, false) != 0) {
		return -EIO;
	}

	data_id = ((u32_t) my_buf[0]) << 16;
	data_id |= ((u32_t) my_buf[1]) << 8;
	data_id |= (u32_t) my_buf[2];

	if (data_id != CONFIG_SPI_FLASH_IS25WP016D_DEVICE_ID) {
		return -ENODEV;
	}

	return 0;
}

static u8_t is25wp016d_read_reg(struct device *dev, u8_t reg)
{
	struct is25wp016d_device_data *const driver_data = dev->driver_data;

	if (is25wp016d_access(driver_data, reg,
				false, 0, &reg, 1, false)) {
		return 0;
	}

	return reg;
}
static inline void wait_for_flash_idle(struct device *dev)
{
	u8_t reg;

	do {
		reg = is25wp016d_read_reg(dev, IS25WP016D_CMD_RDSR);
	} while (reg & IS25WP016D_WIP_BIT);
}

static int is25wp016d_write_reg(struct device *dev, u8_t reg)
{
	struct is25wp016d_device_data *const driver_data = dev->driver_data;

	if (is25wp016d_access(driver_data, reg, false, 0,
				NULL, 0, true) != 0) {
		return -EIO;
	}

	return 0;
}

static int is25wp016d_read(struct device *dev, off_t offset, void *data,
			     size_t len)
{
	struct is25wp016d_device_data *const driver_data = dev->driver_data;
	int ret;

	if (offset < 0) {
		return -ENODEV;
	}

	SYNC_LOCK();

	wait_for_flash_idle(dev);

	ret = is25wp016d_access(driver_data, IS25WP016D_CMD_READ,
				  true, offset, data, len, false);

	SYNC_UNLOCK();

	return ret;
}

static int is25wp016d_write_protection_set_with_lock(struct device *dev,
						       bool enable, bool lock)
{
	struct is25wp016d_device_data *const driver_data = dev->driver_data;
	u8_t reg = 0U;
	int ret;

	if (lock) {
		SYNC_LOCK();
	}

	wait_for_flash_idle(dev);

	if (enable) {
		reg = IS25WP016D_CMD_WRDI;
	} else {
		reg = IS25WP016D_CMD_WREN;
	}

	ret = is25wp016d_write_reg(dev, reg);

	if (lock) {
		SYNC_UNLOCK();
	}

	return ret;
}

static int is25wp016d_write_protection(struct device *dev, bool enable)
{
	return is25wp016d_write_protection_set_with_lock(dev, enable, true);
}

static int is25wp016d_program_page(struct device *dev, off_t offset,
		const void *data, size_t len)
{
	u8_t reg;
	struct is25wp016d_device_data *const driver_data = dev->driver_data;

	__ASSERT(len <= CONFIG_SPI_FLASH_IS25WP016D_PAGE_PROGRAM_SIZE,
		 "Maximum length is %d for page programming (actual:%d)",
		 CONFIG_SPI_FLASH_IS25WP016D_PAGE_PROGRAM_SIZE, len);

	wait_for_flash_idle(dev);

	reg = is25wp016d_read_reg(dev, IS25WP016D_CMD_RDSR);
	if (!(reg & IS25WP016D_WEL_BIT)) {
		return -EIO;
	}

	wait_for_flash_idle(dev);

	/* Assume write protection has been disabled. Note that is25wp016d
	 * flash automatically turns on write protection at the completion
	 * of each write or erase transaction.
	 */
	return is25wp016d_access(driver_data, IS25WP016D_CMD_PP,
				  true, offset, (void *)data, len, true);

}

static int is25wp016d_write(struct device *dev, off_t offset,
			      const void *data, size_t len)
{
	int ret;
	off_t page_offset;
	/* Cast `data`  to prevent `void*` arithmetic */
	const u8_t *data_ptr = data;
	struct is25wp016d_device_data *const driver_data = dev->driver_data;

	if (offset < 0) {
		return -ENOTSUP;
	}

	SYNC_LOCK();

	/* Calculate the offset in the first page we write */
	page_offset = offset % CONFIG_SPI_FLASH_IS25WP016D_PAGE_PROGRAM_SIZE;

	/*
	 * Write all data that does not fit into a single programmable page.
	 * By doing this logic, we can safely disable lock protection in
	 * between pages as in case the user did not disable protection then
	 * it will fail on the first write.
	 */
	while ((page_offset + len) >
			CONFIG_SPI_FLASH_IS25WP016D_PAGE_PROGRAM_SIZE) {
		size_t len_to_write_in_page =
			CONFIG_SPI_FLASH_IS25WP016D_PAGE_PROGRAM_SIZE -
			page_offset;

		ret = is25wp016d_program_page(dev, offset,
						data_ptr, len_to_write_in_page);
		if (ret) {
			goto end;
		}

		ret = is25wp016d_write_protection_set_with_lock(dev,
				false, false);
		if (ret) {
			goto end;
		}

		len -= len_to_write_in_page;
		offset += len_to_write_in_page;
		data_ptr += len_to_write_in_page;

		/*
		 * For the subsequent pages we always start at the beginning
		 * of a page
		 */
		page_offset = 0;
	}

	ret = is25wp016d_program_page(dev, offset, data_ptr, len);

end:
	SYNC_UNLOCK();

	return ret;
}

static inline int is25wp016d_erase_internal(struct device *dev,
					      off_t offset, size_t size)
{
	struct is25wp016d_device_data *const driver_data = dev->driver_data;
	bool need_offset = true;
	u8_t erase_opcode;

	if (offset < 0) {
		return -ENOTSUP;
	}

	wait_for_flash_idle(dev);

	/* write enable */
	is25wp016d_write_reg(dev, IS25WP016D_CMD_WREN);

	wait_for_flash_idle(dev);

	switch (size) {
	case IS25WP016D_SECTOR_SIZE:
		erase_opcode =IS25WP016D_CMD_SE ;
		break;
	case IS25WP016D_BLOCK32K_SIZE:
		erase_opcode = IS25WP016D_CMD_BE32K;
		break;
	case IS25WP016D_BLOCK_SIZE:
		erase_opcode = IS25WP016D_CMD_BE;
		break;
	case CONFIG_SPI_FLASH_IS25WP016D_FLASH_SIZE:
		erase_opcode = IS25WP016D_CMD_CE;
		need_offset = false;
		break;
	default:
		return -EIO;

	}

	/* Assume write protection has been disabled. Note that IS25WP016D
	 * flash automatically turns on write protection at the completion
	 * of each write or erase transaction.
	 */
	return is25wp016d_access(driver_data, erase_opcode,
				   need_offset, offset, NULL, 0, true);
}

static int is25wp016d_erase(struct device *dev, off_t offset, size_t size)
{
	struct is25wp016d_device_data *const driver_data = dev->driver_data;
	int ret = 0;
	u32_t new_offset = offset;
	u32_t size_remaining = size;
	u8_t reg;

	if ((offset < 0) || ((offset & IS25WP016D_SECTOR_MASK) != 0) ||
	    ((size + offset) > CONFIG_SPI_FLASH_IS25WP016D_FLASH_SIZE) ||
	    ((size & IS25WP016D_SECTOR_MASK) != 0)) {
		return -ENODEV;
	}

	SYNC_LOCK();

	reg = spi_flash_wb_reg_read(dev, IS25WP016D_CMD_RDSR);

	if (!(reg & IS25WP016D_WEL_BIT)) {
		SYNC_UNLOCK();
		return -EIO;
	}

	while ((size_remaining >= IS25WP016D_SECTOR_SIZE) && (ret == 0)) {
		if (size_remaining == CONFIG_SPI_FLASH_IS25WP016D_FLASH_SIZE) {
			ret = is25wp016d_erase_internal(dev, offset, size);
			break;
		}

		if (size_remaining >= IS25WP016D_BLOCK_SIZE) {
			ret = is25wp016d_erase_internal(dev, new_offset,
							  IS25WP016D_BLOCK_SIZE);
			new_offset += IS25WP016D_BLOCK_SIZE;
			size_remaining -= IS25WP016D_BLOCK_SIZE;
			continue;
		}

		if (size_remaining >= IS25WP016D_BLOCK32K_SIZE) {
			ret = is25wp016d_erase_internal(dev, new_offset,
							  IS25WP016D_BLOCK32K_SIZE);
			new_offset += IS25WP016D_BLOCK32K_SIZE;
			size_remaining -= IS25WP016D_BLOCK32K_SIZE;
			continue;
		}

		if (size_remaining >= IS25WP016D_SECTOR_SIZE) {
			ret = is25wp016d_erase_internal(dev, new_offset,
							  IS25WP016D_SECTOR_SIZE);
			new_offset += IS25WP016D_SECTOR_SIZE;
			size_remaining -= IS25WP016D_SECTOR_SIZE;
			continue;
		}
	}

	SYNC_UNLOCK();

	return ret;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static struct flash_pages_layout dev_layout;

static void is25wp016d_pages_layout(struct device *dev,
				  const struct flash_pages_layout **layout,
				  size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api spi_flash_api = {
	.read = is25wp016d_read,
	.write =is25wp016d_write,
	.erase =is25wp016d_erase,
	.write_protection = is25wp016d_write_protection,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = is25wp016d_pages_layout,
#endif
	.write_block_size = 1,
};


static int spi_flash_configuration(struct device *dev)
{
	struct spi_flashis25_device_data *data = dev->driver_data;

	data->spi = device_get_binding(DT_IS25WP016D_W25Q16_0_BUS_NAME);
	if (!data->spi) {
		return -EINVAL;
	}

	data->spi_cfg.frequency = DT_IS25WP016D_W25Q16_0_SPI_MAX_FREQUENCY;
	data->spi_cfg.operation = SPI_WORD_SET(8);
	data->spi_cfg.slave = DT_IS25WP016D_W25Q16_0_BASE_ADDRESS;

#if defined(CONFIG_SPI_FLASH_IS25WP016D_GPIO_SPI_CS)
	data->cs_ctrl.gpio_dev = device_get_binding(
		DT_IS25WP016D_W25Q16_0_CS_GPIO_CONTROLLER);
	if (!data->cs_ctrl.gpio_dev) {
		return -ENODEV;
	}

	data->cs_ctrl.gpio_pin = DT_IS25WP016D_W25Q16_0_CS_GPIO_PIN;
	data->cs_ctrl.delay = CONFIG_SPI_FLASH_IS25WP016D_GPIO_CS_WAIT_DELAY;

	data->spi_cfg.cs = &data->cs_ctrl;
#endif /* CONFIG_SPI_FLASH_IS25WP016D_GPIO_SPI_CS */

	return spi_flash_wb_id(dev);
}

static int spi_flash_init(struct device *dev)
{
	int ret;
	SYNC_INIT();
	ret = spi_flash_configuration(dev);

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	/*
	 * Note: we use the sector size rather than the page size as some
	 * modules that consumes the flash page layout assume the page
	 * size is the minimal size they can erase.
	 */
	dev_layout.pages_count = (CONFIG_SPI_FLASH_IS25WP016D_FLASH_SIZE / IS25WP016D_SECTOR_SIZE);
	dev_layout.pages_size = IS25WP016D_SECTOR_SIZE;
#endif

	return ret;
}

static struct spi_flashis25_device_data spi_flash_memory_data;

DEVICE_AND_API_INIT(spi_flash_memory, CONFIG_SPI_FLASH_IS25WP016D_DRV_NAME,
	    spi_flash_init, &spi_flash_memory_data, NULL, POST_KERNEL,
	    CONFIG_SPI_FLASH_IS25WP016D_INIT_PRIORITY, &spi_flash_api);
