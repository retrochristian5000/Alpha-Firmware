/* Basic ATA/ATAPI support for the Alpha firmware console.
 *
 * Copyright (C) 2026 World History Project contributors
 *
 * This file is part of Alpha Firmware.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef IDE_H
#define IDE_H 1

#include "protos.h"

#define IDE_MAX_DEVICES 4

enum ide_device_type
{
  IDE_DEVICE_NONE = 0,
  IDE_DEVICE_ATA,
  IDE_DEVICE_ATAPI
};

struct ide_device
{
  enum ide_device_type type;
  uint8_t channel;
  uint8_t drive;
  uint16_t io_base;
  uint16_t control_base;
  uint64_t blocks;
  uint32_t block_size;
  char model[41];
};

extern void ide_setup(void);
extern unsigned int ide_device_count(void);
extern const struct ide_device *ide_get_device(unsigned int index);
extern int ide_read_blocks(unsigned int index, uint64_t lba,
                           unsigned int count, void *buffer);

#endif /* IDE_H */
