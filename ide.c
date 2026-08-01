/* Basic polled ATA/ATAPI support for the Alpha firmware console.
 *
 * Copyright (C) 2026 World History Project contributors
 *
 * This file is part of Alpha Firmware.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * The initial implementation intentionally uses PIO.  That keeps early
 * firmware storage independent of interrupts, DMA windows, and an ARC
 * memory allocator while still providing the sector reads needed by a
 * future ARC/AlphaBIOS-compatible loader.
 */

#include "protos.h"
#include "pci.h"
#include "pci_regs.h"
#include "ide.h"

#define PCI_VENDOR_ID_CMD             0x1095
#define PCI_DEVICE_ID_CMD_646         0x0646
#define PCI_CLASS_STORAGE_IDE         0x0101

#define CMD646_CNTRL                  0x51
#define CMD646_CNTRL_EN_CH0           0x04
#define CMD646_CNTRL_EN_CH1           0x08

#define ATA_REG_DATA                  0
#define ATA_REG_ERROR                 1
#define ATA_REG_FEATURES              1
#define ATA_REG_NSECT                 2
#define ATA_REG_LBAL                  3
#define ATA_REG_LBAM                  4
#define ATA_REG_LBAH                  5
#define ATA_REG_DEVICE                6
#define ATA_REG_STATUS                7
#define ATA_REG_COMMAND               7

#define ATA_STATUS_ERR                0x01
#define ATA_STATUS_DRQ                0x08
#define ATA_STATUS_DF                 0x20
#define ATA_STATUS_BSY                0x80

#define ATA_DEVICE_MASTER             0xa0
#define ATA_DEVICE_SLAVE              0xb0
#define ATA_DEVICE_LBA                0x40

#define ATA_DEVCTL_NIEN               0x02
#define ATA_DEVCTL_SRST               0x04

#define ATA_CMD_READ_SECTORS          0x20
#define ATA_CMD_PACKET                0xa0
#define ATA_CMD_IDENTIFY_PACKET       0xa1
#define ATA_CMD_IDENTIFY              0xec

#define ATAPI_CMD_READ_10             0x28

#define ATA_SECTOR_SIZE               512
#define ATAPI_SECTOR_SIZE             2048

#define IDE_POLL_DELAY_USEC           10
#define IDE_TIMEOUT_USEC              (2 * 1000 * 1000)
#define IDE_POLL_COUNT                (IDE_TIMEOUT_USEC / IDE_POLL_DELAY_USEC)

static struct ide_device ide_devices[IDE_MAX_DEVICES];
static unsigned int ide_devices_found;

static uint8_t
ide_status(const struct ide_device *dev)
{
  return inb(dev->io_base + ATA_REG_STATUS);
}

static uint8_t
ide_alt_status(const struct ide_device *dev)
{
  return inb(dev->control_base + 2);
}

static void
ide_delay_400ns(const struct ide_device *dev)
{
  ide_alt_status(dev);
  ide_alt_status(dev);
  ide_alt_status(dev);
  ide_alt_status(dev);
}

static void
ide_select(const struct ide_device *dev, bool lba, uint8_t high_lba)
{
  uint8_t value = dev->drive ? ATA_DEVICE_SLAVE : ATA_DEVICE_MASTER;

  if (lba)
    value |= ATA_DEVICE_LBA | (high_lba & 0x0f);

  outb(value, dev->io_base + ATA_REG_DEVICE);
  ide_delay_400ns(dev);
}

static int
ide_wait(const struct ide_device *dev, uint8_t set, uint8_t clear)
{
  unsigned long i;

  for (i = 0; i < IDE_POLL_COUNT; ++i)
    {
      uint8_t status = ide_status(dev);

      if (status == 0xff)
        return -1;
      if (status & clear & (ATA_STATUS_ERR | ATA_STATUS_DF))
        return -1;
      if ((status & clear) == 0 && (status & set) == set)
        return status;

      udelay(IDE_POLL_DELAY_USEC);
    }

  return -1;
}

static void
ide_read_words(const struct ide_device *dev, void *buffer,
               unsigned int words)
{
  uint8_t *out = buffer;
  unsigned int i;

  for (i = 0; i < words; ++i)
    {
      uint16_t value = inw(dev->io_base + ATA_REG_DATA);
      out[i * 2] = value;
      out[i * 2 + 1] = value >> 8;
    }
}

static void
ide_discard_words(const struct ide_device *dev, unsigned int words)
{
  while (words--)
    inw(dev->io_base + ATA_REG_DATA);
}

static void
ide_write_packet(const struct ide_device *dev, const uint8_t packet[12])
{
  unsigned int i;

  for (i = 0; i < 12; i += 2)
    outw(packet[i] | ((uint16_t)packet[i + 1] << 8),
         dev->io_base + ATA_REG_DATA);
}

static void
ide_model_string(char model[41], const uint16_t identify[256])
{
  unsigned int i;

  for (i = 0; i < 40; ++i)
    {
      uint16_t value = identify[27 + i / 2];
      model[i] = (i & 1) ? value : value >> 8;
    }
  model[40] = 0;

  while (i > 0 && model[i - 1] == ' ')
    model[--i] = 0;
}

static uint64_t
ide_identify_blocks(const uint16_t identify[256])
{
  if (identify[83] & (1 << 10))
    {
      return ((uint64_t)identify[100]
              | (uint64_t)identify[101] << 16
              | (uint64_t)identify[102] << 32
              | (uint64_t)identify[103] << 48);
    }

  return (uint64_t)identify[60] | (uint64_t)identify[61] << 16;
}

static int
ide_identify(struct ide_device *dev)
{
  uint16_t identify[256];
  uint8_t status;
  uint8_t signature_mid;
  uint8_t signature_high;
  unsigned int i;

  ide_select(dev, false, 0);

  outb(0, dev->io_base + ATA_REG_NSECT);
  outb(0, dev->io_base + ATA_REG_LBAL);
  outb(0, dev->io_base + ATA_REG_LBAM);
  outb(0, dev->io_base + ATA_REG_LBAH);
  outb(ATA_CMD_IDENTIFY, dev->io_base + ATA_REG_COMMAND);

  status = ide_status(dev);
  if (status == 0 || status == 0xff)
    return -1;

  if (ide_wait(dev, 0, ATA_STATUS_BSY) < 0)
    return -1;

  status = ide_status(dev);
  signature_mid = inb(dev->io_base + ATA_REG_LBAM);
  signature_high = inb(dev->io_base + ATA_REG_LBAH);

  if (status & ATA_STATUS_ERR)
    {
      bool packet_device =
        ((signature_mid == 0x14 && signature_high == 0xeb)
         || (signature_mid == 0x69 && signature_high == 0x96));

      if (!packet_device)
        return -1;

      outb(ATA_CMD_IDENTIFY_PACKET, dev->io_base + ATA_REG_COMMAND);
      if (ide_wait(dev, ATA_STATUS_DRQ,
                   ATA_STATUS_BSY | ATA_STATUS_ERR | ATA_STATUS_DF) < 0)
        return -1;

      dev->type = IDE_DEVICE_ATAPI;
      dev->block_size = ATAPI_SECTOR_SIZE;
    }
  else
    {
      if (ide_wait(dev, ATA_STATUS_DRQ,
                   ATA_STATUS_BSY | ATA_STATUS_ERR | ATA_STATUS_DF) < 0)
        return -1;

      dev->type = IDE_DEVICE_ATA;
      dev->block_size = ATA_SECTOR_SIZE;
    }

  for (i = 0; i < 256; ++i)
    identify[i] = inw(dev->io_base + ATA_REG_DATA);

  ide_model_string(dev->model, identify);
  if (dev->type == IDE_DEVICE_ATA)
    dev->blocks = ide_identify_blocks(identify);
  else
    dev->blocks = 0;

  return 0;
}

static void
ide_reset_channel(uint16_t control_base)
{
  outb(ATA_DEVCTL_NIEN | ATA_DEVCTL_SRST, control_base + 2);
  udelay(5);
  outb(ATA_DEVCTL_NIEN, control_base + 2);
  udelay(2000);
}

static uint16_t
ide_io_bar(int bdf, int offset, uint16_t fallback)
{
  uint32_t bar = pci_config_readl(bdf, offset);

  if ((bar & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO)
    {
      uint32_t base = bar & PCI_BASE_ADDRESS_IO_MASK;
      if (base != 0)
        return base;
    }

  return fallback;
}

static int
ide_find_controller(void)
{
  int bdf, max;
  int class_match = -1;

  foreachpci (bdf, max)
    {
      uint16_t vendor = pci_config_readw(bdf, PCI_VENDOR_ID);
      uint16_t device = pci_config_readw(bdf, PCI_DEVICE_ID);
      uint16_t class_id = pci_config_readw(bdf, PCI_CLASS_DEVICE);

      if (vendor == PCI_VENDOR_ID_CMD && device == PCI_DEVICE_ID_CMD_646)
        return bdf;
      if (class_match < 0 && class_id == PCI_CLASS_STORAGE_IDE)
        class_match = bdf;
    }

  return class_match;
}

static void
ide_probe_channel(uint8_t channel, uint16_t io_base, uint16_t control_base)
{
  uint8_t drive;

  ide_reset_channel(control_base);

  for (drive = 0; drive < 2 && ide_devices_found < IDE_MAX_DEVICES; ++drive)
    {
      struct ide_device dev;

      memset(&dev, 0, sizeof(dev));
      dev.channel = channel;
      dev.drive = drive;
      dev.io_base = io_base;
      dev.control_base = control_base;

      if (ide_identify(&dev) == 0)
        {
          ide_devices[ide_devices_found++] = dev;
          printf("IDE: %s %s at channel %u drive %u",
                 dev.type == IDE_DEVICE_ATAPI ? "ATAPI" : "ATA",
                 dev.model[0] ? dev.model : "(unknown)",
                 (unsigned int)channel, (unsigned int)drive);
          if (dev.blocks != 0)
            printf(" (%lu blocks)", (unsigned long)dev.blocks);
          printf("\r\n");
        }
    }
}

void
ide_setup(void)
{
  int bdf = ide_find_controller();
  uint16_t command;
  uint8_t control;
  uint16_t primary_io;
  uint16_t primary_control;
  uint16_t secondary_io;
  uint16_t secondary_control;

  ide_devices_found = 0;
  memset(ide_devices, 0, sizeof(ide_devices));

  if (bdf < 0)
    {
      printf("IDE: no PCI IDE controller found\r\n");
      return;
    }

  command = pci_config_readw(bdf, PCI_COMMAND);
  command |= PCI_COMMAND_IO | PCI_COMMAND_MASTER;
  pci_config_writew(bdf, PCI_COMMAND, command);

  /*
   * QEMU creates both CMD646 channels, but the secondary channel starts
   * disabled unless the machine explicitly requests it.  Firmware is the
   * correct place to enable both channels before device discovery.
   */
  control = pci_config_readb(bdf, CMD646_CNTRL);
  control |= CMD646_CNTRL_EN_CH0 | CMD646_CNTRL_EN_CH1;
  pci_config_writeb(bdf, CMD646_CNTRL, control);

  primary_io = ide_io_bar(bdf, PCI_BASE_ADDRESS_0, 0x1f0);
  primary_control = ide_io_bar(bdf, PCI_BASE_ADDRESS_1, 0x3f4);
  secondary_io = ide_io_bar(bdf, PCI_BASE_ADDRESS_2, 0x170);
  secondary_control = ide_io_bar(bdf, PCI_BASE_ADDRESS_3, 0x374);

  printf("IDE: controller %04x:%04x at %02x:%02x:%x\r\n",
         (unsigned int)pci_config_readw(bdf, PCI_VENDOR_ID),
         (unsigned int)pci_config_readw(bdf, PCI_DEVICE_ID),
         PCI_BUS(bdf), PCI_SLOT(bdf), PCI_FUNC(bdf));

  ide_probe_channel(0, primary_io, primary_control);
  ide_probe_channel(1, secondary_io, secondary_control);

  if (ide_devices_found == 0)
    printf("IDE: controller present, no drives attached\r\n");
}

unsigned int
ide_device_count(void)
{
  return ide_devices_found;
}

const struct ide_device *
ide_get_device(unsigned int index)
{
  if (index >= ide_devices_found)
    return NULL;
  return &ide_devices[index];
}

static int
ide_read_ata_block(const struct ide_device *dev, uint64_t lba, void *buffer)
{
  int status;

  if (lba > 0x0fffffff)
    return -1;

  if (ide_wait(dev, 0, ATA_STATUS_BSY) < 0)
    return -1;

  ide_select(dev, true, lba >> 24);
  outb(1, dev->io_base + ATA_REG_NSECT);
  outb(lba, dev->io_base + ATA_REG_LBAL);
  outb(lba >> 8, dev->io_base + ATA_REG_LBAM);
  outb(lba >> 16, dev->io_base + ATA_REG_LBAH);
  outb(ATA_CMD_READ_SECTORS, dev->io_base + ATA_REG_COMMAND);

  status = ide_wait(dev, ATA_STATUS_DRQ,
                    ATA_STATUS_BSY | ATA_STATUS_ERR | ATA_STATUS_DF);
  if (status < 0)
    return -1;

  ide_read_words(dev, buffer, ATA_SECTOR_SIZE / 2);

  status = ide_status(dev);
  if (status & (ATA_STATUS_ERR | ATA_STATUS_DF))
    return -1;

  return 0;
}

static int
ide_read_atapi_block(const struct ide_device *dev, uint64_t lba, void *buffer)
{
  uint8_t packet[12];
  int status;
  unsigned int bytes;
  unsigned int copy_bytes;

  if (lba > 0xffffffffUL)
    return -1;

  memset(packet, 0, sizeof(packet));
  packet[0] = ATAPI_CMD_READ_10;
  packet[2] = lba >> 24;
  packet[3] = lba >> 16;
  packet[4] = lba >> 8;
  packet[5] = lba;
  packet[8] = 1;

  if (ide_wait(dev, 0, ATA_STATUS_BSY) < 0)
    return -1;

  ide_select(dev, false, 0);
  outb(0, dev->io_base + ATA_REG_FEATURES);
  outb(0, dev->io_base + ATA_REG_LBAM);
  outb(ATAPI_SECTOR_SIZE >> 8, dev->io_base + ATA_REG_LBAH);
  outb(ATA_CMD_PACKET, dev->io_base + ATA_REG_COMMAND);

  if (ide_wait(dev, ATA_STATUS_DRQ,
               ATA_STATUS_BSY | ATA_STATUS_ERR | ATA_STATUS_DF) < 0)
    return -1;

  ide_write_packet(dev, packet);

  status = ide_wait(dev, ATA_STATUS_DRQ,
                    ATA_STATUS_BSY | ATA_STATUS_ERR | ATA_STATUS_DF);
  if (status < 0)
    return -1;

  bytes = inb(dev->io_base + ATA_REG_LBAM);
  bytes |= (unsigned int)inb(dev->io_base + ATA_REG_LBAH) << 8;
  if (bytes == 0)
    bytes = 0x10000;

  copy_bytes = bytes;
  if (copy_bytes > ATAPI_SECTOR_SIZE)
    copy_bytes = ATAPI_SECTOR_SIZE;

  ide_read_words(dev, buffer, copy_bytes / 2);
  if (bytes > copy_bytes)
    ide_discard_words(dev, (bytes - copy_bytes) / 2);

  if (copy_bytes < ATAPI_SECTOR_SIZE)
    memset((uint8_t *)buffer + copy_bytes, 0,
           ATAPI_SECTOR_SIZE - copy_bytes);

  status = ide_wait(dev, 0, ATA_STATUS_BSY | ATA_STATUS_DRQ);
  if (status < 0 || (status & (ATA_STATUS_ERR | ATA_STATUS_DF)))
    return -1;

  return 0;
}

int
ide_read_blocks(unsigned int index, uint64_t lba,
                unsigned int count, void *buffer)
{
  const struct ide_device *dev;
  uint8_t *out = buffer;
  unsigned int i;

  if (index >= ide_devices_found || buffer == NULL)
    return -1;

  dev = &ide_devices[index];

  for (i = 0; i < count; ++i)
    {
      int error;

      if (dev->type == IDE_DEVICE_ATA)
        error = ide_read_ata_block(dev, lba + i, out);
      else if (dev->type == IDE_DEVICE_ATAPI)
        error = ide_read_atapi_block(dev, lba + i, out);
      else
        return -1;

      if (error)
        return error;

      out += dev->block_size;
    }

  return 0;
}
