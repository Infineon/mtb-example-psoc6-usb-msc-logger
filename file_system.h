/*****************************************************************************
* File Name: file_system.h
*
* Description:
*  This file contains the function prototypes and constants used in
*  the file_system.c.
*
* Note:
*
******************************************************************************
* Copyright (2020), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#if !defined(FILE_SYSTEM_H)
#define FILE_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>
#include "cy_device_headers.h"
#include "config.h"

#define FILE_SYSTEM_ROOT_DIRECTORIES_ENTRIES    16
#define FILE_SYSTEM_SECTORS_PER_FAT             4
#define FILE_SYSTEM_SECTOR_SIZE                 512
#define FILE_SYSTEM_SECTORS_PER_CLUSTER         (CONFIG_LOG_FILE_SIZE/FILE_SYSTEM_SECTOR_SIZE)
#define FILE_SYSTEM_DATASIZE                    (FILE_SYSTEM_SECTOR_SIZE*FILE_SYSTEM_SECTORS_PER_CLUSTER)
#define FILE_SYSTEM_SECTORS_PER_FS              (7+FILE_SYSTEM_SECTORS_PER_CLUSTER)
#define FILE_SYSTEM_SECTORS_PER_TRACK           0x12
#define FILE_SYSTEM_SIGNATURE                   0xAA55
#define FILE_SYSTEM_JUMP_BOOTSTRAP_SIZE         3
#define FILE_SYSTEM_OEM_SIZE                    8
#define FILE_SYSTEM_BOOTSTRAP_SIZE              480
#define FILE_SYSTEM_FILENAME_SIZE               11
#define FILE_SYSTEM_RESERVED_SIZE               10

typedef struct
{
    uint8_t  jump_bootstrap[FILE_SYSTEM_JUMP_BOOTSTRAP_SIZE];
    char     oem[FILE_SYSTEM_OEM_SIZE];
    uint16_t bytes_per_sector;
    uint8_t  sectors_per_cluster;
    uint16_t reserved_sectors;
    uint8_t  fat_copies;
    uint16_t root_entries;
    uint16_t sectors_per_fs;
    uint8_t  media_type;
    uint16_t sectors_per_fat;
    uint16_t sectors_per_track;
    uint16_t heads;
    uint16_t hidden_sectors;
    uint8_t  bootstrap[FILE_SYSTEM_BOOTSTRAP_SIZE];
    uint16_t signature;
} __PACKED boot_sector_t;

typedef struct
{
    char     filename[FILE_SYSTEM_FILENAME_SIZE];
    uint8_t  attribute;
    uint8_t  reserved[FILE_SYSTEM_RESERVED_SIZE];
    uint16_t time;
    uint16_t date;
    uint16_t cluster;
    uint32_t filesize;
} __PACKED directory_entry_t;

typedef struct
{
    uint8_t content[FILE_SYSTEM_SECTOR_SIZE*FILE_SYSTEM_SECTORS_PER_CLUSTER];
} cluster_t;

typedef struct
{
    boot_sector_t boot;
    uint8_t fat_area[FILE_SYSTEM_SECTOR_SIZE*FILE_SYSTEM_SECTORS_PER_FAT];
    directory_entry_t entries[FILE_SYSTEM_ROOT_DIRECTORIES_ENTRIES];
    cluster_t clusters[1];
} __PACKED fat_fs_t;

void file_system_init(uint8_t **mem);
bool file_system_write(const char *buf, uint32_t len);
void file_system_erase_all(void);

#endif /* FILE_SYSTEM_H */
