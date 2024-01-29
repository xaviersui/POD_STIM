/*
 * flash.h
 *
 *  Created on: 30 nov. 2023
 *      Author: ebt
 */

#ifndef PLM_FLASH_H_
#define PLM_FLASH_H_
#include <stdint.h>

typedef struct
{
    uint16_t offSetBio[2];
    uint16_t versionId_t[6];
}flash_user_data_t;


void flash_init(void);
void flash_read_user_data(flash_user_data_t *flashData);
void flash_write_data(flash_user_data_t flashData); // Store new value

#endif /* PLM_FLASH_H_ */
