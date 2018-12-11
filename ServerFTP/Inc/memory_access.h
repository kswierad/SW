#ifndef MEMORY_ACCESS_H_
#define MEMORY_ACCESS_H_


#include "fatfs.h"
#include "ff.h"
#include "usb_host.h"

int mount_usb();

int unmount_usb();

char *get_current_directory();

char *change_directory(char *path) ;

char *list_directory() ;

FIL *open_file(char *filename);

FIL *create_file(char *filename) ;

void close_file(FIL *file) ;

uint16_t write_to_file(FIL *file, char *buf, uint16_t size);

uint16_t read_file(FIL *file, char *buf, uint16_t size) ;

int delete_file(char *filename);

void USB_Process(ApplicationTypeDef Appli_state);

#define MAX_PATH 128
#define MAX_LIST 1024


#endif /* MEMORY_ACCESS_H_ */
