#include "memory_access.h"

FATFS *fs;
char *current_dir;

int _is_full_path(char *path) {
    if (path[0] == '/') {
        return 1;
    }
    return 0;
}

char *_get_full_path(char *path) {
    char *new_path;
    if (_is_full_path(path)) {
        new_path = malloc(sizeof(char) * strlen(path) + 2);
        sprintf(new_path, "%s/", path);
    } else {
        new_path = malloc(sizeof(char) * (strlen(current_dir) + strlen(path) + 2));
        sprintf(new_path, "%s%s/", current_dir, path);
    }
    return new_path;
}

int mount_usb() {
    fs = malloc(sizeof(FATFS));

    if (f_mount(fs, "", 1) != FR_OK) {
        xprintf("usb mount error\n");
        free(fs);
        return -1;
    }

    current_dir = malloc(sizeof(char) * 2);
    strcpy(current_dir, "/");

    xprintf("mounted USB\n");
    return 1;
}

int unmount_usb() {
    if (f_mount(0, "", 0) != FR_OK) {
        xprintf("usb unmount error\n");
        return -1;
    }
    free(current_dir);
    free(fs);

    xprintf("unmounted USB\n");
    return 1;
}

char *get_current_directory() {
    return current_dir;
}

char *change_directory(char *path) {
//    char *full_path = _get_full_path(path);

    DIR dir;
    if (f_opendir(&dir, path) != FR_OK) {
        xprintf("cannot change directory: %s\n", path);
//        free(path);
        return NULL;
    }

    f_closedir(&dir);
//    free(current_dir);
    current_dir = path;
    xprintf("%s\n",current_dir);
    return current_dir;
}

char *list_directory() {
    DIR dir;
    if (f_opendir(&dir, current_dir) != FR_OK) {
        xprintf("cannot open directory %s\n", current_dir);
        return NULL;
    }
    char *list = malloc(sizeof(char) * 10);
    list[0] = 0;
    FILINFO finfo;
    while (1) {
        if (f_readdir(&dir, &finfo) != FR_OK || finfo.fname[0] == 0)
            break;
        strcat(list, finfo.fname);
        strcat(list, "\n");
    }
    f_closedir(&dir);
    return list;
}

FIL *open_file(char *filename) {
    char *path;
    sprintf(path, current_dir, filename);
    FIL *file = malloc(sizeof(FIL));

    if (f_open(file, path, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
        xprintf("%s\n", path);
        xprintf("cannot open file %s\n", path);
        free(path);
        free(file);
        return NULL;
    }
    free(path);
    return file;
}

FIL *create_file(char *filename) {
    char *path = _get_full_path(filename);
    FIL *file = malloc(sizeof(FIL));

    if (f_open(file, path + 1, FA_CREATE_NEW | FA_WRITE) != FR_OK) {
        xprintf("cannot create file %s\n", path);
        free(path);
        free(file);
        return NULL;
    }
    free(path);
    return file;
}

void close_file(FIL *file) {
    f_close(file);
    free(file);
}

uint16_t write_to_file(FIL *file, char *buf, uint16_t size) {
    uint16_t bw;
    if (f_write(file, buf, size, &bw) != FR_OK) {
        xprintf("write to file error\n");
        return -1;
    }
    return bw;
}

uint16_t read_file(FIL *file, char *buf, uint16_t size) {
    uint16_t br;
    if (f_read(file, buf, size, &br) != FR_OK) {
        xprintf("read file error\n");
        return -1;
    }
    return br;
}

int delete_file(char *filename) {


    if (f_unlink(filename) != FR_OK) {
        xprintf("cannot delete file: %s\n", filename);
        free(filename);
        return -1;
    }

    xprintf("deleted file: %s\n", filename);
    free(filename);
    return 1;
}

void USB_Process(ApplicationTypeDef Appli_state) {
    switch (Appli_state) {
        case APPLICATION_START:
            xprintf("Device connected.\n");
            break;
        case APPLICATION_READY:
            mount_usb();
            xprintf("Device ready.\n");
            break;
        case APPLICATION_DISCONNECT:
            unmount_usb();
            xprintf("Device disconnected.\n");
            break;
        default:
            break;
    }
}
