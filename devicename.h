#ifndef DEVICENAME_H
#define DEVICENAME_H

constexpr unsigned MAX_NAME_LENGTH = 8;

struct __attribute__((packed)) DeviceName {
    DeviceName();

    bool verify() const;

    char value[MAX_NAME_LENGTH + 1];
};

static_assert(sizeof(DeviceName) == sizeof(char) * 9, "Data is not packed");

#endif /* DEVICENAME_H */
