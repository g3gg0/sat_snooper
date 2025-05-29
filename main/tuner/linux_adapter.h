#pragma once

#include <stdint.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Define dummy macros for Linux kernel module parameters */
/* module_param(name, type, perm) */
/* For a dummy implementation, we can make it a no-op, */
/* assuming the variable \'name\' is declared elsewhere or globally. */
#define module_param(name, type, perm) /* (name) */

/* MODULE_PARM_DESC(parm, desc) */
/* This is purely for documentation in the kernel. */
#define MODULE_PARM_DESC(parm, desc) /* (parm, desc) */

#define MODULE_DESCRIPTION(parm)
#define MODULE_AUTHOR(parm)
#define MODULE_LICENSE(parm)

#define EXPORT_SYMBOL_GPL(parm)

#define IS_REACHABLE(x) 1

#define KERN_INFO ""
#define KERN_WARNING ""

void printk(const char *fmt, ...);

typedef uint64_t __u64;
typedef uint32_t __u32;
typedef uint16_t __u16;
typedef uint8_t __u8;

typedef int64_t __s64;
typedef int32_t __s32;
typedef int16_t __s16;
typedef int8_t __s8;

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;
typedef uint32_t ktime_t;

struct i2c_msg
{
    u8 addr;
    u8 flags;
    u8 *buf;
    u8 len;
};

struct i2c_adapter
{
    struct
    {
        void *parent;
    } dev;
};

struct kref
{
    u8 ref;
};

#define I2C_M_RD 1

#define kmalloc(size, unused) calloc(1, size)
#define kzalloc(size, unused) calloc(1, size)
#define kfree(ptr) free(ptr)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

//#define __func__ ""

#define EOPNOTSUPP -1
#define ETIMEDOUT -1
#define EINVAL -1
#define ENOMEM -1
#define ENOMEM -1
#define EREMOTEIO -1
#define GFP_KERNEL -1
#define KERN_ERR "E"

#define HZ 1000

// #define BIT(x) (1<<(x))
#define msleep(x) vTaskDelay(pdMS_TO_TICKS(x))


struct firmware
{
    size_t size;
    u8 *data;
};

int i2c_transfer(struct i2c_adapter *state, struct i2c_msg *msg, uint32_t msg_count);
int request_firmware(const struct firmware **fw, const char *name, void *parent);
int release_firmware(const struct firmware *fw);