/* 
 * Windows User Space AppleSMC Driver
 * Copyright (C) 2015 Yudi Xie <xieyudi1990@gmail.com>
 *
 * Based on InpOutx64
 * an open source windows DLL and Driver to give direct access to hardware ports
 * Copyright Logix4U & Phillip Gibbons [Highresolution Enterprises]
 *
 * Based on applesmc.c Linux kernel driver
 * 
 * drivers/hwmon/applesmc.c - driver for Apple's SMC (accelerometer, temperature
 * sensors, fan control, keyboard backlight control) used in Intel-based Apple
 * computers.
 *
 * Copyright (C) 2007 Nicolas Boichat <nicolas@boichat.ch>
 * Copyright (C) 2010 Henrik Rydberg <rydberg@euromail.se>
 *
 * Based on hdaps.c driver:
 * Copyright (C) 2005 Robert Love <rml@novell.com>
 * Copyright (C) 2005 Jesper Juhl <jj@chaosbits.net>
 *
 * Fan control based on smcFanControl:
 * Copyright (C) 2006 Hendrik Holtmann <holtmann@mac.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#include <stdio.h>
#include <Windows.h>
#include "inpout32.h"
#include "applesmc-win32.h"

#undef DEBUG
#ifdef DEBUG
#define pr_warn                 printf
#define pr_info					printf
#else
#define pr_warn                 
#define pr_info					
#endif
#define inb(addr)               (u8)(Inp32((short)addr))
#define outb(value, addr)       Out32((short)addr, value)
#define udelay(x)               Sleep((x>>10)+1)
#define msleep                  Sleep
#define be32_to_cpu(x)          (((x&0x000000ff)<<24) | ((x&0x0000ff00)<<8) | ((x&0x00ff0000)>>8) | ((x&0xff000000)>>24))
#define cpu_to_be32				be32_to_cpu
#define ERR_PTR(x)              ((void *)x)
#define IS_ERR(x)               ((unsigned long)x >= (unsigned long)-4095)
#define PTR_ERR(x)				((long)x)
#define kcalloc(n, size, flags) calloc(n, size)
#define kfree(objp)             free(objp)

#define EIO             5       /* I/O error */
#define ENOMEM          12      /* Out of memory */
#define EINVAL          22      /* Invalid argument */

/* data port used by Apple SMC */
#define APPLESMC_DATA_PORT	0x300
/* command/status port used by Apple SMC */
#define APPLESMC_CMD_PORT	0x304

#define APPLESMC_NR_PORTS	32 /* 0x300-0x31f */

#define APPLESMC_MAX_DATA_LENGTH 32

/* all delays are adapted to millisecond granularity */
#define APPLESMC_MIN_WAIT	0x0010
#define APPLESMC_RETRY_WAIT	0x0100
#define APPLESMC_MAX_WAIT	0x400000

#define APPLESMC_READ_CMD	0x10
#define APPLESMC_WRITE_CMD	0x11
#define APPLESMC_GET_KEY_BY_INDEX_CMD	0x12
#define APPLESMC_GET_KEY_TYPE_CMD	0x13

#define KEY_COUNT_KEY		"#KEY" /* r-o ui32 */

#define LIGHT_SENSOR_LEFT_KEY	"ALV0" /* r-o {alv (6-10 bytes) */
#define LIGHT_SENSOR_RIGHT_KEY	"ALV1" /* r-o {alv (6-10 bytes) */
#define BACKLIGHT_KEY		"LKSB" /* w-o {lkb (2 bytes) */

#define CLAMSHELL_KEY		"MSLD" /* r-o ui8 (unused) */

#define MOTION_SENSOR_X_KEY	"MO_X" /* r-o sp78 (2 bytes) */
#define MOTION_SENSOR_Y_KEY	"MO_Y" /* r-o sp78 (2 bytes) */
#define MOTION_SENSOR_Z_KEY	"MO_Z" /* r-o sp78 (2 bytes) */
#define MOTION_SENSOR_KEY	"MOCN" /* r/w ui16 */

#define FANS_COUNT		"FNum" /* r-o ui8 */
#define FANS_MANUAL		"FS! " /* r-w ui16 */
#define FAN_ID_FMT		"F%dID" /* r-o char[16] */

#define TEMP_SENSOR_TYPE	"sp78"

/* List of keys used to read/write fan speeds */
static const char *const fan_speed_fmt[] = {
	"F%dAc",		/* actual speed */
	"F%dMn",		/* minimum speed (rw) */
	"F%dMx",		/* maximum speed */
	"F%dSf",		/* safe speed - not all models */
	"F%dTg",		/* target speed (manual: rw) */
};

#define INIT_TIMEOUT_MSECS	5000	/* wait up to 5s for device init ... */
#define INIT_WAIT_MSECS		50	/* ... in 50ms increments */

#define APPLESMC_POLL_INTERVAL	50	/* msecs */
#define APPLESMC_INPUT_FUZZ	4	/* input event threshold */
#define APPLESMC_INPUT_FLAT	4

/* Register lookup and registers common to all SMCs */
struct applesmc_registers smcreg = {0};

static int wait_read(void)
{
	u8 status;
	int us;
	for (us = APPLESMC_MIN_WAIT; us < APPLESMC_MAX_WAIT; us <<= 1) {
		udelay(us);
		status = inb(APPLESMC_CMD_PORT);
		/* read: wait for smc to settle */
		if (status & 0x01)
			return 0;
	}

	pr_warn("wait_read() fail: 0x%02x\n", status);
	return -EIO;
}

/*
 * send_byte - Write to SMC port, retrying when necessary. Callers
 * must hold applesmc_lock.
 */
static int send_byte(u8 cmd, u16 port)
{
	u8 status;
	int us;

	outb(cmd, port);
	for (us = APPLESMC_MIN_WAIT; us < APPLESMC_MAX_WAIT; us <<= 1) {
		udelay(us);
		status = inb(APPLESMC_CMD_PORT);
		/* write: wait for smc to settle */
		if (status & 0x02)
			continue;
		/* ready: cmd accepted, return */
		if (status & 0x04)
			return 0;
		/* timeout: give up */
		if (us << 1 == APPLESMC_MAX_WAIT)
			break;
		/* busy: long wait and resend */
		udelay(APPLESMC_RETRY_WAIT);
		outb(cmd, port);
	}

	pr_warn("send_byte(0x%02x, 0x%04x) fail: 0x%02x\n", cmd, port, status);
	return -EIO;
}

static int send_command(u8 cmd)
{
	return send_byte(cmd, APPLESMC_CMD_PORT);
}

static int send_argument(const char *key)
{
	int i;

	for (i = 0; i < 4; i++)
		if (send_byte(key[i], APPLESMC_DATA_PORT))
			return -EIO;
	return 0;
}

static int read_smc(u8 cmd, const char *key, u8 *buffer, u8 len)
{
	u8 status, data = 0;
	int i;

	if (send_command(cmd) || send_argument(key)) {
		pr_warn("%.4s: read arg fail\n", key);
		return -EIO;
	}

	/* This has no effect on newer (2012) SMCs */
	if (send_byte(len, APPLESMC_DATA_PORT)) {
		pr_warn("%.4s: read len fail\n", key);
		return -EIO;
	}

	for (i = 0; i < len; i++) {
		if (wait_read()) {
			pr_warn("%.4s: read data[%d] fail\n", key, i);
			return -EIO;
		}
		buffer[i] = inb(APPLESMC_DATA_PORT);
	}

	/* Read the data port until bit0 is cleared */
	for (i = 0; i < 16; i++) {
		udelay(APPLESMC_MIN_WAIT);
		status = inb(APPLESMC_CMD_PORT);
		if (!(status & 0x01))
			break;
		data = inb(APPLESMC_DATA_PORT);
	}
	if (i)
		pr_warn("flushed %d bytes, last value is: %d\n", i, data);

	return 0;
}

static int write_smc(u8 cmd, const char *key, const u8 *buffer, u8 len)
{
	int i;

	if (send_command(cmd) || send_argument(key)) {
		pr_warn("%s: write arg fail\n", key);
		return -EIO;
	}

	if (send_byte(len, APPLESMC_DATA_PORT)) {
		pr_warn("%.4s: write len fail\n", key);
		return -EIO;
	}

	for (i = 0; i < len; i++) {
		if (send_byte(buffer[i], APPLESMC_DATA_PORT)) {
			pr_warn("%s: write data fail\n", key);
			return -EIO;
		}
	}

	return 0;
}

static int read_register_count(unsigned int *count)
{
	__be32 be;
	int ret;

	ret = read_smc(APPLESMC_READ_CMD, KEY_COUNT_KEY, (u8 *)&be, 4);
	if (ret)
		return ret;

	*count = be32_to_cpu(be);
	return 0;
}

/*
 * Serialized I/O
 *
 * Returns zero on success or a negative error on failure.
 * All functions below are concurrency safe - callers should NOT hold lock.
 */

static int applesmc_read_entry(const struct applesmc_entry *entry,
			       u8 *buf, u8 len)
{
	int ret;

	if (entry->len != len)
		return -EINVAL;
	ret = read_smc(APPLESMC_READ_CMD, entry->key, buf, len);

	return ret;
}

static int applesmc_write_entry(const struct applesmc_entry *entry,
				const u8 *buf, u8 len)
{
	int ret;

	if (entry->len != len)
		return -EINVAL;
	ret = write_smc(APPLESMC_WRITE_CMD, entry->key, buf, len);
	return ret;
}

static const struct applesmc_entry *applesmc_get_entry_by_index(int index)
{
	struct applesmc_entry *cache = &smcreg.cache[index];
	u8 key[4], info[6];
	__be32 be;
	int ret = 0;

	if (cache->valid)
		goto out;
    
	be = cpu_to_be32(index);
	ret = read_smc(APPLESMC_GET_KEY_BY_INDEX_CMD, (u8 *)&be, key, 4);
	if (ret)
		goto out;
	ret = read_smc(APPLESMC_GET_KEY_TYPE_CMD, key, info, 6);
	if (ret)
		goto out;

	memcpy(cache->key, key, 4);
	cache->len = info[0];
	memcpy(cache->type, &info[1], 4);
	cache->flags = info[5];
	cache->valid = 1;

out:
	if (ret)
		return ERR_PTR(ret);
	return cache;
}

static int applesmc_get_lower_bound(unsigned int *lo, const char *key)
{
	int begin = 0, end = smcreg.key_count;
	const struct applesmc_entry *entry;

	while (begin != end) {
		int middle = begin + (end - begin) / 2;
		entry = applesmc_get_entry_by_index(middle);
		if (IS_ERR(entry)) {
			*lo = 0;
			return PTR_ERR(entry);
		}
		if (strcmp(entry->key, key) < 0)
			begin = middle + 1;
		else
			end = middle;
	}

	*lo = begin;
	return 0;
}

static int applesmc_get_upper_bound(unsigned int *hi, const char *key)
{
	int begin = 0, end = smcreg.key_count;
	const struct applesmc_entry *entry;

	while (begin != end) {
		int middle = begin + (end - begin) / 2;
		entry = applesmc_get_entry_by_index(middle);
		if (IS_ERR(entry)) {
			*hi = smcreg.key_count;
			return PTR_ERR(entry);
		}
		if (strcmp(key, entry->key) < 0)
			end = middle;
		else
			begin = middle + 1;
	}

	*hi = begin;
	return 0;
}

static const struct applesmc_entry *applesmc_get_entry_by_key(const char *key)
{
	int begin, end;
	int ret;

	ret = applesmc_get_lower_bound(&begin, key);
	if (ret)
		return ERR_PTR(ret);
	ret = applesmc_get_upper_bound(&end, key);
	if (ret)
		return ERR_PTR(ret);
	if (end - begin != 1)
		return ERR_PTR(-EINVAL);

	return applesmc_get_entry_by_index(begin);
}

static int applesmc_read_key(const char *key, u8 *buffer, u8 len)
{
	const struct applesmc_entry *entry;

	entry = applesmc_get_entry_by_key(key);
	if (IS_ERR(entry))
		return PTR_ERR(entry);

	return applesmc_read_entry(entry, buffer, len);
}

static int applesmc_write_key(const char *key, const u8 *buffer, u8 len)
{
	const struct applesmc_entry *entry;

	entry = applesmc_get_entry_by_key(key);
	if (IS_ERR(entry))
		return PTR_ERR(entry);

	return applesmc_write_entry(entry, buffer, len);
}

static int applesmc_has_key(const char *key, bool *value)
{
	const struct applesmc_entry *entry;

	entry = applesmc_get_entry_by_key(key);
	if (IS_ERR(entry) && PTR_ERR(entry) != -EINVAL)
		return PTR_ERR(entry);

	*value = !IS_ERR(entry);
	return 0;
}

/* applesmc_read_s16 - Read 16-bit signed big endian register
 */
static int applesmc_read_s16(const char *key, s16 *value)
{
        u8 buffer[2];
        int ret;

        ret = applesmc_read_key(key, buffer, 2);
        if (ret)
                return ret;

        *value = ((s16)buffer[0] << 8) | buffer[1];
        return 0;
}

static int applesmc_init_index(struct applesmc_registers *s)
{
	const struct applesmc_entry *entry;
	unsigned int i;

	if (s->index)
		return 0;

	s->index = kcalloc(s->temp_count, sizeof(s->index[0]), GFP_KERNEL);
	if (!s->index)
		return -ENOMEM;

	for (i = s->temp_begin; i < s->temp_end; i++) {
		entry = applesmc_get_entry_by_index(i);
		if (IS_ERR(entry))
			continue;
		if (strcmp(entry->type, TEMP_SENSOR_TYPE))
			continue;
		s->index[s->index_count++] = entry->key;
	}

	return 0;
}

/*
 * applesmc_init_smcreg_try - Try to initialize register cache. Idempotent.
 */
static int applesmc_init_smcreg_try(void)
{
	struct applesmc_registers *s = &smcreg;
	unsigned int count;
	u8 tmp[1];
	int ret;

	if (s->init_complete)
		return 0;

	ret = read_register_count(&count);
	if (ret)
		return ret;

	if (s->cache && s->key_count != count) {
		pr_warn("key count changed from %d to %d\n",
			s->key_count, count);
		kfree(s->cache);
		s->cache = NULL;
	}
	s->key_count = count;

	if (!s->cache)
		s->cache = kcalloc(s->key_count, sizeof(*s->cache), GFP_KERNEL);
	if (!s->cache)
		return -ENOMEM;

	ret = applesmc_read_key(FANS_COUNT, tmp, 1);
	if (ret)
		return ret;
	s->fan_count = tmp[0];

	ret = applesmc_get_lower_bound(&s->temp_begin, "T");
	if (ret)
		return ret;
	ret = applesmc_get_lower_bound(&s->temp_end, "U");
	if (ret)
		return ret;
	s->temp_count = s->temp_end - s->temp_begin;

	ret = applesmc_init_index(s);
	if (ret)
		return ret;

	s->init_complete = true;

	pr_info("key=%d fan=%d temp=%d index=%d\n",
	       s->key_count, s->fan_count, s->temp_count, s->index_count);

	return 0;
}


/* 
 * global procedure 
 */

void applesmc_destroy_smcreg(void)
{
	kfree(smcreg.index);
	smcreg.index = NULL;
	kfree(smcreg.cache);
	smcreg.cache = NULL;
	smcreg.init_complete = false;
}

/*
 * applesmc_init_smcreg - Initialize register cache.
 *
 * Retries until initialization is successful, or the operation times out.
 *
 */
int applesmc_init_smcreg(void)
{
	int ms, ret;

	for (ms = 0; ms < INIT_TIMEOUT_MSECS; ms += INIT_WAIT_MSECS) {
		ret = applesmc_init_smcreg_try();
		if (!ret) {
			pr_info("init_smcreg() took %d ms\n", ms);
			return 0;
		}
		msleep(INIT_WAIT_MSECS);
	}

	applesmc_destroy_smcreg();

	return ret;
}

/* Displays sensor key as label */
int applesmc_show_sensor_label(int index, char *buf)
{
	const char *key = smcreg.index[index];

	return snprintf(buf, 16, "%s", key);
}

/* Displays degree Celsius * 1000 */
int applesmc_show_temperature(int index, char *buf)
{
	const char *key = smcreg.index[index];
	int ret;
	s16 value;
	int temp;

	ret = applesmc_read_s16(key, &value);
	if (ret)
		return ret;

	temp = 250 * (value >> 6);
	
	if(buf)
		snprintf(buf, 16, "%d", temp);
	
	return temp;
}

int applesmc_show_fan_speed(int index, int option, char *buf)
{
	int ret;
	unsigned int speed = 0;
	char newkey[5];
	u8 buffer[2];

	sprintf(newkey, fan_speed_fmt[option], index);

	ret = applesmc_read_key(newkey, buffer, 2);
	speed = ((buffer[0] << 8 | buffer[1]) >> 2);

	if (ret)
		return ret;
	else
	{
		if(buf)
			snprintf(buf, 16, "%u", speed);
		return (int)speed;
	}
}

int applesmc_store_fan_speed(int index, unsigned long speed)
{
        int ret;
        char newkey[5];
        u8 buffer[2];

        if (speed >= 0x4000)
            return -EINVAL;         /* Bigger than a 14-bit value */

		/* "F%dTg", target speed (manual: rw) */
        sprintf(newkey, fan_speed_fmt[4], index);

        buffer[0] = (speed >> 6) & 0xff;
        buffer[1] = (speed << 2) & 0xff;
        ret = applesmc_write_key(newkey, buffer, 2);
		
        if (ret)
			return ret;
        else
			return 0;
}

//static ssize_t applesmc_show_fan_manual(struct device *dev,
//			struct device_attribute *attr, char *sysfsbuf)
//{
//	int ret;
//	u16 manual = 0;
//	u8 buffer[2];
//
//	ret = applesmc_read_key(FANS_MANUAL, buffer, 2);
//	manual = ((buffer[0] << 8 | buffer[1]) >> to_index(attr)) & 0x01;
//
//	if (ret)
//		return ret;
//	else
//		return snprintf(sysfsbuf, PAGE_SIZE, "%d\n", manual);
//}
//
int applesmc_store_fan_manual(int index, int manual)
{
	int ret;
	u8 buffer[2];
	u16 val;

	ret = applesmc_read_key(FANS_MANUAL, buffer, 2);
	val = (buffer[0] << 8 | buffer[1]);
	if (ret)
		goto out;

	if (manual)
		val = val | (0x01 << index);
	else
		val = val & ~(0x01 << index);

	buffer[0] = (val >> 8) & 0xFF;
	buffer[1] = val & 0xFF;

	ret = applesmc_write_key(FANS_MANUAL, buffer, 2);

out:
	if (ret)
		return ret;
	else
		return 0;
}

//static ssize_t applesmc_show_fan_position(struct device *dev,
//				struct device_attribute *attr, char *sysfsbuf)
//{
//	int ret;
//	char newkey[5];
//	u8 buffer[17];
//
//	sprintf(newkey, FAN_ID_FMT, to_index(attr));
//
//	ret = applesmc_read_key(newkey, buffer, 16);
//	buffer[16] = 0;
//
//	if (ret)
//		return ret;
//	else
//		return snprintf(sysfsbuf, PAGE_SIZE, "%s\n", buffer+4);
//}
//
//static ssize_t applesmc_calibrate_show(struct device *dev,
//				struct device_attribute *attr, char *sysfsbuf)
//{
//	return snprintf(sysfsbuf, PAGE_SIZE, "(%d,%d)\n", rest_x, rest_y);
//}
//
//static ssize_t applesmc_calibrate_store(struct device *dev,
//	struct device_attribute *attr, const char *sysfsbuf, size_t count)
//{
//	applesmc_calibrate();
//
//	return count;
//}
//
//static void applesmc_backlight_set(struct work_struct *work)
//{
//	applesmc_write_key(BACKLIGHT_KEY, backlight_state, 2);
//}
//static DECLARE_WORK(backlight_work, &applesmc_backlight_set);
//
//static void applesmc_brightness_set(struct led_classdev *led_cdev,
//						enum led_brightness value)
//{
//	int ret;
//
//	backlight_state[0] = value;
//	ret = queue_work(applesmc_led_wq, &backlight_work);
//
//	if (debug && (!ret))
//		dev_dbg(led_cdev->dev, "work was already on the queue.\n");
//}
//
//static ssize_t applesmc_key_count_show(struct device *dev,
//				struct device_attribute *attr, char *sysfsbuf)
//{
//	int ret;
//	u8 buffer[4];
//	u32 count;
//
//	ret = applesmc_read_key(KEY_COUNT_KEY, buffer, 4);
//	count = ((u32)buffer[0]<<24) + ((u32)buffer[1]<<16) +
//						((u32)buffer[2]<<8) + buffer[3];
//
//	if (ret)
//		return ret;
//	else
//		return snprintf(sysfsbuf, PAGE_SIZE, "%d\n", count);
//}
//
//static ssize_t applesmc_key_at_index_read_show(struct device *dev,
//				struct device_attribute *attr, char *sysfsbuf)
//{
//	const struct applesmc_entry *entry;
//	int ret;
//
//	entry = applesmc_get_entry_by_index(key_at_index);
//	if (IS_ERR(entry))
//		return PTR_ERR(entry);
//	ret = applesmc_read_entry(entry, sysfsbuf, entry->len);
//	if (ret)
//		return ret;
//
//	return entry->len;
//}
//
//static ssize_t applesmc_key_at_index_data_length_show(struct device *dev,
//				struct device_attribute *attr, char *sysfsbuf)
//{
//	const struct applesmc_entry *entry;
//
//	entry = applesmc_get_entry_by_index(key_at_index);
//	if (IS_ERR(entry))
//		return PTR_ERR(entry);
//
//	return snprintf(sysfsbuf, PAGE_SIZE, "%d\n", entry->len);
//}
//
//static ssize_t applesmc_key_at_index_type_show(struct device *dev,
//				struct device_attribute *attr, char *sysfsbuf)
//{
//	const struct applesmc_entry *entry;
//
//	entry = applesmc_get_entry_by_index(key_at_index);
//	if (IS_ERR(entry))
//		return PTR_ERR(entry);
//
//	return snprintf(sysfsbuf, PAGE_SIZE, "%s\n", entry->type);
//}
//
//static ssize_t applesmc_key_at_index_name_show(struct device *dev,
//				struct device_attribute *attr, char *sysfsbuf)
//{
//	const struct applesmc_entry *entry;
//
//	entry = applesmc_get_entry_by_index(key_at_index);
//	if (IS_ERR(entry))
//		return PTR_ERR(entry);
//
//	return snprintf(sysfsbuf, PAGE_SIZE, "%s\n", entry->key);
//}
//
//static ssize_t applesmc_key_at_index_show(struct device *dev,
//				struct device_attribute *attr, char *sysfsbuf)
//{
//	return snprintf(sysfsbuf, PAGE_SIZE, "%d\n", key_at_index);
//}
//
//static ssize_t applesmc_key_at_index_store(struct device *dev,
//	struct device_attribute *attr, const char *sysfsbuf, size_t count)
//{
//	unsigned long newkey;
//
//	if (kstrtoul(sysfsbuf, 10, &newkey) < 0
//	    || newkey >= smcreg.key_count)
//		return -EINVAL;
//
//	key_at_index = newkey;
//	return count;
//}
//
//static struct led_classdev applesmc_backlight = {
//	.name			= "smc::kbd_backlight",
//	.default_trigger	= "nand-disk",
//	.brightness_set		= applesmc_brightness_set,
//};
//
//static struct applesmc_node_group info_group[] = {
//	{ "name", applesmc_name_show },
//	{ "key_count", applesmc_key_count_show },
//	{ "key_at_index", applesmc_key_at_index_show, applesmc_key_at_index_store },
//	{ "key_at_index_name", applesmc_key_at_index_name_show },
//	{ "key_at_index_type", applesmc_key_at_index_type_show },
//	{ "key_at_index_data_length", applesmc_key_at_index_data_length_show },
//	{ "key_at_index_data", applesmc_key_at_index_read_show },
//	{ }
//};
//
//static struct applesmc_node_group accelerometer_group[] = {
//	{ "position", applesmc_position_show },
//	{ "calibrate", applesmc_calibrate_show, applesmc_calibrate_store },
//	{ }
//};
//
//static struct applesmc_node_group light_sensor_group[] = {
//	{ "light", applesmc_light_show },
//	{ }
//};
//
//static struct applesmc_node_group fan_group[] = {
//	{ "fan%d_label", applesmc_show_fan_position },
//	{ "fan%d_input", applesmc_show_fan_speed, NULL, 0 },
//	{ "fan%d_min", applesmc_show_fan_speed, applesmc_store_fan_speed, 1 },
//	{ "fan%d_max", applesmc_show_fan_speed, NULL, 2 },
//	{ "fan%d_safe", applesmc_show_fan_speed, NULL, 3 },
//	{ "fan%d_output", applesmc_show_fan_speed, applesmc_store_fan_speed, 4 },
//	{ "fan%d_manual", applesmc_show_fan_manual, applesmc_store_fan_manual },
//	{ }
//};
//
//static struct applesmc_node_group temp_group[] = {
//	{ "temp%d_label", applesmc_show_sensor_label },
//	{ "temp%d_input", applesmc_show_temperature },
//	{ }
//};
//


