#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>

#ifdef __APPLE__
#include <machine/endian.h>
#include <libkern/OSByteOrder.h>

#define htobe16(x) OSSwapHostToBigInt16(x)
#define htole16(x) OSSwapHostToLittleInt16(x)
#define be16toh(x) OSSwapBigToHostInt16(x)
#define le16toh(x) OSSwapLittleToHostInt16(x)

#define htobe32(x) OSSwapHostToBigInt32(x)
#define htole32(x) OSSwapHostToLittleInt32(x)
#define be32toh(x) OSSwapBigToHostInt32(x)
#define le32toh(x) OSSwapLittleToHostInt32(x)

#define htobe64(x) OSSwapHostToBigInt64(x)
#define htole64(x) OSSwapHostToLittleInt64(x)
#define be64toh(x) OSSwapBigToHostInt64(x)
#define le64toh(x) OSSwapLittleToHostInt64(x)

#define __BIG_ENDIAN    BIG_ENDIAN
#define __LITTLE_ENDIAN LITTLE_ENDIAN
#define __BYTE_ORDER    BYTE_ORDER

#define B921600				921600
#else
#include <endian.h>
#endif

#define APP_NAME			"MtkFlash"
#define APP_VERSION			"0.1" // Major, minor
#define COPYRIGHT_DATE			"2023"
#define AUTHOR_NAME			"Jean-Christophe Rona"

#define DEFAULT_TTY_PATH		"/dev/ttyACM0"
#define DEFAULT_DA_PATH			"MTK_AllInOne_DA.bin"
#define DEFAULT_FILE_PATH		"tee.img"

#define TTY_SPEED			B921600

#define READ_BUF_SIZE			1024 // bytes
#define HEX_DUMP_LINE_LENGTH		32 // bytes

#define DA_OFFSET_INDEX			0x7C
#define DA_OFFSET_COUNT			0x7E
#define DA_OFFSET_HEADERS		0x80

#define DA_OFFSET_HEADER(i)		(DA_OFFSET_HEADERS + (i * 0x14))
#define DA_OFFSET_DATA_OFFSET(i)	(DA_OFFSET_HEADER(i) + 0x00)
#define DA_OFFSET_SIZE(i)		(DA_OFFSET_HEADER(i) + 0x04)
#define DA_OFFSET_START_ADDR(i)		(DA_OFFSET_HEADER(i) + 0x08)
#define DA_OFFSET_START_OFFSET(i)	(DA_OFFSET_HEADER(i) + 0x0C)
#define DA_OFFSET_SIG_LEN(i)		(DA_OFFSET_HEADER(i) + 0x10)

#define DA_SYNC				0xC0

#define DA_STATUS_OK			0x00
#define DA_STATUS_UNSUPPORTED_CTRL_CODE	0xC0010004

// TODO: Fix this (only ints)
typedef struct  {
	unsigned char start_addr[4];
	unsigned char size[4];
	unsigned int size_int;
	unsigned char sig_len[4];
	unsigned int sig_len_int;
	unsigned int data_offset;
} da_info_t;


static unsigned int log_level = 0;

static unsigned char mtk_cmd_start[] = {0xa0, 0x0a, 0x50, 0x05}; // START magic
static unsigned char mtk_cmd_start_reply[] = {0x5f, 0xf5, 0xaf, 0xfa}; // Expexted reply to START magic

static unsigned char mtk_cmd_get_version[] = {0xff};
static unsigned char mtk_cmd_bl_ver[] = {0xfe};
static unsigned char mtk_cmd_hw_sw_ver[] = {0xfc};
static unsigned char mtk_cmd_hw_code[] = {0xfd};
static unsigned char mtk_cmd_send_da[] = {0xd7};
static unsigned char mtk_cmd_jump_da[] = {0xd5};
static unsigned char mtk_cmd_get_target_config[] = {0xd8};
static unsigned char mtk_cmd_read16[] = {0xa2};
static unsigned char mtk_cmd_write16[] = {0xd2};
static unsigned char mtk_cmd_read32[] = {0xd1};
static unsigned char mtk_cmd_write32[] = {0xd4};
static unsigned char mtk_cmd_pwr_init[] = {0xc4};
static unsigned char mtk_cmd_pwr_deinit[] = {0xc5};
static unsigned char mtk_cmd_pwr_read16[] = {0xc6};
static unsigned char mtk_cmd_pwr_write16[] = {0xc7};

/* Those are commands and data understood/used by the DA (ints are little-endian) */
static unsigned char mtk_da_cmd_magic[] = {0xef, 0xee, 0xee, 0xfe};
static unsigned char mtk_da_dt_protocol_flow[] = {0x01, 0x00, 0x00, 0x00};
static unsigned char mtk_da_cmd_sync[] = {0x53, 0x59, 0x4e, 0x43};
static unsigned char mtk_da_cmd_set_env[] = {0x00, 0x01, 0x01, 0x00};
static unsigned char mtk_da_env_params[] = {0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}; // 5 ints: da_loglevel = 2, log_channel = 1, os = 1 (Linux), ufs_provision = 0, ending 1
static unsigned char mtk_da_cmd_hw_init[] = {0x01, 0x01, 0x01, 0x00};
static unsigned char mtk_da_hw_init_params[] = {0x00, 0x00, 0x00, 0x00}; // No config
static unsigned char mtk_da_cmd_write_data[] = {0x04, 0x00, 0x01, 0x00};
static unsigned char mtk_da_cmd_read_data[] = {0x05, 0x00, 0x01, 0x00};
static unsigned char mtk_da_cmd_shutdown[] = {0x07, 0x00, 0x01, 0x00};
static unsigned char mtk_da_cmd_boot_to[] = {0x08, 0x00, 0x01, 0x00};
static unsigned char mtk_da_cmd_dev_ctrl[] = {0x09, 0x00, 0x01, 0x00};

/* DEV_CTRL commands */
static unsigned char mtk_da_dev_ctrl_set_bmt_percentage[] = {0x01, 0x00, 0x02, 0x00};		//0x020001
static unsigned char mtk_da_dev_ctrl_set_battery_opt[] = {0x02, 0x00, 0x02, 0x00};		//0x020002
static unsigned char mtk_da_dev_ctrl_set_checksum_level[] = {0x03, 0x00, 0x02, 0x00};		//0x020003
static unsigned char mtk_da_dev_ctrl_set_reset_key[] = {0x04, 0x00, 0x02, 0x00};		//0x020004
static unsigned char mtk_da_dev_ctrl_set_host_info[] = {0x05, 0x00, 0x02, 0x00};		//0x020005
static unsigned char mtk_da_dev_ctrl_set_meta_boot_mode[] = {0x06, 0x00, 0x02, 0x00};		//0x020006
static unsigned char mtk_da_dev_ctrl_set_emmc_hwreset_pin[] = {0x07, 0x00, 0x02, 0x00};		//0x020007
static unsigned char mtk_da_dev_ctrl_set_generate_gpx[] = {0x08, 0x00, 0x02, 0x00};		//0x020008
static unsigned char mtk_da_dev_ctrl_set_register_value[] = {0x09, 0x00, 0x02, 0x00};		//0x020009
static unsigned char mtk_da_dev_ctrl_set_external_sig[] = {0x0a, 0x00, 0x02, 0x00};		//0x02000a
static unsigned char mtk_da_dev_ctrl_set_remote_sec_policy[] = {0x0b, 0x00, 0x02, 0x00};	//0x02000b
static unsigned char mtk_da_dev_ctrl_set_all_in_one_sig[] = {0x0c, 0x00, 0x02, 0x00};		//0x02000c
static unsigned char mtk_da_dev_ctrl_set_rsc_info[] = {0x0d, 0x00, 0x02, 0x00};			//0x02000d
static unsigned char mtk_da_dev_ctrl_set_update_fw[] = {0x10, 0x00, 0x02, 0x00};		//0x020010
static unsigned char mtk_da_dev_ctrl_set_ufs_config[] = {0x11, 0x00, 0x02, 0x00};		//0x020011

static unsigned char mtk_da_dev_ctrl_get_emmc_info[] = {0x01, 0x00, 0x04, 0x00};		//0x040001
static unsigned char mtk_da_dev_ctrl_get_nand_info[] = {0x02, 0x00, 0x04, 0x00};		//0x040002
static unsigned char mtk_da_dev_ctrl_get_nor_info[] = {0x03, 0x00, 0x04, 0x00};			//0x040003
static unsigned char mtk_da_dev_ctrl_get_ufs_info[] = {0x04, 0x00, 0x04, 0x00};			//0x040004
static unsigned char mtk_da_dev_ctrl_get_da_version[] = {0x05, 0x00, 0x04, 0x00};		//0x040005
static unsigned char mtk_da_dev_ctrl_get_expire_data[] = {0x06, 0x00, 0x04, 0x00};		//0x040006
static unsigned char mtk_da_dev_ctrl_get_packet_length[] = {0x07, 0x00, 0x04, 0x00};		//0x040007
static unsigned char mtk_da_dev_ctrl_get_random_id[] = {0x08, 0x00, 0x04, 0x00};		//0x040008
static unsigned char mtk_da_dev_ctrl_get_partition_tbl_cata[] = {0x09, 0x00, 0x04, 0x00};	//0x040009
static unsigned char mtk_da_dev_ctrl_get_connection_agent[] = {0x0a, 0x00, 0x04, 0x00};		//0x04000a
static unsigned char mtk_da_dev_ctrl_get_usb_speed[] = {0x0b, 0x00, 0x04, 0x00};		//0x04000b
static unsigned char mtk_da_dev_ctrl_get_ram_info[] = {0x0c, 0x00, 0x04, 0x00};			//0x04000c
static unsigned char mtk_da_dev_ctrl_get_chip_id[] = {0x0d, 0x00, 0x04, 0x00};			//0x04000d
static unsigned char mtk_da_dev_ctrl_get_otp_lock_status[] = {0x0e, 0x00, 0x04, 0x00};		//0x04000e
static unsigned char mtk_da_dev_ctrl_get_battery_voltage[] = {0x0f, 0x00, 0x04, 0x00};		//0x04000f
static unsigned char mtk_da_dev_ctrl_get_rpmb_status[] = {0x10, 0x00, 0x04, 0x00};		//0x040010
static unsigned char mtk_da_dev_ctrl_get_expire_date[] = {0x11, 0x00, 0x04, 0x00};		//0x040011
static unsigned char mtk_da_dev_ctrl_get_dram_type[] = {0x12, 0x00, 0x04, 0x00};		//0x040012
static unsigned char mtk_da_dev_ctrl_get_dev_fw_info[] = {0x13, 0x00, 0x04, 0x00};		//0x040013
static unsigned char mtk_da_dev_ctrl_get_hrid[] = {0x14, 0x00, 0x04, 0x00};			//0x040014
static unsigned char mtk_da_dev_ctrl_get_error_detail[] = {0x15, 0x00, 0x04, 0x00};		//0x040015


static int is_print_enabled(unsigned int level)
{
	return (level <= log_level);
}

static void dbg_printf(unsigned int level, char *buff, ...)
{
	va_list arglist;

	if ((int) level > log_level) {
		return;
	}

	va_start(arglist, buff);

	vfprintf(stdout, buff, arglist);

	va_end(arglist);
}

static int set_interface_attribs(int fd, int speed, int parity)
{
	struct termios tty;

	memset(&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0) {
		fprintf(stderr, "Cannot get attr: %d\n", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	// 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;		// disable break processing
	tty.c_iflag &= ~ICRNL;		// disable CR to NL
	tty.c_lflag = 0;		// no signaling chars, no echo,
					// no canonical processing
	tty.c_oflag = 0;		// no remapping, no delays

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);	// shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls,
						// enable reading
	tty.c_cflag &= ~CRTSCTS;		// disable flow control
	tty.c_cflag &= ~(PARENB | PARODD);	// shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~HUPCL;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "Cannot set attr: %d\n", errno);
		return -1;
	}

	/* After all the standard stuff, make sure we are in RAW mode */
	cfmakeraw(&tty);

	return 0;
}

static int set_blocking_timings(int fd)
{
	struct termios tty;

	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		fprintf(stderr, "Cannot get attr: %d\n", errno);
		return -1;
	}

	tty.c_cc[VMIN]  = 1; // Wait for at lest one char
	tty.c_cc[VTIME] = 0; // No timeout for subsequent chars

	if (tcsetattr (fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "Cannot set attr: %d\n", errno);
		return -1;
	}

	return 0;
}

static void hex_dump(unsigned char *buf, size_t size, char indicator)
{
	size_t i, j;

	dbg_printf(2, "%ld bytes %c", size, indicator);
	for (j = 0; j < size; j += HEX_DUMP_LINE_LENGTH) {
		dbg_printf(2, "\n");

		for (i = j; i < (j + HEX_DUMP_LINE_LENGTH); i++) {
			if (i < size) {
				dbg_printf(2, "%02X ", buf[i]);
			} else {
				dbg_printf(2, "   ");
			}
		}
		
		dbg_printf(2, "   ");

		for (i = j; i < (j + HEX_DUMP_LINE_LENGTH) && i < size; i++) {
			dbg_printf(2, "%c", (buf[i] >= 0x20 && buf[i] <= 0x7E) ? buf[i] : '.');
		}
	}
	dbg_printf(2, "\n\n");
}

static size_t receive_data(int fd, unsigned char *buf, size_t buf_size, size_t min_size, unsigned int timeout_sec)
{
	fd_set rfds;
	struct timeval tv;
	size_t size = 0;

	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);

	if (timeout_sec > 0) {
		tv.tv_sec = timeout_sec;
		tv.tv_usec = 0;

		if (select(fd + 1, &rfds, NULL, NULL, &tv) <= 0) {
			fprintf(stderr, "Oops: timeout reached\n");
			return 0;
		}
	}

	while (size < min_size) {
		size += read(fd, buf + size, buf_size - size);
	}

	if (is_print_enabled(2)) {
		hex_dump(buf, size, '<');
	}

	return size;
}

static size_t send_data(int fd, unsigned char *buf, size_t buf_size)
{
	if (write(fd, buf, buf_size) != buf_size) {
		fprintf(stderr, "Error: failed to send data (%s)\n", strerror(errno));
		return 0;
	}

	if (is_print_enabled(2)) {
		hex_dump(buf, buf_size, '>');
	}

	return buf_size;
}

static size_t da_receive_data(int fd, unsigned char **buf, size_t buf_size, unsigned int timeout_sec)
{
	unsigned char header_buf[12];
	size_t length;

	if (receive_data(fd, header_buf, 12, 12, timeout_sec) < 12) {
		fprintf(stderr, "Error: DA packet header received\n");
		return 0;
	}

	if (header_buf[0] != mtk_da_cmd_magic[0] || header_buf[1] != mtk_da_cmd_magic[1] || header_buf[2] != mtk_da_cmd_magic[2] || header_buf[3] != mtk_da_cmd_magic[3]) {
		fprintf(stderr, "Error: wrong magic received in DA packet header\n");
		return 0;
	}

	length = header_buf[8] | (header_buf[9] << 8) | (header_buf[10] << 16) | (header_buf[11] << 24);

	if (*buf == NULL) {
		/* Allocate the buffer */
		buf_size = length;
		*buf = (unsigned char *) malloc(buf_size);
		if (!*buf) {
			fprintf(stderr, "Error: cannot allocate memory for read buffer\n");
			return 0;
		}
	}

	if (buf_size < length) {
		fprintf(stderr, "Error: buffer too small to received the DA packet\n");
		return 0;
	}

	return receive_data(fd, *buf, length, length, timeout_sec);
}

static void da_send_data(int fd, unsigned char *buf, size_t buf_size)
{
	unsigned char size[4];
	size_t sent_size = 0;

	size[0] = buf_size & 0xFF;
	size[1] = (buf_size >> 8) & 0xFF;
	size[2] = (buf_size >> 16) & 0xFF;
	size[3] = (buf_size >> 24) & 0xFF;

	send_data(fd, mtk_da_cmd_magic, 4);
	send_data(fd, mtk_da_dt_protocol_flow, 4);
	send_data(fd, size, 4);

	while (buf_size > 0) {
		sent_size = send_data(fd, buf, (buf_size > 0x200) ? 0x200 : buf_size);
		buf_size -= sent_size;
		buf += sent_size;
	}
}

static unsigned int da_get_status(int fd, unsigned int timeout_sec)
{
	unsigned char status[4] = {0}; // Can be 2 or 4
	unsigned char *status_addr = status;
	size_t length;

	length = da_receive_data(fd, &status_addr, 4, timeout_sec);
	if (length != 2 && length != 4) {
		fprintf(stderr, "Error: wrong size for status\n");
		return -1;
	}

	return (status[0] | (status[1] << 8) | (status[2] << 16) | (status[3] << 24));
}

static size_t da_receive_data_check_status(int fd, unsigned char *buf, size_t buf_size, unsigned int timeout_sec)
{
	size_t length;
	unsigned int status;

	length = da_receive_data(fd, &buf, buf_size, timeout_sec);

	status = da_get_status(fd, timeout_sec);
	if (status != DA_STATUS_OK) {
		fprintf(stderr, "Error: wrong status received 0x%x\n", status);
		return 0;
	}

	return length;
}

static int da_send_data_check_status(int fd, unsigned char *buf, size_t buf_size, unsigned int timeout_sec)
{
	unsigned int status;

	da_send_data(fd, buf, buf_size);
	status = da_get_status(fd, timeout_sec);
	if (status != DA_STATUS_OK) {
		fprintf(stderr, "Error: wrong status received 0x%x\n", status);
		return -1;
	}
}

static int da_dev_ctrl_set(int fd, unsigned char *cmd, unsigned char *buf, size_t buf_size, unsigned int timeout_sec)
{
	if (da_send_data_check_status(fd, mtk_da_cmd_dev_ctrl, sizeof(mtk_da_cmd_dev_ctrl), timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_dev_ctrl\n");
		return -1;
	}

	if (da_send_data_check_status(fd, cmd, 4, timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for command (mtk_da_cmd_dev_ctrl)\n");
		return -1;
	}

	if (da_send_data_check_status(fd, buf, buf_size, timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for parameters (mtk_da_cmd_dev_ctrl)\n");
		return -1;
	}

	return 0;
}

static size_t da_dev_ctrl_get(int fd, unsigned char *cmd, unsigned char *buf, size_t buf_size, unsigned int timeout_sec)
{
	if (da_send_data_check_status(fd, mtk_da_cmd_dev_ctrl, sizeof(mtk_da_cmd_dev_ctrl), timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_dev_ctrl\n");
		return 0;
	}

	if (da_send_data_check_status(fd, cmd, 4, timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for DEV CTRL command (mtk_da_cmd_dev_ctrl)\n");
		return 0;
	}

	return da_receive_data_check_status(fd, buf, buf_size, timeout_sec);
}

static int da_read_flash(int fd, size_t address, size_t length, unsigned char *buf, size_t buf_size, unsigned int timeout_sec)
{
	unsigned char params[56] = {0};
	unsigned char ack[4] = {0};
	unsigned char *read_buf = NULL;
	size_t read_size;

	/* TODO: support more storage */
	/* Storage = EMMC */
	params[0] = 0x01;
	params[1] = 0x00;
	params[2] = 0x00;
	params[3] = 0x00;

	/* Parttype = EMMC_USER */
	params[4] = 0x08;
	params[5] = 0x00;
	params[6] = 0x00;
	params[7] = 0x00;

	/* Address */
	params[8] = address & 0xFF;
	params[9] = (address >> 8) & 0xFF;
	params[10] = (address >> 16) & 0xFF;
	params[11] = (address >> 24) & 0xFF;
	params[12] = (address >> 32) & 0xFF;
	params[13] = (address >> 40) & 0xFF;
	params[14] = (address >> 48) & 0xFF;
	params[15] = (address >> 56) & 0xFF;

	/* Length */
	params[16] = length & 0xFF;
	params[17] = (length >> 8) & 0xFF;
	params[18] = (length >> 16) & 0xFF;
	params[19] = (length >> 24) & 0xFF;
	params[20] = (length >> 32) & 0xFF;
	params[21] = (length >> 40) & 0xFF;
	params[22] = (length >> 48) & 0xFF;
	params[23] = (length >> 56) & 0xFF;

	if (da_send_data_check_status(fd, mtk_da_cmd_read_data, sizeof(mtk_da_cmd_read_data), timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_read_data (DA)\n");
		return -1;
	}

	if (da_send_data_check_status(fd, params, sizeof(params), timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_read_data params (DA)\n");
		return -1;
	}

	if (da_get_status(fd, timeout_sec) != DA_STATUS_OK) {
		fprintf(stderr, "Error: wrong pre-read status for mtk_da_cmd_read_data (DA)\n");
		return -1;
	}

	while (length > 0) {
		read_size = da_receive_data(fd, &read_buf, 0, 0);
		if (length > read_size) {
			length -= read_size;
		} else {
			length = 0;
		}

		/* TODO: Do something with the data received */

		if (read_buf != NULL) {
			free(read_buf);
			read_buf = NULL;
		}

		/* Send an ACK and wait for the status */
		if (da_send_data_check_status(fd, ack, sizeof(ack), timeout_sec) < 0) {
			fprintf(stderr, "Error: wrong status for mtk_da_cmd_read_data ack (DA)\n");
			return -1;
		}
	}

	return 0;
}

static int da_write_flash(int fd, size_t address, unsigned char *buf, size_t length, size_t chunk_size, unsigned int timeout_sec)
{
	unsigned char params[56] = {0};
	unsigned char ack[4] = {0};
	unsigned char checksum[4] = {0};
	unsigned int checksum_int = 0;
	size_t packet_len = 0;
	size_t i;

	/* TODO: support more storage */
	/* Storage = EMMC */
	params[0] = 0x01;
	params[1] = 0x00;
	params[2] = 0x00;
	params[3] = 0x00;

	/* Parttype = EMMC_USER */
	params[4] = 0x08;
	params[5] = 0x00;
	params[6] = 0x00;
	params[7] = 0x00;

	/* Address */
	params[8] = address & 0xFF;
	params[9] = (address >> 8) & 0xFF;
	params[10] = (address >> 16) & 0xFF;
	params[11] = (address >> 24) & 0xFF;
	params[12] = (address >> 32) & 0xFF;
	params[13] = (address >> 40) & 0xFF;
	params[14] = (address >> 48) & 0xFF;
	params[15] = (address >> 56) & 0xFF;

	/* Length */
	params[16] = length & 0xFF;
	params[17] = (length >> 8) & 0xFF;
	params[18] = (length >> 16) & 0xFF;
	params[19] = (length >> 24) & 0xFF;
	params[20] = (length >> 32) & 0xFF;
	params[21] = (length >> 40) & 0xFF;
	params[22] = (length >> 48) & 0xFF;
	params[23] = (length >> 56) & 0xFF;

	if (da_send_data_check_status(fd, mtk_da_cmd_write_data, sizeof(mtk_da_cmd_write_data), timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_write_data (DA)\n");
		return -1;
	}

	if (da_send_data_check_status(fd, params, sizeof(params), timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_write_data params (DA)\n");
		return -1;
	}

	while (length > 0) {
		packet_len = (length > chunk_size) ? chunk_size : length;

		/* Compute checksum for the chunk */
		for (i = 0; i < packet_len; i++) {
			checksum_int += buf[i];
		}
		checksum_int &= 0xFFFFFFFF;
		checksum[0] = checksum_int & 0xFF;
		checksum[1] = (checksum_int >> 8) & 0xFF;
		checksum[2] = (checksum_int >> 16) & 0xFF;
		checksum[3] = (checksum_int >> 24) & 0xFF;
		dbg_printf(3, "Packet length is %u bytes, checksum is 0x%x\n", packet_len, checksum_int);

		/* Send the chunk */
		da_send_data(fd, ack, sizeof(ack));
		da_send_data(fd, checksum, sizeof(checksum));

		if (da_send_data_check_status(fd, buf, packet_len, timeout_sec) < 0) {
			fprintf(stderr, "Error: wrong status for mtk_da_cmd_write_data chunk with %lu bytes remaining (DA)\n", length);
			return -1;
		}

		length -= packet_len;
		buf += packet_len;
	}

	return 0;
}

static int da_shutdown(int fd, unsigned int timeout_sec)
{
	unsigned char params[32] = {0}; // bootmode = shutdown

	if (da_send_data_check_status(fd, mtk_da_cmd_shutdown, sizeof(mtk_da_cmd_shutdown), timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_shutdown (DA)\n");
		return -1;
	}

	if (da_send_data_check_status(fd, params, sizeof(params), timeout_sec) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_shutdown params (DA)\n");
		return -1;
	}

	return 0;
}

static int load_da_info(FILE *f, da_info_t *da, unsigned int index)
{
	unsigned char buf[4];

	/* DA address (little-endian) */
	fseek(f, DA_OFFSET_START_ADDR(index), SEEK_SET);

	if (fread(buf, 1, 4, f) != 4) {
		fprintf(stderr, "Failed to read DA(%u) address: %s\n", index, strerror (errno));
		return -1;
	}
	
	da->start_addr[0] = buf[3];
	da->start_addr[1] = buf[2];
	da->start_addr[2] = buf[1];
	da->start_addr[3] = buf[0];
	dbg_printf(1, "DA(%u) start address: 0x%02x%02x%02x%02x\n", index, da->start_addr[0], da->start_addr[1], da->start_addr[2], da->start_addr[3]);

	/* DA size (little-endian) */
	fseek(f, DA_OFFSET_SIZE(index), SEEK_SET);

	if (fread(buf, 1, 4, f) != 4) {
		fprintf(stderr, "Failed to read DA(%u) size: %s\n", index, strerror (errno));
		return -1;
	}
	
	da->size[0] = buf[3];
	da->size[1] = buf[2];
	da->size[2] = buf[1];
	da->size[3] = buf[0];
	da->size_int = da->size[3] | (da->size[2] << 8) | (da->size[1] << 16) | (da->size[0] << 24);
	dbg_printf(1, "DA(%u) size: %u (0x%02x%02x%02x%02x) bytes\n", index, da->size_int, da->size[0], da->size[1], da->size[2], da->size[3]);

	/* DA signature length (little-endian) */
	fseek(f, DA_OFFSET_SIG_LEN(index), SEEK_SET);

	if (fread(buf, 1, 4, f) != 4) {
		fprintf(stderr, "Failed to read DA(%u) signature length: %s\n", index, strerror (errno));
		return -1;
	}
	
	da->sig_len[0] = buf[3];
	da->sig_len[1] = buf[2];
	da->sig_len[2] = buf[1];
	da->sig_len[3] = buf[0];
	da->sig_len_int = da->sig_len[3] | (da->sig_len[2] << 8) | (da->sig_len[1] << 16) | (da->sig_len[0] << 24);
	dbg_printf(1, "DA(%u) signature length: %u (0x%02x%02x%02x%02x) bytes\n", index, da->sig_len_int, da->sig_len[0], da->sig_len[1], da->sig_len[2], da->sig_len[3]);

	/* DA data offset (little-endian) */
	fseek(f, DA_OFFSET_DATA_OFFSET(index), SEEK_SET);

	if (fread(buf, 1, 4, f) != 4) {
		fprintf(stderr, "Failed to read DA(%u) data offset: %s\n", index, strerror (errno));
		return -1;
	}
	
	da->data_offset = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
	dbg_printf(1, "DA(%u) data offset: 0x%X\n", index, da->data_offset);

	return 0;
}

unsigned char *my_memmem(const unsigned char *haystack, size_t haystacklen, const unsigned char *needle, size_t needlelen)
{
	const unsigned char *haystackend = haystack + haystacklen;

	while (haystack <= haystackend - needlelen) {
		if (memcmp(haystack, needle, needlelen)) {
			return (unsigned char *) haystack;
		}

		haystack++;
	}

	return NULL;
}

static void usage(FILE * fp, int argc, char **argv)
{
	fprintf(fp,
		APP_NAME" v"APP_VERSION" - (C)"COPYRIGHT_DATE" "AUTHOR_NAME"\n\n"
		"Usage: %s [options]\n\n"
		"Options:\n"
		"\t-t | --tty <path>             The TTY to open (default is %s)\n"
		"\t-d | --da <path>              The DA to upload (default is %s)\n"
		"\t-f | --file <path>            The file to write (default is %s)\n"
		"\t-v | --verbose                Show more information like hex dump of the data (-vv and -vvv for even more details)\n"
		"\t-h | --help                   Print this message\n",
		argv[0], DEFAULT_TTY_PATH, DEFAULT_DA_PATH, DEFAULT_FILE_PATH);
}

static const char short_options[] = "t:d:f:vh";

static const struct option long_options[] = {
	{"tty", required_argument, NULL, 't'},
	{"da", required_argument, NULL, 'd'},
	{"file", required_argument, NULL, 'f'},
	{"verbose", no_argument, NULL, 'v'},
	{"help", no_argument, NULL, 'h'},
	{0, 0, 0, 0}
};

int main(int argc, char **argv)
{
	char da_path[256] = DEFAULT_DA_PATH;
	char tty_path[256] = DEFAULT_TTY_PATH;
	char file_path[256] = DEFAULT_FILE_PATH;
	int fd_tty = -1;
	FILE *f_da = NULL;
	FILE *f_file = NULL;
	unsigned char read_buf[READ_BUF_SIZE];
	unsigned char *read_buf_addr = read_buf;
	int ret = 0;
	unsigned int i;
	struct stat st;
	size_t buf_size;
	da_info_t da1, da2;
	unsigned char boot_to_params[16];
	unsigned char *da2_data = NULL;
	unsigned int da_packet_length_write = 0, da_packet_length_read = 0;
	unsigned char *file_data = NULL;
	size_t file_size = 0;
	size_t padded_file_size = 0;

	for (;;) {
		int index;
		int c;

		c = getopt_long(argc, argv, short_options, long_options, &index);

		if (-1 == c)
			break;

		switch (c) {
			case 0:	/* getopt_long() flag */
				break;

			case 't':
				strncpy(tty_path, optarg, 256);
				break;

			case 'd':
				strncpy(da_path, optarg, 256);
				break;

			case 'f':
				strncpy(file_path, optarg, 256);
				break;

			case 'v':
				log_level++;
				break;

			case 'h':
				usage(stdout, argc, argv);
				exit(EXIT_SUCCESS);

			default:
				usage(stderr, argc, argv);
				exit(EXIT_FAILURE);
		}
	}

	f_file = fopen(file_path, "r");
	if (f_file == NULL) {
		fprintf(stderr, "Failed to open %s: %s\n", file_path, strerror (errno));
		ret = -1;
		goto out;
	}

	fseek(f_file, 0, SEEK_END);
	file_size = ftell(f_file);
	padded_file_size = (file_size + (0x200 - 1) & ~(0x200 - 1));
	fseek(f_file, 0, SEEK_SET);

	dbg_printf(1, "Loading file %s with a size of %u bytes padded to %u bytes\n\n", file_path, file_size, padded_file_size);

	/* Pad with 0x00 so that the size is aligned on 512 bytes */
	file_data = (unsigned char *) malloc(padded_file_size);
	if (!file_data) {
		fprintf(stderr, "Error: cannot allocate memory for file data\n");
		ret = -1;
		goto out;
	}

	memset(file_data, 0, padded_file_size);

	if (fread(file_data, 1, file_size, f_file) != file_size) {
		fprintf(stderr, "Failed to read file data: %s\n", strerror (errno));
		ret = -1;
		goto out;
	}

	fclose(f_file);
	f_file = NULL;

	/* Open the DA and get the required information */
	dbg_printf(0, "Using DA file %s\n", da_path);

	f_da = fopen(da_path, "r");
	if (f_da == NULL) {
		fprintf(stderr, "Failed to open %s: %s\n", da_path, strerror (errno));
		ret = -1;
		goto out;
	}

	load_da_info(f_da, &da1, 1);
	load_da_info(f_da, &da2, 2);

	dbg_printf(0, "\n");

	/* Wait until the serial is available */
	dbg_printf(0, "Waiting for %s...\n\n", tty_path);
	while (stat(tty_path, &st) == -1) {
		usleep(100000); // wait for 100 ms
	}

	fd_tty = open(tty_path, O_RDWR | O_NOCTTY | O_SYNC /*| O_NONBLOCK*/);
	if (fd_tty < 0) {
		fprintf(stderr, "Failed to open %s: %s\n", tty_path, strerror (errno));
		ret = -1;
		goto out;
	}

	set_interface_attribs(fd_tty, TTY_SPEED, 0);	// set speed to 921600 bps, 8n1 (no parity)
	set_blocking_timings(fd_tty);

	/* Be sure we start from a sane point */
	tcflush(fd_tty, TCIOFLUSH);

	/* Wait for the "READY" string */
	dbg_printf(0, "Waiting for the device to be ready...\n");
	while (1) {
		buf_size = receive_data(fd_tty, read_buf, 20, 1, 0);
		if (my_memmem(read_buf, buf_size, (const unsigned char *) "READY", 5)) {
			dbg_printf(0, "MTK device is ready\n\n");
			break;
		}

		usleep(100000); // wait for 100 ms
	}

	/* Flush the input buffer before next step */
	tcflush(fd_tty, TCIOFLUSH);

	/* Sync with the device */
	dbg_printf(0, "Syncing with the device...\n");
	while (1) {
		/* Send the first byte of the START magic in loop until we get a reply */
		send_data(fd_tty, mtk_cmd_start, 1);
		buf_size = receive_data(fd_tty, read_buf, 1, 1, 0);
		if (*read_buf == mtk_cmd_start_reply[0]) {
			break;
		}

		usleep(10000); // wait for 100 ms
		// TODO Add timeout
	}

	/* We are now synced, send the remaining bytes of the start command */
	for (i = 1; i < sizeof(mtk_cmd_start); i++) {
		send_data(fd_tty, &mtk_cmd_start[i], 1);
		buf_size = receive_data(fd_tty, read_buf, 1, 1, 0);
		if (*read_buf != mtk_cmd_start_reply[i]) {
			fprintf(stderr, "Wrong reply !\n");
			ret = -1;
			goto out;
		}
	}
	dbg_printf(0, "Device is now synced and accepts commands\n\n");

	/* We can start sending actual commands */
	/* Get the bootloader version */
	send_data(fd_tty, mtk_cmd_bl_ver, 1);
	buf_size = receive_data(fd_tty, read_buf, 1, 1, 0);
	if (buf_size < 1) {
		fprintf(stderr, "Error: wrong reply to mtk_cmd_bl_ver\n");
		ret = -1;
		goto out;
	}
	dbg_printf(0, "Bootloader version: %u\n", *read_buf);

	/* Get the HW code */
	send_data(fd_tty, mtk_cmd_hw_code, 1);
	buf_size = receive_data(fd_tty, read_buf, 5, 5, 0);
	if (buf_size < 5 || read_buf[0] != mtk_cmd_hw_code[0]) {
		fprintf(stderr, "Error: wrong reply to mtk_cmd_hw_code\n");
		ret = -1;
		goto out;
	}
	dbg_printf(0, "HW code: %02x%02x%02x%02x\n", read_buf[1], read_buf[2], read_buf[3], read_buf[4]);

	/* Get the HW/SW versions */
	send_data(fd_tty, mtk_cmd_hw_sw_ver, 1);
	buf_size = receive_data(fd_tty, read_buf, 9, 9, 0);
	if (buf_size < 9 || read_buf[0] != mtk_cmd_hw_sw_ver[0]) {
		fprintf(stderr, "Error: wrong reply to mtk_cmd_hw_sw_ver\n");
		ret = -1;
		goto out;
	}
	dbg_printf(0, "HW/SW version: %02x%02x%02x%02x%02x%02x%02x%02x\n\n", read_buf[1], read_buf[2], read_buf[3], read_buf[4], read_buf[5], read_buf[6], read_buf[7], read_buf[8]);

	/* Send the DA */
	dbg_printf(0, "Sending the DA to the device...\n");
	send_data(fd_tty, mtk_cmd_send_da, 1);
	buf_size = receive_data(fd_tty, read_buf, 1, 1, 0);
	if (buf_size < 1 || read_buf[0] != mtk_cmd_send_da[0]) {
		fprintf(stderr, "Error: wrong reply to mtk_cmd_send_da\n");
		ret = -1;
		goto out;
	}

	/* Set the address of the DA */
	send_data(fd_tty, da1.start_addr, 4);
	buf_size = receive_data(fd_tty, read_buf, 4, 4, 0);
	if (buf_size < 4 || read_buf[0] != da1.start_addr[0] || read_buf[1] != da1.start_addr[1] || read_buf[2] != da1.start_addr[2] || read_buf[3] != da1.start_addr[3]) {
		fprintf(stderr, "Error: wrong reply to da1.start_addr\n");
		ret = -1;
		goto out;
	}

	/* Set the size of the DA */
	send_data(fd_tty, da1.size, 4);
	buf_size = receive_data(fd_tty, read_buf, 4, 4, 0);
	if (buf_size < 4 || read_buf[0] != da1.size[0] || read_buf[1] != da1.size[1] || read_buf[2] != da1.size[2] || read_buf[3] != da1.size[3]) {
		fprintf(stderr, "Error: wrong reply to da1.size\n");
		ret = -1;
		goto out;
	}

	/* Set the signature length of the DA */
	send_data(fd_tty, da1.sig_len, 4);
	buf_size = receive_data(fd_tty, read_buf, 4, 4, 0);
	if (buf_size < 4 || read_buf[0] != da1.sig_len[0] || read_buf[1] != da1.sig_len[1] || read_buf[2] != da1.sig_len[2] || read_buf[3] != da1.sig_len[3]) {
		fprintf(stderr, "Error: wrong reply to da1.sig_len\n");
		ret = -1;
		goto out;
	}

	/* Get the status */
	buf_size = receive_data(fd_tty, read_buf, 2, 2, 0);
	if (buf_size < 2 || read_buf[0] != 0x00 || read_buf[1] != 0x00) {
		fprintf(stderr, "Error: wrong mtk_cmd_send_da status\n");
		ret = -1;
		goto out;
	}

	/* Send the actual DA data */
	fseek(f_da, da1.data_offset, SEEK_SET);

	while (da1.size_int > 0) {
		buf_size = fread(read_buf, 1, (da1.size_int > READ_BUF_SIZE) ? READ_BUF_SIZE : da1.size_int, f_da);
		send_data(fd_tty, read_buf, buf_size);
		da1.size_int -= buf_size;
	}

	/* Get the checksum computed by the device and verify it */
	buf_size = receive_data(fd_tty, read_buf, 2, 2, 0);
	if (buf_size < 2) {
		fprintf(stderr, "Error: wrong DA checksum received\n");
		ret = -1;
		goto out;
	}
	dbg_printf(1, "Received DA checksum: %02x%02x\n", read_buf[0], read_buf[1]);

	/* TODO checksum verification */

	/* Get the upload status */
	buf_size = receive_data(fd_tty, read_buf, 2, 2, 0);
	if (buf_size < 2 || read_buf[0] != 0x00 || read_buf[1] != 0x00) {
		fprintf(stderr, "Error: wrong DA upload status\n");
		ret = -1;
		goto out;
	}
	dbg_printf(1, "Received DA upload status: %02x%02x\n", read_buf[0], read_buf[1]);
	dbg_printf(0, "The DA has been successfully uploaded\n\n");

	/* Jump to the DA */
	dbg_printf(0, "Jumping to the DA...\n");
	send_data(fd_tty, mtk_cmd_jump_da, 1);
	buf_size = receive_data(fd_tty, read_buf, 1, 1, 0);
	if (buf_size < 1 || read_buf[0] != mtk_cmd_jump_da[0]) {
		fprintf(stderr, "Error: wrong reply to mtk_cmd_jump_da\n");
		ret = -1;
		goto out;
	}

	/* Set the address of the DA for the jump */
	send_data(fd_tty, da1.start_addr, 4);
	buf_size = receive_data(fd_tty, read_buf, 4, 4, 0);
	if (buf_size < 4 || read_buf[0] != da1.start_addr[0] || read_buf[1] != da1.start_addr[1] || read_buf[2] != da1.start_addr[2] || read_buf[3] != da1.start_addr[3]) {
		fprintf(stderr, "Error: wrong reply to da1.start_addr\n");
		ret = -1;
		goto out;
	}

	/* Get the status before the jump */
	buf_size = receive_data(fd_tty, read_buf, 2, 2, 0);
	if (buf_size < 2 || read_buf[0] != 0x00 || read_buf[1] != 0x00) {
		fprintf(stderr, "Error: wrong mtk_cmd_jump_da status\n");
		ret = -1;
		goto out;
	}

	/* Get the DA sync byte */
	buf_size = receive_data(fd_tty, read_buf, 1, 1, 0);
	if (buf_size < 1 || read_buf[0] != DA_SYNC) {
		fprintf(stderr, "Error: wrong DA sync byte\n");
		ret = -1;
		goto out;
	}
	dbg_printf(0, "DA properly started\n\n");

	/* Syncing with the DA */
	dbg_printf(0, "Syncing with the DA...\n");
	da_send_data(fd_tty, mtk_da_cmd_sync, sizeof(mtk_da_cmd_sync));

	/* Setting-up environment */
	da_send_data(fd_tty, mtk_da_cmd_set_env, sizeof(mtk_da_cmd_set_env));
	if (da_send_data_check_status(fd_tty, mtk_da_env_params, sizeof(mtk_da_env_params), 0) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_set_env (DA)\n");
		ret = -1;
		goto out;
	}

	/* HW inititialization */
	da_send_data(fd_tty, mtk_da_cmd_hw_init, sizeof(mtk_da_cmd_hw_init));
	if (da_send_data_check_status(fd_tty, mtk_da_hw_init_params, sizeof(mtk_da_hw_init_params), 0) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_hw_init (DA)\n");
		ret = -1;
		goto out;
	}

	/* Check syncing has been successful */
	buf_size = da_receive_data(fd_tty, &read_buf_addr, sizeof(mtk_da_cmd_sync), 0);
	if (buf_size < 4 || read_buf[0] != mtk_da_cmd_sync[0] || read_buf[1] != mtk_da_cmd_sync[1] || read_buf[2] != mtk_da_cmd_sync[2] || read_buf[3] != mtk_da_cmd_sync[3]) {
		fprintf(stderr, "Error: wrong sync status received (DA)\n");
		ret = -1;
		goto out;
	}
	dbg_printf(0, "Synced with the DA\n\n");

	/* Get expire date (no answer) */
	da_dev_ctrl_get(fd_tty, mtk_da_dev_ctrl_get_expire_date, read_buf, READ_BUF_SIZE, 0);

	/* Set reset key */
	read_buf[0] = 0x68;
	read_buf[1] = 0x00;
	read_buf[2] = 0x00;
	read_buf[3] = 0x00;
	if (da_dev_ctrl_set(fd_tty, mtk_da_dev_ctrl_set_reset_key, read_buf, 4, 0) != 0) {
		fprintf(stderr, "Error: failed to set reset key (DA)\n");
		ret = -1;
		goto shutdown;
	}

	/* Set checksum level */
	read_buf[0] = 0x00;
	read_buf[1] = 0x00;
	read_buf[2] = 0x00;
	read_buf[3] = 0x00;
	if (da_dev_ctrl_set(fd_tty, mtk_da_dev_ctrl_set_checksum_level, read_buf, 4, 0) != 0) {
		fprintf(stderr, "Error: failed to set checksum level (DA)\n");
		ret = -1;
		goto shutdown;
	}

	/* Get connection agent */
	buf_size = da_dev_ctrl_get(fd_tty, mtk_da_dev_ctrl_get_connection_agent, read_buf, READ_BUF_SIZE, 0);
	if (buf_size == 0) {
		fprintf(stderr, "Error: failed to receive connection agent (DA)\n");
		ret = -1;
		goto shutdown;
	}

	read_buf[buf_size] = '\0';
	dbg_printf(0, "Connection mode is %s\n\n", read_buf);

	/* Preloader mode is the only one supported
	 * TODO: Add support for Brom mode
	 */
	if (strncmp((char *) read_buf, "preloader", buf_size) != 0) {
		fprintf(stderr, "Error: connection mode %s not supported\n", read_buf);
		ret = -1;
		goto shutdown;
	}

	/* Uploading stage 2 */
	dbg_printf(0, "Uploading DA stage 2...\n");
	da2_data = (unsigned char *) malloc(da2.size_int);
	if (!da2_data) {
		fprintf(stderr, "Error: cannot allocate memory for DA2 data\n");
		ret = -1;
		goto shutdown;
	}

	fseek(f_da, da2.data_offset, SEEK_SET);
	if (fread(da2_data, 1, da2.size_int, f_da) != da2.size_int) {
		fprintf(stderr, "Error: failed to read DA2 data\n");
		ret = -1;
		goto shutdown;
	}

	if (da_send_data_check_status(fd_tty, mtk_da_cmd_boot_to, sizeof(mtk_da_cmd_boot_to), 0) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_boot_to (DA)\n");
		ret = -1;
		goto shutdown;
	}

	boot_to_params[0] = da2.start_addr[3];
	boot_to_params[1] = da2.start_addr[2];
	boot_to_params[2] = da2.start_addr[1];
	boot_to_params[3] = da2.start_addr[0];
	boot_to_params[4] = 0x00;
	boot_to_params[5] = 0x00;
	boot_to_params[6] = 0x00;
	boot_to_params[7] = 0x00;

	boot_to_params[8] = da2.size[3];
	boot_to_params[9] = da2.size[2];
	boot_to_params[10] = da2.size[1];
	boot_to_params[11] = da2.size[0];
	boot_to_params[12] = 0x00;
	boot_to_params[13] = 0x00;
	boot_to_params[14] = 0x00;
	boot_to_params[15] = 0x00;

	da_send_data(fd_tty, boot_to_params, sizeof(boot_to_params));

	/* Upload the actual data without the signature */
	if (da_send_data_check_status(fd_tty, da2_data, da2.size_int - da2.sig_len_int, 0) < 0) {
		fprintf(stderr, "Error: wrong status for mtk_da_cmd_boot_to parameters (DA)\n");
		ret = -1;
		goto shutdown;
	}

	free(da2_data);
	da2_data = NULL;

	dbg_printf(0, "DA stage 2 properly uploaded, jumping to it...\n");

	/* Check if the jump has been successful */
	buf_size = da_receive_data(fd_tty, &read_buf_addr, sizeof(mtk_da_cmd_sync), 0);
	if (buf_size < 4 || read_buf[0] != mtk_da_cmd_sync[0] || read_buf[1] != mtk_da_cmd_sync[1] || read_buf[2] != mtk_da_cmd_sync[2] || read_buf[3] != mtk_da_cmd_sync[3]) {
		fprintf(stderr, "Error: wrong sync status received (DA)\n");
		ret = -1;
		goto shutdown;
	}
	dbg_printf(0, "DA stage 2 properly running and synced\n\n");

	if (da_dev_ctrl_get(fd_tty, mtk_da_dev_ctrl_get_ram_info, read_buf, 48, 0) != 48) {
		fprintf(stderr, "Error: failed to get RAM info (DA)\n");
		ret = -1;
		goto shutdown;
	}

	if (da_dev_ctrl_get(fd_tty, mtk_da_dev_ctrl_get_emmc_info, read_buf, 96, 0) != 96) {
		fprintf(stderr, "Error: failed to get EMMC info (DA)\n");
		ret = -1;
		goto shutdown;
	}

	if (da_dev_ctrl_get(fd_tty, mtk_da_dev_ctrl_get_chip_id, read_buf, 12, 0) != 12) {
		fprintf(stderr, "Error: failed to get chip ID (DA)\n");
		ret = -1;
		goto shutdown;
	}

	if (da_dev_ctrl_get(fd_tty, mtk_da_dev_ctrl_get_da_version, read_buf, 3, 0) != 3) {
		fprintf(stderr, "Error: failed to get DA version (DA)\n");
		ret = -1;
		goto shutdown;
	}

	if (da_dev_ctrl_get(fd_tty, mtk_da_dev_ctrl_get_random_id, read_buf, 16, 0) != 16) {
		fprintf(stderr, "Error: failed to get random ID (DA)\n");
		ret = -1;
		goto shutdown;
	}

	if (da_dev_ctrl_get(fd_tty, mtk_da_dev_ctrl_get_usb_speed, read_buf, 10, 0) != 10) {
		fprintf(stderr, "Error: failed to get USB speed (DA)\n");
		ret = -1;
		goto shutdown;
	}

	/* Get read/write packet lengths */
	if (da_dev_ctrl_get(fd_tty, mtk_da_dev_ctrl_get_packet_length, read_buf, 8, 0) != 8) {
		fprintf(stderr, "Error: failed to get packet length (DA)\n");
		ret = -1;
		goto shutdown;
	}

	da_packet_length_write = read_buf[3] | (read_buf[2] << 8) | (read_buf[1] << 16) | (read_buf[0] << 24);
	da_packet_length_read = read_buf[7] | (read_buf[6] << 8) | (read_buf[5] << 16) | (read_buf[4] << 24);

	dbg_printf(1, "DA packet lengths are %u bytes for WRITE and %u bytes for READ\n\n", da_packet_length_write, da_packet_length_read);


	dbg_printf(0, "Writing %s at 0x%X...\n", file_path, 0x13e20000);
	if (da_write_flash(fd_tty, 0x13e20000, file_data, padded_file_size, da_packet_length_write, 0) < 0) {
		fprintf(stderr, "Error: failed to write file_data (DA)\n");
		ret = -1;
		goto shutdown;
	}
	dbg_printf(0, "Successfully written\n\n");

	dbg_printf(0, "Writing %s at 0x%X...\n", file_path, 0x14320000);
	if (da_write_flash(fd_tty, 0x14320000, file_data, padded_file_size, da_packet_length_write, 0) < 0) {
		fprintf(stderr, "Error: failed to write file_data (DA)\n");
		ret = -1;
		goto shutdown;
	}
	dbg_printf(0, "Successfully written\n\n");

	//da_read_flash(fd_tty, 0x13e20000, 0x500000, read_buf, READ_BUF_SIZE, 0);

	//while (1) receive_data(fd_tty, read_buf, READ_BUF_SIZE, 1, 0);

shutdown:
	dbg_printf(0, "Sending shutdown request...\n");
	if (da_shutdown(fd_tty, 0) < 0) {
		fprintf(stderr, "Error: failed to shutdown the device (DA)\n");
	}
	dbg_printf(0, "The device can be unplugged\n\n");

out:
	if (da2_data != NULL) {
		free(da2_data);
	}

	if (file_data != NULL) {
		free(file_data);
	}

	if (fd_tty > 0) {
		close(fd_tty);
	}

	if (f_da != NULL) {
		fclose(f_da);
	}

	if (f_file != NULL) {
		fclose(f_file);
	}

	return ret;
}
