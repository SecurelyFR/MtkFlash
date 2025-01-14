# MtkFlash

MtkFlash is a command-line tool for interfacing with MT6739 MediaTek-based devices. It facilitates firmware flashing, reading, and other low-level operations over a serial connection.

## Features

- Upload Download Agents (DA) to MediaTek devices
- Flash firmware files to specified partitions
- Read from and write to device memory or partitions
- Format device memory
- Send shutdown commands to the device

## Requirements

- A MediaTek-based device
- A serial connection to the device
- A scatter file for partition mapping
- DA binary file (e.g., `MTK_AllInOne_DA.bin`)

## Compilation

MtkFlash is written in C and can be compiled using `gcc` or any standard C compiler. Use the following command to build the program:

```bash
gcc -o MtkFlash MtkFlash.c
```

## Usage

Run the program from the command line with the following options:

```bash
MtkFlash [options]
```

### Options

| Option | Description |
|--------|-------------|
| `-t, --tty <path>` | Specify the TTY device to use (default: `/dev/ttyACM0`) |
| `-d, --da <path>` | Specify the path to the DA file (default: `MTK_AllInOne_DA.bin`) |
| `-s, --scatter <path>` | Specify the path to the scatter file |
| `-f, --file <path>` | Specify the file to write or read to/from the device |
| `-a, --addr <address>` | Specify the memory address for operations |
| `-n, --name <partition>` | Specify the name of the partition to operate on |
| `-p, --type <type>` | Specify the partition type |
| `-l, --length <length>` | Specify the length of data to read or format |
| `-D, --download` | Download a file to a specified partition |
| `-W, --write` | Write a file to a specific memory address |
| `-R, --read` | Read from a memory address into a file |
| `-F, --format` | Format a specific region of memory |
| `-S, --shutdown` | Send a shutdown command to the device |
| `-v, --verbose` | Increase verbosity level (up to three levels) |
| `-h, --help` | Show help information |

### Example Commands

1. **Flash a firmware file to a partition**:
   ```bash
   MtkFlash -t /dev/ttyUSB0 -d MTK_AllInOne_DA.bin -n boot -f boot.img -D
   ```

2. **Read memory into a file**:
   ```bash
   MtkFlash -t /dev/ttyUSB0 -p 8 -a 0x400000 -l 0x100000 -f dump.bin -R
   ```

3. **Format a specific memory region**:
   ```bash
   MtkFlash -t /dev/ttyUSB0 -p 8 -a 0x0 -l 0x100000 -F
   ```

4. **Send a shutdown command**:
   ```bash
   MtkFlash -t /dev/ttyUSB0 -S
   ```

## Notes

- The DA binary (`MTK_AllInOne_DA.bin`) must match the target device’s requirements.
- Use a compatible scatter file to ensure accurate partition mapping.
- Be cautious when performing write and format operations to avoid bricking the device.

## License

MtkFlash is distributed under the GPLv2 License. See the LICENSE file for details.

## Author

Jean-Christophe Rona (C) 2023
