## mlx_rawdata decoder
This directory contains a utility which reads files ending in the extension "mlx_rawdata" and converts the mlx data to a CSV containing temperatures in degrees celsius. The utility prints the csv file to stdout.

"mlx_rawdata" files are binary files which contain an arbitrarily long array of single MLX readings, each of which is followed by a small amount of metadata.


#### Compiling and running
Run `make` in this directory to compile the `mlx_rawdata_decoder`. After that, you can run it with
```
    ./mlx_rawdata_decoder <input_mlx_rawdata_file> <input_mlx_calibration_file> <output_csv_file>
```

#### mlx_rawdata format
Each MLX reading is an array of 768 `uint16_t`; each reading is followed by a short metadata footer. The metadata is a small struct with the following fields:

```
    uint32_t unix_timestamp;    // big endian
    uint16_t milliseconds;      // big endian
    uint24_t save_time;         // big endian
    uint16_t mlx_ID[3];         // little endian
    uint16_t end_sequence;      // 0x0d 0x0a
```

#### Calibration files
To decode one of these binary files, an mlx calibration file must also be provided. An MLX calibration file just consists of a dump of the mlx's EEPROM. Every time it starts up, the GroundTruthCamera reads the EEPROM of the connected MLX and saves it in a calibration file. Each MLX sensor has a unique 48-bit ID and a unique calibration file; the GroundTruthCamera reads the MLX's ID and appends it in hex to the name of the calibration file.

For the calibration to be correct, the MLX ID in the calibration file should match the one recorded in the above metadata. Typically, the correct calibration file associated with the MLX data on an SD card will be on the SD card. If an MLX is swapped but the same SD card is left in, there will be 2 different mlx calibration files in the SD card. If the wrong MLX calibration file is used for an mlx_rawdata file, a warning will be printed.
