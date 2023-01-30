#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include "MLX90640_API.h"

#define MLX_EEP_SIZE 832

#define RAW_FRAME_METADATA_OFFSET (834 * 2)
#define RAW_FRAME_METADATA_LENGTH (18)
#define RAW_FRAME_LENGTH (RAW_FRAME_METADATA_OFFSET + RAW_FRAME_METADATA_LENGTH)

#define  TA_SHIFT     8 //Default shift for MLX90640 in open air might need to adjust this

typedef struct application_state
{
    uint16_t mlx_eeprom[MLX_EEP_SIZE];
    paramsMLX90640 mlx90640_parameters;

    uint8_t* filedata;
    long int filesize;

    FILE* output_file;
} application_state_t;

const char* helpstr = "Usage: %s -i <rawdata_input_file> -c <mlx_calibration_file>\n";

static int initialize_application_state(application_state_t*, int, char**);
static int decode_one_block(const application_state_t*, int);

static int get_exact_temperature = 1;

static struct option opts[] =
{
    {"approximate-image",    no_argument,  &get_exact_temperature,  0},
    {"calibration",    required_argument,                      0, 'c'},
    {"input",          required_argument,                      0, 'i'},
    {"output",         required_argument,                      0, 'o'}
};

int main(int argc, char** argv)
{
    application_state_t state;
    if (initialize_application_state(&state, argc, argv)) {
        fprintf(stderr, helpstr, argv[0]);
        return -1;
    }

    for (int i = 0; i < (state.filesize / RAW_FRAME_LENGTH); i++) {
        decode_one_block(&state, i);
    }

    if (state.output_file != stdout) {
        fclose(state.output_file);
    }

    return -1;
}

/**
 * This function checks the validity of the arguments passed to mlx_rawdata_decoder and opens the
 * 3 files that it reads and writes.
 *
 * If the files can't be opened, the function returns a nonzero value.
 * If the calibration and mlx_rawdata file don't belong to the same sensor, a warning message is
 * printed.
 */
static int initialize_application_state(application_state_t* state, int argc, char** argv)
{
    // parse args
    int option_index = 0;

    char* calibration_filename = NULL;
    char* rawdata_filename = NULL;
    char* output_filename = NULL;

    while (1) {
        int c;
        c = getopt_long (argc, argv, "c:i:o:", opts, &option_index);

        if (c == -1)
            break;

        switch (c)
        {
            case 0:
                // If this option set a flag, we don't need to do anything else.
                break;

            case 'c':
                calibration_filename = argv[optind - 1];
                break;

            case 'i':
                rawdata_filename = argv[optind - 1];
                break;

            case 'o':
                output_filename = argv[optind - 1];
                break;

            case '?':
                /* getopt_long already printed an error message. */
                break;

            default:
                fprintf(stderr, "Unknown command line option\n");
                return -1;
                break;
        }
    }

    FILE* mlx_rawdata_file;
    FILE* mlx_calibration_file;

    // open files
    if ((calibration_filename == NULL) || (rawdata_filename == NULL)) {
        fprintf(stderr, "An mlx raw data file needs to be specified with the --input flag and \n"
                "A calibration file needs to be specified with the --calibration option.\n"
                "One of them is missing.\n");
        return -1;
    }

    if ((mlx_rawdata_file = fopen(rawdata_filename, "rb")) == NULL) {
        fprintf(stderr, "Error: Couldn't open file %s for reading.\n", rawdata_filename);
        fprintf(stderr, "%s\n", strerror(errno));
        return -1;
    }

    if ((mlx_calibration_file = fopen(calibration_filename, "rb")) == NULL) {
        fprintf(stderr, "Error: Couldn't open file %s for reading.\n", calibration_filename);
        fprintf(stderr, "%s\n", strerror(errno));
        return -1;
    }

    if (output_filename != NULL) {
        state->output_file = fopen(output_filename, "w");
    } else {
        state->output_file = stdout;
    }

    // extract data from mlx calibration file
    size_t words_read = fread(state->mlx_eeprom, sizeof(uint16_t), MLX_EEP_SIZE, mlx_calibration_file);
    if (words_read != MLX_EEP_SIZE) {
        fprintf(stderr, "Error: mlx calibration file too small. Read %li words, expected %i\n", words_read, MLX_EEP_SIZE);
        return -1;
    }

    if (MLX90640_ExtractParameters(state->mlx_eeprom, &state->mlx90640_parameters)) {
        fprintf(stderr, "Error: failed to extract mlx90640 parameters from mlx90640 calibration file %s.\n", argv[2]);
        return -1;
    }

    // read data of file into ram
    fseek(mlx_rawdata_file, 0, SEEK_END);
    state->filesize = ftell(mlx_rawdata_file);
    fseek(mlx_rawdata_file, 0, SEEK_SET);

    state->filedata = malloc(state->filesize);
    if (fread(state->filedata, 1, state->filesize, mlx_rawdata_file) != state->filesize) {
        fprintf(stderr, "Error: didn't read expected number of bytes from mlx rawdata file.\n");
        return -1;
    }

    fclose(mlx_rawdata_file);
    fclose(mlx_calibration_file);

    return 0;
}

/**
 * Decodes the block starting at index offset and prints it to the output file
 */
static int decode_one_block(const application_state_t* state, int offset)
{
    // check id
    uint8_t* ptr = &state->filedata[offset * RAW_FRAME_LENGTH];
    uint8_t* endp = &ptr[RAW_FRAME_METADATA_OFFSET];
    if (memcmp(&state->mlx_eeprom[7], &endp[10], 6)) {
        fprintf(stderr, "Warning: mlx calibration file has id %04x%04x%04x but frame has id %02x%02x%02x%02x%02x%02x\n",
               state->mlx_eeprom[9], state->mlx_eeprom[8], state->mlx_eeprom[7],
               endp[15], endp[14], endp[13], endp[12], endp[11], endp[10]);
    }

    uint64_t seconds = ((((uint64_t)endp[0]) << 24) | (((uint64_t)endp[1]) << 16) |
                        (((uint64_t)endp[2]) << 8) | (((uint64_t)endp[3]) << 0));
    uint64_t millis =  ((((uint64_t)endp[4]) << 8) | (((uint64_t)endp[5]) << 0));

    static float result[768];
    if (get_exact_temperature) {
        //float vdd = MLX90640_GetVdd((uint16_t*)ptr, &state->mlx90640_parameters);
        float Ta = MLX90640_GetTa((uint16_t*)ptr, &state->mlx90640_parameters);
        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;

        MLX90640_CalculateTo((uint16_t*)ptr, &state->mlx90640_parameters, emissivity, tr, result);
    } else {
        MLX90640_GetImage((uint16_t*)ptr, &state->mlx90640_parameters, result);
    }

    // print everything
    fprintf(state->output_file, "%lu, ", 1000 * ((uint64_t)seconds) + millis);
    for (int i = 0; i < 768; i++) {
        fprintf(state->output_file, "%5.2f, ", result[i]);
    }
    fprintf(state->output_file, "\n");

    return 0;
}
