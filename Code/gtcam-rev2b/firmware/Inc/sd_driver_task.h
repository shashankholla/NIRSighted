#ifndef _SD_DRIVER_TASK
#define _SD_DRIVER_TASK

#include "main.h"

/**
 * This enum represents the current state of the SD transfer. This lets the IRQ handler keep track
 * of what to do next.
 */
typedef enum {
    // next interrupt pertains to the CMD55 coming before an ACMD23
    SD_DRIVER_REQUEST_STATE_WRITE_CMD23_CMD55,
    SD_DRIVER_REQUEST_STATE_WRITE_CMD23,
    SD_DRIVER_REQUEST_STATE_WRITE_CMD25,
    SD_DRIVER_REQUEST_STATE_WRITE_CMD12,

    SD_DRIVER_REQUEST_STATE_IDLE
} sd_driver_request_state_e;

/**
 * This enum represents the current state of a single SD operation. Used by the IRQ handler to
 * keep track of where it is in an SD operation.
 */
typedef enum {
    SD_DRIVER_OPERATION_PHASE_COMMAND,
    SD_DRIVER_OPERATION_PHASE_DATA
} sd_driver_operation_phase_e;

/**
 * This enum represents the direction of an sd_driver_rw_request_t.
 */
typedef enum {
    SD_DRIVER_DIRECTION_READ,
    SD_DRIVER_DIRECTION_WRITE
} sd_driver_direction_e;

/**
 *
 */
typedef struct sd_driver_task_state {
    SD_HandleTypeDef* hsd;
} sd_driver_task_state_t;

/**
 * Keeps track of what the SDMMC IRQ handler is doing.
 */
typedef struct sd_driver_irq_state {
    sd_driver_request_state_e request_state;
    sd_driver_operation_phase_e operation_phase;

    // holds the current number of blocks to read or write
    uint32_t nblocks;
    uint32_t addr;
    void* buf;
} sd_driver_irq_state_t;

/**
 * This structure represents a read or write request by a thread.
 */
typedef struct sd_driver_rw_request {
    // Determines whether this request is a read or a write.
    sd_driver_direction_e direction;

    // number of 512-byte blocks to be transferred in this request
    uint32_t nblocks;

    // SD card addr. Currently, this code only supports SDHC and SDXC size cards (4GB - 2TB), so the
    // card addr should be indicated in blocks. e.g. addr 8 points to byte 4096 (8 * 512).
    uint32_t card_addr;

    // Buffer to read from or write to. The user should have nblocks * 512 bytes allocated at this
    // address.
    void* buf;
} sd_driver_rw_request_t;

/**
 * Every SD command issued by the host has a response from the card. Depending on the command, the
 * responses vary in format and length. In total, the SD Physical Layer Spec defines 6 different
 * response types. This enum represents each of them, different commands may need to behave
 * differently depending on which response type they expect.
 */
typedef enum {
    SD_RESPONSE_TYPE_R1,
    SD_RESPONSE_TYPE_R1B,
    SD_RESPONSE_TYPE_R2,
    SD_RESPONSE_TYPE_R3,
    SD_RESPONSE_TYPE_R6,
    SD_RESPONSE_TYPE_R7,
    SD_RESPONSE_TYPE_NONE,
    SD_RESPONSE_TYPE_UNDEF,
} sd_response_type_e;


/**
 * Add a read / write request to the SD driver's internal data structure for things to read / write.
 * Right now, it's just a queue.
 */
int32_t sd_driver_enqueue_request(const sd_driver_rw_request_t*);

void sd_driver_task(void* param);

#endif
