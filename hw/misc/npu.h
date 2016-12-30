
#define MAX_DATA 1024	/* 1024 bytes to be sent/received */

typedef enum NpuOperations{
	GUEST_TO_HOST = 1,	/* Msg comes from guest to host */
	HOST_TO_GUEST = 2	/* Msg is sent from host to guest */
} NpuOperations;

typedef struct NpuData {
	/* TODO: check if you need NpuOperations */
	uint32_t magic;		/* hex value : 47806 */
	uint32_t flags;	/* 1: guest->host, 2: host-> guest */
	uint32_t data_size;
	uint32_t host_req;   /* request from host to guest, 1: write, 2: read */
	uint32_t guest_op;  /* 1: write, 2: read, operation that happened on guest */
	uint64_t address; /* address on which operation was made or shd be made */
	uint64_t temp_data; /* testing */
//	char data[MAX_DATA];
	uint32_t vector; /* used in case of sending interrupt */
	uint8_t data_test[MAX_DATA];
} NpuData;
