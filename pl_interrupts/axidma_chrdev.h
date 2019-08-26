#include <asm/ioctl.h>              // IOCTL macros


struct gpio_data {
	int num;
	int value;
};


struct axidma_transaction {
    bool wait;                      // Indicates if the call is blocking
    void *buf;                      // The buffer used for the transaction
    size_t buf_offset;
    size_t buf_len;                 // The length of the buffer

};

struct axidma_inout_transaction {
    bool wait;                      // Indicates if the call is blocking
    int tx_channel_id;              // The id of the transmit DMA channel
    void *tx_buf;                   // The buffer containing the data to send
    size_t tx_buf_len;              // The length of the transmit buffer

    int rx_channel_id;              // The id of the receive DMA channel
    void *rx_buf;                   // The buffer to place the data in
    size_t rx_buf_len;              // The length of the receive buffer
};

#define AXIDMA_IOCTL_MAGIC              'W'
#define AXIDMA_DMA_READ                 _IOR(AXIDMA_IOCTL_MAGIC, 0, struct axidma_transaction)
#define AXIDMA_DMA_WRITE                _IOW(AXIDMA_IOCTL_MAGIC, 1, struct axidma_transaction)
#define AXIDMA_DMA_READCON            _IOWR(AXIDMA_IOCTL_MAGIC, 2, struct axidma_inout_transaction)
#define AXIDMA_GPIO_READ                _IOR(AXIDMA_IOCTL_MAGIC, 3, unsigned long)
#define AXIDMA_GPIO_WRITE               _IOW(AXIDMA_IOCTL_MAGIC, 4, unsigned long)
