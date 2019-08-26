#include<linux/module.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
struct dma_slave_config test_config=
{   
    .src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES,
    .src_maxburst = 1,
    .dst_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES,
    .dst_maxburst = 1,
    .device_fc = false,
    .direction= DMA_MEM_TO_MEM,
};
static struct dma_chan *test_ch=NULL;
static void dma_buffer_done(void*arg)
{
    printk("dma_buffer_done\n");
    dmaengine_terminate_all(test_ch);
}
static int test_init(void)
{
    struct dma_async_tx_descriptor *tx_desc;
    unsigned char *src_buf;
    dma_addr_t src_dma_buf;
    unsigned char *dst_buf;
    dma_addr_t dst_dma_buf;
    dma_cookie_t cookie;
    struct class * class;
    struct device *dev;
    class=class_create(THIS_MODULE, "test");  
    dev=device_create(class, NULL, MKDEV(22, 22), NULL, "subname");

    test_ch =dma_request_slave_channel(dev, "test ch");
    if(!test_ch)
    {
        printk(" dma_request_slave_channel error\n");
    }
    src_buf=dma_alloc_coherent(dev,64,&src_dma_buf,GFP_KERNEL);
    if(!src_buf)
    {
        printk("src_buf alloc error\n");
    }
    return 0;
    dst_buf=dma_alloc_coherent(dev,64,&dst_dma_buf,GFP_KERNEL);
    if(!dst_buf)
    {
        printk("dst_buf alloc error\n");
    }
    test_config.src_addr=src_dma_buf;
    test_config.dst_addr=dst_dma_buf;
    if(dmaengine_slave_config(test_ch,&test_config))
    {
        printk("dmaengine_slave_config error\n");
    }
    tx_desc=dmaengine_prep_dma_cyclic(test_ch,dst_dma_buf,32,16,DMA_MEM_TO_MEM,DMA_PREP_INTERRUPT);
    if(!tx_desc)
    {
        printk("dmaengine_prep_dma_cyclic error\n");
    }
    tx_desc->callback = dma_buffer_done;
    tx_desc->callback_param = NULL;
    cookie=dmaengine_submit(tx_desc);
    dma_async_issue_pending(test_ch);
    return 0;
}

static void test_exit(void)
{
}
module_init(test_init);
module_exit(test_exit);
MODULE_LICENSE("GPL");
