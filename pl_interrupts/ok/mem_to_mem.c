#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/dma-buf.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/dmaengine.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/pagemap.h>
#include <linux/errno.h>	/* error codes */
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/notifier.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include<linux/module.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

//
static char 			devname[16];
static int 				major;
static int             	mijor;
static struct class*	cls;
//static void __iomem*	base_address;	
//static resource_size_t  remap_size;  
static int	            irq;
static struct device*	dev;           
//
#define DEVICE_NAME "irq_drv"
static volatile int irq_is_open = 0;
static struct fasync_struct *irq_async;
static int irq_drv_open(struct inode *Inode, struct file *File)
{
	irq_is_open = 1;
	return 0;
}
int irq_drv_release (struct inode *inode, struct file *file)
{
	irq_is_open = 0;
	return 0;
}
static ssize_t irq_drv_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}
static ssize_t irq_drv_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}
static int irq_drv_fasync (int fd, struct file *filp, int on)
{
	return fasync_helper (fd, filp, on, &irq_async);
}
static struct file_operations irq_fops = {	
	.owner  		= THIS_MODULE,
	.open 			= irq_drv_open,
	.read 			= irq_drv_read, 
	.write 			= irq_drv_write,
	.fasync		 	= irq_drv_fasync,
	.release		= irq_drv_release,
};


static void __iomem  *addr=NULL;
static unsigned int count=0;
struct dma_slave_config test_config=
{   
    .src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES,
    .src_maxburst = 1,
    .dst_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES,
    .dst_maxburst = 1,
    .device_fc = false,
//    .direction= DMA_MEM_TO_MEM,
};

struct dma_test_data
{
    struct dma_chan *test_ch;
    struct dma_async_tx_descriptor *tx_desc;
    unsigned char *src_buf;
    dma_addr_t src_dma_buf;
    unsigned char *dst_buf;
    dma_addr_t dst_dma_buf;
    dma_cookie_t cookie;
};

static struct dma_test_data *test_data;

static void dma_buffer_done(void*arg)
{
    int i;
    struct dma_test_data *dma_data=arg;
    dma_data->tx_desc=dmaengine_prep_dma_memcpy(dma_data->test_ch,dma_data->dst_dma_buf,dma_data->src_dma_buf,32,DMA_PREP_INTERRUPT);
    if(!dma_data->tx_desc)
    {
        printk("dmaengine_prep_dma_cyclic error\n");
        return ;
    }
    dma_data->tx_desc->callback = dma_buffer_done;
    dma_data->tx_desc->callback_param = dma_data;
    dmaengine_submit(dma_data->tx_desc);
    dma_async_issue_pending(dma_data->test_ch);
    printk("dma_buffer_done\n");
    printk("src:\n");
    for(i=0;i<38;i++)
        printk("%x,",test_data->src_buf[i]);
    printk("dst:\n");

    for(i=0;i<38;i++)
        printk("%x,",test_data->dst_buf[i]);
    //    dmaengine_terminate_all(test_ch);
}

static bool filter(struct dma_chan *chan, void *params)
{
    return(!strcmp(dma_chan_name(chan), params));
}

static int dma_init(struct device *dev,struct dma_test_data *dma_data,dma_addr_t dma_addr,void __iomem *vaddr)
{
    dma_cap_mask_t mask;
    dma_cap_zero(mask);
    dma_cap_set(DMA_MEMCPY, mask);
    dma_data->test_ch=dma_request_channel(mask,filter, "dma0chan6");
    if(!dma_data->test_ch)
    {
        printk(" dma_request_slave_channel error\n");
        return -1;
    }
    dma_data->src_buf=dma_alloc_coherent(dev,PAGE_SIZE,&dma_data->src_dma_buf,GFP_KERNEL);
    if(!dma_data->src_buf)
    {
        printk("src_buf alloc error\n");
        return -1;
    }
    memset(dma_data->src_buf,0x55,38);
    dma_data->dst_buf=dma_alloc_coherent(dev,PAGE_SIZE,&dma_data->dst_dma_buf,GFP_KERNEL);
    if(!dma_data->dst_buf)
    {
        printk("dst_buf alloc error\n");
        return -1;
    }
    memset(dma_data->dst_buf,0xff,38);
    //   test_config.src_addr=dma_data->src_dma_buf;
    //    test_config.dst_addr=dma_data->dst_dma_buf;
#if 0
    if(dmaengine_slave_config(dma_data->test_ch,&test_config))
    {
        printk("dmaengine_slave_config error\n");
        return -1;
    }
#endif

    dma_data->tx_desc=dmaengine_prep_dma_memcpy(dma_data->test_ch,dma_data->dst_dma_buf,dma_data->src_dma_buf,32,DMA_PREP_INTERRUPT);
    if(!dma_data->tx_desc)
    {
        printk("dmaengine_prep_dma_cyclic error\n");
        return -1;
    }
    dma_data->tx_desc->callback = dma_buffer_done;
    dma_data->tx_desc->callback_param = dma_data;
    // dma_data->cookie=dmaengine_submit(dma_data->tx_desc);
    dmaengine_submit(dma_data->tx_desc);
    dma_async_issue_pending(dma_data->test_ch);
    return 0;
}



static irqreturn_t irq_interrupt(int irq, void *dev_id)
{
    unsigned int data;
    count++;
    printk("irq-1= %d,count=%d", irq,count);
    writel_relaxed(count, addr);
    gpio_direction_output(913,count%2);
    data = readl_relaxed(addr);
    printk("data=0x%x\n",data);
    if(irq_is_open)
    {
        kill_fasync (&irq_async, SIGIO, POLL_IN);
    }
    return IRQ_HANDLED;
}
static int irq_probe(struct platform_device *pdev)
{
    int					err;
    struct resource *res;
#if 0
    struct device *tmp_dev;

    memset(devname,0,16);
    strcpy(devname, DEVICE_NAME);
    major = register_chrdev(0, devname, &irq_fops);
    cls = class_create(THIS_MODULE, devname);
    mijor = 1;
    tmp_dev = device_create(cls, &pdev->dev, MKDEV(major, mijor), NULL, devname);
    if (IS_ERR(tmp_dev)) {
        class_destroy(cls);
        unregister_chrdev(major, devname);
        return 0;
    }
#endif

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    printk("res->start=0x%x,res->end:%x\n",res->start,res->end);

    addr = devm_ioremap_resource(&pdev->dev, res);
    test_data=devm_kzalloc(&pdev->dev, sizeof(struct dma_test_data), GFP_KERNEL);
    dma_init(&pdev->dev,test_data,res->start,addr);
    return 0;
    irq = platform_get_irq(pdev,1);
    irq = gpio_to_irq(956);
    if (irq <= 0)
        return -ENXIO;
    dev = &pdev->dev;
    err = request_threaded_irq(irq, NULL,
            irq_interrupt,
            IRQF_TRIGGER_RISING | IRQF_ONESHOT,
            devname, NULL);				   
    if (err) {
        printk(KERN_ALERT "irq_probe irq	error=%d\n", err);
        goto fail;
    }
    else
    {
        printk("irq = %d\n", irq);
        printk("devname = %s\n", devname);
    }

    if (IS_ERR(addr))
        return PTR_ERR(addr);

    //    addr = ioremap(VERSION_ADDR, 20);
    //保存dev
    //platform_set_drvdata(pdev, &xxx);	
    err = gpio_request(913,"mio7");
    if (err) {
        printk("can't request mio7\n");
        return -1;
    }

    return 0;
fail:
    free_irq(irq, NULL);
    device_destroy(cls, MKDEV(major, mijor));	
    class_destroy(cls);
    unregister_chrdev(major, devname);
    return -ENOMEM;
}
static int irq_remove(struct platform_device *pdev)
{
    device_destroy(cls, MKDEV(major, mijor));	
    class_destroy(cls);
    unregister_chrdev(major, devname);
    free_irq(irq, NULL);
    gpio_free(913);
    printk("irq = %d\n", irq);
    return 0;
}
static int irq_suspend(struct device *dev)
{
    return 0;
}
static int irq_resume(struct device *dev)
{
    return 0;
}
static const struct dev_pm_ops irq_pm_ops = {
    .suspend = irq_suspend,
    .resume  = irq_resume,
};
//MODULE_DEVICE_TABLE(platform, irq_driver_ids);
static const struct of_device_id irq_of_match[] = {
    {.compatible = "pl-0,irq" },
    { }
};
MODULE_DEVICE_TABLE(of, irq_of_match);
static struct platform_driver irq_driver = {
    .probe = irq_probe,
    .remove	= irq_remove,
    .driver = {
        .owner   		= THIS_MODULE,
        .name	 		= "irq@0",
        .pm    			= &irq_pm_ops,
        .of_match_table	= irq_of_match,		
    },
};
module_platform_driver(irq_driver);
MODULE_LICENSE("GPL v2");
