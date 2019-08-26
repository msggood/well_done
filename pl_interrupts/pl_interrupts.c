#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/delay.h>
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
#include <linux/errno.h>	
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
#include <linux/module.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_iommu.h>
#include <linux/delay.h>
#include <asm/delay.h>



#include "axidma_chrdev.h"



static struct class * axidma_class;
//static void __iomem*	base_address;	
//static resource_size_t  remap_size;  
static int	            irq;
#define DEVICE_NAME "axi_dma"
static volatile int axi_dma_is_open = 0;
static struct fasync_struct *axi_dma_async;

static void __iomem  *addr_base=NULL;
static struct dma_txrx_data *txrx_data;

// A structure that represents a DMA buffer allocation
struct axidma_dma_allocation {
    size_t size;                // Size of the buffer
    void *user_addr;            // User virtual address of the buffer
    void *kern_addr;            // Kernel virtual address of the buffer
    dma_addr_t dma_addr;        // DMA bus address of the buffer
//    struct list_head list;      // List node pointers for allocation list
};


struct dma_txrx_data
{
	struct axidma_dma_allocation tx_dma_alloc;
	unsigned long rx_buf_offset;
//	struct axidma_dma_allocation *rx_dma_alloc;
	struct list_head dmabuf_list;
	struct device	*dev;
	struct completion tx_comp;
	struct completion rx_comp;
	struct completion tx_irq_comp;
	bool wait;
    struct dma_chan *tx_ch;
    struct dma_chan *rx_ch;
    struct dma_async_tx_descriptor *tx_desc;
    struct dma_async_tx_descriptor *rx_desc;
	struct tasklet_struct tasklet;
    unsigned char *src_buf;
    dma_addr_t src_dma_buf;
    unsigned char *dst_buf;
    dma_addr_t dst_dma_buf;
    dma_cookie_t tx_cookie;
    dma_cookie_t rx_cookie;
};


static int axi_dma_open(struct inode *Inode, struct file *File)
{
//	printk("%s\n",__func__);
	axi_dma_is_open = 1;
	return 0;
}
int axi_dma_release (struct inode *inode, struct file *file)
{
	axi_dma_is_open = 0;
	return 0;
}
static ssize_t axi_dma_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}
static ssize_t axi_dma_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}
static int axi_dma_fasync (int fd, struct file *filp, int on)
{
	return fasync_helper (fd, filp, on, &axi_dma_async);
}

static void dma_send_done(void*arg)
{
//    printk("%s\n",__func__);	
	complete(&txrx_data->tx_comp);	
#if 0
	for(i=0;i<3;i++)
	{
	    data = readl_relaxed(addr+8+i*4);
	    printk("data=0x%x\n",data);
	}
#endif	
}

static void dma_recv_done(void*arg)
{
//    printk("%s\n",__func__);	
	complete(&txrx_data->rx_comp);	
//	printk("%s\n,",(char *)(txrx_data->tx_dma_alloc.dma_addr+PAGE_SIZE*2048));
#if 0
	for(i=0;i<3;i++)
	{
	    data = readl_relaxed(addr+8+i*4);
	    printk("data=0x%x\n",data);
	}
#endif	
}

static int test_len=32768;
static long axidma_transfer(struct axidma_transaction *trans)
{


	unsigned long timeout, time_remain;
	enum dma_status status;
	int rc;
	struct scatterlist sg[3];
#if 1	
	sg_init_one(sg, txrx_data->tx_dma_alloc.kern_addr+trans->buf_offset,trans->buf_len);
#else	
	sg_init_table(sg, 3);
	sg_set_buf(&sg[0], txrx_data->tx_dma_alloc.kern_addr+trans->buf_offset, 8);
	sg_set_buf(&sg[1], txrx_data->tx_dma_alloc.kern_addr+trans->buf_offset+trans->buf_len+8, 8);
	sg_set_buf(&sg[2], txrx_data->tx_dma_alloc.kern_addr+trans->buf_offset+trans->buf_len+16, trans->buf_len-16);
#endif	
	dma_map_sg(txrx_data->dev, sg, 1, DMA_TO_DEVICE);
	txrx_data->tx_desc=dmaengine_prep_slave_sg(txrx_data->tx_ch,sg,1,DMA_MEM_TO_DEV,DMA_PREP_INTERRUPT|DMA_CTRL_ACK);
    if(!txrx_data->tx_desc)
    {
        printk("dmaengine_prep_slave_sg error\n");
        return -1;
    }
    txrx_data->tx_desc->callback = dma_send_done;
    txrx_data->tx_desc->callback_param = txrx_data;
    txrx_data->tx_cookie=dmaengine_submit(txrx_data->tx_desc);

#if 0	
	timeout = msecs_to_jiffies(2000);
	time_remain = wait_for_completion_timeout(&txrx_data->tx_irq_comp, timeout);
	reinit_completion(&txrx_data->tx_irq_comp);
	if(time_remain==0){
		rc = -ETIME;
		goto stop_dma;
	}
#endif

    dma_async_issue_pending(txrx_data->tx_ch);
    if (txrx_data->wait) {
        timeout = msecs_to_jiffies(10000);
        time_remain = wait_for_completion_timeout(&txrx_data->tx_comp, timeout);
        //wait_for_completion(&txrx_data->tx_comp);
        status = dma_async_is_tx_complete(txrx_data->tx_ch, txrx_data->tx_cookie, NULL, NULL);

        if (time_remain == 0) {
            printk("transaction timed out.\n");
            rc = -ETIME;
		
            goto stop_dma;
        } else if (status != DMA_COMPLETE) {
            printk("transaction did not succceed. Status is %d.\n",status);
            rc = -EBUSY;
            goto stop_dma;
        }
//		printk("tx done!\n");
    }

	return 0;
stop_dma:
//	dmaengine_terminate_all(txrx_data->tx_ch);
	return rc;
	
}


static long axidma_receive(struct axidma_transaction *trans)
{

	struct scatterlist sg;
	unsigned long timeout, time_remain;
	enum dma_status status;
	int rc;
#if 0
	txrx_data->dst_buf=kzalloc(test_len, GFP_KERNEL);
#endif

//	txrx_data->src_buf=kmalloc(PAGE_SIZE, GFP_KERNEL);

	/* Queue the DMA data transfer. */
//	printk("%s,trans->buf:%x,trans->buf_len:%x\n",__func__,trans->buf,trans->buf_len);
#if 1
//	sg_init_one(&sg, txrx_data->rx_dma_alloc->kern_addr,trans->buf_len);
	sg_init_table(&sg, 1);
	sg_set_buf(&sg, (void*)((unsigned int)txrx_data->tx_dma_alloc.kern_addr+txrx_data->rx_buf_offset+trans->buf_offset), trans->buf_len);
//	sg_set_buf(&sg, txrx_data->dst_buf, trans->buf_len);
	dma_map_sg(txrx_data->dev, &sg, 1, DMA_FROM_DEVICE);
#else	
	sg_init_table(&sg, 1);
	sg_dma_address(&sg) = txrx_data->tx_dma_alloc.dma_addr+txrx_data->rx_buf_offset+trans->buf_offset;
	sg_dma_len(&sg) = trans->buf_len;
#endif
#if 0
	printk("sg_list.length=0x%x\n",sg.length);
	printk("sg_list.offset=0x%x\n",sg.offset);
	printk("sg_list.dma_address=0x%x\n",sg.dma_address);
	printk("sg_list.page_link=%lu\n",sg.page_link);
#endif
	txrx_data->rx_desc=dmaengine_prep_slave_sg(txrx_data->rx_ch,&sg,1,DMA_DEV_TO_MEM,DMA_PREP_INTERRUPT|DMA_CTRL_ACK);
    if(!txrx_data->rx_desc)
    {
        printk("dmaengine_prep_slave_sg error\n");
        return -1;
    }
    txrx_data->rx_desc->callback = dma_recv_done;
    txrx_data->rx_desc->callback_param = txrx_data;
    txrx_data->rx_cookie=dmaengine_submit(txrx_data->rx_desc);
    dma_async_issue_pending(txrx_data->rx_ch);
    if (txrx_data->wait) {
        timeout = msecs_to_jiffies(10000);
        time_remain = wait_for_completion_timeout(&txrx_data->rx_comp, timeout);
//        wait_for_completion(&txrx_data->rx_comp);
        status = dma_async_is_tx_complete(txrx_data->rx_ch, txrx_data->rx_cookie, NULL, NULL);

        if (time_remain == 0) {
            printk("receive timed out.\n");
            rc = -ETIME;
            goto stop_dma;
        } else if (status != DMA_COMPLETE) {
            printk("transaction did not succceed. Status is %d.\n",status);
            rc = -EBUSY;
            goto stop_dma;
        }
#if 0
		printk("trans->buf_offset:%d\n",trans->buf_offset);
		int i;
		for(i=0;i<32;i=i+4)
		{
			printk("%x,",*(int*)((unsigned long)txrx_data->tx_dma_alloc.kern_addr+txrx_data->rx_buf_offset+trans->buf_offset+i));
//			printk("%x,",*(int*)((unsigned long)txrx_data->tx_dma_alloc.kern_addr+txrx_data->rx_buf_offset+i));
		}			
		printk("\n");
#endif		
//		printk("rx done!\n");		
//		printk("%s\n",(char*)(txrx_data->tx_dma_alloc.kern_addr));
//		printk("%s\n",txrx_data->dst_buf);
//		printk("%s\n",(char*)(txrx_data->tx_dma_alloc.kern_addr +txrx_data->rx_buf_offset));
		return 0;
    }

	return 0;
stop_dma:
//	dmaengine_terminate_all(txrx_data->rx_ch);
	return rc;
	
}


static long axi_dma_ioctl(struct file *f, unsigned int cmd,unsigned long arg)
{
	long rc=0;
	void *__user arg_ptr;
    struct axidma_transaction trans;
	struct gpio_data data;
	
    arg_ptr = (void __user *)arg;
	
    switch (cmd) {
        case AXIDMA_DMA_READ:
			if (copy_from_user(&trans, arg_ptr, sizeof(trans)) != 0) {
				printk("Unable to copy transfer info from userspace for "
						   "AXIDMA_DMA_READ.\n");
				return -EFAULT;
			}
			rc = axidma_receive(&trans);

			break;
        case AXIDMA_DMA_WRITE:
            if (copy_from_user(&trans, arg_ptr, sizeof(trans)) != 0) {
                printk("Unable to copy transfer info from userspace for "
                           "AXIDMA_DMA_WRITE.\n");
                return -EFAULT;
            }
            rc = axidma_transfer(&trans);
			
			break;
			
		case AXIDMA_DMA_READCON:
			
			break;
			
		case AXIDMA_GPIO_READ:
			break;

		case AXIDMA_GPIO_WRITE:			
			if (copy_from_user(&data, arg_ptr, sizeof(data)) != 0) {
				printk("Unable to copy transfer info from userspace for "
						   "AXIDMA_GPIO_WRITE.\n");
				return -EFAULT;
			}
			rc = gpio_direction_output(data.num,data.value);
			break;
			
		default:
			break;
   	}
	return rc;
}

static int axi_dma_mmap(struct file *file, struct vm_area_struct *vma)
{
	int rc;
	unsigned long pfn_start;
	size_t size;
	unsigned long virt_start;
    struct axidma_dma_allocation *dma_alloc;
	
	dma_alloc=&txrx_data->tx_dma_alloc;
	pfn_start=(virt_to_phys(dma_alloc->kern_addr)>>PAGE_SHIFT)+vma->vm_pgoff;
	if(vma->vm_pgoff)
		txrx_data->rx_buf_offset=vma->vm_pgoff<<PAGE_SHIFT;
//	printk("pnf_start=%d\n",pfn_start);
//    vma->vm_flags |= VM_DONTCOPY;	
	virt_start=(unsigned long)dma_alloc->kern_addr+(vma->vm_pgoff<<PAGE_SHIFT);

	size=vma->vm_end-vma->vm_start;
	rc=remap_pfn_range(vma, vma->vm_start, pfn_start,size, vma->vm_page_prot);
	if(rc){
		printk("remap_pfn_range error\n");
		return rc;
	}
//	printk("map 0x%x to 0x%x, size: 0x%x\n", vma->vm_start,virt_start,size);                                            
    return rc;	
}



static struct file_operations axi_dma_fops = {	
	.owner  		= THIS_MODULE,
	.unlocked_ioctl	= axi_dma_ioctl,
	.mmap            =axi_dma_mmap,
	.open 			= axi_dma_open,
	.read 			= axi_dma_read, 
	.write 			= axi_dma_write,
	.fasync		 	= axi_dma_fasync,
	.release		= axi_dma_release,
};


#if 0
static bool filter(struct dma_chan *chan, void *params)
{
    return(!strcmp(dma_chan_name(chan), params));
}
#endif

static int dma_init(void)
{

	struct axidma_dma_allocation *dma_alloc;

	txrx_data->tx_ch=dma_request_slave_channel(txrx_data->dev,"tx_channel");
	if(!txrx_data->tx_ch)
	{
		printk(" dma_request_slave_channel tx_channel error\n");
		return -1;
	}

	txrx_data->rx_ch=dma_request_slave_channel(txrx_data->dev,"rx_channel");
	if(!txrx_data->rx_ch)
	{
		printk(" dma_request_slave_channel rx_channel error\n");
		return -1;
	}

	dma_alloc=&txrx_data->tx_dma_alloc;
	dma_alloc->size=PAGE_SIZE*4096;
	dma_alloc->kern_addr = dma_alloc_coherent(txrx_data->dev, dma_alloc->size,&dma_alloc->dma_addr, GFP_KERNEL);
//	printk("dma_alloc->kern_addr:0x%x\n",dma_alloc->kern_addr);
//	dma_alloc->kern_addr = kzalloc(dma_alloc->size, GFP_KERNEL);
	if(!dma_alloc->kern_addr)
	{
		printk(" dma_alloc_coherent error\n");
		return -1;
	}

#if 0	
	axidma_transfer(NULL);
	axidma_receive(NULL);
	mdelay(1000);
	if(strncmp(txrx_data->src_buf,txrx_data->dst_buf,16384)==0)
	{
		printk("same!!!!!!!!!!\n");
	}
	else
	{
		printk("diff!!!!!!!!!!\n");
	}

	txrx_data->src_buf=dma_alloc_coherent(dev,PAGE_SIZE,&txrx_data->src_dma_buf,GFP_KERNEL);
	if(!txrx_data->src_buf)
	{
		printk("src_buf alloc error\n");
		return -1;
	}
#endif


    return 0;
}

static int dma_test(void)
{
	struct scatterlist sg;

	txrx_data->src_buf=kmalloc(PAGE_SIZE, GFP_KERNEL);

	memset(txrx_data->src_buf,0x00,64);


	/* Queue the DMA data transfer. */
	sg_init_one(&sg, txrx_data->src_buf, 64);
	dma_map_sg(txrx_data->dev, &sg, 1, DMA_TO_DEVICE);

	txrx_data->tx_desc=dmaengine_prep_slave_sg(txrx_data->tx_ch,&sg,1,DMA_MEM_TO_DEV,DMA_PREP_INTERRUPT|DMA_CTRL_ACK);
    if(!txrx_data->tx_desc)
    {
        printk("dmaengine_prep_dma_cyclic error\n");
        return -1;
    }
    txrx_data->tx_desc->callback = dma_send_done;
    txrx_data->tx_desc->callback_param = txrx_data;
    txrx_data->tx_cookie=dmaengine_submit(txrx_data->tx_desc);
    dma_async_issue_pending(txrx_data->tx_ch);
	return 0;
}

static void dma_do_tasklet(unsigned long data)
{
//	printk("%s\n",__func__);
}


static irqreturn_t irq_interrupt(int irq, void *dev_id)
{
//	struct dma_txrx_data *p=(struct dma_txrx_data *)dev_id;
//    printk("irq_interrupt:%d\n",irq);
//	complete(&txrx_data->tx_irq_comp);
//	tasklet_schedule(&p->tasklet);

    return IRQ_HANDLED;
}

static char *chrdev_name = DEVICE_NAME;


static dev_t dev_id;	   
static struct device *axidma_dev;
static struct cdev axidma_cdev;

static int axi_dma_probe(struct platform_device *pdev)
{
    int	err;
//    struct resource *res;

    alloc_chrdev_region(&dev_id, 0, 32768, chrdev_name);  

    axidma_class=class_create(THIS_MODULE, chrdev_name);  
	axidma_dev=device_create(axidma_class, NULL, dev_id, NULL, "axidma0");
    if (IS_ERR(axidma_dev)) {
		goto err_cdev_del;
    }
	cdev_init(&axidma_cdev , &axi_dma_fops);
	err = cdev_add(&axidma_cdev, dev_id, 1);
    if (err < 0) {
        printk("Unable to add a character device.\n");
        goto err_irq_fail;
    }

    txrx_data=devm_kzalloc(&pdev->dev, sizeof(struct dma_txrx_data), GFP_KERNEL);
    INIT_LIST_HEAD(&txrx_data->dmabuf_list);
	txrx_data->dev=&pdev->dev;

    irq = platform_get_irq(pdev,0);
    if (irq <= 0){
		printk("request irq error\n");
   	}
#if 0	
    err = request_threaded_irq(irq, NULL,irq_interrupt,
            IRQF_TRIGGER_RISING | IRQF_ONESHOT,
            DEVICE_NAME, txrx_data);				   
    if (err) {
        printk(KERN_ALERT "irq_probe irq	error=%d\n", err);
        goto err_irq_fail;
    }
#endif

#if 0		
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    addr_base = devm_ioremap_resource(&pdev->dev, res);
#endif	
#if 0	
#define FPGA_VERSION_OFFSET    0x00    
#define DMA_LENGTH_OFFSET      0x04
#define INTR_GAP_OFFSET        0x08
#define TX_10M_GAP_OFFSET      0X0c

	writel_relaxed(0x20,addr_base+DMA_LENGTH_OFFSET);
	writel_relaxed(0x5F5E100,addr_base+INTR_GAP_OFFSET);
	writel_relaxed(0x61A800,addr_base+TX_10M_GAP_OFFSET);  //0x61A800=> 0.5s,0x186A000;=>2s
#endif	
#if 0
    err = gpio_request(913,"mio7");
    if (err) {
        printk("can't request mio7\n");
        goto err_request_gpio_fail;
    }
#endif	
	init_completion(&txrx_data->tx_comp);
	init_completion(&txrx_data->rx_comp);
	init_completion(&txrx_data->tx_irq_comp);
	txrx_data->wait=true;
//	txrx_data->wait=false;
    dma_init();
	tasklet_init(&txrx_data->tasklet, dma_do_tasklet,
			(unsigned long)txrx_data);

//	dma_test();
    return 0;
err_request_gpio_fail:
err_irq_fail:	
    device_destroy(axidma_class, dev_id);
err_cdev_del:
	cdev_del(&axidma_cdev);
	class_destroy(axidma_class);
	unregister_chrdev_region(dev_id, 1);
		return -EFAULT;
    return -ENOMEM;
}
static int axi_dma_remove(struct platform_device *pdev)
{
	struct axidma_dma_allocation *dma_alloc;
	cdev_del(&axidma_cdev);
    device_destroy(axidma_class, dev_id);	
    class_destroy(axidma_class);
 	unregister_chrdev_region(dev_id, 1);
//    free_irq(irq, NULL);
	dma_alloc=&txrx_data->tx_dma_alloc;
	dma_free_coherent(txrx_data->dev, dma_alloc->size,dma_alloc->kern_addr,dma_alloc->dma_addr);
	kfree(txrx_data);
    gpio_free(913);
	dma_release_channel(txrx_data->tx_ch);
	dma_release_channel(txrx_data->rx_ch);
//    printk("irq = %d\n", irq);
    return 0;
}
static int axi_dma_suspend(struct device *dev)
{
    return 0;
}
static int axi_dma_resume(struct device *dev)
{
    return 0;
}
static const struct dev_pm_ops axi_dma_pm_ops = {
    .suspend = axi_dma_suspend,
    .resume  = axi_dma_resume,
};
//MODULE_DEVICE_TABLE(platform, axi_dma_driver_ids);
static const struct of_device_id axi_dma_of_match[] = {
    {.compatible = "pl-0,axi_dma" },
    { }
};
MODULE_DEVICE_TABLE(of, axi_dma_of_match);
static struct platform_driver axi_dma_driver = {
    .probe = axi_dma_probe,
    .remove	= axi_dma_remove,
    .driver = {
        .owner   		= THIS_MODULE,
        .name	 		= "axi_dma@0",
        .pm    			= &axi_dma_pm_ops,
        .of_match_table	= axi_dma_of_match,		
    },
};
module_platform_driver(axi_dma_driver);
MODULE_LICENSE("GPL v2");
