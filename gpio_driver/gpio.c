#include <linux/init.h>
#include <linux/types.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/major.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio.h>


static struct class *gpio_class;

static int gpio_open(struct inode *inode,struct file *file)
{
    printk("gpio_open\n");

    return 0;    
}
static ssize_t gpio_read(struct file *file, char __user *user_buf, size_t size, loff_t *ppos)
{
    unsigned char data[3]="100";
    printk("gpio_read\n");
    copy_to_user(user_buf,&data,size);
    return 0;
}
static ssize_t gpio_write(struct file *filp, const char __user *buf,	size_t count, loff_t *f_pos)
{
    unsigned char gpio_buf[4];
//    printk("kernel write\n");
    copy_from_user(gpio_buf,buf,3);
    gpio_buf[3]='\0';
    printk("kernel receive gpio_buf=%s\n",gpio_buf);
    return 0;
}

static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch(cmd)
    {
        case 1:
            printk("1 args's %ld\n",arg);
            break;
        case 2:            
            printk("2 args's %ld\n",arg);
            break;
        default:
            printk("cmd:%x,arg=%ld",cmd,arg);        
    }
    
    return 0;
}

static const struct file_operations gpio_fops =
{                   
    .owner = THIS_MODULE,
    .open  = gpio_open,
    .read  = gpio_read,
    .write = gpio_write,
    .unlocked_ioctl = gpio_ioctl,
};
static struct cdev *gpio_cdev;
struct device *class_dev;
static irqreturn_t func_intr(int irq, void *data)
{
    printk("irq:%d\n",irq);
	return IRQ_NONE;
}

static int mio=906+7;
static int gpio_setup(void)
{
    int ret;
    int irq;
    irq=gpio_to_irq(mio);
    printk("gpio to irq=%d\n",irq);
    ret = gpio_request(mio,"mio7" );
	if (ret) {
        printk("can't request mio7\n");
        return -1;
	}
    gpio_direction_input(mio);

    ret = request_irq(gpio_to_irq(mio), func_intr, IRQF_SHARED,"zynq-gpio", "mdio7");
	if (ret) {
        printk("can't request irq 61 ret=%d\n",ret);
        return -1;
	}

    ret = request_irq(62, func_intr, IRQF_SHARED|IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"irq62", NULL);
	if (ret) {
        printk("can't request irq 62 ret=%d\n",ret);
        return -1;
	}

    return 0;
}
static int __init gpio_init(void)
{

    dev_t hell_devid;
    alloc_chrdev_region(&hell_devid, 0, 32768, "gpio_test");
#if 1
    gpio_cdev = cdev_alloc();
    cdev_init(gpio_cdev, &gpio_fops);
    gpio_cdev->owner = THIS_MODULE;
    cdev_add(gpio_cdev, hell_devid, 1); 
#else    
    register_chrdev(231, "gpio", &gpio_fops);
#endif
    gpio_class=class_create(THIS_MODULE, "gpio_test");
    class_dev=device_create(gpio_class, NULL, hell_devid, NULL, "mio7");
    gpio_setup();
    return 0;
}
static void __exit gpio_exit(void)
{
    printk("gpio_exit\n");    
    free_irq(gpio_to_irq(mio),NULL);
//    unregister_chrdev(231, "gpio");
//    cdev_del(gpio_cdev);
    device_unregister(class_dev);
    class_destroy(gpio_class);
}
module_init(gpio_init);
module_exit(gpio_exit);
MODULE_LICENSE("GPL");
