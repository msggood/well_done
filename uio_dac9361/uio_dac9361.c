#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/uio_driver.h>

static irqreturn_t uio_handler(int irq, struct uio_info *dev_info)
{
	return IRQ_HANDLED;
}

static int uio_probe(struct platform_device *pdev)
{
    struct uio_info *info;
	struct resource *res;
	int ret;
	int gpio;

	info = devm_kzalloc(&pdev->dev, (sizeof(struct uio_info)), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	info->mem[0].name = "ad9361";
	info->mem[0].addr = res->start;
	info->mem[0].size = resource_size(res);
	info->mem[0].memtype = UIO_MEM_PHYS;


	info->name = "ad9361";
	info->version = "1.0";
	gpio=906+50;
	ret=gpio_request(gpio, "btn9");
	info->irq=gpio_to_irq(gpio);
	printk("ret=%d,irq=%ld\n",ret,info->irq);
	info->handler=uio_handler;
	
	ret = uio_register_device(&pdev->dev, info);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register to UIO\n");
	}

    platform_set_drvdata(pdev, info);
    
	return ret;
}

static int uio_remove(struct platform_device *pdev)
{
    struct uio_info *info = platform_get_drvdata(pdev);
	uio_unregister_device(info);
    return 0; 
}

static const struct of_device_id axi_dma_of_match[] = {
    {.compatible = "xlnx,axi-ad9361-1.0" },
    { }
};
MODULE_DEVICE_TABLE(of, axi_dma_of_match);
static struct platform_driver axi_dma_driver = {
    .probe = uio_probe,
    .remove	= uio_remove,
    .driver = {
        .owner   		= THIS_MODULE,
        .name           = "xlnx,axi-ad9361-1.0",
        .of_match_table	= axi_dma_of_match,
    },
};
module_platform_driver(axi_dma_driver);
MODULE_LICENSE("GPL v2");
