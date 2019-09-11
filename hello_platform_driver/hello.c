#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static int axi_dma_probe(struct platform_device *pdev)
{
    return 0;
}

static int axi_dma_remove(struct platform_device *pdev)
{
        return 0;
}

static const struct of_device_id axi_dma_of_match[] = {
    {.compatible = "hello", },
    { }
};
MODULE_DEVICE_TABLE(of, axi_dma_of_match);
static struct platform_driver axi_dma_driver = {
    .probe = axi_dma_probe,
    .remove	= axi_dma_remove,
    .driver = {
        .owner   		= THIS_MODULE,
        .name           = "hello world",
        .of_match_table	= axi_dma_of_match,
    },
};
module_platform_driver(axi_dma_driver);
MODULE_LICENSE("GPL v2");
