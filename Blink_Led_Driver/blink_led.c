#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/gpio.h>

/**
 * @brief This is info of module
 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("devauto devauto.id.vn");
MODULE_DESCRIPTION("A basic kernel module");

#define NPAGES  1

uint32_t __iomem *gpio0_ptr;
#define GPIO0_30                    (1 << 30) 
#define GPIO0_BASE_ADDRESS          0x044E07000
#define GPIO0_END_ADDRESS           0x044E07FFF
#define SIZE_OF_GPIO0               (0x044E07FFF - 0x044E07000)
#define GPIO_OE_OFFSET              0x0134
#define GPIO_DATAOUT_OFFSET         0x013C
#define GPIO_SETDATAOUT_OFFSET      0x0194
#define GPIO_CLEARDATAOUT_OFFSET    0x0190

struct char_device {
    int length;
    dev_t cdev_number;
    struct class *my_class;
    struct cdev m_cdev;
    char *kernel_buffer;
} m_dev;

static int      __init m_module_init(void);
static void     __exit m_module_exit(void);
static int      driver_open(struct inode *inode, struct file *file);
static ssize_t  driver_read(struct file *file, char __user *user_buffer, size_t length, loff_t *offset);
static ssize_t  driver_write(struct file *file, const char __user *user_buffer, size_t length, loff_t *offset);
static int      driver_release(struct inode *inode, struct file *file);
// static int     led_control(int state);

static const struct file_operations fops = {
    .owner      = THIS_MODULE,
    .open       = driver_open,
    .read       = driver_read,
    .write      = driver_write,
    .release    = driver_release
};


static int driver_open(struct inode *inode, struct file *file) {
    printk(KERN_INFO "Open function called...\n");
    return 0;
}

static ssize_t driver_read(struct file *file, char __user *user_buffer, size_t length, loff_t *offset) {
    size_t byte_read = 0;
    printk(KERN_INFO "read function called...\n");
    byte_read = (m_dev.length - *offset <= length) ? (m_dev.length - *offset) : length;
    if (copy_to_user(user_buffer, m_dev.kernel_buffer, byte_read) != 0 ) {
        return -EFAULT;
    }
    *offset += byte_read;
    return byte_read;
}

static ssize_t driver_write(struct file *file, const char __user *user_buffer, size_t length, loff_t *offset) {
    size_t byte_write = 0;
    printk(KERN_INFO "write function called...\n");
    memset(m_dev.kernel_buffer, 0, sizeof(m_dev.kernel_buffer));
    byte_write = (length + *offset <= 10) ? (length) : (10 - *offset);
    if (copy_from_user(m_dev.kernel_buffer, user_buffer, byte_write) != 0) {
        printk(KERN_ERR "Failed to copy data from user space\n");
        return -EFAULT;
    }
    printk(KERN_INFO "Data from user: %s", m_dev.kernel_buffer);
    
    if (*(m_dev.kernel_buffer) == '1') {
        // led_control(1);
        *(gpio0_ptr + GPIO_CLEARDATAOUT_OFFSET/4) |= GPIO0_30;    //ON
        
    }
    else if (*(m_dev.kernel_buffer) == '0') {
        // led_control(0);
        *(gpio0_ptr + GPIO_SETDATAOUT_OFFSET/4) |= GPIO0_30;   //OFF

    }
    *offset = *offset + length;
    byte_write = length;
    m_dev.length = *offset;
    return byte_write;
}

static int driver_release(struct inode *inode, struct file *file) {
    printk(KERN_ERR "release function called...\n");
    return 0;    
}

/**
 * @brief This function will be called when insert module with modprobe or insmod
 */
static int __init m_module_init(void) {
    printk("Hello Kernel\n");    
    if (alloc_chrdev_region(&m_dev.cdev_number, 1, 1, "my_driver") < 0) {
        printk(KERN_ERR "Failed to allocate device number.\n");
        return -1;
    }
    printk(KERN_INFO "Major is: %d, Minor is: %d\n", MAJOR(m_dev.cdev_number), MINOR(m_dev.cdev_number));

    cdev_init(&m_dev.m_cdev, &fops);

    if (cdev_add(&m_dev.m_cdev, m_dev.cdev_number, 1) < 0 ) {
        printk(KERN_ERR "Failed to bind the number for device.\n");
        goto rm_alloc;
    }

    if ((m_dev.my_class = class_create(THIS_MODULE,"my_class")) == NULL) {
        printk(KERN_ERR "Failed to create class for device.\n");
        goto rm_alloc;
    }

    if (device_create(m_dev.my_class, NULL, m_dev.cdev_number, NULL, "my_device") == NULL) {
        printk(KERN_ERR "Failed to create device.\n");
        goto rm_class;
    }

    if ((m_dev.kernel_buffer = kmalloc(10, GFP_KERNEL)) == NULL) {
        printk(KERN_ERR "Failed to allocate kernel buffer.\n");
        goto rm_device;
    }

    gpio0_ptr = ioremap(GPIO0_BASE_ADDRESS, SIZE_OF_GPIO0);
    if (gpio0_ptr == NULL) {
        return -ENOMEM;
    }    
    *(gpio0_ptr + GPIO_OE_OFFSET/4) &= ~GPIO0_30;
    *(gpio0_ptr + GPIO_SETDATAOUT_OFFSET/4) |= GPIO0_30;   // PIN = 1 --> OFF

    return 0;

rm_device:
    device_destroy(m_dev.my_class, m_dev.cdev_number);
rm_class:
    class_destroy(m_dev.my_class);
rm_alloc:
    unregister_chrdev_region(m_dev.cdev_number, 1);
    return -1;
}

/** 
 * @brief This function will be called when remove module out of kernel with modprobe or rmmod
 */
static void __exit m_module_exit(void) {
    printk("Goodbye Kernel\n");
    *(gpio0_ptr + GPIO_OE_OFFSET/4) |= GPIO0_30;
    iounmap(gpio0_ptr);
    kfree(m_dev.kernel_buffer);
    device_destroy(m_dev.my_class, m_dev.cdev_number);
    class_destroy(m_dev.my_class);
    cdev_del(&m_dev.m_cdev);
    unregister_chrdev_region(m_dev.cdev_number, 1);
}


module_init(m_module_init);
module_exit(m_module_exit);

