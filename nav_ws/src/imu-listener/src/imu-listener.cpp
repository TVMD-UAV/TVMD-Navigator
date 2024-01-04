#include <cstdio>
#include <unistd.h>
#include <cstdlib>
#include <cstring>

#include <i2c/i2c.h>

void print_i2c_data(const unsigned char *data, size_t len);

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    I2C_READ_HANDLE i2c_read_handle = i2c_ioctl_read;
    unsigned int addr = 0x66;

    /* Open i2c bus */
    int bus;
    // I2C Bus 1 SDA is on Pin 3
    // I2C Bus 1 SCL is on Pin 5
    char bus_name[] = "/dev/i2c-1";

    if ((bus = i2c_open(bus_name)) == -1) {

        fprintf(stderr, "Open i2c bus:%s error!\n", bus_name);
        exit(-3);
    }

    /* Init i2c device */
    I2CDevice device;
    memset(&device, 0, sizeof(device));
    i2c_init_device(&device);

    device.bus = bus;
    device.addr = addr & 0x3ff;
    device.page_bytes = 0;
    device.iaddr_bytes = 0;

    ssize_t ret = 0;
    unsigned char buf[256];
    size_t buf_size = sizeof(buf);

    ret = i2c_read_handle(&device, 0x0, buf, buf_size);
    if (ret == -1 || (size_t)ret != buf_size)
    {

        fprintf(stderr, "Read i2c error!\n");
        exit(-5);
    }

    /* Print read result */
    fprintf(stdout, "Read data:\n");
    print_i2c_data(buf, buf_size);

    i2c_close(bus);
    return 0;
}


void print_i2c_data(const unsigned char *data, size_t len)
{
    size_t i = 0;

    for (i = 0; i < len; i++) {

        if (i % 16 == 0) {

            fprintf(stdout, "\n");
        }

        fprintf(stdout, "%02x ", data[i]);
    }

    fprintf(stdout, "\n");
}
