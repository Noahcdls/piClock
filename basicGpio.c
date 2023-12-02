#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>

int main(){
    int fd;
    struct gpiohandle_request led, button;
    struct gpiohandle_data data;

    fd = open("/dev/gpiochip0", O_RDWR);
    if(fd < 0)
    {
        printf("Failed to open gpio\n");
        return -1;
    }
    led.flags = GPIOHANDLE_REQUEST_OUTPUT;
    strcpy(led.consumer_label, "DC");
    memset(led.default_values, 0, sizeof(led.default_values));
    led.lines = 1;
    led.lineoffsets[0] = 16;

    if(ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &led) < 0)
    {
        printf("Error setting GPIO 16 to output\n");
        close(led.fd);
        close(fd);
        return -1;
    }
    
    /*Settting the GPIO*/
    data.values[0] = 1;
    if(ioctl(led.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0)
    {
        printf("Failed to set pin to high\n");
        close(fd);
        close(led.fd);
        return -1;
    }
    
    sleep(2);

    return 0;
    


}