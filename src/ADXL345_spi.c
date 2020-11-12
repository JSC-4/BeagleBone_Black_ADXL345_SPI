/*============================================================================
 Name        : ADXL345_spi.c
 Author      : JSC
 Website	 : https://www.jscblog.com/post/bbb-adxl345-3-axis-accelerometer-spi-part-2
 Description : This project is to communicate with the ADXL345 3-axis Accelerometer
 	 	 	   sensor using the BeagleBone Black (SPI).
 ============================================================================*/

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPI_PATH	"/dev/spidev0.0"

// Registers taken from the ADXL345 Datasheet
#define ADXL345_REG_BW_RATE		0x2C
#define ADXL345_REG_POWERCTL	0x2D
#define ADXL345_REG_DATA_FORMAT	0x31

const unsigned char acc_reg[6] = {
		0x32, 0x33, 0x34,
		0x35, 0x36, 0x37
};

int fd;

int main()
{
	//Stored in unsigned char since it's 8 bit data, normal (signed) char is 7 bits
	unsigned char tx_buf[2];// = {0x00};
	unsigned char rx_buf[2]; //= {0};
	struct spi_ioc_transfer xfer[1] = {0};

	uint8_t mode = SPI_MODE_3, bits = 8;
	uint32_t speed = 5000000;

	// Open SPI path
	if ((fd = open(SPI_PATH, O_RDWR)) < 0)
	{
		perror("SPI ERROR: Can't open device.");
		return -1;
	}

	// Set SPI mode
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode)==-1){
	   perror("SPI: Can't set SPI mode.");
	   return -1;
	}
	if (ioctl(fd, SPI_IOC_RD_MODE, &mode)==-1){
	   perror("SPI: Can't get SPI mode.");
	   return -1;
	} else
		printf("SPI mode set to %d\n", mode);

	// Set SPI write bits
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
	  perror("SPI: Can't set bits per word.");
	  return -1;
	}
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1){
	  perror("SPI: Can't get bits per word.");
	  return -1;
	}

	// Set SPI speed
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
	  perror("SPI: Can't set max speed HZ");
	  return -1;
	}
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
	  perror("SPI: Can't get max speed HZ.");
	  return -1;
	}
	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	printf("Writing register...\n");
	tx_buf[0] = (0x00 | ADXL345_REG_BW_RATE); // 0x00 adds a 0 to the transmit register for writing (check datasheet timing diagram)
	tx_buf[1] = (0x0A);
	xfer[0].tx_buf = (unsigned long)tx_buf;
	xfer[0].rx_buf = (unsigned long)rx_buf;
	xfer[0].len = 2;

	if (ioctl(fd, SPI_IOC_MESSAGE(1), xfer) < 0)
		 perror("SPI_IOC_MESSAGE");

	printf("Writing register...\n");
	tx_buf[0] = (0x00 | ADXL345_REG_DATA_FORMAT); // 0x00 adds a 0 to the transmit register for writing (check datasheet timing diagram)
	tx_buf[1] = (0x08);
	xfer[0].tx_buf = (unsigned long)tx_buf;
	xfer[0].rx_buf = (unsigned long)rx_buf;
	xfer[0].len = 2;

	if (ioctl(fd, SPI_IOC_MESSAGE(1), xfer) < 0)
		 perror("SPI_IOC_MESSAGE");

	printf("Writing register...\n");
	tx_buf[0] = (0x00 | ADXL345_REG_POWERCTL); // 0x00 adds a 0 to the transmit register for writing (check datasheet timing diagram)
	tx_buf[1] = (0x08);
	xfer[0].tx_buf = (unsigned long)tx_buf;
	xfer[0].rx_buf = (unsigned long)rx_buf;
	xfer[0].len = 2;

	if (ioctl(fd, SPI_IOC_MESSAGE(1), xfer) < 0)
		 perror("SPI_IOC_MESSAGE");

	printf("Reading register...\n");
		tx_buf[0] = (0x80 | ADXL345_REG_BW_RATE); // 0x80 adds a one to the transmit register for reading (check datasheet timing diagram)
		xfer[0].tx_buf = (unsigned long)tx_buf;
		xfer[0].rx_buf = (unsigned long)rx_buf;
		xfer[0].len = 2;

		if (ioctl(fd, SPI_IOC_MESSAGE(1), xfer) < 0)
			 perror("SPI_IOC_MESSAGE");
		else
			 printf("Return value: %d\n", rx_buf[1]);

	printf("Reading register...\n");
		tx_buf[0] = (0x80 | ADXL345_REG_DATA_FORMAT); // 0x80 adds a one to the transmit register for reading (check datasheet timing diagram)
		xfer[0].tx_buf = (unsigned long)tx_buf;
		xfer[0].rx_buf = (unsigned long)rx_buf;
		xfer[0].len = 2;

		if (ioctl(fd, SPI_IOC_MESSAGE(1), xfer) < 0)
			 perror("SPI_IOC_MESSAGE");
		else
			 printf("Return value: %d\n", rx_buf[1]);

	printf("Reading register...\n");
	tx_buf[0] = (0x80 | ADXL345_REG_POWERCTL); // 0x80 adds a one to the transmit register for reading (check datasheet timing diagram)
	xfer[0].tx_buf = (unsigned long)tx_buf;
	xfer[0].rx_buf = (unsigned long)rx_buf;
	xfer[0].len = 2;

	if (ioctl(fd, SPI_IOC_MESSAGE(1), xfer) < 0)
		 perror("SPI_IOC_MESSAGE");
	else
		 printf("Return value: %d\n", rx_buf[1]);

	short acc_x, acc_y, acc_z;
	char acc_buffer[6];

	while(1)
	{
		for(int i = 0; i < 6;i++)
		{
			//printf("Reading acc register...\n");
			tx_buf[0] = (0x80 | acc_reg[i]);
			xfer[0].tx_buf = (unsigned long)tx_buf;
			xfer[0].rx_buf = (unsigned long)rx_buf;
			xfer[0].len = 2;

			if (ioctl(fd, SPI_IOC_MESSAGE(1), xfer) < 0)
				 perror("SPI_IOC_MESSAGE");
			else
				acc_buffer[i] = rx_buf[1];
				 //printf("Return value: %d\n", rx_buf[1]);
		}

		acc_x = ((short)acc_buffer[1] << 8) | (short)acc_buffer[0];
		acc_y = ((short)acc_buffer[3] << 8) | (short)acc_buffer[2];
		acc_z = ((short)acc_buffer[5] << 8) | (short)acc_buffer[4];

		printf("Acc => X:%d Y:%d Z:%d\n", acc_x,acc_y,acc_z);
		usleep(1000*500);
	}

	return 0;
}

