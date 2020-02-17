/*
Subject: OpenRex position shows in Blender
Description: This example shows how to read Openrex sensors, transfer the values into Blender and use them for 3D virtualization
Web: http://www.openrexkit.org/
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <errno.h>
#include <termios.h>

// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR   0x1C  //this is address of the sensor on I2C bus
#define FXOS8700CQ_I2C_BUS              "/dev/i2c-0"

// FXOS8700CQ internal register addresses
#define FXOS8700CQ_STATUS               0x00
#define FXOS8700CQ_OUT_X_MSB    0x01
#define FXOS8700CQ_OUT_X_LSB    0x02
#define FXOS8700CQ_OUT_Y_MSB    0x03
#define FXOS8700CQ_OUT_Y_LSB    0x04
#define FXOS8700CQ_OUT_Z_MSB    0x05
#define FXOS8700CQ_OUT_Z_LSB    0x06
#define FXOS8700CQ_WHOAMI               0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1    0x2A
#define FXOS8700CQ_CTRL_REG2    0x2B
#define FXOS8700CQ_OFF_X                0x2F
#define FXOS8700CQ_OFF_Y                0x30
#define FXOS8700CQ_OFF_Z                0x31
#define FXOS8700CQ_M_CTRL_REG1  0x5B
#define FXOS8700CQ_M_CTRL_REG2  0x5C
#define FXOS8700CQ_WHOAMI_VAL   0xC7

#define SENSITIVITY_2G                  4096
#define SENSITIVITY_MAG                 10

char *portname = "/dev/ttymxc0";

typedef struct
{
        int16_t x;
        int16_t y;
        int16_t z;
} SRAWDATA;

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 10;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}


const char *word_to_binary(int x)
{
    static char b[17];
    b[0] = '\0';

    int z;
    for (z = 32768; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

int i2c_smbus_read(int file, char *result, unsigned int bytes)
{
        if (read(file, result, bytes) != bytes)
        {
                printf("Failed to read\n");
                return -1;
        }
        return 0;
}

int i2c_smbus_write(int file, char *buffer, unsigned int bytes)
{
        if (write(file, buffer, bytes) != bytes)
        {
                printf("Failed to write\n");
                return -1;
        }
        return 0;
}

static int i2c_smbus_write_read(int file, unsigned char addr, unsigned char reg, unsigned char *val)
{
    unsigned char inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = addr;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("Unable to send data");
        return 1;
    }
    *val = inbuf;

    return 0;
}

int i2c_smbus_write_register(int file, char reg, char value)
{
        char buffer[4] = {0};

        buffer[0] = reg;
        buffer[1] = value;
        if (i2c_smbus_write(file,buffer,2) < 0)
                return -1;
        return 0;
}

void FXOS8700CQ_Mag_Calibration (int file)
{
        SRAWDATA pMagnData,pMagnData_max,pMagnData_min,pMagnData_avg;
        char buffer[100] = {0};
        unsigned char value;

        char i = 0;
        /*
        //Enable this to get calibration data
        //When rununnig calibration, put the board on your table and rotate it to get max and min values for x and y
        while (i<255)
        {
                value = 0;
                usleep(1000000); //read one sample per second so you have time to rotate the board
                while (value & 0x08) //wait until data are ready
                        i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x32,&value);

                //Read magnetometer data manually
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x33,&value);
                buffer[7] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x34,&value);
                buffer[8] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x35,&value);
                buffer[9] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x36,&value);
                buffer[10] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x37,&value);
                buffer[11] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x38,&value);
                buffer[12] = value;

                pMagnData.x = (buffer[7] << 8) | buffer[8];
                pMagnData.y = (buffer[9] << 8) | buffer[10];
                pMagnData.z = (buffer[11] << 8) | buffer[12];

                //set the initial values
                if (i == 0)
                {
                        pMagnData_max.x = pMagnData.x;
                        pMagnData_min.x = pMagnData.x;

                        pMagnData_max.y = pMagnData.y;
                        pMagnData_min.y = pMagnData.y;

                        pMagnData_max.z = pMagnData.z;
                        pMagnData_min.z = pMagnData.z;
                }

                // Check to see if current sample is the maximum or minimum X-axis value
                if (pMagnData.x > pMagnData_max.x)    {pMagnData_max.x = pMagnData.x;}
                if (pMagnData.x < pMagnData_min.x)    {pMagnData_min.x = pMagnData.x;}

                if (pMagnData.y > pMagnData_max.y)    {pMagnData_max.y = pMagnData.y;}
                if (pMagnData.y < pMagnData_min.y)    {pMagnData_min.y = pMagnData.y;}

                if (pMagnData.z > pMagnData_max.z)    {pMagnData_max.z = pMagnData.z;}
                if (pMagnData.z < pMagnData_min.z)    {pMagnData_min.z = pMagnData.z;}

                printf("Loop %d\n",i);
                printf("X MAX: %04x / %04d  MIN: %04x / %04d  VAL: %04x\n",pMagnData_max.x,pMagnData_max.x,pMagnData_min.x,pMagnData_min.x,pMagnData.x,pMagnData.x);
                printf("Y MAX: %04x / %04d  MIN: %04x / %04d  VAL: %04x\n",pMagnData_max.y,pMagnData_max.y,pMagnData_min.y,pMagnData_min.y,pMagnData.y,pMagnData.y);
                printf("Z MAX: %04x / %04d  MIN: %04x / %04d  VAL: %04x\n\n",pMagnData_max.z,pMagnData_max.z,pMagnData_min.z,pMagnData_min.y,pMagnData.z,pMagnData.z);

                i++;
        }

        //calculate the offset
        pMagnData_avg.x = (pMagnData_max.x + pMagnData_min.x) / 2;            // X-axis hard-iron offset
        pMagnData_avg.y = (pMagnData_max.y + pMagnData_min.y) / 2;            // Y-axis hard-iron offset
        pMagnData_avg.z = (pMagnData_max.z + pMagnData_min.z) / 2;            // Z-axis hard-iron offset

        //Left-shift by one as magnetometer offset registers are 15-bit only, left justified, see datasheet
        pMagnData_avg.x <<= 1;
        pMagnData_avg.y <<= 1;
        pMagnData_avg.z <<= 1;

        //write the values into offset registers
        i2c_smbus_write_register(file,FXOS8700CQ_CTRL_REG1,0x00);

        i2c_smbus_write_register(file,0x40,(char) (pMagnData_avg.x & 0xFF)); //M_OFF_X_LSB
        i2c_smbus_write_register(file,0x3F,(char) ((pMagnData_avg.x >> 8) & 0xFF)); //M_OFF_X_MSB

        i2c_smbus_write_register(file,0x42,(char) (pMagnData_avg.y & 0xFF)); //M_OFF_X_LSB
        i2c_smbus_write_register(file,0x41,(char) ((pMagnData_avg.y >> 8) & 0xFF)); //M_OFF_X_MSB

        i2c_smbus_write_register(file,0x44,(char) (pMagnData_avg.z & 0xFF)); //M_OFF_X_LSB
        i2c_smbus_write_register(file,0x43,(char) ((pMagnData_avg.z >> 8) & 0xFF)); //M_OFF_X_MSB

        //Use these values in the: Insert Calibration data manually to skip calibration (see below)
        printf("\nCalibration values: \n");
        printf("XM: %02x ",(char) ((pMagnData_avg.x >> 8) & 0xFF));
        printf("XL: %02x \n",(char) (pMagnData_avg.x & 0xFF));

        printf("YM: %02x ",(char) ((pMagnData_avg.y >> 8) & 0xFF));
        printf("YL: %02x \n",(char) (pMagnData_avg.y & 0xFF));

        printf("ZM: %02x ",(char) ((pMagnData_avg.z >> 8) & 0xFF));
        printf("ZL: %02x \n",(char) (pMagnData_avg.z & 0xFF));
        */

        //START: Insert Calibration data manually to skip calibration
        //Once you run calibration, enable this
        //XM: ec XL: fa
        //YM: f9 YL: c8
        //ZM: f4 ZL: b6
        i2c_smbus_write_register(file,FXOS8700CQ_CTRL_REG1,0x00); //stop the sensor to rewrite values
        i2c_smbus_write_register(file,0x3F,0xec); //M_OFF_X_MSB
        i2c_smbus_write_register(file,0x40,0xfa); //M_OFF_X_LSB

        i2c_smbus_write_register(file,0x41,0xf9); //M_OFF_X_MSB
        i2c_smbus_write_register(file,0x42,0xc8); //M_OFF_X_LSB

        i2c_smbus_write_register(file,0x43,0xf4); //M_OFF_X_MSB
        i2c_smbus_write_register(file,0x44,0xb6); //M_OFF_X_LSB
        //END: Insert Calibration data manually to skip calibration

        //Enable the sensor again
        i2c_smbus_write_register(file,FXOS8700CQ_CTRL_REG1,0x01);

}

int main(int argc, char *argv[])
{
        printf("OpenRex I2C Read/Write example\n");

        int file;

        char buffer[100] = {0};
        unsigned char value;

        //I2C Init
        printf("I2C init\n");
        char *filename = FXOS8700CQ_I2C_BUS;
        if ((file = open(filename, O_RDWR)) < 0)
        {
                        perror("Failed to open the i2c bus\n");
                        return 0;
        }

        if (ioctl(file, I2C_SLAVE, FXOS8700CQ_SLAVE_ADDR) < 0)
        {
                        printf("Failed to acquire bus access and/or talk to slave.\n");
                        return 0;
        }

        //Serial port init
        printf("Serial port init\n");
        int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
        return 0;
        }
        set_interface_attribs (fd, B115200, 0);         // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking (fd, 0);                           // set no blocking

        //RESET sensor
        printf("reseting sensor\n");
        i2c_smbus_write_register(file,FXOS8700CQ_CTRL_REG2,0x40);
        usleep(1000000);

        //Detect the sensor
        i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,FXOS8700CQ_WHOAMI,&value);              //Read sensor ID
        printf("Checking the sensor. Expected 0xC7. Reads: %#02x\n",value);

        //Initialize sensor
        i2c_smbus_write_register(file,0x5B,0x03);
        i2c_smbus_write_register(file,0x2A,0x01); //Enable the sensor

        printf("Compass calibration\n");
        FXOS8700CQ_Mag_Calibration(file); //see the notes in calibration function

        SRAWDATA pAccelData,pAccelData2;
        SRAWDATA pMagnData,pMagnData2;

        int16_t buff_x[255],buff_y[255];

        unsigned int i=0;
        int j=0;
        unsigned int loop=0;
        double sum = 0;
        double ave_x,ave_y;
        float Xout_uT,Yout_uT,Zout_uT;
        float mRex;
        int samples;
        double heading_double;
        char heading[50];
        char output[50];

        while (1)
        {
                //This is a simple example. In a real application you may want to consider using Interrupt
                //Read ERRATA: https://developer.mbed.org/media/uploads/GregC/fxos8700cq_er.pdf
                /*
                A timing conflict exists where signals from the I2C bus are being coupled into the magnetometer’s
                analog front end (AFE), creating noise that exceeds the noise specification of the device.
                The issue only occurs when the AFE and I2
                C signal are active at the same time (during polling method).
                The resulting noise is transient noise of repeatable magnitude (in static environments) occurring at 20–
                70 µTesla from the nominal output samples. The effect is observed mainly on the x-axis and y-axis and
                negligibly on the z-axis.
                */

                //wait to send next data
                //printf("Loop: %d\n",i);
                usleep(100);
                value = 0;
                while (value & 0x08) //wait until data are ready
                        i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x32,&value);

                //Read magnetometer data manually
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x33,&value);
                buffer[7] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x34,&value);
                buffer[8] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x35,&value);
                buffer[9] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x36,&value);
                buffer[10] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x37,&value);
                buffer[11] = value;
                i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x38,&value);
                buffer[12] = value;

                pMagnData.x = (buffer[7] << 8) | buffer[8];
                pMagnData.y = (buffer[9] << 8) | buffer[10];
                pMagnData.z = (buffer[11] << 8) | buffer[12];

                /*
                value = 0;
                while (value & 0x08) //wait until data are ready
                        i2c_smbus_write_read(file,FXOS8700CQ_SLAVE_ADDR,0x32,&value);

                //read accelerometer and magnetomer at once
                i2c_smbus_read(file,buffer,13); //read 13 bytes

                // copy the 14 bit accelerometer byte data into 16 bit words
                pAccelData.x = (int16_t)(((buffer[1] << 8) | buffer[2]))>> 2;
                pAccelData.y = (int16_t)(((buffer[3] << 8) | buffer[4]))>> 2;
                pAccelData.z = (int16_t)(((buffer[5] << 8) | buffer[6]))>> 2;
                // copy the magnetometer byte data into 16 bit words
                pMagnData.x = (buffer[7] << 8) | buffer[8];
                pMagnData.y = (buffer[9] << 8) | buffer[10];
                pMagnData.z = (buffer[11] << 8) | buffer[12];
                */

                //calculate average from last samples
                sum = 0;
                samples = 100; //number of samples in buffer used for average calculation

                //X
                for (j=1;j<samples;j++)
                {
                        buff_x[j-1]=buff_x[j];
                        sum = sum + buff_x[j-1];
                }
                buff_x[j-1] = pMagnData.x;
                sum = sum + buff_x[j-1];
                ave_x = sum / (j-1);

                //Y
                sum = 0;
                for (j=1;j<samples;j++)
                {
                        buff_y[j-1]=buff_y[j];
                        sum = sum + buff_y[j-1];
                }
                buff_y[j-1] = pMagnData.y;
                sum = sum + buff_y[j-1];
                ave_y = sum / (j-1);

                //change it to uT
                Xout_uT = (float) (ave_x) / SENSITIVITY_MAG;
                Yout_uT = (float) (ave_y) / SENSITIVITY_MAG;
                Zout_uT = (float) (pMagnData.z) / SENSITIVITY_MAG;

                heading_double = atan2(Yout_uT,Xout_uT) * (180/3.14159265);     //calculate heading in -180 + 180 degrees, we ignore Z

                //prepare the string which is going to be send through serial port to blender
                //the string will look simple ASNNNNN; where A is axis, S is sign, NNNNN is number, ; is used to separate the strings
                strcpy (output,"z"); //sart creating our string, we will be rotating around Z axis

                if (!(heading_double<0))
                        strcat (output,"+"); //add + sign into the string for positive numbers

                snprintf(heading, 50, "%06.2f", heading_double); //make string_number from the number
                strcat (output,heading); //copy the string_number into the string which we will be sending out

                strcat (output,";"); //add ; to separate the numbers

                printf("Sending ===> %s Loop: %d i= %d\n", output,loop,i); //double check what we are sending out
                write (fd, output, strlen(output)); //send the string to serial port

        }
        printf("\n");
        return 0;
}
