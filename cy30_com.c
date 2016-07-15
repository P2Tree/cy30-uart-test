/***************************************************************
 *
 * @file    cy30-com.c
 * @brief    cy30 communication and control file
 * @author    PWE <dicksonliuming@gmail.com>
 * @date    2016-7-12
 *
 ***************************************************************/

#include <stdio.h>
#include <termios.h>
#include <memory.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "cy30_com.h"
#include <stdlib.h>

// because of 'Segmentation Fault', set 'dev' to global variable.
unsigned int dev = 1;        // com port. 1 is ttymxc1, 2 is ttymxc2

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0) {
        perror("Backup SerialSetting");
        return -1;
    }

    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_iflag &= ~(INPCK | ISTRIP     // close parity bits setting
                | IUCLC);    // close auto case change

    switch(nBits) {
    case 7:
        newtio.c_cflag &= ~CSIZE;
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag &= ~CSIZE;
        newtio.c_cflag |= CS8;
        break;
    default:
        perror("Set_opt wrong bits");
        return -1;
    }

    switch(nEvent) {
    case 'O':
        newtio.c_cflag &= ~PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        perror("Set_opt wrong event");
        return -1;
    }

    switch(nSpeed) {
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    default:
        perror("Set_opt wrong speed OR other speed");
        return -1;
    }

    if ( 1 == nStop)
        newtio.c_cflag &= ~CSTOPB;        // stop 1 bit
    else if ( 2 == nStop)
        newtio.c_cflag |= CSTOPB;        // stop 2 bits

    newtio.c_cc[VTIME] = 0;                // waitting time
    newtio.c_cc[VMIN] = 0;                // min receive data

    tcflush(fd, TCIOFLUSH);

    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        tcsetattr(fd, TCSANOW, &oldtio);
        perror("Set_opt com set error");
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    printf("set down!\n");
    return 0;
}

int open_port() {
    int fd;
    if (1 == dev) {
        if (-1 == (fd = open("/dev/ttymxc1", O_RDWR | O_NOCTTY | O_NONBLOCK)) ) {
            perror ("Can't Open Serial Port");
            return -1;
        }
    }
    else if (2 == dev) {
        if (-1 == (fd = open("/dev/ttymxc2", O_RDWR | O_NOCTTY | O_NONBLOCK)) ) {
            perror ("Can't Open Serial Port");
            return -1;
        }
    }
    if (isatty(STDIN_FILENO) == 0) {
        printf("standard input is not a terminal device \n");
    }
    else {
        printf("isatty success\n");
    }
    return fd;
}

int com_control() {
    int fd;
    if ((fd = open_port()) < 0) {
        perror("open_port error");
        return -1;
    }

    if (set_opt(fd, 9600, 8, 'N', 1) < 0) {
        perror("set_opt error");
        return -1;
    }

    printf("fd = %d\n", fd);

    return fd;
}

unsigned char calculateCS(unsigned char * precommand, unsigned int len) {
    char cs = 0x00;
    unsigned int i;
    if (len <= 0){
        printf("Error: calculateCS wrong argument: len\n");
        return -1;
    }
    for (i=0; i<len; i++) {
        cs = precommand[i] + cs;
    }
    cs = ~cs + 1;
    return cs;
}

int checkCS(unsigned char * origin, unsigned int len) {
    char value  = 0x00;
    unsigned int i;
    if (len <= 0) {
        printf("Error: checkCS wrong argument: len\n");
        return -1;
    }

    for (i=0; i<len-1; i++) {
        value = value + origin[i];
    }
    value = ~value + 1;
    if (value == origin[len-1])
        return 0;   // right cs
    else
        return 1;   // wrong cs
}

int constructCommand(Mode mode, unsigned char address, 
        Action action, unsigned char **cmd ) {
    unsigned int len = 0;
    unsigned char cs = 0x00;
    switch(action) {
        case ReadArguments :
            break;
        case ReadMachineNum :
            break;
        case SetAddress :
            break;
        case CalibrateDistance :
            break;
        case SetMeaInterver :
            break;
        case SetPosition :
            break;
        case SetRange :
            break;
        case SetFrequence :
            break;
        case SetResolution :
            break;
        case SetMeasureInBoot :
            break;
        case MeasureOnceInBuffer :
            break;
        case ReadBuffer :
            break;
        case MeasureOnce :
            len = 4;
            *cmd = (unsigned char*)calloc(len, sizeof(unsigned char));
            (*cmd)[0] = address;
            (*cmd)[1] = MEASURE_READ;
            (*cmd)[2] = MEASURE_READ_READONCE;
            len = 3;
            break;
        case MeasureContinuous :
            break;
        case SetLaser :
            break;
        case Shutdown :
            break;
        default:
            printf("Constructing command: Error Action\n");
            return -1;
    }
    cs = calculateCS(*cmd, len);
    (*cmd)[len] = cs;
    len += 1;
    return len;
}

int resultProcess(DistanceContainer *container, unsigned char *origin, 
        unsigned int len, Action action) {
    unsigned char originDist[7];
    int i;
    if (0x45 == origin[3] && 0x52 == origin[4] && 0x52 == origin[5]) {
        printf("wrong data: receive ERR message.\n");
        return -1;
    }
    if (checkCS(origin, len)) {
        printf("Error: receive error result, checkCS stop.\n");
        return -1;
    }
    switch(action) {
        case ReadArguments :
            break;
        case ReadMachineNum :
            break;
        case SetAddress :
            break;
        case CalibrateDistance :
            break;
        case SetMeaInterver :
            break;
        case SetPosition :
            break;
        case SetRange :
            break;
        case SetFrequence :
            break;
        case SetResolution :
            break;
        case SetMeasureInBoot :
            break;
        case MeasureOnceInBuffer :
            break;
        case ReadBuffer :
            break;
        case MeasureOnce :
            if (!(RECEIVE_READONCE == origin[2]))
                return -1;
            (*container).address = origin[0];
            for (i=0; i<7; i++)
                originDist[i] = origin[3+i];
            (*container).distance = calculateDistance(originDist);
            break;
        case MeasureContinuous :
            break;
        case SetLaser :
            break;
        case Shutdown :
            break;
        default:
            printf("analysis command: Error Action\n");
            return -1;
    }
    return 0;
}

float calculateDistance(unsigned char * originDist) {
    return (float)(originDist[0] - '0')*100 + (float)(originDist[1] - '0')*10 + 
        (float)(originDist[2] - '0') + (float)(originDist[4] - '0')*0.1 + 
        (float)(originDist[5] - '0')*0.01 + (float)(originDist[6] - '0')*0.001;
}

int cy30_run(void) {
    int fd1, fd2;
    unsigned int nread;
    unsigned char readbuff[READLEN];
    unsigned char *cmd1;
    unsigned int cmdlen1 = 0;
    unsigned char *cmd2;
    unsigned int cmdlen2 = 0;
    DistanceContainer container;
    int i;
    int flag = 0;
    memset(readbuff, 0, sizeof(readbuff));
    dev = 1;
    fd1 = com_control();
    dev = 2;
    fd2 = com_control();
    if (!( cmdlen1 = constructCommand(Measure, 0x80, MeasureOnce, &cmd1))) {
        printf("Construct command fault\n");
        return -1;
    }
    if (!( cmdlen2 = constructCommand(Measure, 0x81, MeasureOnce, &cmd2))) {
        printf("Construct command fault\n");
        return -1;
    }

    while(1) {
        if (0 == flag) {
            write(fd1, cmd1, cmdlen1);
            tcflush(fd1, TCOFLUSH);
            sleep(1);
            nread = read(fd1, readbuff, READLEN);
            tcflush(fd1, TCIFLUSH);
            flag = 1;
        }
        else {
            write(fd2, cmd2, cmdlen2);
            tcflush(fd2, TCOFLUSH);
            sleep(1);
            nread = read(fd2, readbuff, READLEN);
            tcflush(fd2, TCIFLUSH);
            flag = 0;
        }
        if (!nread) {
            printf("read no data\n");
            continue;
        }
        // process data
        if (0 == resultProcess(&container, readbuff, nread, MeasureOnce)) {
            printf("address is 0x%02X , distance is %.3f\n", 
                    container.address, container.distance);
        }
        memset(readbuff, 0, sizeof(readbuff));
    }
    free(cmd1);
    free(cmd2);
    close(fd1);
    close(fd2);
    return 0;
}
