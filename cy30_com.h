/**
 * @file    cy30-com.h
 * @brief   This file is the header file of cy30 module communication and control.
 * @author  PWE <dicksonliuming@gmail.com>
 * @date    2016-7-12
 */
#ifndef CY30_COM_H
#define CY30_COM_H

/**
 * ** PARAMENTS
 */
/* commands is always follow the form of: [HEADER] [READ/WRITE] [CONTENT] [C] */

#define DEPLOYHEADER                0XFA
#define DEPLOY_READ                 0X06
#define DEPLOY_WRITE                0X04
// measure header is the address of device which want to measure
// #define MEASUREHEADER
#define MEASURE_READ                0X06
#define MEASURE_WRITE               0X04

// deploy function of read
#define DEPLOY_READ_ARGUMENTS       0X01        // like: FA 06 01 FF
#define DEPLOY_READ_MACHINENUM      0X04        // like: FA 06 04 FC
//measure once and save result in module buffer"
#define DEPLOY_READ_ONCEINBUFF      0X06        // like: FA 06 06 FA

// deploy function of write
#define DEPLOY_WRITE_ADDRESS        0X01        // like: FA 04 01 ADDR(1byte) CS
#define DEPLOY_WRITE_DISTANCECALIBRATION    0X06        // like: FA 04 06 MARK('-' is 0x2d, '+' is 0x2b)(1byte) CALIBEATION(1byte) CS
#define DEPLOY_WRITE_MEAINTERVER    0X05        // like: FA 04 05 MEAINTERVER(1byte) CS
#define DEPLOY_WRITE_POSITION       0X08        // like: FA 04 08 POSITION(1byte) CS
#define DEPLOY_WRITE_RANGE          0X09        // like: FA 04 09 RANGE(05,10,30,50,80,1byte) CS
#define DEPLOY_WRITE_FREQUENCE      0X0A        // like: FA 04 0A FREQ(05,10,20,1byte) CS
#define DEPLOY_WRITE_RESOLUTION     0X0C        // like: FA 04 0C RESOLUTION(1(1mm),2(0.1mm),1byte) CS
#define DEPLOY_WRITE_MEASUREINBOOT  0X0D        // like: FA 04 0D Start(0(close),1(open),1byte) CS

// measure function of read
#define MEASURE_READ_BUFFER         0X07        // like: ADDR 06 07 CS
#define MEASURE_READ_READONCE       0X02        // like: ADDR 06 02 CS
#define MEASURE_READ_READCONTINUOUS 0X03        // like: ADDR 06 03 CS
// other functions
// control laser to open or close
#define MEASURE_READ_LASER          0X05        // like: ADDR 06 05 LASER(0(close),1(open),1byte) CS
// shutdown sensor
#define MEASURE_WRITE_SHUTDOWN      0X02        // like: ADDR 04 02 CS

// receive marks
#define RECEIVE_ARGUMENTS           DEPLOY_READ_ARGUMENTS + 0X80        // like: FA 06 81 ADDR XX XX XX ... CS
#define RECEIVE_MACHINENUM          DEPLOY_READ_MACHINENUM + 0x80       // like: FA 06 84 DAT1 DAT2 ... DAT16 CS

#define RECEIVE_MEAINTERVER         DEPLOY_WRITE_MEAINTERVER + 0X80     // like: FA 04 85 7D(is success)
#define RECEIVE_POSITION            DEPLOY_WRITE_POSITION + 0X80        // like: FA 04 88 7A(is success)
#define RECEIVE_RANGE               DEPLOY_WRITE_RANGE + 0X80           // like: FA 04 89 79(is success)
#define RECEIVE_FREQUENCE           DEPLOY_WRITE_FREQUENCE + 0x80       // like: FA 04 8A 78(is success)
#define RECEIVE_RESOLUTION          DEPLOY_WRITE_RESOLUTION + 0x80      // like: FA 04 8C 76(is success)
#define RECEIVE_MEASUREINBOOT       DEPLOY_WRITE_MEASUREINBOOT + 0x80   // like: FA 04 8D 75(is success)

#define RECEIVE_BUFFER              RECEIVE_READONCE
#define RECEIVE_READONCE            MEASURE_READ_READONCE + 0X80         // like: ADDR 06 82 3X 3X 3X 2E 3X 3X 3X CS
#define RECEIVE_READCONTINUOUS      MEASURE_READ_READCONTINUOUS + 0X80   // like: ADDR 06 83 3X 3X 3X 2E 3X 3X 3X CS

#define RECEIVE_LASER               MEASURE_READ_LASER + 0x80           // like: ADDR 06 85 01 CS(is success)
#define RECEIVE_SHUTDOWN            MEASURE_WRITE_SHUTDOWN + 0X80       // like: ADDR 04 82 CS(is success)

/**
 * ** MARKS
 */

typedef enum {
    Deploy          = 0,
    Measure         = 1
}Mode;

typedef enum {
    ReadArguments   = 0,
    ReadMachineNum  = 1,
    SetAddress      = 2,
    CalibrateDistance   = 3,
    SetMeaInterver  = 4,
    SetPosition     = 5,
    SetRange        = 6,
    SetFrequence    = 7,
    SetResolution   = 8,
    SetMeasureInBoot   = 9,
    MeasureOnceInBuffer = 10,
    ReadBuffer      = 11,
    MeasureOnce     = 12,
    MeasureContinuous   = 13,
    SetLaser        = 14,
    Shutdown        = 15
}Action;

/**
 * ** CONTAINER
 */

typedef struct container{
    unsigned char address;
    float distance;
}DistanceContainer;

/**
 * ** FUNCTIONS
 */
/**
 * calculateCS: to calculate CS in command.
 * @param   precommand      part of command without CS
 * @param   len             length of precommand
 * @retval                  return CS, 0x00 is fault
 */
unsigned char calculateCS(unsigned char * precommand, unsigned int len);

/**
 * checkCS: check received data is right with cs bit
 * @param   origin          received data
 * @param   len             lenght of receive command
 * @retval                  0 is right cs, 1 is wrong cs, -1 is fault
 */
int checkCS(unsigned char * origin, unsigned int len);

/**
 * constructCommand: to construct command, full command
 * @param   mode            Mode: Depoly or Measure
 * @param   address         address of sensor
 * @param   action          Action: one of action in Action
 * @param   command         return command
 * @retval                  return length of command, -1 is fault
 *
 */
int constructCommand(Mode mode, unsigned char address, 
        Action action, unsigned char **command);
/**
 * resultProcess: used to analysis result recieved
 * @param   container       a struct form argument to save distance and address
 * @param   origin          received data
 * @param   len             length of received data
 * @param   action          which action return to origin
 * @origin                  0 is down
 */
int resultProcess(DistanceContainer *container, unsigned char * origin, unsigned len, Action action);

/**
 * calculateDistance: used to calculate distance
 * @param   originDist      part chars of distance in received data, always 7 chars
 * @retval                  distance in float form
 */
float calculateDistance(unsigned char *originDist);
/**
 * set_opt: This function is used to set serial communication argument.
 * @param  fd     serial com file point
 * @param  nSpeed serial transfer speed
 * @param  nBits  serial transfer data bits
 * @param  nEvent serial transfer parity mode
 * @param  nStop  serial transfer stop bit
 * @return        0 is down
 */
int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

/**
 * open_port: function for open port, you should change uart device name in it.
 * @return 0 is down
 */
int open_port();

/**
 * cy30_control: main function of cy30 sensor.
 * @return 0 is down
 */
int cy30_control();

/**
 * cy30_run: entrance of cy30 control.
 * @return 0 is down
 */
int cy30_run();

#endif  // CY30-COM
