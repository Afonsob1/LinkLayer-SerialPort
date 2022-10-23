// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#define LOG256_2 (1/8)

#define CONTROL_START 2
#define CONTROL_END 3 

#define FILE_SIZE 0
#define FILE_NAME 1

#define DATA 1
#define START 2
#define END 3

#define FILE_SIZE 0
#define FILE_NAME 1


#define TRANSMITTER 0
#define RECEIVER 1


#define MAX_FILENAME_SIZE 1024
#define DATA_SIZE 1024
/*
A aplicação deve ser capaz de chamar as funções: 
- llopen() - llread() - llwrite() - llclose()
*/

typedef enum{
    StateReceiveDEFAULT,
    StateReceiveSTART,
    StateReceiveDATA,
    StateReceiveERROR,
    StateReceiveEND
} StateReceive;


typedef struct applicationLayer {
    int fileDescriptor; /* Descritor correspondente à porta série */
    int status; /* TRANSMITTER | RECEIVER */
} ApplicationLayer;

// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename);

#endif // _APPLICATION_LAYER_H_
