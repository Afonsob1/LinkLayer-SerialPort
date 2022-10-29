// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_


typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE 0
#define TRUE 1

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source


#define ACKN 1
#define NACKN (-1)

#define CONTROL_FIELD(n) ((n)<<6)

#define BUF_SIZE 256
#define FLAG 0x7E
#define SET 0x03
#define UA 0x07
#define DISC 0x0B
#define TRANSMITTER_COMMAND 0x03
#define RECEIVER_REPLY 0x03
#define RECEIVER_COMMAND 0x01
#define TRANSMITTER_REPLY 0x01
#define SU_BUF_SIZE 5


#define TIMEOUT_SECS 3
#define MAX_TIMEOUTS 3

#define STUFFING 0x7d
#define ESC 0x7d

#define CONTROL(n)  ((n) << 6)
#define ACK(n)  ((n) << 7  | 0x05)
#define NACK(n)  ((n) << 7 | 0x01)


/*Unumbered and Supervions frames State Machine  */
typedef enum{
    U_S_StateSTART,
    U_S_StateFLAG,
    U_S_StateA,
    U_S_StateC,
    U_S_StateBCC1,
    U_S_StateSTOP
} U_S_State;

/*Information Frame State Machine*/
typedef enum{
    IStateSTART,
    IStateFLAG,
    IStateA,
    IStateC,
    IStateBCC1,
    IStateCReply,
    IStateReplyData,
    IStateSTOP
} IState;


// Open a connection using the "port" parameters defined in struct linkLayer.
// Return "1" on success or "-1" on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or "-1" on error.
int llwrite(int fd, const unsigned char *buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or "-1" on error.
int llread(int fd, unsigned char * buffer);

// Close previously opened connection.
// if showStatistics == TRUE, link layer should print statistics in the console on close.
// Return "1" on success or "-1" on error.
int llclose(int fd, int showStatistics,LinkLayerRole ll);

#endif // _LINK_LAYER_H_
