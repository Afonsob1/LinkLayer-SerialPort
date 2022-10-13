// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 256
#define FLAG 0x7E
#define SET 0x03
#define UA 0x07
#define TRANSMITTER_COMMAND 0x03
#define RECEIVER_REPLY 0x03
#define RECEIVER_COMMAND 0x01
#define TRANSMITTER_REPLY 0x01
#define SU_BUF_SIZE 5

#define TRANSMITER 1
#define RECEIVER 0


#define TIMEOUT_SECS 3
#define MAX_TIMEOUTS 3

#define STUFFING 0x7d

#define CONTROL(n)  ((n) << 6)
#define ACK(n)  ((n) << 7  | 0x05)
#define NACK(n)  ((n) << 7 | 0x01)



typedef enum{
    StateSTART,
    StateFLAG,
    StateA,
    StateC,
    StateBCC1,
    StateCReply,
    StateReplyData,
    StateSTOP
} State;

State state;

volatile int STOP = FALSE;
int alarm_enabled = FALSE;
int timeout_count = 0;
struct termios oldtio;
struct termios newtio;

unsigned int ns = 0;

void send_set(int fd){

    const unsigned char BCC1 = TRANSMITTER_COMMAND^SET;
    char su_buf[SU_BUF_SIZE] = {FLAG,TRANSMITTER_COMMAND,SET,BCC1,FLAG};
    write(fd, su_buf, SU_BUF_SIZE);
    printf("Sent set up frame\n");
    // Wait until all bytes have been written to the serial port
    sleep(1);

}

void receive_UA(State * state, unsigned char byte){
    
    unsigned char A = RECEIVER_REPLY;
    unsigned char C = UA;
    
    switch(*state){
    case StateSTART:
        if(byte == FLAG)
            *state = StateFLAG;
        
        break;
    case StateFLAG:
        if(byte == FLAG)
            *state = StateFLAG;
        else if(byte == A)
            *state = StateA;
        else
            *state = StateSTART;
        
        break;
    case StateA:
        if(byte == FLAG)
            *state = StateFLAG;
        else if(byte == C)
            *state = StateC;
        else
            *state = StateSTART;
        
        break;
    case StateC:
        if(byte == FLAG)
            *state = StateFLAG;
        else if(byte == (A^C))
            *state = StateBCC1;
        else
            *state = StateSTART;
        
        break;
        
    case StateBCC1:
        if(byte == FLAG)
            *state = StateSTOP;
        else
            *state = StateSTART;
        
        break;
    }

}


// Alarm function handler
void alarmHandler(int signal)
{
    alarm_enabled = FALSE;
    state = StateSTART;
    timeout_count++;
    printf("Timeout #%d\n", timeout_count);
}


void receive_set(State * state, unsigned char byte){
    
    unsigned char A = TRANSMITTER_COMMAND;
    unsigned char C = SET;
    
    switch(*state){
    case StateSTART:
        if(byte == FLAG)
            *state = StateFLAG;
        
        break;
    case StateFLAG:
        if(byte == FLAG)
            *state = StateFLAG;
        else if(byte == A)
            *state = StateA;
        else
            *state = StateSTART;
        
        break;
    case StateA:
        if(byte == FLAG)
            *state = StateFLAG;
        else if(byte == C)
            *state = StateC;
        else
            *state = StateSTART;
        
        break;
    case StateC:
        if(byte == FLAG)
            *state = StateFLAG;
        else if(byte == (A^C))
            *state = StateBCC1;
        else
            *state = StateSTART;
        
        break;
        
    case StateBCC1:
        if(byte == FLAG)
            *state = StateSTOP;
        else
            *state = StateSTART;
        
        break;
    }

}

int llopen(const char * port, int flag){
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    
    state = StateSTART;
    ns = 0;

    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(port);
        exit(-1);
    }


    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 1;  // Blocking read until 1 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");


    unsigned char in_char;
    

    if(flag == TRANSMITER){
        (void)signal(SIGALRM, alarmHandler);
        (void)siginterrupt(SIGALRM,TRUE); //system call interrupted by alarm isn't restarted
        // Loop for input
        unsigned char in_char;
        
        while (state != StateSTOP && timeout_count < MAX_TIMEOUTS)
        {

            if (alarm_enabled == FALSE)
            {
                send_set(fd);
                alarm(TIMEOUT_SECS); // Set alarm
                alarm_enabled = TRUE;
            }

            // Returns after 1 chars has been input
            read(fd, &in_char, 1);        
            receive_UA(&state,in_char);

        }

        alarm(0);

        if(timeout_count == MAX_TIMEOUTS){
            printf("Max timeouts exceeded\n");
        }
        else{
            printf("Received unnumbered acknowledgment frame\n");
            printf("Connection established\n");
        }

    }else{

        while (state != StateSTOP)
        {
            // Returns after 1 chars has been input
            read(fd, &in_char, 1);        
            receive_set(&state, in_char);
        }
        
        printf("Received set up frame\n");
        
        unsigned char bcc = RECEIVER_REPLY^UA;
        unsigned char su_buf[SU_BUF_SIZE] = {FLAG, RECEIVER_REPLY, UA, bcc, FLAG};
        
        write(fd, su_buf, SU_BUF_SIZE);
        printf("Sent unnumbered acknowledgment frame\n");

        // Wait until all bytes have been written to the serial port
        sleep(1);
        printf("Connection established\n");
    }
    return fd;
}

void sendACK(int fd, bool reply){
    printf("Send ack\n");
    // write ACK
    char C = ACK( (ns)?0:1);

    if(reply){
        C = ACK(ns);
    }
    
    char ack[] = {FLAG,0x03,C,0x03^C,FLAG};
    write(fd, ack, SU_BUF_SIZE);
}

void sendNACK(int fd){
    printf("Send Nack\n");
    // write NACK
    char C = NACK(ns);
    char ack[] = {FLAG,0x03,C,0x03^C,FLAG};
    write(fd, ack, SU_BUF_SIZE);
}

int llread(int fd, char * buffer, int max_size){

    unsigned char A = TRANSMITTER_COMMAND;

    unsigned char C_new = CONTROL(ns);
    unsigned char bcc1 = A^C_new;

    unsigned char C_old = CONTROL((ns==0)?1:0);
    unsigned char bcc1_old = A^C_old;

    bool is_stuffing = false;
    bool is_error = false;
    bool reply = false;
    
    u_char bcc2 = 0;

    size_t data_pos = 0;

    // read mensage

    state = StateSTART;

    while(state!=StateSTOP){
        char byte;
        read(fd, &byte, 1);  

        switch(state){
        case StateSTART:
            if(byte == FLAG)
                state = StateFLAG;
            
            break;
        case StateFLAG:
            if(byte == FLAG)
                state = StateFLAG;
            else if(byte == A)
                state = StateA;
            else
                state = StateSTART;
            
            break;
        case StateA:

            if(byte == FLAG)
                state = StateFLAG;
            else if(byte == C_new){
                state = StateC;
                reply = false;
            }else if(byte == C_old){
                state = StateCReply;
                reply = true;
            }else
                state = StateSTART;
            
            break;
        case StateC:
            if(byte == FLAG)
                state = StateFLAG;
            else if(byte == (A^C_new))
                state = StateBCC1;
            else
                state = StateSTART;
            
            break;
        case StateCReply:
            if(byte == FLAG)
                state = StateFLAG;
            else if(byte == (A^C_old))
                state = StateReplyData;
            else
                state = StateSTART;
            
            break;
        case StateReplyData:
            // fazer byte stuffing
            //printf("Reply \n");
            
            if(!is_stuffing && byte == FLAG){


                state = StateSTOP;
            }
            if(!is_stuffing && byte == STUFFING){
                is_stuffing = true;
            }else{
                is_stuffing = false;
            }
            
            break;
        case StateBCC1:
            // fazer byte stuffing
            
            if(!is_stuffing && byte == FLAG){
                state = StateSTOP;
                bcc2 ^= buffer[data_pos-1]; // antes tinha se feito xor com o bcc2 (enviado), agr faz-se outra vez para reverter esse xor 

                if(buffer[data_pos-1] !=bcc2){
                    is_error = true;        // send NACK
                }else{
                    is_error = false;       // send ACK
                }

                break;
            }


            if(!is_stuffing && byte == STUFFING){
                is_stuffing = true;
            }else{
                if(data_pos>=max_size){
                    is_error = true;        // sendNACK
                    state = StateSTOP;
                }else{
                    bcc2 ^= byte;
                    buffer[data_pos++] = byte;
                }
                is_stuffing = false;
            }   
            break;
        }
        
    }

    if(is_error){
        printf("sending nack");
        sendNACK(fd);
    }else{
        
        sendACK(fd, reply);
        printf("Ns: %d\n", ns);
        if(!reply)
            ns = (ns)?0:1;
        else
            printf("Recv Reply\n");
    }

    buffer[data_pos-1] = 0;

    printf("\nReceive %d bytes \n", data_pos);

    // Wait until all bytes have been written to the serial port
    sleep(1);

}


int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];
    const char *flag = argv[2];


    if (argc < 3)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort> <TRANSMITER/RECEIVER>\n"
               "Example: %s /dev/ttyS1 TRANSMITER \n",
               argv[0],
               argv[0]);
        exit(1);
    }
    int fd = -1;

    // conectar

    if(strncmp(argv[2], "TRANSMITER", 20) == 0){

    fd = llopen(serialPortName, TRANSMITER);
    
    }else if(strncmp(argv[2], "RECEIVER", 20) == 0){

        fd = llopen(serialPortName, RECEIVER);
    }else{
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort> <TRANSMITER/RECEIVER>\n"
               "Example: %s /dev/ttyS1 TRANSMITER \n",
               argv[0],
               argv[0]);
        exit(1);
    }
    
    // mandar I(0)
    printf("Reading mensage\n");
    char buffer[10000];
    while(1){
        llread(fd , buffer ,10000);
        printf("Message received: '%s'\n", buffer);
    }
    // Disconnect

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
