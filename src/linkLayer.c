// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include "link_layer.h"
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


LinkLayer linkLayer;

volatile int STOP = FALSE;
int alarm_enabled = FALSE;
int timeout_count = 0;
struct termios oldtio;
struct termios newtio;

unsigned int ns = 0;

#define ERROR_NACK (1<<0)
#define ERROR_REPLY (1<<1)

// Alarm function handler
void alarmHandler(int signal)
{
    alarm_enabled = FALSE;
    timeout_count++;
    printf("Timeout #%d\n", timeout_count);
}

void send_set(int fd){

    const unsigned char BCC1 = TRANSMITTER_COMMAND^SET;
    char su_buf[SU_BUF_SIZE] = {FLAG,TRANSMITTER_COMMAND,SET,BCC1,FLAG};
    write(fd, su_buf, SU_BUF_SIZE);
    printf("Sent set up frame\n");
    // Wait until all bytes have been written to the serial port

}

void send_UA(int fd){
    unsigned char bcc = TRANSMITTER_COMMAND^UA;
    unsigned char su_buf[SU_BUF_SIZE] = {FLAG, TRANSMITTER_COMMAND, UA, bcc, FLAG};
    write(fd, su_buf, SU_BUF_SIZE);
}

void send_disc(int fd){
    unsigned char bcc = TRANSMITTER_COMMAND^DISC;
    unsigned char su_buf[SU_BUF_SIZE] = {FLAG, TRANSMITTER_COMMAND, DISC, bcc, FLAG};
    write(fd, su_buf, SU_BUF_SIZE);
}



void send_data(int fd,char* buffer, int length){

    printf("Sending %d %d\n", ns, length);

    size_t frame_size = 6 + 2*length;
    unsigned char * frame = (unsigned char*)malloc(frame_size); 

    const unsigned char BCC1 = TRANSMITTER_COMMAND^CONTROL_FIELD(ns); 

    size_t numEsc = 0;
    frame[0] = FLAG; 
    frame[1] = TRANSMITTER_COMMAND; 
    frame[2] = CONTROL_FIELD(ns);
    if(BCC1==FLAG || BCC1==ESC){
        frame[3]=ESC;
        numEsc++;
    }

    frame[3+numEsc] = BCC1; 
    
    unsigned char BCC2 = 0; 

    for(size_t i = 0; i < length; i++) {
        frame[4 + i + numEsc] = buffer[i];
        if(buffer[i]==FLAG || buffer[i]==ESC){
            frame[4+i+(numEsc++)] = ESC;
        }
        frame[4+i+numEsc] = buffer[i];
        BCC2 ^= buffer[i]; 
    } 
    
    if(BCC2==FLAG || BCC2==ESC){
        frame[4 + length + numEsc] = ESC;
        numEsc++;
    }


    frame[4 + length + numEsc] = BCC2; 
    frame[4 + length + numEsc + 1] = FLAG; 


    write(fd,frame,6 + 2*length);
    free(frame);
    //sleep(1);
}

unsigned char A = 0;

void receive_ACK(U_S_State * state,  char * ack,unsigned char byte, int sn) {
    

    switch(*state){
        case U_S_StateSTART:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            break;
        case U_S_StateFLAG:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == 0x03){
                *ack=ACKN;
                A = 0x03;
                *state = U_S_StateA;
            }else if(byte == 0x01){
                A = 0x01;
                *ack=NACKN;
                *state = U_S_StateA;
            }else{
                *state = U_S_StateSTART;
            }
            break;
        case U_S_StateA:

            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == ACK(sn)  && *ack == ACKN) 
                *state = U_S_StateC;
            else if (byte == NACK(sn) && *ack == NACKN){
                *state = U_S_StateC;
            }
            else
                *state = U_S_StateSTART;
            break;
        case U_S_StateC:
            printf("%x %x\n", byte, A);

            if(byte == FLAG){
                *state = U_S_StateFLAG;
                *ack=FALSE;
            }
            else if(byte == (A^(*ack==ACKN?ACK(sn):NACK(sn))))
                *state = U_S_StateBCC1;
            else{
                *state = U_S_StateSTART;
                *ack=FALSE;
            }
            break;
            
        case U_S_StateBCC1:
            if(byte == FLAG)
                *state = U_S_StateSTOP;
            else{
                *state = U_S_StateSTART;
                *ack=FALSE;
            }
            break;
        default:
            break;
    }
    
}

int llwrite(int fd, const unsigned char *buffer, int bufSize) {

    unsigned char in_char;
    char ack=FALSE;
    U_S_State state = U_S_StateSTART;


    alarm_enabled = FALSE;
    timeout_count = 0;
    while (state != U_S_StateSTOP && timeout_count<linkLayer.nRetransmissions){
        if (alarm_enabled == FALSE)
        {
            (void)signal(SIGALRM, alarmHandler);
            send_data(fd,buffer,bufSize);
            alarm(linkLayer.timeout); // Set alarm
            state = U_S_StateSTART;
            alarm_enabled = TRUE;
        }

        // Returns after 1 chars has been input
        read(fd, &in_char, 1);        
        receive_ACK(&state,&ack,in_char,1-ns);
    }
    
    alarm(0); //  alarm
    
    if(ack==ACKN){
        ns=(1-ns);
        printf("Received ACK \n");
    }else if(ack==NACKN){
        printf("Received NACK \n");
        llwrite(fd,buffer,bufSize);
    }
    else{
        printf("Max timeouts Exceeded!\n");
        return -1;
    }
    return 0;
}



void receive_UA(U_S_State * state, unsigned char byte){
    
    unsigned char A = RECEIVER_REPLY;
    unsigned char C = UA;
    
    switch(*state){
        case U_S_StateSTART:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            
            break;
        case U_S_StateFLAG:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == A)
                *state = U_S_StateA;
            else
                *state = U_S_StateSTART;
            
            break;
        case U_S_StateA:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == C)
                *state = U_S_StateC;
            else
                *state = U_S_StateSTART;
            
            break;
        case U_S_StateC:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == (A^C))
                *state = U_S_StateBCC1;
            else
                *state = U_S_StateSTART;
            
            break;
            
        case U_S_StateBCC1:
            if(byte == FLAG)
                *state = U_S_StateSTOP;
            else
                *state = U_S_StateSTART;
            
            break;
        default:
            break;
    }

}

void receive_disc(U_S_State * state, unsigned char byte){
    
    unsigned char A = RECEIVER_REPLY;
    unsigned char C = DISC;
    
    switch(*state){
        case U_S_StateSTART:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            break;
        case U_S_StateFLAG:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == A)
                *state = U_S_StateA;
            else
                *state = U_S_StateSTART;
            
            break;
        case U_S_StateA:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == C)
                *state = U_S_StateC;
            else
                *state = U_S_StateSTART;
            
            break;
        case U_S_StateC:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == (A^C))
                *state = U_S_StateBCC1;
            else
                *state = U_S_StateSTART;
            
            break;
            
        case U_S_StateBCC1:
            if(byte == FLAG)
                *state = U_S_StateSTOP;
            else
                *state = U_S_StateSTART;
            
            break;
        default:
            break;
    }   

}





void receive_set(U_S_State * state, unsigned char byte){
    
    unsigned char A = TRANSMITTER_COMMAND;
    unsigned char C = SET;
    
    switch(*state){
        case U_S_StateSTART:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            
            break;
        case U_S_StateFLAG:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == A)
                *state = U_S_StateA;
            else
                *state = U_S_StateSTART;
            
            break;
        case U_S_StateA:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == C)
                *state = U_S_StateC;
            else
                *state = U_S_StateSTART;
            
            break;
        case U_S_StateC:
            if(byte == FLAG)
                *state = U_S_StateFLAG;
            else if(byte == (A^C))
                *state = U_S_StateBCC1;
            else
                *state = U_S_StateSTART;
            
            break;
            
        case U_S_StateBCC1:
            if(byte == FLAG)
                *state = U_S_StateSTOP;
            else
                *state = U_S_StateSTART;
            
            break;
        default:
            break;
    }

}

int llopen(LinkLayer connectionParameters){
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    
    memcpy(&linkLayer,&connectionParameters,sizeof(connectionParameters));
    U_S_State state = U_S_StateSTART;
    ns = 0;


    int fd = open(linkLayer.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(linkLayer.serialPort);
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

    newtio.c_cflag = linkLayer.baudRate | CS8 | CLOCAL | CREAD;
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
    

    if(linkLayer.role == LlTx){
        (void)signal(SIGALRM, alarmHandler);
        (void)siginterrupt(SIGALRM,TRUE); //system call interrupted by alarm isn't restarted
        // Loop for input
        unsigned char in_char;
        
        while (state != U_S_StateSTOP && timeout_count < linkLayer.nRetransmissions)
        {

            if (alarm_enabled == FALSE)
            {
                (void)signal(SIGALRM, alarmHandler);
                send_set(fd);
                alarm(linkLayer.timeout); // Set alarm
                state = U_S_StateSTART;
                alarm_enabled = TRUE;
            }

            // Returns after 1 chars has been input
            read(fd, &in_char, 1);        
            receive_UA(&state,in_char);

        }

        alarm(0);

        if(timeout_count == linkLayer.nRetransmissions){
            printf("Max timeouts exceeded\n");
            return -1;
        }
        else{
            printf("Received unnumbered acknowledgment frame\n");
            printf("Connection established\n");
        }

    }else{

        while (state != U_S_StateSTOP)
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
        
        printf("Connection established\n");
    }
    return fd;
}

void sendACK(int fd, bool reply){
    printf("Send ack\n");
    // write ACK
    char C = ACK( (1-ns));

    if(reply){
        C = ACK(ns);
    }
    
    char ack[] = {FLAG,0x03,C,0x03^C,FLAG};
    write(fd, ack, SU_BUF_SIZE);
}

void sendNACK(int fd){
    printf("Send Nack\n");
    // write NACK
    char C = NACK((ns)?0:1);
    char ack[] = {FLAG,0x01,C,0x01^C,FLAG};
    write(fd, ack, SU_BUF_SIZE);
}

int receive_message(int fd, unsigned char * buffer, int* error){
    IState state = IStateSTART;
    size_t data_pos = 0;

    unsigned char A = TRANSMITTER_COMMAND;

    unsigned char C_new = CONTROL(ns);

    unsigned char C_old = CONTROL((ns==0)?1:0);

    bool is_stuffing = false;
    bool is_error = false;
    bool reply = false;
    
    unsigned char bcc2 = 0;

    while(state!=IStateSTOP){
        char byte;
        read(fd, &byte, 1);  

        switch(state){
            case IStateSTART:
                if(byte == FLAG)
                    state = IStateFLAG;
                
                break;
            case IStateFLAG:
                if(byte == FLAG)
                    state = IStateFLAG;
                else if(byte == A)
                    state = IStateA;
                else
                    state = IStateSTART;
                
                break;
            case IStateA:

                if(byte == FLAG)
                    state = IStateFLAG;
                else if(byte == C_new){
                    state = IStateC;
                    reply = false;
                }else if(byte == C_old){
                    state = IStateCReply;
                    reply = true;
                }else
                    state = IStateSTART;
                
                break;
            case IStateC:
                if(byte == FLAG)
                    state = IStateFLAG;
                else if(byte == (A^C_new))
                    state = IStateBCC1;
                else
                    state = IStateSTART;
                
                break;
            case IStateCReply:
                if(byte == FLAG)
                    state = IStateFLAG;
                else if(byte == (A^C_old))
                    state = IStateReplyData;
                else
                    state = IStateSTART;
                
                break;
            case IStateReplyData:
                // fazer byte stuffing
                //printf("Reply \n");
                
                if(!is_stuffing && byte == FLAG){


                    state = IStateSTOP;
                }
                if(!is_stuffing && byte == STUFFING){
                    is_stuffing = true;
                }else{
                    is_stuffing = false;
                }
                
                break;
            case IStateBCC1:
                // fazer byte stuffing
                
                if(!is_stuffing && byte == FLAG){
                    state = IStateSTOP;
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
                    if(data_pos>=MAX_PAYLOAD_SIZE){
                        is_error = true;        // sendNACK
                        state = IStateSTOP;
                    }else{
                        bcc2 ^= byte;
                        buffer[data_pos++] = byte;
                    }
                    is_stuffing = false;
                }   
                break;

            default:
                break;
        }  
    
    }

    *error = 0;
    if (reply)
        *error |= ERROR_REPLY;
    
    if (is_error)
        *error |= ERROR_NACK;
    

    data_pos-=1;
    buffer[data_pos-1] = 0;
    return data_pos;
}


int llread(int fd, unsigned char * buffer){
    int error = 0;

    // read menssage
    size_t data_pos = receive_message(fd, buffer, &error);

    if(error & ERROR_NACK){
        printf("Sending NACK %d\n ", ns);
        sendNACK(fd);
        return llread(fd, buffer);
    }else if(error & ERROR_REPLY){
        sendACK(fd, true);
        printf("Ns: %d\n", ns);
        printf("Recv Reply\n");
        return llread(fd, buffer);
    }

    
    sendACK(fd, false);
    printf("Ns: %d\n", ns);
    ns = (ns)?0:1;

    printf("Receive %ld bytes \n\n", data_pos);

    //sleep(1);
    // Wait until all bytes have been written to the serial port
    return data_pos;

}

int llclose(int fd, int showStatistics, LinkLayerRole ll){
    int ret=0;
    char in_char;
    U_S_State state = U_S_StateSTART;
    if(ll==LlTx){
        alarm_enabled = FALSE;
        timeout_count = 0;
        while (state != U_S_StateSTOP && timeout_count < linkLayer.nRetransmissions){
            if (alarm_enabled == FALSE)
                {
                    (void)signal(SIGALRM, alarmHandler);
                    printf("Sending DISC\n");
                    send_disc(fd);
                    alarm(linkLayer.timeout); // Set alarm
                    state = U_S_StateSTART;
                    alarm_enabled = TRUE;
                }
            // Returns after 1 chars has been input
            read(fd, &in_char, 1);        
            receive_disc(&state,in_char);
        }
        alarm(0); //  alarm
        if(state!=U_S_StateSTOP){
            printf("Max timeouts Exceeded!\n");
            ret=-1;
        }
        else{
            printf("Received DISC\n");
            send_UA(fd);
            printf("Sending UA\n");
        }
    }
    else{
        alarm_enabled = FALSE;
        timeout_count = 0;
        while (state != U_S_StateSTOP){
            read(fd, &in_char, 1);        
            receive_disc(&state,in_char);
        }
        printf("Received DISC\n");
        alarm(0); //  alarm
        alarm_enabled = FALSE;
        state=U_S_StateSTART;
        timeout_count = 0;
        while (state != U_S_StateSTOP){
            if (alarm_enabled == FALSE && timeout_count < linkLayer.nRetransmissions)
                {
                    (void)signal(SIGALRM, alarmHandler);
                    send_disc(fd);
                    printf("Sending DISC\n");
                    alarm(linkLayer.timeout); // Set alarm
                    state = U_S_StateSTART;
                    alarm_enabled = TRUE;
                }
            // Returns after 1 chars has been input
            read(fd, &in_char, 1);        
            receive_UA(&state,in_char);
        }
        if(state!=U_S_StateSTOP){
            printf("Max timeouts Exceedefd!\n");
            ret=-1;
        }
        else{
            printf("Received UA\n"); 
        }
        alarm(0); //  alarm
    }

     // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return ret;
}
