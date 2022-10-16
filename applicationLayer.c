#include <stdlib.h>
#include <stdio.h> 
#include <math.h>
#include <string.h>
#include "linkLayer.c"

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


#define TRANSMITTER 1
#define RECEIVER 0


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
} applicationLayer;


double log256(double x){
    return log2(x)*LOG256_2;
}

int ipow(int base, int exp)
{
    int result = 1;
    for(unsigned char i=1;i<=exp;i++){
        result*=base;
    }
    return result;
}

void sendPacket(int fd,unsigned char * buffer,unsigned char sequence_number){   
    buffer[0] = DATA;
    buffer[1] = sequence_number;
    buffer[2] = DATA_SIZE/256;
    buffer[3] = DATA_SIZE%256;
    llwrite(fd,buffer,DATA_SIZE);
}

void sendControl(int fd,unsigned char c,char* file_name, size_t file_size){
    printf("started math");
    unsigned char file_size_bytes = (int)log256((double)file_size) + 1;
    printf("finished math");
    unsigned int file_name_size =  strlen(file_name)+1;
    unsigned int control_DATA_SIZE = 5+file_size_bytes+file_name_size;
    unsigned char* control_packet = (unsigned char*) malloc(control_DATA_SIZE);

    control_packet[0] = c;
    control_packet[1] = FILE_SIZE;
    control_packet[2] = file_size_bytes;

    unsigned char exponent = file_size_bytes - 1;

    for(int i=0;i<file_size_bytes;i++){
        control_packet[3+i]= file_size/ipow(256,exponent);
        exponent--;
        file_size%=ipow(256,exponent);
    }

    strncpy(control_packet+3+file_size_bytes,file_name,file_name_size);

    printf("writing");
    llwrite(fd,control_packet,control_DATA_SIZE);
    printf("Finished writing");
    free(control_packet);
}


void receiveFile(StateReceive * StateReceive,int fd,char * buffer){
    llread(fd,buffer,DATA_SIZE+4);
    switch(*StateReceive){
        case StateReceiveDEFAULT:
            if(buffer[0]==START)
                *StateReceive = StateReceiveSTART;
            else
                *StateReceive = StateReceiveERROR;
        case StateReceiveSTART:
            if(buffer[0]==DATA)
                *StateReceive = StateReceiveDATA;
            else
                *StateReceive = StateReceiveERROR;
            break;
        case StateReceiveDATA:
            if(buffer[0]==END)
                *StateReceive=StateReceiveEND;
            else if(buffer[0]==DATA)
                break;
            else
                *StateReceive=StateReceiveERROR;
            break;
    }
}

void read_control_packet(StateReceive*StateReceive,unsigned char *buffer,unsigned long long *file_size,char* file_name){
    if(buffer[1]!=FILE_SIZE){
        printf("Error getting control packet\n");
        *StateReceive = StateReceiveERROR;
    }

    unsigned char file_size_bytes = buffer[2];
    *file_size = 0;
    for(unsigned char i=0;i<file_size_bytes;i++){
        unsigned char exponent = (file_size_bytes-1) - i;
        *file_size += buffer[2+i]*ipow(8,exponent);
    }

    size_t second_parameter_offset = 3+file_size_bytes;

    if(buffer[second_parameter_offset]!=FILE_NAME){
        printf("Error getting control packet\n");
        *StateReceive = StateReceiveERROR;
    }

    unsigned int file_name_size = buffer[second_parameter_offset+1];
    strncpy(file_name,buffer+second_parameter_offset+2,file_name_size);
}

int main(int argc,char* argv[]) {
    applicationLayer appLayer; 
    appLayer.status = atoi(argv[2]); 
    appLayer.fileDescriptor = llopen(argv[3], appLayer.status); 
    unsigned char sequence_number = 0;
    unsigned int data_packet_size = DATA_SIZE + 4;
    unsigned char* buffer = (unsigned char*) malloc(data_packet_size);


    if(appLayer.status==RECEIVER){   
        printf("Receiver\n");    
        StateReceive StateReceive = StateReceiveSTART;
        unsigned char* start_file_name = (unsigned char*) malloc(MAX_FILENAME_SIZE);
        unsigned char* end_file_name = (unsigned char*)malloc(MAX_FILENAME_SIZE);
        unsigned long long start_file_size,end_file_size;
        unsigned int data_size;
        int fd;
        while (StateReceive!=StateReceiveEND){
            receiveFile(&StateReceive,appLayer.fileDescriptor,buffer);
            if(StateReceiveERROR)
                break;
            switch(StateReceive){
                case StateReceiveSTART:
                    read_control_packet(&StateReceive,buffer,&start_file_size,start_file_name);
                    fd = open(start_file_name,"w");
                case StateReceiveDATA:
                    data_size = 256*buffer[2] + buffer[3];
                    if(buffer[1]!=sequence_number){
                        StateReceive=StateReceiveERROR;
                        printf("Got wrong sequence number\n");
                        break;
                    }
                    sequence_number++;
                    write(fd,buffer+4,data_size);
                case StateReceiveEND:
                    read_control_packet(&StateReceive,buffer,&end_file_size,end_file_name);
                    close(fd);
                    if(start_file_size!=end_file_size)
                        printf("Error getting file size\n");
                    if(strcmp(start_file_name,end_file_name))
                        printf("Error getting file name\n");
            }
        }
        free(start_file_name);
        free(end_file_name);
    }


    else{
        FILE* file = fopen(argv[1],"rb");
        fseek(file, 0L, SEEK_END);
        int file_size = ftell(file);
        fseek(file, 0L, SEEK_SET);
        printf("Finished reading file size");
        sendControl(appLayer.fileDescriptor,START,argv[1],file_size);
        while(fread(buffer+4, 1,DATA_SIZE,file) != 0){
            printf("sending packet\n");
            sendPacket(appLayer.fileDescriptor,buffer,sequence_number);
            sequence_number++;
        }
        sendControl(appLayer.fileDescriptor,END,argv[1],file_size);
    }
    
    free(buffer);
    llclose(appLayer.fileDescriptor); 
    return 0; 
}

