#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "link_layer.h"
#include "application_layer.h"


int ipow(int base, int exp)
{
    int result = 1;
    for(unsigned char i=1;i<=exp;i++){
        result*=base;
    }
    return result;
}

unsigned char log256(unsigned long long x){
    unsigned char count=-1;
    while(x!=0){
        count++;
        x/=256;
    }
    return count;

}

void sendPacket(int fd,unsigned char * buffer,unsigned char sequence_number, size_t size){   
    buffer[0] = DATA;
    buffer[1] = sequence_number;
    buffer[2] = size/256;
    buffer[3] = size%256;
    if (llwrite(fd,buffer,size+4) == -1) exit(-1);
}




void sendControl(int fd,unsigned char c,const char* file_name, unsigned long long file_size){
    printf("start\n");
    unsigned char file_size_bytes = log256(file_size) + 1;
    unsigned int file_name_size =  strlen(file_name)+1;
    unsigned int control_DATA_SIZE = 5+file_size_bytes+file_name_size;
    char* control_packet = (char*) malloc(control_DATA_SIZE);

    control_packet[0] = c;

    // sending file size
    control_packet[1] = FILE_SIZE;
    control_packet[2] = file_size_bytes;

    unsigned char exponent = file_size_bytes - 1;

    printf("%d\n",file_size_bytes);
    for(int i=0;i<file_size_bytes;i++){
        printf("Byte %d\n",i);
        control_packet[3+i]= file_size/ipow(256,exponent);
        exponent--;
        file_size %= 256;
    }
    

    // sending file name
    printf("Finished writing file size\n");
    size_t second_parameter_offset = 3+file_size_bytes;
    control_packet[second_parameter_offset]=FILE_NAME;
    control_packet[second_parameter_offset+1]=file_name_size;
    printf("before strncpy\n");
    strncpy(control_packet+second_parameter_offset+2,file_name,file_name_size);
    control_packet[control_DATA_SIZE]='\0';
    printf("after strncpy\n");

    printf("writing\n");

    // sending control packet
    if(llwrite(fd,control_packet,control_DATA_SIZE)==-1) exit(0);
    printf("Finished writing\n");
    free(control_packet);
}

void read_control_packet(StateReceive*StateReceive,unsigned char *buffer,unsigned long long *file_size,char* file_name){

    // receiving file size
    if(buffer[1]!=FILE_SIZE){
        printf("Error getting control packet file size\n");
        *StateReceive = StateReceiveERROR;
    }
    printf("Print 1\n");

    unsigned char file_size_bytes = buffer[2];
    *file_size = 0;
    for(unsigned char i=0;i<file_size_bytes;i++){
        unsigned char exponent = (file_size_bytes-1) - i;
        *file_size += buffer[2+i]*ipow(256,exponent);
    }
    printf("Print 2\n");
    
    
    // receiving file name
    size_t second_parameter_offset = 3+file_size_bytes;

    if(buffer[second_parameter_offset]!=FILE_NAME){
        printf("Error getting control packet file name\n");
        *StateReceive = StateReceiveERROR;
    }

    
    unsigned int file_name_size = buffer[second_parameter_offset+1];
    printf("%d  %d\n", file_name_size, second_parameter_offset);
    
    strncpy(file_name,buffer+second_parameter_offset+2,file_name_size);
    
    printf("STRCPY \n");
}




void receiveFile(StateReceive * stateReceive,int fd,unsigned char * buffer){
    llread(fd,buffer);
    switch(*stateReceive){
        case StateReceiveDEFAULT:
            printf("DEFAULT\n");
            printf("first byte %d\n",buffer[0]);
            if(buffer[0]==START)
                *stateReceive = StateReceiveSTART;
            else
                *stateReceive = StateReceiveERROR;
            break;
        case StateReceiveSTART:
            printf("START\n");
            if(buffer[0]==DATA)
                *stateReceive = StateReceiveDATA;
            else
                *stateReceive = StateReceiveERROR;
            break;
        case StateReceiveDATA:
            printf("DATA\n");
            if(buffer[0]==END)
                *stateReceive=StateReceiveEND;
            else if(buffer[0]==DATA)
                break;
            else
                *stateReceive=StateReceiveERROR;
            break;
    }
    printf("State %d\n",*stateReceive);
}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    ApplicationLayer appLayer;
    LinkLayerRole llRole;
    if(strcmp("tx",role)==0){
        appLayer.status = TRANSMITTER;
        llRole = LlTx;
    }
    else if (strcmp("rx",role)==0){
        appLayer.status = RECEIVER;
        llRole = LlRx;
    } 
    else
        exit(-1);
    

    LinkLayer connectionParameters;
    strncpy(connectionParameters.serialPort, serialPort, 50);
    connectionParameters.role = llRole;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    
    appLayer.fileDescriptor = llopen(connectionParameters);
    if(appLayer.fileDescriptor==-1){
        printf("Error llopen\n");
        return; 
    }

    char sequence_number = 0;
    unsigned char* buffer = (unsigned char*) malloc(MAX_PAYLOAD_SIZE*2);


    if(appLayer.status==RECEIVER){   
        printf("Receiver\n");    
        StateReceive stateReceive = StateReceiveDEFAULT;
        char start_file_name[MAX_FILENAME_SIZE];
        char end_file_name[MAX_FILENAME_SIZE];
        unsigned long long start_file_size,end_file_size;
        unsigned int data_size;
        FILE* file;
        while (stateReceive!=StateReceiveEND){
            receiveFile(&stateReceive,appLayer.fileDescriptor,buffer);
            printf("RECEIVE\n");
            if(stateReceive==StateReceiveERROR)
                break;
            switch(stateReceive){
                case StateReceiveSTART:
                    read_control_packet(&stateReceive,buffer,&start_file_size,start_file_name);
                    file = fopen(filename,"w");

                    if(file==NULL){
                        printf("Error opening file %s\n",filename);
                        return;
                    }else{
                        printf("Criou ficheiro\n");    
                    }
                    break;
                case StateReceiveDATA:
                    data_size = 256*buffer[2] + buffer[3];
                    if(buffer[1]!=sequence_number){
                        stateReceive=StateReceiveERROR;
                        printf("Got wrong sequence number\n");
                        break;
                    }
                    sequence_number++;
                    printf("Escrever Ficheiro %d\n", data_size);
                    fwrite(buffer+4,1,data_size, file);
                    printf("Fim Escrever Ficheiro\n");          
                    break;
                case StateReceiveEND:
                    read_control_packet(&stateReceive,buffer,&end_file_size,end_file_name);
                    printf("COMPARACOES INICIO\n");
                    if(start_file_size != end_file_size)
                        printf("Error getting file size %d %d\n",start_file_size,end_file_size);
                    if(strcmp(start_file_name,end_file_name) != 0)
                        printf("Error getting file name\n");

                    printf("COMPARACOES FIM\n");
                    break;
                    
            }
        }
        fclose(file);
    }else{
        FILE* file = fopen(filename,"rb");
        if(file==NULL){
            printf("Error opening file %s\n",filename);
            return;
        }

        fseek(file, 0, SEEK_END);
        unsigned long long file_size = ftell(file);
        printf("File size%ld\n",file_size);
        fclose(file);

        file = fopen(filename,"rb");
        if(file==NULL){
            printf("Error opening file %s\n",filename);
            return ;
        }
        
        sendControl(appLayer.fileDescriptor,START,filename,file_size);
        
        printf("sendControl\n");
        size_t bytes_read = 0;
        while((bytes_read=fread(buffer+4, 1,MAX_PAYLOAD_SIZE-10,file)) != 0){
            printf("sending packet\n");
            sendPacket(appLayer.fileDescriptor,buffer,sequence_number,bytes_read);
            sequence_number++;
        }
        sendControl(appLayer.fileDescriptor,END,filename,file_size);
        fclose(file);
    }
    free(buffer);
    llclose(appLayer.fileDescriptor,0,llRole); 
}






