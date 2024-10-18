// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "serial_port.h"
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

extern int alarmEnabled;
extern int alarmCount;
extern int timeout;
extern int nRetransmissions;

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    LinkLayer linkLayerConfig;
    strcpy(linkLayerConfig.serialPort, serialPort);
    linkLayerConfig.role = strcmp(role, "tx") ? LlRx : LlTx;
    linkLayerConfig.baudRate = baudRate;
    linkLayerConfig.nRetransmissions = nTries;
    linkLayerConfig.timeout = timeout;

    int linkDescriptor = llopen(linkLayerConfig);
    if (linkDescriptor < 0) {
        perror("Connection error\n");
        exit(-1);
    }

    switch (linkLayerConfig.role) {

        case LlTx: {
            char f[] = {0x01, 0x02, 0x03};
            int i = llwrite(f, 3);
            if (i == -1) {
                printf("Exit: error in start packet\n");
                exit(-1);
            }

            llclose(linkDescriptor);
            break;
        }

        case LlRx: {

            char *receivedPacket = (char *)malloc(4);
            int packetLength = -1;
            printf("About to read!\n");
            packetLength = llread(receivedPacket);
            printf("Package Received: ");
            for (int j = 0; j < packetLength; j++) {
                printf("%d ", receivedPacket[j]);
            }
            break;
        }

        default:
            exit(-1);
            break;
    }
}
