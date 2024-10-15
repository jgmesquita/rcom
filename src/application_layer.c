// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
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

            FILE *fileStream = fopen(filename, "rb");
            if (fileStream == NULL) {
                perror("File not found\n");
                exit(-1);
            }

            int currentPos = ftell(fileStream);
            fseek(fileStream, 0L, SEEK_END);
            long int totalFileSize = ftell(fileStream) - currentPos;
            fseek(fileStream, currentPos, SEEK_SET);

            unsigned int controlPacketSize;
            unsigned char *startControlPacket = createControlPacket(2, filename, totalFileSize, &controlPacketSize);
            if (llwrite(startControlPacket, controlPacketSize) == -1) {
                printf("Exit: error in start packet\n");
                exit(-1);
            }


            unsigned char seqNum = 0;
            unsigned char *fileContent = loadData(fileStream, totalFileSize);
            long int remainingBytes = totalFileSize;

            while (remainingBytes >= 0) {

                int chunkSize = remainingBytes > (long int) MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : remainingBytes;
                unsigned char *dataChunk = (unsigned char *)malloc(chunkSize);
                memcpy(dataChunk, fileContent, chunkSize);
                int dataPacketSize;
                unsigned char *dataPacket = createDataPacket(seqNum, dataChunk, chunkSize, &dataPacketSize);

                if (llwrite(dataPacket, dataPacketSize) == -1) {
                    printf("Exit: error in data packets\n");
                    exit(-1);
                }

                remainingBytes -= (long int) MAX_PAYLOAD_SIZE;
                fileContent += chunkSize;
                seqNum = (seqNum + 1) % 255;
            }

            unsigned char *endControlPacket = createControlPacket(3, filename, totalFileSize, &controlPacketSize);
            if (llwrite(endControlPacket, controlPacketSize) == -1) {
                printf("Exit: error in end packet\n");
                exit(-1);
            }
            llclose(linkDescriptor);
            break;
        }

        case LlRx: {

            unsigned char *receivedPacket = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
            int packetLength = -1;
            printf("About to read!\n");
            packetLength = llread(receivedPacket);
            printf("out!\n");
            unsigned long int receivedFileSize = 0;
            unsigned char *receivedFilename = processControlPacket(receivedPacket, packetLength, &receivedFileSize);

            FILE *outputFile = fopen((char *)receivedFilename, "wb+");
            while (1) {
                while ((packetLength = llread(receivedPacket)) < 0);
                if (packetLength == 0) break;
                else if (receivedPacket[0] != 3) {
                    unsigned char *dataBuffer = (unsigned char *)malloc(packetLength);
                    processDataPacket(receivedPacket, packetLength, dataBuffer);
                    fwrite(dataBuffer, sizeof(unsigned char), packetLength - 4, outputFile);
                    free(dataBuffer);
                }
                else continue;
            }

            fclose(outputFile);
            break;

        default:
            exit(-1);
            break;
    }
    }
}

unsigned char *processControlPacket(unsigned char *packet, int size, unsigned long int *fileSize) {

    unsigned char fileSizeByteCount = packet[2];
    unsigned char fileSizeBuffer[fileSizeByteCount];
    memcpy(fileSizeBuffer, packet + 3, fileSizeByteCount);
    for (unsigned int i = 0; i < fileSizeByteCount; i++)
        *fileSize |= (fileSizeBuffer[fileSizeByteCount - i - 1] << (8 * i));

    unsigned char filenameByteCount = packet[3 + fileSizeByteCount + 1];
    unsigned char *filename = (unsigned char *)malloc(filenameByteCount);
    memcpy(filename, packet + 3 + fileSizeByteCount + 2, filenameByteCount);
    return filename;
}

unsigned char *createControlPacket(const unsigned int controlType, const char *filename, long int fileSize, unsigned int *packetSize) {

    const int fileSizeLength = 1;
    const int filenameLength = strlen(filename);
    *packetSize = 1 + 2 + fileSizeLength + 2 + filenameLength;
    unsigned char *packet = (unsigned char *)malloc(*packetSize);

    unsigned int pos = 0;
    packet[pos++] = controlType;
    packet[pos++] = 0;
    packet[pos++] = fileSizeLength;

    for (unsigned char i = 0; i < fileSizeLength; i++) {
        packet[2 + fileSizeLength - i] = fileSize & 0xFF;
        fileSize >>= 8;
    }
    pos += fileSizeLength;
    packet[pos++] = 1;
    packet[pos++] = filenameLength;
    memcpy(packet + pos, filename, filenameLength);
    return packet;
}

unsigned char *createDataPacket(unsigned char sequenceNumber, unsigned char *data, int dataSize, int *packetSize) {

    *packetSize = 1 + 1 + 2 + dataSize;
    unsigned char *packet = (unsigned char *)malloc(*packetSize);

    packet[0] = 1;
    packet[1] = sequenceNumber;
    packet[2] = dataSize >> 8 & 0xFF;
    packet[3] = dataSize & 0xFF;
    memcpy(packet + 4, data, dataSize);

    return packet;
}

unsigned char *loadData(FILE *file, long int fileSize) {
    unsigned char *fileContent = (unsigned char *)malloc(sizeof(unsigned char) * fileSize);
    fread(fileContent, sizeof(unsigned char), fileSize, file);
    return fileContent;
}

void processDataPacket(const unsigned char *packet, const unsigned int packetSize, unsigned char *buffer) {
    memcpy(buffer, packet + 4, packetSize - 4);
    buffer += packetSize + 4;
}