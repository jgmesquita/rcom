// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

unsigned char tramaTx = 0;
unsigned char tramaRx = 1;
volatile int STOP = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;
int timeout = 0;
int nRetransmissions = 0;
int fd;


void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    /* This will open the socket between the computers */
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate)
    if (fd < 0) {
        return -1;
    }

    /* Start of the state machine */
    LinkStateMachine state = START;

    /* This will store the information about the connection */
    LinkLayerRole role = connectionParameters.role;
    int baudRate = connectionParameters.baudRate;
    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    /* Store our current byte */
    unsigned char byte;

    /* Handle if we are hadling the transmiter (role = LlTx) or the receiver (role = LlRx) */
    switch (role) {
        case LlTx:
            (void) signal(SIGALRM, alarmHandler);
            while (nRetransmissions != 0 && state != SSTOP) {
                unsigned char frame[5] = {FLAG, A_TR, C_SET, A_TR ^ C_SET, FLAG};
                write(fd, frame, 5);
                alarm(timeout);
                alarmEnabled = TRUE;
                while (alarmEnabled == TRUE && state != SSTOP) {
                    if (read(fd, &byte, 1) > 0) {
                        switch (state) {
                            case START:
                                if (byte == FLAG) {
                                    state = FLAG_RCV;
                                }
                                break;
                            case FLAG_RCV:
                                if (byte == A_RT) {
                                    state = A_RCV;
                                }
                                else if (byte != FLAG) {
                                    state = START;
                                }
                                break;
                            case A_RCV:
                                if (byte == C_UA) {
                                    state = C_RCV;
                                }
                                else if (byte == FLAG) {
                                    state = FLAG_RCV;
                                }
                                else {
                                    state = START;
                                }
                                break;
                            case C_RCV:
                                if (byte == (A_RT ^ C_UA)) {
                                    state = BCC_OK;
                                }
                                else if (byte == FLAG) {
                                    state = FLAG_RCV;
                                }
                                else {
                                    state = START;
                                }
                                break;
                            case BCC_OK:
                                if (byte == FLAG) {
                                    state = SSTOP;
                                }
                                else {
                                    state = START;
                                }
                                break;
                            default: 
                                break;
                        }
                    }
                } 
                nRetransmissions--;
            }
            break;  
        case LlRx:
            while (state != SSTOP) {
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) {
                                state = FLAG_RCV;
                            }
                            break;
                        case FLAG_RCV:
                            if (byte == A_ER) {
                                state = A_RCV;
                            }
                            else if (byte != FLAG) {
                                state = START;
                            }
                            break;
                        case A_RCV:
                            if (byte == C_SET) {
                                state = C_RCV;
                            }
                            else if (byte == FLAG) {
                                state = FLAG_RCV;
                            }
                            else {
                                state = START;
                            }
                            break;
                        case C_RCV:
                            if (byte == (A_TR ^ C_SET)) {
                                state = BCC_OK;
                            }
                            else if (byte == FLAG) {
                                state = FLAG_RCV;
                            }
                            else {
                                state = START;
                            }
                            break;
                        case BCC_OK:
                            if (byte == FLAG) {
                                state = SSTOP;
                            }
                            else {
                                state = START;
                            }
                            break;
                        default: 
                            break;
                    }
                }
            }  
            unsigned char frame[5] = {FLAG, A_RT, C_UA, A_RT ^ C_UA, FLAG};
            write(fd, frame, 5);
            break; 
        default:
            return -1;
            break;
    }
    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    /* Make sure it has enough space for the main components! */
    int frameSize = 6+bufSize; 

    /*  Allocate the space in memory */
    unsigned char *frame = (unsigned char *)malloc(frameSize); 

    /* Set the main components */
    frame[0] = FLAG;
    frame[1] = A_TR;
    frame[2] = C_N(tramaTx);
    frame[3] = FLAG ^ A_TR;

    /* Copy directly into the frame from the buffer */
    memcpy(frame+4,buf, bufSize);
    
    /* Use strategy explained in sliders to ensure the message is correct later */
    unsigned char BCC2 = buf[0];
    for (unsigned int i = 1 ; i < bufSize ; i++) {
        BCC2 ^= buf[i];
    }

    /* Current size of the frame */
    int size = 4;

    /* Adapt size of the frame */
    for (unsigned int i = 0 ; i < bufSize ; i++) {
        if (buf[i] == FLAG || buf[i] == ESC) {
            frame = realloc(frame,++frameSize);
            frame[size++] = ESC;
        }
        frame[size++] = buf[i];
    }

    /* Add final components */
    frame[size++] = BCC2;
    frame[size++] = FLAG;

    /* Bool values to handle current status */
    int current = 0;
    int rejected = 0;
    int accepted = 0;

    while (current < nRetransmissions) { 
        alarmEnabled = TRUE;
        alarm(timeout);
        rejected = 0;
        accepted = 0;
        while (alarmEnabled == TRUE && !rejected && !accepted) {
            write(fd, frame, size);
            unsigned char result = controlRead(fd);
            if (!result) {
                continue;
            }
            else if (result == C_REJ0 || result == C_REJ1) {
                rejected = 1;
            }
            else if (result == C_RR0 || result == C_RR1) {
                accepted = 1;
                tramaTx = (tramaTx+1) % 2;
            }
            else {
                continue;
            }
        }
        if (accepted) {
            break;
        }
        current++;
    }
    
    /* No memory leak! */
    free(frame);

    if (accepted) {
        return frameSize;
    }
    else{
        llclose(fd);
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}

/* Auxiliary Function */
unsigned char controlRead(int fd){
    unsigned char byte = 0;
    unsigned char temp = 0;
    LinkLayerState state = START;
    while (state != SSTOP && alarmEnabled == TRUE) {  
        if (read(fd, &byte, 1) > 0 || 1) {
            switch (state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_RE) {
                        state = A_RCV;
                    }
                    else if (byte != FLAG) {
                        state = START;
                    }
                    break;
                case A_RCV:
                    if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == c_DISC) {
                        state = C_RCV;
                        temp = byte;   
                    }
                    else if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    else {
                        state = START;
                    }
                    break;
                case C_RCV:
                    if (byte == (A_RT ^ temp)) {
                        state = BCC_OK;
                    }
                    else if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    else {
                        state = START;
                    }
                    break;
                case BCC_OK:
                    if (byte == FLAG){
                        state = SSTOP;
                    }
                    else {
                        state = START;
                    }
                    break;
                default: 
                    break;
            }
        } 
    } 
    return temp;
}