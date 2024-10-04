// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

volatile int STOP = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;

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
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate)
    if (fd < 0) {
        return -1;
    }

    /* Start of the state machine */
    LinkStateMachine state = START;

    /* This will store the information about the connection */
    LinkLayerRole role = connectionParameters.role;
    int baudRate = connectionParameters.baudRate;
    int nRetransmissions = connectionParameters.nRetransmissions;
    int timeout = connectionParameters.timeout;

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
    // TODO

    return 0;
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
