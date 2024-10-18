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


void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm %d is ticking!\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    /* This will open the socket between the computers */
    
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        return -1;
    }
    

    /* Start of the state machine */
    LinkStateMachine state = START;

    /* This will store the information about the connection */
    LinkLayerRole role = connectionParameters.role;
    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    /* Store our current byte */
    char byte = 0;

    /* Handle if we are hadling the transmiter (role = LlTx) or the receiver (role = LlRx) */
    switch (role) {
        case LlTx:
            (void)signal(SIGALRM, alarmHandler);
            while (nRetransmissions > 0 && state != SSTOP) {
                printf("Retransmission: %d\n", nRetransmissions);
                char frame[] = {FLAG, A_TR, C_SET, A_TR ^ C_SET, FLAG};
                writeBytes(frame, 5);
                alarm(timeout);
                alarmEnabled = TRUE;
                while (alarmEnabled == TRUE && state != SSTOP) {
                    if (readByte(&byte) > 0) {
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
                                    printf("Connection was successful!\n");
                                    alarmCount = 0;
                                    nRetransmissions = connectionParameters.nRetransmissions;
                                    alarm(0);
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
                if (readByte(&byte) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) {
                                state = FLAG_RCV;
                            }
                            break;
                        case FLAG_RCV:
                            if (byte == A_TR) {
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
            char frame[5] = {FLAG, A_RT, C_UA, A_RT ^ C_UA, FLAG};
            writeBytes(frame, 5);
            printf("Connection was successful!\n");
            break; 
        default:
            return -1;
            break;
    }
    if (nRetransmissions == 0) {
        return -1;
    }
    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const char *buf, int bufSize)
{
    /* Make sure it has enough space for the main components! */
    int frameSize = bufSize + 6; 

    /*  Allocate the space in memory */
    char *frame = (char *)malloc(frameSize); 

    /* Set the main components */
    frame[0] = FLAG;
    frame[1] = A_TR;
    frame[2] = C_I0;
    frame[3] = C_SET ^ A_TR;

    /* Copy directly into the frame from the buffer */
    memcpy(frame+4,buf, bufSize);
    
    /* Use strategy explained in sliders to ensure the message is correct later */
    char BCC2 = buf[0];
    for (int i = 1 ; i < bufSize ; i++) {
        BCC2 ^= buf[i];
    }

    /* First byte with payload */
    int size = 4;

    /* Handle FLAG and ESC in the middle of the payload */
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
    while (current <= nRetransmissions) { 
        alarmEnabled = TRUE;
        alarm(timeout);
        rejected = 0;
        accepted = 0;
        while (alarmEnabled == TRUE && !rejected && !accepted) {
            printf("Sending: ");
            for (int i = 0; i < size; i++) {
                printf("%d ", frame[i]);
            }
            int a = writeBytes(frame, size);
            printf("\nNrBytes: %d\n", a);
            char result = controlRead();
            if (!result) {
                continue;
            }
            else if (result == C_REJ0 || result == C_REJ1) {
                rejected = 1;
                printf("Rejected!\n");
            }
            else if (result == C_RR0 || result == C_RR1) {
                accepted = 1;
                printf("Accepted!\n");
                tramaTx = (tramaTx+1) % 2;
            }
            else {
                continue;
            }
        }
        if (accepted) {
            printf("Package Accepted!\n");
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
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(char *packet)
{
    char byte = 0;
    int readL = readByte(&byte);
    printf("%d", readL);
    char temp = 0;
    int index = 0;
    LinkStateMachine state = START;
    printf("Reading: ");
    while (state != SSTOP) {  
        readL = readByte(&byte);
        if (readL > 0) {
            printf("%d", byte);
            switch (state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_TR) {
                        state = A_RCV;
                    }
                    else if (byte != FLAG) {
                        state = START;
                    }
                    break;
                case A_RCV:
                    if (byte == C_I0 || byte == C_I1) {
                        state = C_RCV;
                        temp = byte; // We store this to check de BCC1 later!  
                    }
                    else if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    else if (byte == C_DISC) {
                        char frame[5] = {FLAG, A_RT, C_DISC, A_RT ^ C_DISC, FLAG};
                        writeBytes(frame, 5);  
                        return 0;
                    }
                    else {
                        state = START;
                    }
                    break;
                case C_RCV:
                    if (byte == (A_TR ^ temp)) {
                        state = READING;
                    }
                    else if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    else {
                        state = START;
                    }
                    break;
                case READING:
                    if (byte == ESC) {
                        state = SPECIAL_FOUND;
                    }
                    else if (byte == FLAG){
                        char BCC2 = packet[index-1];
                        index--;
                        packet[index] = '\0';
                        char BCC2_ACC = packet[0];
                        for (int j = 1; j < index; j++) {
                            BCC2_ACC ^= packet[j];
                        }
                        char RR;
                        char REJ;
                        switch (tramaRx) {
                                case 0: 
                                    RR = C_RR0;
                                    REJ = C_REJ0;
                                    break;
                                case 1:
                                    RR = C_RR1;
                                    REJ = C_REJ1;
                                default:
                                    return -1;
                            }
                        if (BCC2 == BCC2_ACC){
                            state = SSTOP;
                            char frame[5] = {FLAG, A_RT, RR, A_RT ^ RR, FLAG};
                            writeBytes(frame, 5);
                            tramaRx = (tramaRx + 1) % 2;
                            return index; 
                        }
                        else{
                            printf("Error!\n");
                            char frame[5] = {FLAG, A_RT, REJ, A_RT ^ REJ, FLAG};
                            writeBytes(frame, 5);
                            return -1;
                        };
                    }
                    else {
                        packet[index++] = byte;
                    }
                    break;
                case SPECIAL_FOUND:
                    state = READING;
                    if (byte == ESC || byte == FLAG) {
                        packet[index++] = byte;
                    }
                    else{
                        packet[index++] = ESC;
                        packet[index++] = byte;
                    }
                    break;
                default: 
                    break;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    LinkStateMachine state = START;
    char byte;
    (void) signal(SIGALRM, alarmHandler);
    
    while (nRetransmissions != 0 && state != SSTOP) {
        char frame[5] = {FLAG, A_TR, C_SET, A_TR ^ C_SET, FLAG};
        writeBytes(frame, 5);
        alarm(timeout);
        alarmEnabled = TRUE;
        while (alarmEnabled == TRUE && state != SSTOP) {
            if (readByte(&byte) > 0) {
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
                        if (byte == C_DISC) {
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
                        if (byte == (A_RT ^ C_DISC)) {
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

    if (state != SSTOP) {
        return -1;
    }
    char frame[5] = {FLAG, A_TR, C_SET, A_TR ^ C_SET, FLAG};
    writeBytes(frame, 5);
    int clstat = closeSerialPort();
    return clstat;
}

/* Auxiliary Function */
char controlRead(){
    char byte = 0;
    char temp = 0;
    LinkStateMachine state = START;
    while (state != SSTOP && alarmEnabled == TRUE) {  
        if (readByte(&byte) > 0) {
            printf("Byte: %d\n", byte);
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
                    if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == C_DISC) {
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