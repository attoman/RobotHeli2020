#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <stdint.h>
#include <netinet/in.h>
#include <string.h>

#define XAVIER2MISSION_PacketSize 9
#define MISSION2FCS_PacketSize 17


char packet_xavier2mission[XAVIER2MISSION_PacketSize];

int fd0;
int socket_Local_flag;
int port_xavier2mission = 44666;
struct sockaddr_in mission_socket;
struct sockaddr_in unity_socket;
int dgc_serial_setparams(int fd, int baudrate, char parity);
long int dgc_serial_bytes_available(int fd);
int dgc_serial_clear_input_buffer(int fd);
int dgc_serial_writen(int fd, unsigned char* buffer, int n, double timeout);
int dgc_serial_readn(int fd, unsigned char* buffer, int n, double timeout);
int baudcode(int baudrate);
void udp_init();
int udp_thread_gen();
void* udp_thread(void* thread_id);
void Serial_init(int fd);
int serial2fcs_thread_gen();
void* fcsprotocol_thread(void* thread_id);

int missionLen;
uint32_t HumanX;
uint32_t HumanY;
struct termios term0;


static struct Mission_Tracker_Output
{
    uint16_t Centerpoint_X;
    uint16_t Centerpoint_Y;
    uint16_t Box_width;
    uint16_t Box_height;
    uint8_t Class_number;

}Mission_Tracker_Output;

#pragma pack(push, 1)
unsigned char packet_mission2FCS[MISSION2FCS_PacketSize];
struct UARTXavierSendBuffer
{
    uint8_t header0;
    uint8_t header1;
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint16_t target_heading;
    uint8_t checksum;
}UARTXavierSendBuffer;
#pragma pack(pop)

using namespace std;

int main()
{
    missionLen = sizeof(mission_socket);
    socket_Local_flag = -1;
    int udp_flag = -1;
    int fcsprotocol_flag = -1;
    udp_init();
    Serial_init(fd0);

    udp_flag = udp_thread_gen();
    if(udp_flag == 0)
    {
        cout << endl << "<<<< Locol UDP Thread On !! >>>>" << endl;
    }
    else
    {
        cout << endl << "Locol UDP Thread Fault !!" << endl;
    }
    fcsprotocol_flag = serial2fcs_thread_gen();
    if (fcsprotocol_flag == 0)
    {
        cout << endl << "<<<< FCS Protocol Thread On !! >>>>" << endl;
    }
    else
    {
        cout << endl << "FCS Protocol Thread Fault !!" << endl;
    }
    cout << endl << "Hello World!!" << endl;
}
void udp_init()
{
    int rec;
    int clientAddr;
    socket_Local_flag = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_Local_flag == -1)
    {
        fprintf(stderr, "socket() failed");
        exit(1);
    }
    else
    {
        printf("Socket open!!!!");
    }
    memset(&mission_socket, 0, sizeof(mission_socket));
    mission_socket.sin_family = AF_INET;
    mission_socket.sin_addr.s_addr = INADDR_ANY;
    mission_socket.sin_port = htons(port_xavier2mission);

    if (bind(socket_Local_flag, (struct sockaddr*)&mission_socket, sizeof(mission_socket)) == -1)
    {
        printf("Bind error\n");
    }
    else
    {
        //Coding Rule
    }
}

void Serial_init(int fd)
{
    //struct termios term;
    struct serial_struct serial;
    const char* dev_name = "/dev/ttyUSB0";
    //initialize parameter
    HumanX = 0;
    HumanY = 0;
    /* open the serial port */
    fd = open(dev_name, O_RDWR | O_SYNC | O_NOCTTY, S_IRUSR | S_IWUSR);
    if (fd0 < 0)
    {
        fprintf(stderr, "Error: could not open port %s\n", dev_name);
        exit(1);
    }
    else
    {
        printf("Serial port is open\n");
    }
    /* configure it for raw read/write */
    if (tcgetattr(fd, &term0) < 0)
    {
        fprintf(stderr, "Error: could not get terminal attributes.\n");
        exit(1);
    }
    cfmakeraw(&term0);
    cfsetispeed(&term0, B115200);
    cfsetospeed(&term0, B115200);
    if (tcsetattr(fd, TCSAFLUSH, &term0) < 0)
    {
        fprintf(stderr, "Error: could not set terminal attributes.\n");
        exit(1);
    }
}


int udp_thread_gen()
{
    int co_thr;
    pthread_t M_P_1_thread;
    const char* message_udp = "UDP Thread";
    co_thr = pthread_create(&M_P_1_thread, NULL, udp_thread, (void*)message_udp);
    return co_thr;
}

int serial2fcs_thread_gen()
{
    int co_thr;
    pthread_t M_P_1_thread;
    const char* message_udp = "FCS Protocol Thread";
    co_thr = pthread_create(&M_P_1_thread, NULL, fcsprotocol_thread, (void*)message_udp);
    return co_thr;
}

void* fcsprotocol_thread(void* thread_id)
{
    cout << endl << " Mission2FCS Protocol thread Open" << endl;
    while(1)
    {
        dgc_serial_clear_input_buffer(fd0);
        dgc_serial_writen(fd0, packet_mission2FCS, MISSION2FCS_PacketSize, 0.1);
    }
}

void* udp_thread(void* thread_id)
{
    while (1)
    {
        missionLen = sizeof(mission_socket);
        if ((recvLen = recvfrom(socket_Local_flag, packet_xavier2mission, XAVIER2MISSION_PacketSize - 1, 0, (sockaddr*)&mission_socket, (socklen_t*)&missionLen)) == -1) {
            perror("recvfrom failed");
            exit(1);
        }
        else
        {
            printf("Receive Complete");
        }
        memcpy(&Mission_Tracker_Output, &packet_xavier2mission, sizeof(struct Mission_Tracker_Output));
    }
}




int baudcode(int baudrate) {

    switch (baudrate) {

        case 9600:
            return (B9600);
            break;

        case 57600:
            return (B57600);
            break;

        case 115200:
            return (B115200);
            break;

        case 230400:
            return (B230400);
            break;

        default:
            return (B115200);
            break;

    }
}

int dgc_serial_setparams(int fd, int baudrate, char parity) {
    struct termios term;
    struct serial_struct serial;

    if (tcgetattr(fd, &term) < 0) {
        fprintf(stderr, "Error: could not get terminal attributes.\n");
        return -1;
    }
    cfmakeraw(&term);

    if (0 && baudrate == 500000) {
        cfsetispeed(&term, baudcode(38400));
        cfsetospeed(&term, baudcode(38400));
        if (ioctl(fd, TIOCGSERIAL, &serial) < 0) {
            fprintf(stderr,
                    "Error: could not get high speed serial parameters.\n");
            return -1;
        }
        serial.flags |= ASYNC_SPD_CUST;
        serial.custom_divisor = 48; // for FTDI USB/serial converter
        // divisor is 240/5
        if (ioctl(fd, TIOCSSERIAL, &serial) < 0) {
            fprintf(stderr,
                    "Error: could not set high speed serial parameters.\n");
            return -1;
        }
    }
    else {
        cfsetispeed(&term, baudcode(baudrate));
        cfsetospeed(&term, baudcode(baudrate));
    }
    if (parity == 'E' || parity == 'e')
        term.c_cflag |= PARENB;
    else if (parity == 'O' || parity == 'o')
        term.c_cflag |= (PARENB | PARODD);
    if (tcsetattr(fd, TCSAFLUSH, &term) < 0) {
        fprintf(stderr, "Error: could not set terminal attributes.\n");
        return -1;
    }
    return 0;
}

long int dgc_serial_bytes_available(int fd) {
    long available = 0;

    if (ioctl(fd, FIONREAD, &available) == 0)
        return available;
    else
        return -1;
}

int dgc_serial_clear_input_buffer(int fd) {
    int dummy;
    long int val;
    char* buffer;

    val = dgc_serial_bytes_available(fd);
    if (val > 0) {
        buffer = (char*)malloc(val);
        if (buffer == NULL) {
            fprintf(stderr, "Error: could not allocate temporary buffer.\n");
            return -1;
        }
        dummy = read(fd, buffer, val);
        free(buffer);
    }
    return 0;
}

int dgc_serial_writen(int fd, unsigned char* buffer, int n, double timeout) {
    struct timeval t;
    fd_set set;
    int err, start_n, bytes_written;

    start_n = n;
    do {
        if (timeout != -1) {
            t.tv_sec = (int)floor(timeout);;
            t.tv_usec = (timeout - t.tv_sec) * 1e6;
        }
        FD_ZERO(&set);
        FD_SET(fd, &set);
        if (timeout == -1)
            err = select(fd + 1, NULL, &set, NULL, NULL);
        else
            err = select(fd + 1, NULL, &set, NULL, &t);
        if (err == 0)
            return start_n - n;
        bytes_written = write(fd, buffer, n);
        if (bytes_written < 0 || (bytes_written == 0 && n == start_n))
            return -1;
        else {
            buffer += bytes_written;
            n -= bytes_written;
        }
    } while (n > 0);
    return start_n;
}

int dgc_serial_readn(int fd, unsigned char* buffer, int n, double timeout) {
    struct timeval t;
    fd_set set;
    int err, start_n, bytes_read;

    start_n = n;
    while (n > 0) {
        if (timeout != -1) {
            t.tv_sec = (int)floor(timeout);
            t.tv_usec = (timeout - t.tv_sec) * 1e6;
        }
        FD_ZERO(&set);
        FD_SET(fd, &set);
        if (timeout == -1)
            err = select(fd + 1, &set, NULL, NULL, NULL);
        else
            err = select(fd + 1, &set, NULL, NULL, &t);
        if (err == 0)
            return start_n - n;
        else if (err < 0)
            return -1;
        bytes_read = read(fd, buffer, n);
        if (bytes_read < 0 || (bytes_read == 0 && n == start_n))
            return -1;
        else {
            buffer += bytes_read;
            n -= bytes_read;
        }
    }
    return start_n;
}

void dgc_serial_close(int fd) {
    close(fd);
}