/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "deepstream_app.h"
#include "deepstream_config_file_parser.h"
#include "nvds_version.h"
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <sys/socket.h>


 //////////////////////////////////////////////////////////
 //200624_Jinhyun
 //#include "nvdstracker.h"
 //UDP
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>

//200714_Jinhyun
//UART
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#define MAX_INSTANCES 128
#define APP_TITLE "DeepStream"
#define DEFAULT_X_WINDOW_WIDTH 1920
#define DEFAULT_X_WINDOW_HEIGHT 1080

////////////////////////////////////////////////////////
//200722
//Keyboard Event
#include "linux_kbhit.h"
#define UARTSendBufferSize 17
#define PI  3.141592
////////////////////////////////////////////////////////
//  200723_SeJun
#define UDPSendBufferSize 16
#define UDPReceiveBufferSize 40
#define XAVIER2UNITY_PORT	55777
#define UNITY2XAVIER_PORT 55888
//  Debugging Parameter
#define Debug 1
////////////////////////////////////////////////////////
//  200723_SeJun
void* acsl_UDP_event_thread(void* threadid);

int slen = 0;
int recvlen = 0;
int udp_initialize = 0;
int hUnitySock = 0;
char* UnityIP = "165.246.75.189";
char UDP_Xavier_send_buf[UDPSendBufferSize];
char UDP_Xavier_receive_buf[UDPReceiveBufferSize];
struct sockaddr_in Xavieraddr;	/* our address */
struct sockaddr_in Unityaddr;	/* remote address */
int protocol = 0;

struct UDPXavierSendBuffer
{
    float panCMD;
    float tiltCMD;
    float FOV;
    float zoomspeed;
}UDPXavierSendBuffer;

struct UDPXavierReceiveBuffer
{
    float Gimbaltilt;
    float Gimbalpan;
    float Gimbalroll;
    float FOV;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
}UDPXavierReceiveBuffer;

//GLib Thread
static GThread* protocol_event_thread = NULL;
static GMutex protocol_lock;




////////////////////////////////////////////////////////
//  Deepstream_app Basic Code

AppCtx* appCtx[MAX_INSTANCES];
static guint cintr = FALSE;
static GMainLoop* main_loop = NULL;
static gchar** cfg_files = NULL;
static gchar** input_files = NULL;
static gboolean print_version = FALSE;
static gboolean show_bbox_text = FALSE;
static gboolean print_dependencies_version = FALSE;
static gboolean quit = FALSE;
static gint return_value = 0;
static guint num_instances;
static guint num_input_files;
static GMutex fps_lock;
static gdouble fps[MAX_SOURCE_BINS];
static gdouble fps_avg[MAX_SOURCE_BINS];
static guint num_fps_inst = 0;
static Display* display = NULL;
static Window windows[MAX_INSTANCES] = { 0 };
static gint source_ids[MAX_INSTANCES];
static GThread* x_event_thread = NULL;
static GMutex disp_lock;

////////////////////////////////////////////////////////////
//  200714_Jinhyun
int com_period = 10;       // ms
guint32 HumanX;
guint32 HumanY;

//UART global variable
gint fd;
struct termios term0;

#pragma pack(push, 1)
unsigned char UART_Xavier_send[UARTSendBufferSize];

struct UARTXavierSendBuffer
{
    guint8 header0;
    guint8 header1;
    guint32 x;
    guint32 y;
    guint32 z;
    guint16 target_heading;
    guint8 checksum;
}UARTXavierSendBuffer;
#pragma pack(pop)

/////////////////////////////////////////////////////////////



GST_DEBUG_CATEGORY(NVDS_APP);

GOptionEntry entries[] = {
  {"version", 'v', 0, G_OPTION_ARG_NONE, &print_version,
      "Print DeepStreamSDK version", NULL}
  ,
  {"tiledtext", 't', 0, G_OPTION_ARG_NONE, &show_bbox_text,
      "Display Bounding box labels in tiled mode", NULL}
  ,
  {"version-all", 0, 0, G_OPTION_ARG_NONE, &print_dependencies_version,
      "Print DeepStreamSDK and dependencies version", NULL}
  ,
  {"cfg-file", 'c', 0, G_OPTION_ARG_FILENAME_ARRAY, &cfg_files,
      "Set the config file", NULL}
  ,
  {"input-file", 'i', 0, G_OPTION_ARG_FILENAME_ARRAY, &input_files,
      "Set the input file", NULL}
  ,
  {NULL}
  ,
};

/**
 * Callback function to be called once all inferences (Primary + Secondary)
 * are done. This is opportunity to modify content of the metadata.
 * e.g. Here Person is being replaced with Man/Woman and corresponding counts
 * are being maintained. It should be modified according to network classes
 * or can be removed altogether if not required.
 */
static void
all_bbox_generated(AppCtx* appCtx, GstBuffer* buf,
    NvDsBatchMeta* batch_meta, guint index)
{
    guint num_male = 0;
    guint num_female = 0;
    guint num_objects[128];

    memset(num_objects, 0, sizeof(num_objects));

    for (NvDsMetaList* l_frame = batch_meta->frame_meta_list; l_frame != NULL;
        l_frame = l_frame->next) {
        NvDsFrameMeta* frame_meta = l_frame->data;
        for (NvDsMetaList* l_obj = frame_meta->obj_meta_list; l_obj != NULL;
            l_obj = l_obj->next) {
            NvDsObjectMeta* obj = (NvDsObjectMeta*)l_obj->data;
            if (obj->unique_component_id ==
                (gint)appCtx->config.primary_gie_config.unique_id) {
                if (obj->class_id >= 0 && obj->class_id < 128) {
                    num_objects[obj->class_id]++;
                }
                if (appCtx->person_class_id > -1
                    && obj->class_id == appCtx->person_class_id) {
                    if (strstr(obj->text_params.display_text, "Man")) {
                        str_replace(obj->text_params.display_text, "Man", "");
                        str_replace(obj->text_params.display_text, "Person", "Man");
                        num_male++;
                    }
                    else if (strstr(obj->text_params.display_text, "Woman")) {
                        str_replace(obj->text_params.display_text, "Woman", "");
                        str_replace(obj->text_params.display_text, "Person", "Woman");
                        num_female++;
                    }
                }
            }
        }
    }
}

/**
 * Function to handle program interrupt signal.
 * It installs default handler after handling the interrupt.
 */
static void
_intr_handler(int signum)
{
    struct sigaction action;
    NVGSTDS_ERR_MSG_V("User Interrupted.. \n");
    memset(&action, 0, sizeof(action));
    action.sa_handler = SIG_DFL;
    sigaction(SIGINT, &action, NULL);
    cintr = TRUE;
}

/**
 * callback function to print the performance numbers of each stream.
 */
static void
perf_cb(gpointer context, NvDsAppPerfStruct* str)
{
    static guint header_print_cnt = 0;
    guint i;
    AppCtx* appCtx = (AppCtx*)context;
    guint numf = (num_instances == 1) ? str->num_instances : num_instances;
    g_mutex_lock(&fps_lock);
    if (num_instances > 1) {
        fps[appCtx->index] = str->fps[0];
        fps_avg[appCtx->index] = str->fps_avg[0];
    }
    else {
        for (i = 0; i < numf; i++) {
            fps[i] = str->fps[i];
            fps_avg[i] = str->fps_avg[i];
        }
    }
    num_fps_inst++;
    if (num_fps_inst < num_instances) {
        g_mutex_unlock(&fps_lock);
        return;
    }
    num_fps_inst = 0;
    if (header_print_cnt % 20 == 0) {
        g_print("\n**PERF: ");
        for (i = 0; i < numf; i++) {
            g_print("FPS %d (Avg)\t", i);
        }
        g_print("\n");
        header_print_cnt = 0;
    }
    header_print_cnt++;
    g_print("**PERF: ");
    for (i = 0; i < numf; i++) {
        g_print("%.2f (%.2f)\t", fps[i], fps_avg[i]);
    }
    g_print("\n");
    g_mutex_unlock(&fps_lock);
}

/**
 * Loop function to check the status of interrupts.
 * It comes out of loop if application got interrupted.
 */
static gboolean
check_for_interrupt(gpointer data)
{
    if (quit) {
        return FALSE;
    }
    if (cintr) {
        cintr = FALSE;
        quit = TRUE;
        g_main_loop_quit(main_loop);
        return FALSE;
    }
    return TRUE;
}

/*
 * Function to install custom handler for program interrupt signal.
 */
static void
_intr_setup(void)
{
    struct sigaction action;

    memset(&action, 0, sizeof(action));
    action.sa_handler = _intr_handler;

    sigaction(SIGINT, &action, NULL);
}

static gboolean
kbhit(void)
{
    struct timeval tv;
    fd_set rdfs;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET(STDIN_FILENO, &rdfs);

    select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);
}

/*
 * Function to enable / disable the canonical mode of terminal.
 * In non canonical mode input is available immediately (without the user
 * having to type a line-delimiter character).
 */
static void
changemode(int dir)
{
    static struct termios oldt, newt;

    if (dir == 1) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    }
    else
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

static void
print_runtime_commands(void)
{
    g_print("\nRuntime commands:\n"
        "\th: Print this help\n"
        "\tq: Quit\n\n" "\tp: Pause\n" "\tr: Resume\n\n");

    if (appCtx[0]->config.tiled_display_config.enable) {
        g_print
        ("NOTE: To expand a source in the 2D tiled display and view object details,"
            " left-click on the source.\n"
            "      To go back to the tiled display, right-click anywhere on the window.\n\n");
    }
}
static guint rrow, rcol;
static gboolean rrowsel = FALSE, selecting = FALSE;

///////////////////////////////////////////////////////
//  200723_SeJun
int UDP_init()
{
    hUnitySock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (hUnitySock == -1)
    {
        fprintf(stderr, "Socket create failed");
        exit(1);
    }

    memset((char*)&Xavieraddr, 0, sizeof(Xavieraddr));
    Xavieraddr.sin_family = AF_INET;
    Xavieraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    Xavieraddr.sin_port = htons(UNITY2XAVIER_PORT);

    if (bind(hUnitySock, (struct sockaddr*)&Xavieraddr, sizeof(Xavieraddr)) < 0) {
        perror("bind failed");
        return 0;
    }
    else {
        if (Debug)
        {
            printf("-----------------<< UDP socket opened >>------------------------\n");
        }
    }

    memset((char*)&Unityaddr, 0, sizeof(Unityaddr));
    Unityaddr.sin_family = AF_INET;
    Unityaddr.sin_port = htons(XAVIER2UNITY_PORT);
    if (inet_aton(UnityIP, &Unityaddr.sin_addr) == 0) {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
    else {
        //Coding Rule
    }
    slen = sizeof(Unityaddr);

    protocol = 1;
    return 1;
}

/////////////tttttttttttttt
void dskfjdsk()
{
    if (Debug)
    {
        printf("UDP Thread On!!!\n");
    }
    else {
        //Coding Rule
    }
    while (protocol)
    {
        ////////////////////////////////////////////////////////////////////////////////////


        memset(&UDP_Xavier_send_buf, 0, sizeof(UDP_Xavier_send_buf));

        if (Debug)
        {
            printf("Send packet buffer clear!!!\n");
        }
        else {
            //Coding Rule
        }
        memcpy(&UDP_Xavier_send_buf, &UDPXavierSendBuffer, sizeof(struct UDPXavierSendBuffer));
        if (Debug)
        {
            printf("Send packet buffer parsing complete!!!\n");
        }
        else {
            //Coding Rule
        }
        if (Debug)
        {
            printf("Ready to Send packet to %s port %d\n", UnityIP, XAVIER2UNITY_PORT);
        }
        else {
            //Coding Rule
        }
        if (sendto(hUnitySock, UDP_Xavier_send_buf, strlen(UDP_Xavier_send_buf), 0, (struct sockaddr*)&Unityaddr, slen) == -1) {
            perror("sendto");
            exit(1);
        }
        else {
            //Coding Rule
        }
        recvlen = recvfrom(hUnitySock, UDP_Xavier_receive_buf, UDPReceiveBufferSize, 0, (struct sockaddr*)&Unityaddr, &slen);
        if (recvlen == -1) {
            UDP_Xavier_receive_buf[UDPReceiveBufferSize] = 0;
            printf("Receive failed\n");
            exit(1);
        }
        else if (recvlen > 0)
        {
            printf("Receive Success\n");
        }
        else {
            //Coding Rule
        }

    }
}

static gboolean protocl_thread_func(gpointer arg)
{
    guint i;
    gboolean ret = TRUE;

    // Check if all instances have quit
    for (i = 0; i < num_instances; i++) {
        if (!appCtx[i]->quit)
            break;
    }

    if (i == num_instances)
    {
        quit = TRUE;
        g_main_loop_quit(main_loop);
        return FALSE;
    }
}


///////////////////////////////////////////////////////


long int dgc_serial_bytes_available(int fd)
{
    long available = 0;

    if (ioctl(fd, FIONREAD, &available) == 0)
        return available;
    else
        return -1;
}

gint dgc_serial_clear_input_buffer(int fd)
{
    int dummy;
    long int val;
    char* buffer;

    val = dgc_serial_bytes_available(fd);
    if (val > 0)
    {
        buffer = (char*)malloc(val);
        if (buffer == NULL)
        {
            fprintf(stderr, "Error: could not allocate temporary buffer.\n");
            return -1;
        }
        dummy = read(fd, buffer, val);
        free(buffer);
    }
    return 0;
}

gint dgc_serial_writen(int fd, unsigned char* buffer, int n, double timeout)
{
    struct timeval t;
    fd_set set;
    int err, start_n, bytes_written;
    float PastHumanX, PastHumanY, PresentHumanX, PresentHumanY, HumanHeading, HeliX, HeliY;
    
    start_n = n;
    do
    {
        if (timeout != -1)
        {
            t.tv_sec = floor(timeout);;
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

        memset(&buffer, 0, sizeof(buffer));
        ////////////////////////////////////////////////////////////////////////////////////
        // 200722_Jinhyun
        // Human X, Y, Z, Heading estimate
        PastHumanX = (float) HumanX / 1e7;
        PastHumanY = (float) HumanY / 1e7;
        HeliX = (float) UDPXavierReceiveBuffer.x / 1e7;
        HeliY = (float) UDPXavierReceiveBuffer.y / 1e7;

        PresentHumanX = HeliX + sin(UDPXavierReceiveBuffer.Gimbalpan) * (UDPXavierReceiveBuffer.z) / tan(UDPXavierReceiveBuffer.Gimbaltilt);
        PresentHumanY = HeliY + cos(UDPXavierReceiveBuffer.Gimbalpan) * (UDPXavierReceiveBuffer.z) / tan(UDPXavierReceiveBuffer.Gimbaltilt);
        
        HumanHeading = atan2((PresentHumanX - PastHumanX), (PresentHumanY - PastHumanY)) * 1e2;
        if(HumanHeading < 0)
        {
            HumanHeading = 360 + HumanHeading;
        }
        else
        {
            //Coding Rule
        }

        //Data type 변환
        HumanHeading = HumanHeading * 1e2;
        HumanX = (guint32) PresentHumanX * 1e7;
        HumanY = (guint32) PresentHumanY * 1e7;
        ////////////////////////////////////////////////////////////////////////////////////

        UARTXavierSendBuffer.header0 = 'X';
        UARTXavierSendBuffer.header1 = 'F';
        UARTXavierSendBuffer.x = HumanX;
        UARTXavierSendBuffer.y = HumanY;
        UARTXavierSendBuffer.z = (guint32) 0;
        UARTXavierSendBuffer.target_heading = (guint16) HumanHeading;

        memcpy(&buffer, &UARTXavierSendBuffer, UARTSendBufferSize);
        for (int i = 2; i < UARTSendBufferSize - 1; i++)
        {
            UARTXavierSendBuffer.checksum ^= buffer[i];
        }
        memcpy(&buffer, &UARTXavierSendBuffer, UARTSendBufferSize);

        bytes_written = write(fd, buffer, n);

        if (bytes_written < 0 || (bytes_written == 0 && n == start_n))
            return -1;
        else
        {
            buffer += bytes_written;
            n -= bytes_written;
        }
    } while (n > 0);
    return start_n;
}

gint uart_send(gpointer data)
{
    dgc_serial_writen(fd, UART_Xavier_send, UARTSendBufferSize, 0.1);
}
////////////////////////////////////////////////////////////////////////

/**
 * Loop function to check keyboard inputs and status of each pipeline.
 */
static gboolean event_thread_func(gpointer arg)
{
    guint i;
    gboolean ret = TRUE;

    // Check if all instances have quit
    for (i = 0; i < num_instances; i++) {
        if (!appCtx[i]->quit)
            break;
    }

    if (i == num_instances)
    {
        quit = TRUE;
        g_main_loop_quit(main_loop);
        return FALSE;
    }
    // Check for keyboard input
    if (!kbhit())
    {
        //continue;
        return TRUE;
    }
    int c = fgetc(stdin);
    g_print("\n");

    gint source_id;
    GstElement* tiler = appCtx[0]->pipeline.tiled_display_bin.tiler;
    g_object_get(G_OBJECT(tiler), "show-source", &source_id, NULL);

    if (selecting)
    {
        if (rrowsel == FALSE)
        {
            if (c >= '0' && c <= '9')
            {
                rrow = c - '0';
                if (rrow < appCtx[0]->config.tiled_display_config.rows)
                {
                    g_print("--selecting source  row %d--\n", rrow);
                    rrowsel = TRUE;
                }
                else
                {
                    g_print("--selected source  row %d out of bound, reenter\n", rrow);
                }
            }
        }
        else
        {
            if (c >= '0' && c <= '9')
            {
                unsigned int tile_num_columns = appCtx[0]->config.tiled_display_config.columns;
                rcol = c - '0';
                if (rcol < tile_num_columns)
                {
                    selecting = FALSE;
                    rrowsel = FALSE;
                    source_id = tile_num_columns * rrow + rcol;
                    g_print("--selecting source  col %d sou=%d--\n", rcol, source_id);
                    if (source_id >= (gint)appCtx[0]->config.num_source_sub_bins)
                    {
                        source_id = -1;
                    }
                    else
                    {
                        source_ids[0] = source_id;
                        appCtx[0]->show_bbox_text = TRUE;
                        g_object_set(G_OBJECT(tiler), "show-source", source_id, NULL);
                    }
                }
                else
                {
                    g_print("--selected source  col %d out of bound, reenter\n", rcol);
                }
            }
        }
    }
    switch (c)
    {
    case 'h':
        print_runtime_commands();
        break;
    case 'p':
        for (i = 0; i < num_instances; i++)
            pause_pipeline(appCtx[i]);
        break;
    case 'r':
        for (i = 0; i < num_instances; i++)
            resume_pipeline(appCtx[i]);
        break;
    case 'q':
        quit = TRUE;
        g_main_loop_quit(main_loop);
        ret = FALSE;
        break;
    case 'z':
        if (source_id == -1 && selecting == FALSE)
        {
            g_print("--selecting source --\n");
            selecting = TRUE;
        }
        else
        {
            if (!show_bbox_text)
                appCtx[0]->show_bbox_text = FALSE;
            g_object_set(G_OBJECT(tiler), "show-source", -1, NULL);
            source_ids[0] = -1;
            selecting = FALSE;
            g_print("--tiled mode --\n");
        }
        break;
    case 'w':
        UDPXavierSendBuffer.tiltCMD = (float)0.1;
        break;
    case 'a':
        UDPXavierSendBuffer.panCMD = (float)-0.1;
        break;
    case 's':
        UDPXavierSendBuffer.tiltCMD = (float)-0.1;
        break;
    case 'd':
        UDPXavierSendBuffer.panCMD = (float)0.1;
        break;
    case 'f':
        UDPXavierSendBuffer.panCMD = (float)0;
        UDPXavierSendBuffer.tiltCMD = (float)0;
        break;
    default:
        break;
    }
    return ret;
}

static int
get_source_id_from_coordinates(float x_rel, float y_rel)
{
    int tile_num_rows = appCtx[0]->config.tiled_display_config.rows;
    int tile_num_columns = appCtx[0]->config.tiled_display_config.columns;

    int source_id = (int)(x_rel * tile_num_columns);
    source_id += ((int)(y_rel * tile_num_rows)) * tile_num_columns;

    /* Don't allow clicks on empty tiles. */
    if (source_id >= (gint)appCtx[0]->config.num_source_sub_bins)
        source_id = -1;

    return source_id;
}

/**
 * Thread to monitor X window events.
 */
static gpointer nvds_x_event_thread(gpointer data)
{
    g_mutex_lock(&disp_lock);
    while (display)
    {
        XEvent e;
        guint index;
        while (XPending(display))
        {
            XNextEvent(display, &e);
            switch (e.type)
            {
            case ButtonPress:
            {
                XWindowAttributes win_attr;
                XButtonEvent ev = e.xbutton;
                gint source_id;
                GstElement* tiler;

                XGetWindowAttributes(display, ev.window, &win_attr);

                for (index = 0; index < MAX_INSTANCES; index++)
                    if (ev.window == windows[index])
                        break;

                tiler = appCtx[index]->pipeline.tiled_display_bin.tiler;
                g_object_get(G_OBJECT(tiler), "show-source", &source_id, NULL);

                if (ev.button == Button1 && source_id == -1)
                {
                    source_id = get_source_id_from_coordinates(ev.x * 1.0 / win_attr.width, ev.y * 1.0 / win_attr.height);
                    if (source_id > -1)
                    {
                        g_object_set(G_OBJECT(tiler), "show-source", source_id, NULL);
                        source_ids[index] = source_id;
                        appCtx[index]->show_bbox_text = TRUE;
                    }
                }
                else if (ev.button == Button3)
                {
                    g_object_set(G_OBJECT(tiler), "show-source", -1, NULL);
                    source_ids[index] = -1;
                    if (!show_bbox_text)
                        appCtx[index]->show_bbox_text = FALSE;
                }
            }
            break;
            case KeyRelease:
            case KeyPress:
            {
                KeySym p, r, q, w, a, s, d, f;
                guint i;
                p = XKeysymToKeycode(display, XK_P);
                r = XKeysymToKeycode(display, XK_R);
                q = XKeysymToKeycode(display, XK_Q);
                w = XKeysymToKeycode(display, XK_W);
                a = XKeysymToKeycode(display, XK_A);
                s = XKeysymToKeycode(display, XK_S);
                d = XKeysymToKeycode(display, XK_D);
                f = XKeysymToKeycode(display, XK_F);
                if (e.xkey.keycode == p)
                {
                    for (i = 0; i < num_instances; i++)
                        pause_pipeline(appCtx[i]);
                    break;
                }
                if (e.xkey.keycode == r)
                {
                    for (i = 0; i < num_instances; i++)
                        resume_pipeline(appCtx[i]);
                    break;
                }
                if (e.xkey.keycode == q)
                {
                    quit = TRUE;
                    g_main_loop_quit(main_loop);
                }
                if (e.xkey.keycode == w)
                {
                    UDPXavierSendBuffer.tiltCMD = (float)0.1;
                }
                if (e.xkey.keycode == a)
                {
                    UDPXavierSendBuffer.panCMD = (float)-0.1;
                }
                if (e.xkey.keycode == s)
                {
                    UDPXavierSendBuffer.tiltCMD = (float)-0.1;
                }
                if (e.xkey.keycode == d)
                {
                    UDPXavierSendBuffer.panCMD = (float)0.1;
                }
                if (e.xkey.keycode == f)
                {
                    UDPXavierSendBuffer.panCMD = (float)0;
                    UDPXavierSendBuffer.tiltCMD = (float)0;
                }
            }
            break;
            case ClientMessage:
            {
                Atom wm_delete;
                for (index = 0; index < MAX_INSTANCES; index++)
                    if (e.xclient.window == windows[index])
                        break;
                wm_delete = XInternAtom(display, "WM_DELETE_WINDOW", 1);
                if (wm_delete != None && wm_delete == (Atom)e.xclient.data.l[0])
                {
                    quit = TRUE;
                    g_main_loop_quit(main_loop);
                }
            }
            break;
            }
        }
        g_mutex_unlock(&disp_lock);
        g_usleep(G_USEC_PER_SEC / 20);
        g_mutex_lock(&disp_lock);
    }
    g_mutex_unlock(&disp_lock);
    return NULL;
}

/**
 * callback function to add application specific metadata.
 * Here it demonstrates how to display the URI of source in addition to
 * the text generated after inference.
 */
static gboolean overlay_graphics(AppCtx* appCtx, GstBuffer* buf, NvDsBatchMeta* batch_meta, guint index)
{
    if (source_ids[index] == -1)
        return TRUE;

    NvDsFrameLatencyInfo* latency_info = NULL;
    NvDsDisplayMeta* display_meta = nvds_acquire_display_meta_from_pool(batch_meta);

    display_meta->num_labels = 1;
    display_meta->text_params[0].display_text = g_strdup_printf("Source: %s", appCtx->config.multi_source_config[source_ids[index]].uri);

    display_meta->text_params[0].y_offset = 20;
    display_meta->text_params[0].x_offset = 20;
    display_meta->text_params[0].font_params.font_color = (NvOSD_ColorParams){ 0, 1, 0, 1 };
    display_meta->text_params[0].font_params.font_size = appCtx->config.osd_config.text_size * 1.5;
    display_meta->text_params[0].font_params.font_name = "Serif";
    display_meta->text_params[0].set_bg_clr = 1;
    display_meta->text_params[0].text_bg_clr = (NvOSD_ColorParams){ 0, 0, 0, 1.0 };


    if (nvds_enable_latency_measurement)
    {
        g_mutex_lock(&appCtx->latency_lock);
        latency_info = &appCtx->latency_info[index];
        display_meta->num_labels++;
        display_meta->text_params[1].display_text = g_strdup_printf("Latency: %lf", latency_info->latency);
        g_mutex_unlock(&appCtx->latency_lock);

        display_meta->text_params[1].y_offset = (display_meta->text_params[0].y_offset * 2) + display_meta->text_params[0].font_params.font_size;
        display_meta->text_params[1].x_offset = 20;
        display_meta->text_params[1].font_params.font_color = (NvOSD_ColorParams){ 0, 1, 0, 1 };
        display_meta->text_params[1].font_params.font_size = appCtx->config.osd_config.text_size * 1.5;
        display_meta->text_params[1].font_params.font_name = "Arial";
        display_meta->text_params[1].set_bg_clr = 1;
        display_meta->text_params[1].text_bg_clr = (NvOSD_ColorParams){ 0, 0, 0, 1.0 };
    }

    nvds_add_display_meta_to_frame(nvds_get_nth_frame_meta(batch_meta->frame_meta_list, 0), display_meta);
    return TRUE;
}





int main(int argc, char* argv[])
{
    udp_initialize = UDP_init();
    if (Debug == 1 && udp_initialize == 0)
    {
        printf("---------------------- UDP Link Fault --------------------\n");
    }
    else
    {
        //coding rule
    }


    GOptionContext* ctx = NULL;
    GOptionGroup* group = NULL;
    GError* error = NULL;
    guint i;

    ctx = g_option_context_new("Nvidia DeepStream Demo");
    group = g_option_group_new("abc", NULL, NULL, NULL, NULL);
    g_option_group_add_entries(group, entries);

    g_option_context_set_main_group(ctx, group);
    g_option_context_add_group(ctx, gst_init_get_option_group());

    GST_DEBUG_CATEGORY_INIT(NVDS_APP, "NVDS_APP", 0, NULL);

    if (!g_option_context_parse(ctx, &argc, &argv, &error))
    {
        NVGSTDS_ERR_MSG_V("%s", error->message);
        return -1;
    }

    if (print_version)
    {
        g_print("deepstream-app version %d.%d.%d\n", NVDS_APP_VERSION_MAJOR, NVDS_APP_VERSION_MINOR, NVDS_APP_VERSION_MICRO);
        nvds_version_print();
        return 0;
    }

    if (print_dependencies_version)
    {
        g_print("deepstream-app version %d.%d.%d\n", NVDS_APP_VERSION_MAJOR, NVDS_APP_VERSION_MINOR, NVDS_APP_VERSION_MICRO);
        nvds_version_print();
        nvds_dependencies_version_print();
        return 0;
    }

    if (cfg_files)
    {
        num_instances = g_strv_length(cfg_files);
    }
    if (input_files)
    {
        num_input_files = g_strv_length(input_files);
    }

    memset(source_ids, -1, sizeof(source_ids));

    if (!cfg_files || num_instances == 0)
    {
        NVGSTDS_ERR_MSG_V("Specify config file with -c option");
        return_value = -1;
        goto done;
    }

    for (i = 0; i < num_instances; i++)
    {
        appCtx[i] = g_malloc0(sizeof(AppCtx));
        appCtx[i]->person_class_id = -1;
        appCtx[i]->car_class_id = -1;
        appCtx[i]->index = i;
        if (show_bbox_text)
        {
            appCtx[i]->show_bbox_text = TRUE;
        }

        if (input_files && input_files[i])
        {
            appCtx[i]->config.multi_source_config[0].uri = g_strdup_printf("file://%s", input_files[i]);
            g_free(input_files[i]);
        }

        if (!parse_config_file(&appCtx[i]->config, cfg_files[i]))
        {
            NVGSTDS_ERR_MSG_V("Failed to parse config file '%s'", cfg_files[i]);
            appCtx[i]->return_value = -1;
            goto done;
        }
    }

    for (i = 0; i < num_instances; i++)
    {
        if (!create_pipeline(appCtx[i], NULL, all_bbox_generated, perf_cb, overlay_graphics))
        {
            NVGSTDS_ERR_MSG_V("Failed to create pipeline");
            return_value = -1;
            goto done;
        }
    }

    main_loop = g_main_loop_new(NULL, FALSE);

    _intr_setup();

    g_timeout_add(400, check_for_interrupt, NULL);

    g_mutex_init(&disp_lock);
    display = XOpenDisplay(NULL);
    for (i = 0; i < num_instances; i++)
    {
        guint j;

        if (gst_element_set_state(appCtx[i]->pipeline.pipeline, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE)
        {
            NVGSTDS_ERR_MSG_V("Failed to set pipeline to PAUSED");
            return_value = -1;
            goto done;
        }

        if (!appCtx[i]->config.tiled_display_config.enable)
            continue;

        for (j = 0; j < appCtx[i]->config.num_sink_sub_bins; j++)
        {
            XTextProperty xproperty;
            gchar* title;
            guint width, height;

            if (!GST_IS_VIDEO_OVERLAY(appCtx[i]->pipeline.instance_bins[0].sink_bin.sub_bins[j].sink))
            {
                continue;
            }

            if (!display)
            {
                NVGSTDS_ERR_MSG_V("Could not open X Display");
                return_value = -1;
                goto done;
            }
            if (!protocol)
            {
                printf("Could not open Protocol");
                    return_value = -1;
            }

            if (appCtx[i]->config.sink_bin_sub_bin_config[j].render_config.width)
                width = appCtx[i]->config.sink_bin_sub_bin_config[j].render_config.width;
            else
                width = appCtx[i]->config.tiled_display_config.width;
            if (appCtx[i]->config.sink_bin_sub_bin_config[j].render_config.height)
                height = appCtx[i]->config.sink_bin_sub_bin_config[j].render_config.height;
            else
                height = appCtx[i]->config.tiled_display_config.height;

            width = (width) ? width : DEFAULT_X_WINDOW_WIDTH;
            height = (height) ? height : DEFAULT_X_WINDOW_HEIGHT;

            windows[i] = XCreateSimpleWindow(display, RootWindow(display, DefaultScreen(display)), 0, 0, width, height, 2, 0x00000000, 0x00000000);

            if (num_instances > 1)
                title = g_strdup_printf(APP_TITLE "-%d", i);
            else
                title = g_strdup(APP_TITLE);
            if (XStringListToTextProperty((char**)&title, 1, &xproperty) != 0)
            {
                XSetWMName(display, windows[i], &xproperty);
                XFree(xproperty.value);
            }

            XSetWindowAttributes attr = { 0 };
            if ((appCtx[i]->config.tiled_display_config.enable &&
                appCtx[i]->config.tiled_display_config.rows *
                appCtx[i]->config.tiled_display_config.columns == 1) ||
                (appCtx[i]->config.tiled_display_config.enable == 0 &&
                    appCtx[i]->config.num_source_sub_bins == 1))
            {
                attr.event_mask = KeyPress;
            }
            else
            {
                attr.event_mask = ButtonPress | KeyRelease;
            }
            XChangeWindowAttributes(display, windows[i], CWEventMask, &attr);

            Atom wmDeleteMessage = XInternAtom(display, "WM_DELETE_WINDOW", False);
            if (wmDeleteMessage != None)
            {
                XSetWMProtocols(display, windows[i], &wmDeleteMessage, 1);
            }
            XMapRaised(display, windows[i]);
            XSync(display, 1);       //discard the events for now
            gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(appCtx[i]->pipeline.instance_bins[0].sink_bin.sub_bins[j].sink), (gulong)windows[i]);
            gst_video_overlay_expose(GST_VIDEO_OVERLAY(appCtx[i]->pipeline.instance_bins[0].sink_bin.sub_bins[j].sink));
            if (!x_event_thread)
                x_event_thread = g_thread_new("nvds-window-event-thread", nvds_x_event_thread, NULL);
        }
    }

    /* Dont try to set playing state if error is observed */
    if (return_value != -1)
    {
        for (i = 0; i < num_instances; i++)
        {
            if (gst_element_set_state(appCtx[i]->pipeline.pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
            {
                g_print("\ncan't set pipeline to playing state.\n");
                return_value = -1;
                goto done;
            }
        }
    }

    print_runtime_commands();

    changemode(1);
    g_timeout_add(40, event_thread_func, NULL);

 
    g_main_loop_run (main_loop);

    changemode(0);

done:

    g_print("Quitting\n");
    for (i = 0; i < num_instances; i++)
    {
        if (appCtx[i]->return_value == -1)
            return_value = -1;
        destroy_pipeline(appCtx[i]);
        g_mutex_lock(&disp_lock);
        g_mutex_lock(&disp_lock);
        if (windows[i])
            XDestroyWindow(display, windows[i]);
        windows[i] = 0;
        g_mutex_unlock(&disp_lock);
        g_free(appCtx[i]);
        close(fd);
    }

    g_mutex_lock(&disp_lock);
    if (display)
        XCloseDisplay(display);
    display = NULL;
    g_mutex_unlock(&disp_lock);
    g_mutex_clear(&disp_lock);

    if (main_loop)
    {
        g_main_loop_unref(main_loop);
    }

    if (ctx)
    {
        g_option_context_free(ctx);
    }

    if (return_value == 0)
    {
        g_print("App run successful\n");
    }
    else
    {
        g_print("App run failed\n");
    }

    gst_deinit();
    close(fd);
    return return_value;
}
