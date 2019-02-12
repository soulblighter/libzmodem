#if defined(__CYGWIN__) || defined(_WIN32)
#define _GNU_SOURCE
#include "string.h"
#endif

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <limits.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include "zmodem.h"


int fd = -1;

static
long long timeInMilliseconds() {
  struct timeval tv;

  gettimeofday(&tv, NULL);
  return (((long long) tv.tv_sec) * 1000) + (tv.tv_usec / 1000);
}

volatile long long baseTimestamp = 0;

static bool tick_cb(const char *fname, long bytes_sent, long bytes_total, long last_bps, int min_left, int sec_left) {
  //if( (bytes_sent<1000000) || ((bytes_sent%1000000)/2==1) ) return true;

  static long last_sec_left = 0;
  if ((timeInMilliseconds() - baseTimestamp) > 1000 && (last_sec_left != sec_left) && (sec_left != 0)) {
    //fprintf(stderr, "%s: Bytes Sent:%7ld/%7ld   BPS:%-8ld ETA %02d:%02d\n", fname, bytes_sent, bytes_total, last_bps, min_left, sec_left);
    last_sec_left = sec_left;
    baseTimestamp = timeInMilliseconds();
  }
  //usleep(10000);
  return true;
}

void complete_cb(const char *filename, int result, size_t size, time_t date, int file_num, int file_count) {
  if (result == RZSZ_NO_ERROR) {
    //fprintf(stderr, "'%s (%zu bytes)': successful send [%d/%d]\n", filename, size, file_num, file_count);
  } else {
    fprintf(stderr, "'%s': failed to send [%d/%d]\n", filename, file_num, file_count);
  }
}

int open_serial(const char *ttyDev, speed_t baudr) {
  int fd;
  struct termios tty;

  fd = open(ttyDev, O_RDWR | O_NOCTTY | O_SYNC);

  if (fd < 0) {
    fprintf(stderr, "Opening %s [%d]: errno(%d) - %s\n", ttyDev, fd, errno, strerror(errno));
    return -1;
  }

  /* Erase all flags */
  fcntl(fd, F_SETFL, 0);
  tcgetattr(fd, &tty);

  speed_t ispeed = cfgetispeed(&tty);
  speed_t ospeed = cfgetospeed(&tty);

  // set baudrate
  cfsetispeed(&tty, baudr);
  cfsetospeed(&tty, baudr);
  cfmakeraw(&tty); // avoid changing CR to LF
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remaping, no delays
  tty.c_cc[VMIN] = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem controls

  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;        // DISABLE hardware flow control

  tcsetattr(fd, TCSANOW, &tty);

  return fd;
}

int main(int argc, char *argv[]) {
  int c;
  bool bps_flag = false;
  uint64_t bps = 0u;

  bool hold_flag = false;

  bool com_flag = false;
  uint64_t com = 0u;

  int n_filenames = 0;
  const char **filenames = NULL;

  baseTimestamp = timeInMilliseconds();

  while ((c = getopt(argc, argv, "c:hq:")) != -1)
    switch (c) {
      case 'c':
        com = strtoul(optarg, NULL, 10);
        if (com >= 0) com_flag = true;
        break;
      case 'h':
        hold_flag = true;
        break;
      case '?':
        if (optopt == 'c') fprintf(stderr, "Option -c requires an integer argument.\n");
        else if (isprint(optopt)) fprintf(stderr, "Unknown option '-%c'.\n", optopt);
        else fprintf(stderr, "Unknown option '\\x%x'.\n", optopt);
        return 1;
      default:
        abort();
    }

  struct stat st;
  int ret;
  filenames = (const char **) malloc(argc * sizeof(char *));
  memset(filenames, 0, argc * sizeof(char *));
  for (int i = optind; i < argc; i++) {
    // These should be filenames of regular files
    ret = stat(argv[i], &st);
    if (ret == -1) fprintf(stderr, "'%s' does not exist.\n", argv[i]);
    else {
      if (S_ISDIR(st.st_mode)) fprintf(stderr, "'%s' is a directory.\n", argv[i]);
      else if (!S_ISREG(st.st_mode)) fprintf(stderr, "'%s' is not a regular file.\n", argv[i]);
      else {
        filenames[n_filenames] = strdup(argv[i]);
        n_filenames++;
      }
    }
  }

  if (com_flag == false || n_filenames == 0) {
    fprintf(stderr, "Usage:\n");
#if defined(__CYGWIN__) || defined(_WIN32)
    fprintf(stderr, "./msz -c <CDC COM number> <file>\n");
#else
    fprintf(stderr, "./msz -c <TTY ACM for Linux number> <file>\n");
#endif
    fprintf(stderr, "Ex.:\n");
    fprintf(stderr, "./msz -c 6 file.txt\n");
    free(filenames);
    return 0;
  }

  if (hold_flag == true) {
    fprintf(stderr, "Waiting for gdb\n");
    while (hold_flag == true)
      sleep(1);
  }

  char com_std[30];

#if defined(__CYGWIN__) || defined(_WIN32)
  sprintf(com_std, "/dev/ttyS%d", com - 1);
#else
  sprintf(com_std, "/dev/ttyACM%d", com);
#endif


  fd = open_serial(com_std, B115200);
  if (fd == -1) {
    exit(1);
  }

  for (int i = 0; i < n_filenames; i++) {

    size_t bytes = zmodem_send(fd, 1, &filenames[i], tick_cb, complete_cb, bps_flag ? bps : 0, RZSZ_FLAGS_NONE);

    struct timeval t;
    fd_set f;
    //unsigned char msg_char;
    char msg_buf[128];
    char msg[1024];
    memset(msg, 0, sizeof(msg));

    t.tv_sec = 15;
    t.tv_usec = 0;

    FD_ZERO(&f);
    FD_SET(fd, &f);

    int find_hash = 0;
    while (select( FD_SETSIZE, &f, NULL, NULL, &t)) {
      memset(msg_buf, 0, sizeof(msg_buf));

      int read_len = read(fd, msg_buf, 1);

      if(find_hash == 1) {
        strcat(msg, msg_buf);
      }

      if( read_len == 0 || msg_buf[0] == '\0' ) {
        break;
      } else {
        if(msg_buf[0] == '#') {
          find_hash = 1;
        }
      }
    }
    printf("%s: %s \r\n", filenames[i], msg);

    char ack = 1;
    write(fd, &ack, 1);
  }

  for (int i = 0; i < n_filenames; i++) {
    free(filenames[i]);
  }
  free(filenames);
  return 0;
}
