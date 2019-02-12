#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include <limits.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include "zmodem.h"

#include <signal.h>

#if defined(_MSC_VER)
#include <direct.h>
#define getcwd _getcwd
#elif defined(__GNUC__)
#include <unistd.h>
#endif


static bool
approver_cb(const char *filename, size_t size, time_t date, int file_num, int file_count)
{
  fprintf(stderr, "Sender requests to send %s: %zu bytes [%d/%d]\n", filename, size, file_num, file_count);
  return true;
}

static
bool tick_cb(const char *fname, long bytes_sent, long bytes_total, long last_bps, int min_left, int sec_left)
{
  //if( (bytes_sent<1000000) || ((bytes_sent%1000000)/2==1) ) return true;

  static long last_sec_left = 0;
  if ((last_sec_left != sec_left) && (sec_left != 0)) {
    fprintf(stderr, "%s: Bytes Received:%7ld/%7ld   BPS:%-8ld ETA %02d:%02d\n",
	    fname, bytes_sent, bytes_total,
	    last_bps, min_left, sec_left);
    last_sec_left = sec_left;
  }
  //usleep(10000);
  return true;
}


void complete_cb(const char *filename, int result, size_t size, time_t date, int file_num, int file_count)
{
  if (result == RZSZ_NO_ERROR)
    fprintf(stderr, "'%s': received [%d/%d]\n", filename, file_num, file_count);
  else
    fprintf(stderr, "'%s': failed to receive [%d/%d]\n", filename, file_num, file_count);
}


int open_serial(const char *ttyDev, speed_t baudr)
{
  int fd;
  struct termios tty;

  fd = open(ttyDev, O_RDWR | O_NOCTTY | O_SYNC);

  if(fd < 0) {
    fprintf(stderr, "Opening %s [%d]: errno(%d) - %s", ttyDev, fd, errno, strerror(errno));
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
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.1 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem controls

  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;        // DISABLE hardware flow control

  tcsetattr(fd, TCSANOW, &tty);

  return fd;
}






int
main(int argc, char *argv[])
{
  int c;
  bool bps_flag = false;
  uint64_t bps = 0u;

  while ((c = getopt(argc, argv, "b:q")) != -1)
    switch(c)
      {
      case 'b':
	bps = strtoul(optarg, NULL, 10);
	if (bps > 0)
	  bps_flag = true;
	break;
      case '?':
	if (optopt == 'b')
	  fprintf(stderr, "Option -b requires an integer argument.\n");
	else if (isprint (optopt))
	  fprintf(stderr, "Unknown option '-%c'.\n", optopt);
	else
	  fprintf (stderr, "Unknown option '\\x%x'.\n", optopt);
	return 1;
      default:
	abort ();
      }

  char cwd[PATH_MAX];
  getcwd(cwd, sizeof(cwd));

  int fd = open_serial("/dev/ttyGS0", B115200);

  size_t bytes = zmodem_receive(fd, cwd, /* use current directory */
				15,
				approver_cb, /* receive everything */
				tick_cb,
				complete_cb,
				NULL,
				bps_flag ? bps : 0,
				RZSZ_FLAGS_NONE);
  fprintf(stderr, "Received %zu bytes.\n", bytes);
  return 0;
}
