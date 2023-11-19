#include <bsp.h>
#include <stdio.h>

void *POSIX_Init()
{
  printf("Hello World 932274\n");
  exit(0);
}

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_POSIX_THREADS              10
#define CONFIGURE_POSIX_INIT_THREAD_TABLE
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
