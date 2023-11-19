#include <sched.h>
#include <bsp.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

double n_4(int n)
{
  int a,b,c,d;
  double s=0;
  for(a=1; a<=n; a++)
    for (b=1; b <=n; b++)
      for (c=1; c <= n; c++)
        for (d=1; d <=n; d++)
          s+=1;
  return s;
}


void *POSIX_Init()
{
  struct timeval start, end;
  double result = 0.0;

  for(int n = 0; n <= 270; n+=30)
  {
    gettimeofday(&start, NULL);
    result = n_4(n);
    gettimeofday(&end, NULL);

    printf("n = %i\nn_4 = %.0f\ntime = %ld seconds\n\n", n, result,
    ((end.tv_sec + end.tv_usec / 1000000) - (start.tv_sec + start.tv_usec / 1000000)));
  }

  exit(0);
}

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_POSIX_THREADS              10
#define CONFIGURE_POSIX_INIT_THREAD_TABLE
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
