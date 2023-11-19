#include <sys/time.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <errno.h>

pthread_mutex_t mutex;
pthread_cond_t cond;

void * print_hello(void * arg)
{
  struct timeval time;
  struct timespec timeout;

  pthread_mutex_lock(&mutex); //

  gettimeofday(&time, NULL);
  timeout.tv_sec = time.tv_sec + 2;
  timeout.tv_nsec = time.tv_usec * 1000;
  printf("<child>: Hello World coming to wait!\n");

  if (errno = pthread_cond_timedwait(&cond, &mutex, &timeout))
    perror ("Error on pthread_cond_timedwait");

  printf("<child>: Hello World exit to wait!\n");
  
  pthread_mutex_unlock(&mutex); //

  return NULL;
}

void *POSIX_Init(void *arg)
{
  pthread_t child;
  if ( pthread_create( &child, NULL, print_hello, NULL ))
    perror("Error on pthread_create");
  printf("<main>: Wait for child thread...\n");
  if ( pthread_join( child, NULL ))
    perror("Error on pthread_join");
  // return 0;
  exit(0);
}

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_POSIX_THREADS              10
#define CONFIGURE_POSIX_INIT_THREAD_TABLE

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
