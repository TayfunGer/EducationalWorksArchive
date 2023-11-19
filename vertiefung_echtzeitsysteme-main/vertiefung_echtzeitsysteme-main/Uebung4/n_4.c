#include <sched.h>
#include <bsp.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <rtems/shell.h>
#include <pthread.h>

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

void start_shell(void)
{
    printf(" =========================\n");
    printf(" starting shell\n");
    printf(" =========================\n");
    rtems_shell_init(
        "SHLL",                       /* task name */
        RTEMS_MINIMUM_STACK_SIZE * 4, /* task stack size */
        100,                          /* task priority */
        "/dev/console",               /* device name */
        false,                        /* run forever */
        true,                         /* wait for shell to terminate */
        NULL                          /* login check function */
    );
    printf("Shell initilisiert");
}

// void *cpu_info(void *args)
// {
//   while(1)
//   {
//     rtems_cpu_info_report();
//     sleep();
//   }
// }

void *thread_job(void *args)
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

  pthread_exit(0);
}

int main_usercmd(int argc, char **argv)
{
  // pthread_t cpu_child;
  // pthread_attr_t attr;
  // struct sched_param param;

  // pthread_attr_init(&attr);
  // pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  // param.sched_priority = sched_get_priority_max(SCHED_FIFO);

  // if(pthread_create(&cpu_child, &attr, thread_job, NULL) || 
  //    pthread_setschedparam(cpu_child, SCHED_FIFO, &param)) 
  //   perror("Error on create child");

  if (argc == 1)
  {
    pthread_t child;

    if(pthread_create(&child, NULL, thread_job, NULL)) 
      perror("Error on create child");

    pthread_join(child, NULL);

    return 0;
  }

  pthread_t child[argc-1];

  for(int i = 1; i < argc; i++)
  {
    if (atoi(argv[i]) < sched_get_priority_min(SCHED_RR) || 
        atoi(argv[i]) > sched_get_priority_max(SCHED_RR))
      continue;

    printf("<< Create child %i with prio %i >>\n", i, atoi(argv[i]));

    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    param.sched_priority = atoi(argv[i]);

    if(pthread_create(&child[i], &attr, thread_job, NULL) || 
       pthread_setschedparam(child[i], SCHED_RR, &param)) 
      perror("Error on create child");
  }

  for(int i = 1; i < argc; i++)
  {
    pthread_join(child[i], NULL);
  } 

  //pthread_join(cpu_child, NULL);
  
  return 0;
}

rtems_shell_cmd_t Shell_USERCMD_Command = {
    "usercmd",                                                   /* name */
    "usercmd n1 [n2 [n3...]]",                                   /* usage */
    "user",                                                      /* topic */
    main_usercmd,                                                /* command */
    NULL,                                                        /* alias */
    NULL,                                                        /* next */
    S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH,   /* mode */
    0,                                                           /* uid */
    0                                                            /* gid */
};

void *POSIX_Init()
{
  printf("\n\n*** Aufgabe 4 N_4 mit argc, argv ***\n");
  
  start_shell();

  exit(0);
}


#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_LIBBLOCK

#define CONFIGURE_MAXIMUM_POSIX_THREADS              10
#define CONFIGURE_MAXIMUM_POSIX_KEYS (5)
#define CONFIGURE_POSIX_INIT_THREAD_TABLE
#define CONFIGURE_INIT
#include <rtems/confdefs.h>

#define CONFIGURE_SHELL_COMMANDS_INIT
#define CONFIGURE_SHELL_COMMANDS_ALL
#define CONFIGURE_SHELL_USER_COMMANDS &Shell_USERCMD_Command
#define CONFIGURE_SHELL_MOUNT_MSDOS
#include <rtems/shellconfig.h>