#include <linux/module.h>
#include <linux/random.h>


static char data[6];

int __init bad_access(void)
{
     int i;
     int num;

     num = get_random_int();

     /* overflow here */
     for (i = 0; i < num+6; i ++) data[i] = '0' + i;

     return 0;
}

module_init(bad_access);
