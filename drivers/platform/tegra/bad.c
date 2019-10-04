#include <module.h>

static char data[6];
static char the_rest[10 - sizeof(data)];

int __init bad_access()
{
     int i;

     /* overflow here */
     for (i = 0; i < 10; i ++) data[i] = '0' + i;

     return 0;
}

module_init(bad_access);
