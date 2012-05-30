
#include "lwip/tcp.h"

void net_init( void );
void net_close( struct tcp_pcb *pcb );

void net_ext_send( unsigned char *data, int len );

