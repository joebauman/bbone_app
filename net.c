

#include "net.h"

#include "uartStdio.h"

#define ECHO_SERVER_PORT        2000

// The poll delay is X*500ms
#define ECHO_POLL_INTERVAL      4

// Priority for tcp pcbs
#define ECHO_TCP_PRIO           TCP_PRIO_MAX

#define MAX_SIZE                4096

err_t net_send( struct tcp_pcb *pcb, struct pbuf *p );
err_t net_recv( void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err );
err_t net_accept( void *arg, struct tcp_pcb *pcb, err_t err );

extern unsigned int runCommand;

// Global copy buffer for echoing data back to the sender
unsigned char mydata[ MAX_SIZE ];


void net_init( void )
{
    struct tcp_pcb *pcb;

    pcb = tcp_new();
    tcp_bind( pcb, IP_ADDR_ANY, ECHO_SERVER_PORT );
    pcb = tcp_listen( pcb );
    tcp_arg( pcb, pcb );
    tcp_accept( pcb, net_accept );
}

void net_close( struct tcp_pcb *pcb )
{
    tcp_recv( pcb, NULL );
    tcp_close( pcb );
    tcp_arg( pcb, NULL );
    tcp_sent( pcb, NULL );
}

err_t net_accept( void *arg, struct tcp_pcb *pcb, err_t err )
{
    //LWIP_UNUSED_ARG( err );

    UARTPuts( "Enter net_accept\n\r", -1 );

    // Decrease the listen backlog counter
    tcp_accepted( (struct tcp_pcb_listen*)arg );

    tcp_setprio( pcb, ECHO_TCP_PRIO );

    // Set up the various callback functions
    tcp_recv( pcb, net_recv );
    tcp_err( pcb,  NULL );
    tcp_poll( pcb, NULL, ECHO_POLL_INTERVAL );
    tcp_sent( pcb, NULL );

    UARTPuts( "Exit net_accept\n\r", -1 );
    return ERR_OK;
}

err_t net_recv( void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err )
{
    err_t err_send;

    UARTPuts( "Enter net_recv\n\r", -1 );

    if( p != NULL )
    {
        // Inform TCP that we have taken the data.
        tcp_recved( pcb, p->tot_len );
    }
  
    if( ( err != ERR_OK ) || ( p == NULL ) )
    {
        // error or closed by other side
        if( p != NULL )
        {
            pbuf_free( p );
        }

        net_close( pcb );

        return ERR_OK;
    }

    // Do something weith the data?
    err_send = net_send( pcb, p );

    runCommand = 1;

    UARTPuts( "Exit net_recv\n\r", -1 );

    return err_send;
}

err_t net_send( struct tcp_pcb *pcb, struct pbuf *p )
{
    err_t err;
    char *data;
    unsigned int cnt = 0, j, i;
    unsigned int len, tot_len;
    struct pbuf *temp = p;

    tot_len = p->tot_len;

    // Traverse pbuf chain and store payload of each pbuf into buffer 
    do
    {
        UARTPuts( "net_send do...\n\r", -1 );

        data = (char*)p->payload;
        len  = p->len;

        for( i = 0, j = 0; i < len; i++, j++, cnt++ )
        {
            mydata[ cnt ] = data[ j ];
        }
        p = p->next;

    } while( p != NULL );

    // free pbuf's
    pbuf_free( temp );

    // send the data in buffer over network with tcp header attached
    err = tcp_write( pcb, mydata, tot_len, TCP_WRITE_FLAG_COPY );

    return err;
}

