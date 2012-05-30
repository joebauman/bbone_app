

#include "net.h"

#include "i2c.h"

#include "uartStdio.h"

#define ECHO_SERVER_PORT        3891

// The poll delay is X*500ms
#define ECHO_POLL_INTERVAL      4

// Priority for tcp pcbs
#define ECHO_TCP_PRIO           TCP_PRIO_MAX

#define MAX_SIZE                4096

err_t net_send( struct tcp_pcb *pcb, struct pbuf *p );
err_t net_recv( void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err );
err_t net_accept( void *arg, struct tcp_pcb *pcb, err_t err );
void net_err( void *arg, err_t err );

extern unsigned char runData[ 32 ];
extern unsigned int runCommand;

// Global copy buffer for echoing data back to the sender
unsigned char mydata[ MAX_SIZE ];

// Global struct for tcp coms
struct tcp_pcb *global_pcb;

unsigned char char2num( unsigned char c )
{
    unsigned char ret = 0;

    // The next char should be a byte val - 0-F
    if( c >= 0x30 && c <= 0x39 ) // 0-9
    {
        ret = c - 0x30;
    }
    else if( c >= 0x41 && c <= 0x46 ) // A-F
    {
        ret = c - 0x37;
    }
    else if( c >= 0x61 && c <= 0x66 ) // a-f
    {
        ret = c - 0x57;
    }

    return ret;
}

unsigned char num2char( unsigned char c )
{
    unsigned char ret = ' ';

    // The next char should be a byte val - 0-F
    if( c < 0x0A )
    {
        ret = c + 0x30; // 0-9
    }
    else if( c < 0x10 )
    {
        ret = c - 0x0A + 0x41; // A-F
    }

    return ret;
}

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

    global_pcb = NULL;
}

err_t net_accept( void *arg, struct tcp_pcb *pcb, err_t err )
{
    //LWIP_UNUSED_ARG( err );

    // Decrease the listen backlog counter
    tcp_accepted( (struct tcp_pcb_listen*)arg );

    tcp_setprio( pcb, ECHO_TCP_PRIO );

    // Set up the various callback functions
    tcp_recv( pcb, net_recv );
    tcp_err( pcb,  net_err );
    tcp_poll( pcb, NULL, ECHO_POLL_INTERVAL );
    tcp_sent( pcb, NULL );

    global_pcb = pcb;

    return ERR_OK;
}

err_t net_recv( void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err )
{
    err_t err_send = ERR_OK;

    char *data;
    unsigned int cnt = 0, j, i;
    unsigned int len, tot_len;

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
    tot_len = p->tot_len;

    // Traverse pbuf chain and store payload of each pbuf into buffer 
    do
    {
        data = (char*)p->payload;
        len  = p->len;

        for( i = 0, j = 0; i < len; i++, j++, cnt++ )
        {
            mydata[ cnt ] = data[ j ];
        }
        p = p->next;

    } while( p != NULL );

    UARTPuts( "Received: \n\r", -1 );
    for( i = 0; i < tot_len; ++i )
    {
        UARTPutc( num2char( mydata[ i ] >> 4 ) );
        UARTPutc( num2char( mydata[ i ] & 0x0F ) );
        UARTPutc( ' ' );
    }
    UARTPuts( "\n\r", -1 );

    // Send the data to the main loop
    if( runCommand == 0 )
    {
        for( i = 0; i < tot_len; ++i )
        {
            runData[ i ] = mydata[ i ];
        }

        runCommand = 1;
    }

    // free pbuf's
    pbuf_free( p );

    // send the data in buffer over network with tcp header attached
    err_send = tcp_write( pcb, mydata, tot_len, TCP_WRITE_FLAG_COPY );

    //err_send = net_send( pcb, p );

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

void net_ext_send( unsigned char *data, int len )
{
    if( global_pcb != NULL && data != NULL && len > 0 )
    {
        tcp_write( global_pcb, data, len, TCP_WRITE_FLAG_COPY );
        tcp_output( global_pcb );
    }
}

void net_err( void *arg, err_t err )
{
    UARTPuts( "ERROR\n\r", -1 );
}
