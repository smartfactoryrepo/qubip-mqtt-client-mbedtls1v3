/*
 * nanomodbus_interface.c
 *
 *  Created on: Aug 13, 2024
 *      Author: federico
 */
#include "main.h"
#include "nanomodbus_interface.h"
#include "nanomodbus.h"
#include "lwip.h"
#include "lwip/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "ethernetif.h"
#include "cmsis_os.h"


int8_t nmbs_platform_setup(int* fd, nmbs_t *nmbs)
{
	if(*fd >= 0)
	{
		nmbs_platform_disconnect(*fd);
	}

	// Mi devo connettere
	LOG_DEBUG("Connect to modbus server (PLC)\n");
	*fd = nmbs_platform_connect_tcp(MOBBUS_PLC_IP, MODBUS_PLC_PORT);
	if(*fd < 0)
	{
		// Non sono connesso c'è un errore
		LOG_DEBUG("Error while connecting to plc\n");
		return -1;
	}

	nmbs_platform_conf platform_conf;
	platform_conf.transport = NMBS_TRANSPORT_TCP;
	platform_conf.read = nmbs_platform_read_timeout; //nmbs_platform_read;
	platform_conf.write = nmbs_platform_send;
	platform_conf.arg = (void*) fd;    // Passing our TCP connection handle to the read/write functions

	// Create the modbus client
	nmbs_error err = nmbs_client_create(nmbs, &platform_conf);
	if (err != NMBS_ERROR_NONE)
	{
		LOG_DEBUG("Error creating modbus client\n");
	    if (!nmbs_error_is_exception(err))
	    {
	    	LOG_DEBUG("Exception occurred in nmbs_client_create\n");
	    	nmbs_platform_disconnect(*fd);
	    }
	    return -1;
	}

	// Set only the response timeout. Byte timeout will be handled by the TCP connection
	nmbs_set_read_timeout(nmbs, 5000); // 5 seconds read timeout
	nmbs_set_byte_timeout(nmbs, 1000); // 1 second byte timeout

	return 0;
}


int32_t nmbs_platform_connect_tcp(const char *host, const char *port)
{
	int ret = -1;
	int fd = -1;
	struct addrinfo hints;
	struct addrinfo *addr_list;
	struct addrinfo *cur;

	/* Do name resolution with both IPv6 and IPv4 */
	memset( &hints, 0, sizeof( hints ) );
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	if( getaddrinfo( host, port, &hints, &addr_list ) != 0 )
	{
		LOG_DEBUG("Error in getaddrinfo\n");
		return -1;
	}

	/* Try the sockaddrs until a connection succeeds */
	for( cur = addr_list; cur != NULL; cur = cur->ai_next )
	{
		fd = socket( cur->ai_family, cur->ai_socktype, cur->ai_protocol );
	    if(fd < 0)
	    {
	    	LOG_DEBUG("Error net socket failed\n");
	    	ret = -1;
	    	continue;
	    }

	    if( connect( fd, cur->ai_addr, cur->ai_addrlen ) == 0 )
	    {
	    	ret = 0;
	    	break;
	    }

	    nmbs_platform_disconnect(fd);

	    LOG_DEBUG("Error net connect failed\n");
	    ret = -1;
	}

	freeaddrinfo(addr_list);

	return (ret == -1) ? ret : fd;
}


int32_t nmbs_platform_send(const uint8_t* buf, uint16_t len, int32_t timeout_ms, void* arg )
{
  int fd = *(int*) arg;
  int ret = 0;

  if( fd < 0 )
  {
	  LOG_DEBUG("Error write, net invalid context\n");
	  return -1;
  }

  ret = write( fd, buf, len );

  if( ret < 0 )
  {
	  if( errno == EPIPE || errno == ECONNRESET )
	  {
		  LOG_DEBUG("Error write, net conn reset or broken pipe\n");
		  return -1;
	  }

	  if( errno == EINTR )
	  {
		  LOG_DEBUG("Error write, Interrupted system call\n");
		  return -1;
	  }

	  LOG_DEBUG("Error write, net send failed\n");
	  return -1;
  }

  return ret;
}

int32_t nmbs_platform_read(uint8_t* buf, uint16_t count, int32_t timeout_ms, void* arg)
{
    int ret = 0;
    int fd = *(int*) arg;

    if( fd < 0 )
    {
    	LOG_DEBUG("Error read, net invalid context\n");
    	return -1;
    }

    ret = read( fd, buf, count );

    if( ret < 0 )
    {
        if( errno == EPIPE || errno == ECONNRESET )
        {
        	LOG_DEBUG("Error read, net conn reset or broken pipe\n");
        	return -1;
        }

        if( errno == EINTR )
        {
        	LOG_DEBUG("Error read, Interrupted system call \n");
        	return -1;
        }

        LOG_DEBUG("Error write, net send failed\n");
        return -1;
  }

  return ret;
}

int32_t nmbs_platform_read_timeout(uint8_t* buf, uint16_t count, int32_t timeout_ms, void* arg)
{
    int ret = 0;
    int fd = *(int*) arg;

    if( fd < 0 )
    {
        LOG_DEBUG("Error read, net invalid context\n");
        return -1;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(fd + 1, &readfds, NULL, NULL, &timeout);

    if (ret == -1)
    {
        LOG_DEBUG("Error select\n");
        return -1;
    }
    else if (ret == 0)
    {
        //LOG_DEBUG("Timeout read\n");
        return 0; // Timeout
    }
    else
    {
        // Dati disponibili per la lettura
        ret = read( fd, buf, count );

        if( ret < 0 )
        {
            if( errno == EPIPE || errno == ECONNRESET )
            {
                LOG_DEBUG("Error read, net conn reset or broken pipe\n");
                return -1;
            }

            if( errno == EINTR )
            {
                LOG_DEBUG("Error read, Interrupted system call \n");
                return -1;
            }

            LOG_DEBUG("Error write, net send failed\n");
            return -1;
        }
    }

    return ret;
}


void nmbs_platform_disconnect(int fd)
{
    close(fd);
    printf("Closed connection %d\n", fd);
}





