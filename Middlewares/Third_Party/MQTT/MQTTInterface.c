/**
 ******************************************************************************
 * File Name          : MQTTInterface.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 SmartFactory s.r.l.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * Contributors:
 *    Federico Parente - initial API and implementation and/or initial documentation
 ******************************************************************************
 */
#include "MQTTInterface.h"
#include "stm32f4xx_hal.h"

#include <string.h>
#include "lwip.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include "leds.h"

#ifdef MQTT_LWIP_SOCKET_TLS
#include "mbedtls/net_sockets.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/debug.h"
#endif

uint32_t MilliTimer;

#ifdef MQTT_LWIP_SOCKET_TLS
mbedtls_net_context server_fd;
const char *pers = "mbedtls";

mbedtls_entropy_context entropy;
mbedtls_ctr_drbg_context ctr_drbg;
mbedtls_ssl_context ssl;
mbedtls_ssl_config conf;
mbedtls_x509_crt cacert;
mbedtls_x509_crt clicert;
mbedtls_pk_context pkey;
#endif

const char mbedtls_root_certificate[] =
		"-----BEGIN CERTIFICATE-----\r\n"
		    "MIIDlTCCAn2gAwIBAgIUKCfpw6t5lK6GDuZDtgD9w7FHqEMwDQYJKoZIhvcNAQEL\r\n"
		    "BQAwWjELMAkGA1UEBhMCaXQxCzAJBgNVBAgMAmFsMQswCQYDVQQHDAJ0cDELMAkG\r\n"
		    "A1UECgwCc2YxCzAJBgNVBAsMAmViMRcwFQYDVQQDDA4xOTIuMTY4LjEwMS42MzAe\r\n"
		    "Fw0yNDA3MTAxMzE4MjFaFw0yNTA3MTAxMzE4MjFaMFoxCzAJBgNVBAYTAml0MQsw\r\n"
		    "CQYDVQQIDAJhbDELMAkGA1UEBwwCdHAxCzAJBgNVBAoMAnNmMQswCQYDVQQLDAJl\r\n"
		    "YjEXMBUGA1UEAwwOMTkyLjE2OC4xMDEuNjMwggEiMA0GCSqGSIb3DQEBAQUAA4IB\r\n"
		    "DwAwggEKAoIBAQCXDHuChb178kyELFJ8Pxw1lNVt5mPlu8yndBZqU3dONo+NF6bw\r\n"
		    "o5cfQgZhNveSRNeuGVPOuDe/KkjaTsYpwqIFpNE6fYN9mpJmPImDox9z6qEiyOS9\r\n"
		    "/b4DlKLYcQ3ypWf4DPvi9Zpm2md7FTXfYUEI/HxJlSJUhF6VDsGxwS2UcTOalTZx\r\n"
		    "tcfIt+v0UChiYhybdD6u7+dBEMPkt200xKE48OOfDSeUGqM5LAYsQGNZstcxpwrS\r\n"
		    "r4gdjS9pmCxUqT99LoWidwrmxiZLuYwfH0ap2KQEZdQZIb25j+nX2LbS3D1sL5YW\r\n"
		    "kXNrmpXEmmKFIAIM/U/tddbTt6OGKv2V/zppAgMBAAGjUzBRMB0GA1UdDgQWBBSH\r\n"
		    "Hm+ISZlaDqa8FSoix0SLxdePVzAfBgNVHSMEGDAWgBSHHm+ISZlaDqa8FSoix0SL\r\n"
		    "xdePVzAPBgNVHRMBAf8EBTADAQH/MA0GCSqGSIb3DQEBCwUAA4IBAQAe3mV1yCN4\r\n"
		    "kq92BGbeY0W+KkQ8HoJYsulINaLxpLvxM/JnxXvZBknmWRPRQ5G4bKCPhLDgoj2U\r\n"
		    "w+PGqV71FPGUhFlF+ofKiARh4FsAlpxiTuC2RGYr/9TYRHJn1a3eCWc1+SNQBK8e\r\n"
		    "F2z6NbPZX1Ow32TU0KOjXaZ3im0lQLTBUNlymrXup/7P4eIuexQY8+BzQXLIw+DY\r\n"
		    "KAwcNnBcDtEc/I3uLzrF1LsYGZbJ8lElxjY13l2TzkAvonO90ZXMZSmWpssLW7k4\r\n"
		    "z81M5G1nHW10HNzukG1nrahcZMHsq9bxM86yA4k/0eIINxbxhByTNMYhq6744BDh\r\n"
		    "bBeMGN3kxMWs\r\n"
		    "-----END CERTIFICATE-----\r\n";

const size_t mbedtls_root_certificate_len = sizeof(mbedtls_root_certificate);


const char client_cert[] =
	    "-----BEGIN CERTIFICATE-----\r\n"
	    "MIIDhDCCAmygAwIBAgIUc+vC6JB/wiRlgXzEyCM31gLDFOowDQYJKoZIhvcNAQEL\r\n"
	    "BQAwWjELMAkGA1UEBhMCaXQxCzAJBgNVBAgMAmFsMQswCQYDVQQHDAJ0cDELMAkG\r\n"
	    "A1UECgwCc2YxCzAJBgNVBAsMAmViMRcwFQYDVQQDDA4xOTIuMTY4LjEwMS42MzAe\r\n"
	    "Fw0yNDA3MTAxMzE5NTVaFw0yNTA3MTAxMzE5NTVaMFoxCzAJBgNVBAYTAml0MQsw\r\n"
	    "CQYDVQQIDAJhbDELMAkGA1UEBwwCdG8xCzAJBgNVBAoMAnNmMQswCQYDVQQLDAJl\r\n"
	    "YjEXMBUGA1UEAwwOMTkyLjE2OC4xMDEuNjMwggEiMA0GCSqGSIb3DQEBAQUAA4IB\r\n"
	    "DwAwggEKAoIBAQC+edJqS8QecC0Q/+dR4h/89oSzhT9U7Es1DPYLFXekKz70T30n\r\n"
	    "qWZTBvIhGtmS6ZeDkjDIpQDf50KgqsPlHI84KNEPrcB3ZGlmvmUIvDILTIuSDWAF\r\n"
	    "WpO1ovRbwmU5GDqWWw4qMCT0QdvYbtbHkmMDyJ3x1gFtDejsXggFrlfvTvAJx/AH\r\n"
	    "lXytCsXpsCn+3+N2kXhABR3K7k/jMo60D924hi/980drYrwLxpKuV6+QFgpTAoZ4\r\n"
	    "aabwu4os6BdrS0O9IEzi7GADs0AlA8ewNTVjNqqUDgTzC7XdaMg4fwJsB9IKiLGZ\r\n"
	    "qzAWxyT6pxiGj8Ji8Y0zJHAKZW0sdzcIiwBdAgMBAAGjQjBAMB0GA1UdDgQWBBQ+\r\n"
	    "tgdM1sGHb+WtGBAxo+Suc8sXqjAfBgNVHSMEGDAWgBSHHm+ISZlaDqa8FSoix0SL\r\n"
	    "xdePVzANBgkqhkiG9w0BAQsFAAOCAQEAJ95diL7bnVOs2KCQbzv5SYyAUxuLLdBm\r\n"
	    "JUAbi4RNrFeHmgdiHIeAGXamlxZglAn8ovdp06DRNdMZRkG9kN3L+mnh4TIH7cUu\r\n"
	    "NbzBFzwkNcEdavyRn3lzjWXLQYtS+ixry1kUJ5rVPKFsuR9pRf19Wjd+ZWX0GO9Z\r\n"
	    "HdZ0QkSPosleYtHn0TKexs7+P+1x8vtI86cmiZPwn02tGh1OpTkvKmenzRxWw7m+\r\n"
	    "4YHwSump5LqkFyk+N+DnITXwJBn+l9RdZ2b1hVww1L/jlLAT1fxi+7FxFp2nZbwF\r\n"
	    "jasYJBVqhNm7As4Z85JGBjZI7HPwMXM2n1Aw5vV8s8W7DK+qJ+17og==\r\n"
	    "-----END CERTIFICATE-----\r\n";

const size_t client_cert_len = sizeof(client_cert);


const char client_key[] =
	    "-----BEGIN PRIVATE KEY-----\r\n"
	    "MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQC+edJqS8QecC0Q\r\n"
	    "/+dR4h/89oSzhT9U7Es1DPYLFXekKz70T30nqWZTBvIhGtmS6ZeDkjDIpQDf50Kg\r\n"
	    "qsPlHI84KNEPrcB3ZGlmvmUIvDILTIuSDWAFWpO1ovRbwmU5GDqWWw4qMCT0QdvY\r\n"
	    "btbHkmMDyJ3x1gFtDejsXggFrlfvTvAJx/AHlXytCsXpsCn+3+N2kXhABR3K7k/j\r\n"
	    "Mo60D924hi/980drYrwLxpKuV6+QFgpTAoZ4aabwu4os6BdrS0O9IEzi7GADs0Al\r\n"
	    "A8ewNTVjNqqUDgTzC7XdaMg4fwJsB9IKiLGZqzAWxyT6pxiGj8Ji8Y0zJHAKZW0s\r\n"
	    "dzcIiwBdAgMBAAECggEAQFEfYWAzGXoUZZybjfU2ivLs7Tdtoq3lWUUGoch+bTNj\r\n"
	    "HxjmMGnNkPST9uS7mhWBYV6QVXgN+wz6XQk8e5UwsSxrJ4mqp0YDJzvcHt55YYJX\r\n"
	    "0JnulfA3V0pui7tw9Z3+Tn3xowI9wDKq2wLWSG5gO8tWte3m7l2XjJTVzaFItRfT\r\n"
	    "0DCWUfb6jel+P6b0jffunMs11KI4tMGOIlb3VWt7ljTaXvebPpoZ9Kt5E0W0vqHs\r\n"
	    "J7JD1iNvw2bL2IQ6itF+g+Cf2YrBJuyaGpsrYPHRJpxUC1Z/Bd5tz3e6vPVeCOSi\r\n"
	    "goy3l1VdMjnUk4oYigNaH1JkAMyxShGoKw5vi77cCwKBgQDlEy6zxZXDidfZL4tT\r\n"
	    "wZkJ+iBkmSra5feIDknyNkVhilpVjJm1zdunrL727k43ClmJ1aaDdKOZOKHJdx9N\r\n"
	    "NuTL4uM8EnZx/ePc41jDmb2lxo9q/MFFRpNavAmhZonuoEIN4SaiUMbqIURIr+86\r\n"
	    "zUqAtWuRz/4gxHi58nAasCdaOwKBgQDU3TObp6nGyhCbB8O98+HMvy/XhBPoaFvw\r\n"
	    "ylQLNmwypPvg+Ce5dzKuvyJJq6ISyvzjN7sest+jUOqvNZ/ZXUa9A864GGcLghXH\r\n"
	    "wlyipZxLf7FAzz5UmuvwMclefYH8FXRj8MD1V6FTziHqOLjHcp/XHtiR42YkcEhO\r\n"
	    "dXPouzlORwKBgQCFwbSkXbu8CHHTrDJDfqiYrcdaViEy3dKyS/2bg1rxwHJMv6NF\r\n"
	    "B+W5O2HqF23uL4nmtKzc1y9rmSjG1VqeoG3qKxoaCoHEv8XcRZef5tZYxN8bTmif\r\n"
	    "xbzm3yMUbiYeAs9vAUeowVfUgAY6FxiuEg7tpoEgC/3MLkx77vbMbo0b3wKBgGEU\r\n"
	    "VRIbQDnSNAqQWvxJuuRHGYmfyfiHh87kZ7oJYwUh62HpqyxRqYK61udkaHFLtFPo\r\n"
	    "OeXBTG9OWwn3WeSnPri7gM7DClPcSxSkltzyzLo+DVfybInncc1E14LJmLugCUn/\r\n"
	    "JfF+uqve6ebJYbRMmYthnQHEBPR/ZOqrdGZi5LrHAoGAOxjXBJr9m7OWtWc6QCv2\r\n"
	    "ZJ+Fjiuxr60KGBwjK3Fgeo2AsDpP72wtEBFKjCQijbJLHm1bhUvw8ray8qO/81ox\r\n"
	    "6H3gcyUul76LlfHPpKOa/IV7F5R6GvvkQyxerJkOBsJ7UNg3lJ66JJEbQMoA7we5\r\n"
	    "jPOMOOrhlRIhcb7des6DPgU=\r\n"
	    "-----END PRIVATE KEY-----\r\n";

const size_t client_key_len = sizeof(client_key);


#ifdef MQTT_LWIP_SOCKET
void mqtt_network_init(Network *n) {
	n->socket = 0; //clear
	n->mqttread = mqtt_network_read; //receive function
	n->mqttwrite = mqtt_network_write; //send function
	n->disconnect = mqtt_network_disconnect; //disconnection function
}

int mqtt_network_connect(Network *n, char *ip, int port) {
	struct sockaddr_in server_addr;

	if(n->socket)
	{
		close(n->socket);
	}

	n->socket = socket(PF_INET, SOCK_STREAM, 0); //create socket
	if(n->socket < 0)
	{
		n->socket = 0;
		return -1;
	}

	memset(&server_addr, 0, sizeof(struct sockaddr_in)); //broker address info
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(ip);
	server_addr.sin_port = htons(port);

	if(connect(n->socket, (struct sockaddr*)&server_addr, sizeof(struct sockaddr_in)) < 0) //connect to the broker
	{
		close(n->socket);
		return -1;
	}
	return 0;
}

int mqtt_network_read(Network *n, unsigned char *buffer, int len, int timeout_ms) {
	int available;

	/* !!! LWIP_SO_RCVBUF must be enabled !!! */
	if(ioctl(n->socket, FIONREAD, &available) < 0) return -1; //check receive buffer

	if(available > 0)
	{
		return recv(n->socket, buffer, len, 0);
	}

	return 0;
}

int mqtt_network_write(Network *n, unsigned char *buffer, int len, int timeout_ms) {
	return send(n->socket, buffer, len, 0);
}

void mqtt_network_disconnect(Network *n) {
	close(n->socket);
	n->socket = 0;
}
#endif
#ifdef MQTT_LWIP_SOCKET_TLS

static void my_debug(void *ctx, int level, const char *file, int line, const char *str) {
	((void) level);
	//mbedtls_fprintf((FILE*) ctx, "%s:%04d: %s", file, line, str);
	//fprintf((FILE*) ctx, "%s:%04d: %s", file, line, str);
	MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE]: %s:%04d: %s", file, line, str);
	fflush((FILE*) ctx);
}


void mqtt_network_init(Network *n) {
	n->socket = 0; //clear
	n->mqttread = mqtt_network_read; //receive function
	n->mqttwrite = mqtt_network_write; //send function
	n->disconnect = mqtt_network_disconnect; //disconnection function
}

int mqtt_network_connect(Network *n, char *ip, char * port) {
	int ret = 0;

#if defined(MBEDTLS_DEBUG_C) && defined(DEBUG)
	mbedtls_debug_set_threshold(99);
#endif

	// Initialize the network interface
	mqtt_network_init(n);
	mqtt_network_clear();

	//mbedtls_net_init( &server_fd ); // MX_LWIP_Init() is called already
	mbedtls_ssl_init(&ssl);
	mbedtls_ssl_config_init(&conf);
	mbedtls_x509_crt_init(&cacert);
	mbedtls_x509_crt_init(&clicert);
	mbedtls_pk_init(&pkey);
	mbedtls_ctr_drbg_init(&ctr_drbg);
	mbedtls_entropy_init(&entropy);

	ret = psa_crypto_init();
	if(ret != PSA_SUCCESS) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: psa_crypto_init failed.\n");
		return -1;
	}

	if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
	                           (const unsigned char *) pers,
	                           strlen( pers ) ) ) != 0 )
	{
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_ctr_drbg_seed returned %d\n", ret );
	    return -1;
	}

	// Processi SSL/TLS
	ret = mbedtls_x509_crt_parse(&cacert, (const unsigned char*) mbedtls_root_certificate, mbedtls_root_certificate_len);
	if (ret < 0) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_x509_crt_parse failed.\n");
	    return -1;
	}

	// START
	// TLS V1.3
#if !defined(TLS_1V2) && defined(TLS_1V3)
	ret = mbedtls_x509_crt_parse(&clicert, (const unsigned char *)client_cert, client_cert_len);
	if (ret != 0) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_x509_crt_parse failed\n");
	    return -1;
	}

	// Aggiungi caricamento della chiave cliente
	ret = mbedtls_pk_parse_key(&pkey, (const unsigned char *) client_key, client_key_len, NULL, 0, mbedtls_ctr_drbg_random, &ctr_drbg);
	if (ret != 0) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_pk_parse_key failed.\n");
	    return -1;
	}

	// Configura il certificato e la chiave privata nel contesto SSL
	ret = mbedtls_ssl_conf_own_cert(&conf, &clicert, &pkey);
	if (ret != 0) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_ssl_conf_own_cert failed.\n");
	    return -1;
	}
#endif
	// END

	ret = mbedtls_ssl_config_defaults(&conf, MBEDTLS_SSL_IS_CLIENT,
	        MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
	if (ret < 0) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_ssl_config_defaults failed.\n");
	    return -1;
	}

	mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_REQUIRED);
	mbedtls_ssl_conf_ca_chain(&conf, &cacert, NULL);
	mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr_drbg);
	mbedtls_ssl_conf_dbg(&conf, my_debug, stdout);

	// TLS V1.2
#if defined(TLS_1V2) && !defined(TLS_1V3)
	mbedtls_ssl_conf_min_version(&conf, MBEDTLS_SSL_MAJOR_VERSION_3, MBEDTLS_SSL_MINOR_VERSION_3);
	mbedtls_ssl_conf_max_version(&conf, MBEDTLS_SSL_MAJOR_VERSION_3, MBEDTLS_SSL_MINOR_VERSION_3);
#endif
	// TLS V1.3
#if !defined(TLS_1V2) && defined(TLS_1V3)
	mbedtls_ssl_conf_min_version(&conf, MBEDTLS_SSL_MAJOR_VERSION_3, MBEDTLS_SSL_MINOR_VERSION_4);
	mbedtls_ssl_conf_max_version(&conf, MBEDTLS_SSL_MAJOR_VERSION_3, MBEDTLS_SSL_MINOR_VERSION_4);
#endif

	ret = mbedtls_ssl_setup(&ssl, &conf);
	if (ret < 0) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_ssl_setup failed.\n");
	    return -1;
	}

	ret = mbedtls_ssl_set_hostname(&ssl, ip); // if the handshake fail check here
	if (ret < 0) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_ssl_set_hostname failed.\n");
	    return -1;
	}

	mbedtls_ssl_set_bio(&ssl, &server_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

	// register functions
	n->mqttread = mqtt_network_read; //receive function
	n->mqttwrite = mqtt_network_write; //send function
	n->disconnect = mqtt_network_disconnect; //disconnection function



	// Connect

	ret = mbedtls_net_connect(&server_fd, (const char*)ip, port, MBEDTLS_NET_PROTO_TCP);
	if (ret < 0) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_net_connect failed.\n");
		return -1;
	}

	while ((ret = mbedtls_ssl_handshake(&ssl)) != 0) {
		if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
			MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_ssl_handshake failed.\n");
			return -2;
		}
	}

	ret = mbedtls_ssl_get_verify_result(&ssl);
	if (ret < 0) {
		MQTT_INTERFACE_DEBUG_LOG("[MQTT_INTERFACE] ERROR: mbedtls_ssl_get_verify_result failed.\n");
		return -1;
	}

	return 0;
}


int mqtt_network_read(Network *n, unsigned char *buffer, int len, int timeout_ms) {
	int ret;
	int received = 0;
	int error = 0;
	int complete = 0;

	//set timeout
	if (timeout_ms != 0) {
		mbedtls_ssl_conf_read_timeout(&conf, timeout_ms);
	}

	//read until received length is bigger than variable len
	do {
		ret = mbedtls_ssl_read(&ssl, buffer, len);
		if (ret > 0) {
			received += ret;
		} else if (ret != MBEDTLS_ERR_SSL_WANT_READ) {
			error = 1;
		}
		if (received >= len) {
			complete = 1;
		}
	} while (!error && !complete);

	return received;
}


int mqtt_network_write(Network *n, unsigned char *buffer, int len, int timeout_ms) {
	int ret;
	int written;

	//check all bytes are written
	for (written = 0; written < len; written += ret) {
		while ((ret = mbedtls_ssl_write(&ssl, buffer + written, len - written)) <= 0) {
			if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
				return ret;
			}
		}
	}

	return written;
}


void mqtt_network_disconnect(Network *n) {
	int ret;

	do {
		ret = mbedtls_ssl_close_notify(&ssl);
	} while (ret == MBEDTLS_ERR_SSL_WANT_WRITE);

	mbedtls_ssl_session_reset(&ssl);
	mbedtls_net_free(&server_fd);
}


void mqtt_network_clear() {
	mbedtls_net_free(&server_fd);
	mbedtls_x509_crt_free(&cacert);
	mbedtls_x509_crt_free(&clicert);
	mbedtls_pk_free(&pkey);
	mbedtls_psa_crypto_free();
	mbedtls_ssl_free(&ssl);
	mbedtls_ssl_config_free(&conf);
	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);
}

#endif
#ifdef MQTT_LWIP_NETCONN
void mqtt_network_init(Network *n) {
	n->conn = NULL;
	n->buf = NULL;
	n->offset = 0;

	n->mqttread = mqtt_network_read;
	n->mqttwrite = mqtt_network_write;
	n->disconnect = mqtt_network_disconnect;
}

int mqtt_network_connect(Network *n, char *ip, int port) {
	err_t err;
	ip_addr_t server_ip;

	ipaddr_aton(ip, &server_ip);

	n->conn = netconn_new(NETCONN_TCP);
	if (n->conn != NULL) {
		err = netconn_connect(n->conn, &server_ip, port);

		if (err != ERR_OK) {
			netconn_delete(n->conn); //free memory
			return -1;
		}
	}

	return 0;
}

int mqtt_network_read(Network *n, unsigned char *buffer, int len, int timeout_ms) {
	int rc;
	struct netbuf *inbuf;
	int offset = 0;
	int bytes = 0;

	while(bytes < len) {
		if(n->buf != NULL) {
			inbuf = n->buf;
			offset = n->offset;
			rc = ERR_OK;
		} else {
			rc = netconn_recv(n->conn, &inbuf);
			offset = 0;
		}

		if(rc != ERR_OK) {
			if(rc != ERR_TIMEOUT) {
				bytes = -1;
			}
			break;
		} else {
			int nblen = netbuf_len(inbuf) - offset;
			if((bytes+nblen) > len) {
				netbuf_copy_partial(inbuf, buffer+bytes, len-bytes,offset);
				n->buf = inbuf;
				n->offset = offset + len - bytes;
				bytes = len;
			} else {
				netbuf_copy_partial(inbuf, buffer+bytes, nblen, offset);
				bytes += nblen;
				netbuf_delete(inbuf);
				n->buf = NULL;
				n->offset = 0;
			}
		}
	}
	return bytes;
}

int mqtt_network_write(Network *n, unsigned char *buffer, int len, int timeout_ms) {
	int rc = netconn_write(n->conn, buffer, len, NETCONN_NOCOPY);
	if(rc != ERR_OK) return -1;
	return len;
}

void mqtt_network_disconnect(Network *n) {
	netconn_close(n->conn); //close session
	netconn_delete(n->conn); //free memory
	n->conn = NULL;
}
#endif

#ifdef MQTT_TASK
int ThreadStart(Thread* thread, void (*fn)(void*), void* arg)
{
	int rc = 0;
	uint16_t usTaskStackSize = (configMINIMAL_STACK_SIZE * 5);
	UBaseType_t uxTaskPriority = uxTaskPriorityGet(NULL); /* set the priority as the same as the calling task*/

	rc = xTaskCreate(fn,	/* The function that implements the task. */
		"MQTTTask",			/* Just a text name for the task to aid debugging. */
		usTaskStackSize,	/* The stack size is defined in FreeRTOSIPConfig.h. */
		arg,				/* The task parameter, not used in this case. */
		uxTaskPriority,		/* The priority assigned to the task is defined in FreeRTOSConfig.h. */
		&thread->task);		/* The task handle is not used. */

	return rc;
}


void MutexInit(Mutex* mutex)
{
	mutex->sem = xSemaphoreCreateMutex();
}

int MutexLock(Mutex* mutex)
{
	return xSemaphoreTake(mutex->sem, portMAX_DELAY);
}

int MutexUnlock(Mutex* mutex)
{
	return xSemaphoreGive(mutex->sem);
}
#endif

//Timer functions
char TimerIsExpired(Timer *timer) {
	long left = timer->end_time - MilliTimer;
	return (left < 0);
}

void TimerCountdownMS(Timer *timer, unsigned int timeout) {
	timer->end_time = MilliTimer + timeout;
}

void TimerCountdown(Timer *timer, unsigned int timeout) {
	timer->end_time = MilliTimer + (timeout * 1000);
}

int TimerLeftMS(Timer *timer) {
	long left = timer->end_time - MilliTimer;
	return (left < 0) ? 0 : left;
}

void TimerInit(Timer *timer) {
	timer->end_time = 0;
}

