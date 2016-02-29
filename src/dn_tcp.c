/**
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "bcap/stdint.h"
#include <string.h>

#if defined(_USE_WIN_API)
#include <winsock2.h>
#include <mstcpip.h>
#pragma comment(lib, "wsock32.lib")
#elif defined(_USE_LINUX_API)
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#else
#include "bcap/dn_additional.h"
#endif

#include "bcap/dn_common.h"
#include "bcap/dn_device.h"
#include "bcap/dn_socket.h"
#include "bcap/dn_tcp.h"

#if defined(_USE_WIN_API)
static HRESULT _tcp_set_keepalive(int sock, int enable, uint32_t idle, uint32_t interval, uint32_t count)
{
	int ret;
	DWORD dwBytes;
	struct tcp_keepalive stParameter = {0}, stReturned = {0};

	stParameter.onoff = (enable != 0) ? 1: 0;
	stParameter.keepalivetime = (idle == 0) ? 1 : (idle * 1000);
	stParameter.keepaliveinterval = interval * 1000;
	ret = WSAIoctl(sock, SIO_KEEPALIVE_VALS, &stParameter, sizeof(stParameter), &stReturned, sizeof(stReturned), &dwBytes, NULL, NULL);

	return (ret == SOCKET_ERROR) ? OSERR2HRESULT(ret) : S_OK;
};
#elif defined(_USE_LINUX_API)
static HRESULT _tcp_set_keepalive(int sock, int enable, uint32_t idle, uint32_t interval, uint32_t count)
{
	int ret;

	enable = (enable != 0) ? 1: 0;
	ret = setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, (char *)&enable, sizeof(enable));
	if(ret < 0){
		ret = DNGetLastError();
		return OSERR2HRESULT(ret);
	}

	idle = (idle == 0) ? 1 : idle;
	ret = setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, (char *)&idle, sizeof(idle));
	if(ret < 0){
		ret = DNGetLastError();
		return OSERR2HRESULT(ret);
	}

	ret = setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, (char *)&interval, sizeof(interval));
	if(ret < 0){
		ret = DNGetLastError();
		return OSERR2HRESULT(ret);
	}

	ret = setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, (char *)&count, sizeof(count));
	if(ret < 0){
		ret = DNGetLastError();
		return OSERR2HRESULT(ret);
	}

	return S_OK;
};
#endif

HRESULT tcp_open_client(void *param, int *sock)
{
	int ret;
	HRESULT hr;
	struct sockaddr_in server;
	const struct CONN_PARAM_ETH *eth_param = (const struct CONN_PARAM_ETH *)param;

	if(param == NULL || sock == NULL)
		return E_INVALIDARG;

	hr = socket_open(SOCK_STREAM, sock);
	if(FAILED(hr)) return hr;

	memset(&server, 0, sizeof(server));
	server.sin_addr.s_addr = eth_param->dst_addr;
	server.sin_port = eth_param->dst_port;
	server.sin_family = AF_INET;

	ret = connect(*sock, (struct sockaddr *)&server, sizeof(server));
	if(ret < 0){
		ret = DNGetLastError();
		socket_close(sock);
		hr = OSERR2HRESULT(ret);
	}

	return hr;
}

HRESULT tcp_open_server(void *param, int *sock)
{
	int ret, flag;
	HRESULT hr;
	const struct CONN_PARAM_ETH *eth_param = (const struct CONN_PARAM_ETH *)param;

	if(param == NULL || sock == NULL)
		return E_INVALIDARG;

	if(eth_param->src_port == 0)
		return E_INVALIDARG;

	hr = socket_open(SOCK_STREAM, sock);
	if(FAILED(hr)) return hr;

	/* Sets reuse address option */
	flag = 1;
	ret = setsockopt(*sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&flag, sizeof(flag));
	if(ret < 0){
		ret = DNGetLastError();
		socket_close(sock);
		return OSERR2HRESULT(ret);
	}

	/* Binds the created socket */
	hr = socket_bind(eth_param, sock);
	if(FAILED(hr)){
		socket_close(sock);
		return hr;
	}

	if(SUCCEEDED(hr)){
		ret = listen(*sock, 1);
		if(ret < 0){
			ret = DNGetLastError();
			socket_close(sock);
			hr = OSERR2HRESULT(ret);
		}
	}

	return hr;
}

HRESULT tcp_close(int *sock)
{
	return socket_close(sock);
}

HRESULT tcp_send(int sock, const char *buf, uint32_t len_buf, void *arg)
{
	int ret, flag = 0;
	uint32_t len_send, len_sended;

	if(sock <= 0) return E_HANDLE;
	if(buf == NULL || strlen(buf) == 0) return E_INVALIDARG;

#ifdef MSG_NOSIGNAL
	flag |= MSG_NOSIGNAL;
#endif

	if(arg != NULL){
		flag |= *(int *)arg;
	}

	len_send = (len_buf != 0) ? len_buf : strlen(buf);

	ret = send(sock, buf, len_send, flag);
	len_sended = ret;

	if(ret < 0){
		ret = DNGetLastError();
		return OSERR2HRESULT(ret);
	}

	if(len_send > len_sended){
		return E_TIMEOUT;
	}

	return S_OK;
}

HRESULT tcp_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved, uint32_t timeout, void *arg)
{
	int ret, flag = 0;
	HRESULT hr;

	if(sock <= 0) return E_HANDLE;
	if(buf == NULL || len_recved == NULL) return E_INVALIDARG;

	if(arg != NULL){
		flag |= *(int *)arg;
	}

	hr = check_timeout(sock, timeout);
	if(FAILED(hr)) return hr;

	ret = recv(sock, buf, len_buf, flag);
	*len_recved = ret;

	if(ret < 0){
		ret = DNGetLastError();
		return OSERR2HRESULT(ret);
	}

	if(*len_recved == 0){
		return E_ACCESSDENIED;
	}

	return S_OK;
}

HRESULT tcp_set_timeout(int sock, uint32_t timeout)
{
	return socket_set_timeout(sock, timeout);
}

HRESULT tcp_clear(int sock, uint32_t timeout)
{
	uint32_t len_recv = DEV_BUF_MAX, len_recved;
	char buf_tmp[DEV_BUF_MAX];
	HRESULT hr;

	if(sock <= 0) return E_HANDLE;

	hr = tcp_set_timeout(sock, 0);
	if(FAILED(hr)) return hr;

	do{
		hr = tcp_recv(sock, buf_tmp, len_recv, &len_recved, 0, NULL);
	}while(SUCCEEDED(hr));

	hr = tcp_set_timeout(sock, timeout);

	return hr;
}

HRESULT tcp_accept(int sock, int *client)
{
	int ret;
	extern int socket_counter;
	HRESULT hr;

	if(sock <= 0) return E_HANDLE;
	if(client == NULL) return E_INVALIDARG;

	hr = check_timeout(sock, 0);
	if(FAILED(hr)) return hr;

	*client = accept(sock, NULL, NULL);
	if(*client < 0){
		*client = 0;
		ret = DNGetLastError();
		return OSERR2HRESULT(ret);
	}

	socket_counter++;

	return hr;
}

HRESULT tcp_set_keepalive(int sock, int enable, uint32_t idle, uint32_t interval, uint32_t count)
{
	if(sock <= 0) return E_HANDLE;
	return _tcp_set_keepalive(sock, enable, idle, interval, count);
}

HRESULT tcp_set_nodelay(int sock, int enable)
{
	int ret;

	if(sock <= 0) return E_HANDLE;
	
	enable = (enable != 0) ? 1 : 0;
	ret = setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&enable, sizeof(enable));
	if(ret < 0){
		ret = DNGetLastError();
		return OSERR2HRESULT(ret);
	}

	return S_OK;
}
