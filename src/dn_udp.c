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
#pragma comment(lib, "wsock32.lib")
typedef int socklen_t;
#elif defined(_USE_LINUX_API)
#include <arpa/inet.h>
#include <errno.h>
#include <sys/socket.h>
#else
#include "bcap/dn_additional.h"
#endif

#include "bcap/dn_common.h"
#include "bcap/dn_device.h"
#include "bcap/dn_socket.h"
#include "bcap/dn_udp.h"

HRESULT udp_open(void *param, int *sock)
{
	HRESULT hr;
	const struct CONN_PARAM_ETH *eth_param = (const struct CONN_PARAM_ETH *)param;

	if(param == NULL || sock == NULL)
		return E_INVALIDARG;

	hr = socket_open(SOCK_DGRAM, sock);
	if(FAILED(hr)) return hr;

	/* Binds the created socket */
	hr = socket_bind(eth_param, sock);
	if(FAILED(hr)){
		socket_close(sock);
		return hr;
	}

	return hr;
}

HRESULT udp_close(int *sock)
{
	return socket_close(sock);
}

HRESULT udp_send(int sock, const char *buf, uint32_t len_buf, void *arg)
{
	int ret, flag = 0;
	uint32_t len_send, len_sended;
	struct udpaddr *opt = (struct udpaddr *)arg;

	if(sock <= 0) return E_HANDLE;
	if(buf == NULL || strlen(buf) == 0 || arg == NULL) return E_INVALIDARG;

	len_send = (len_buf != 0) ? len_buf : strlen(buf);

	flag |= opt->flag;

	ret = sendto(sock, buf, len_send, flag, (struct sockaddr *)&opt->addr, sizeof(struct sockaddr_in));
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

HRESULT udp_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved, uint32_t timeout, void *arg)
{
	int ret, flag = 0;
	socklen_t len_from = sizeof(struct sockaddr_in);
	struct udpaddr *opt = (struct udpaddr *)arg;
	HRESULT hr;

	if(sock <= 0) return E_HANDLE;
	if(buf == NULL || len_recved == NULL || arg == NULL) return E_INVALIDARG;

	hr = check_timeout(sock, timeout);
	if(FAILED(hr)) return hr;

	flag |= opt->flag;

	ret = recvfrom(sock, buf, len_buf, flag, (struct sockaddr *)&opt->addr, &len_from);
	*len_recved = ret;

	if(ret < 0){
		ret = DNGetLastError();
		return OSERR2HRESULT(ret);
	}

	return S_OK;
}

HRESULT udp_set_timeout(int sock, uint32_t timeout)
{
	return socket_set_timeout(sock, timeout);
}

HRESULT udp_clear(int sock, uint32_t timeout)
{
	uint32_t len_recv = DEV_BUF_MAX, len_recved;
	char buf_tmp[DEV_BUF_MAX];
	HRESULT hr;
	struct udpaddr opt;

	if(sock <= 0) return E_HANDLE;

	hr = udp_set_timeout(sock, 0);
	if(FAILED(hr)) return hr;

	opt.flag = 0;
	do{
		hr = udp_recv(sock, buf_tmp, len_recv, &len_recved, 0, &opt);
	}while(SUCCEEDED(hr));

	hr = udp_set_timeout(sock, timeout);

	return hr;
}

HRESULT udp_check_sockaddr(const struct sockaddr_in *sock_to, const struct sockaddr_in *sock_from)
{
	if(sock_to == NULL || sock_from == NULL) return E_INVALIDARG;

	if ((sock_to->sin_addr.s_addr != sock_from->sin_addr.s_addr) || (sock_to->sin_port != sock_from->sin_port)) {
		return E_ACCESSDENIED;
	}

	return S_OK;
}
