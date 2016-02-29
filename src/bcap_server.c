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
#include <stdlib.h>
#include <string.h>

#if defined(_USE_WIN_API)
#include <process.h>
#include <winsock2.h>
#pragma comment(lib, "wsock32.lib")
#elif defined(_USE_LINUX_API)
#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <termios.h>
#else
#include "bcap/dn_additional.h"
#endif

#include "bcap/dn_common.h"
#include "bcap/dn_device.h"
#include "bcap/dn_tcp.h"
#include "bcap/dn_udp.h"
#include "bcap/dn_com.h"
#include "bcap/dn_thread.h"
#include "bcap/bcap_common.h"
#include "bcap/bcap_server.h"

/**
 * @def		_BCAP_SERVER_MAX
 * @brief	A definition for the maximum count of servers.
 */
#define _BCAP_SERVER_MAX	(BCAP_TCP_MAX + BCAP_UDP_MAX + BCAP_COM_MAX)

/**
 * @def		_FUNCTION_ID_MAX
 * @brief	A definition for the maximum count of b-CAP function IDs.
 */
#define _FUNCTION_ID_MAX	(137 + 1)

/**
 * @enum	CHANGE_RELATION
 * @brief	change relation information.
 */
enum CHANGE_RELATION {
	ADD_CHILD,		/** Add a child			*/
	DELETE_CHILD,	/** Delete all children	*/
	DESTROY_SELF,	/** Destroy self		*/
};

/**
 * @struct	VEC_OBJECT
 * @brief	A vector for stacking created objects.
 */
struct VEC_OBJECT {
	int32_t id;					/**< Function ID				*/
	uint32_t hObj;				/**< Object handle				*/
	struct VEC_OBJECT *prev;	/**< Object pointer for later	*/
	struct VEC_OBJECT *next;	/**< Object pointer for earlier	*/
};

/**
 * @struct	MAP_ID
 * @brief	A map for function id information.
 */
struct MAP_ID {
	int32_t relation_id;	/**< Function ID */
	int return_flag;		/**< If 1 then sends the response packet with a argument, else without any argument */
};

/**
 * @struct	CONN_BCAP_SERVER
 * @brief	b-CAP server communication object.
 */
struct CONN_BCAP_SERVER {
	struct CONN_PARAM_COMMON device;	/**< Common communication object									*/
	struct BCAP_PACKET last_send;		/**< The last send b-CAP packet										*/
	struct BCAP_PACKET last_recv;		/**< The last received b-CAP packet									*/
	uint32_t exec_timeout;				/**< executing timeout												*/
	uint32_t wdt_interval;				/**< watch dog timer interval										*/
	THREAD main_thread;					/**< Main thread object												*/
	THREAD sub_thread;					/**< Sub thread object												*/
	EVENT term_main_evt;				/**< Event object for terminate main thread 						*/
	EVENT term_sub_evt;					/**< Event object for terminate sub thread							*/
	EVENT exec_evt;						/**< Event object for synchronize with recv_thread and exec_thread	*/
	EVENT comp_evt;						/**< Event object for synchronize with recv_thread and exec_thread	*/
	int num_object;						/**< The number of created object									*/
	struct VEC_OBJECT *stack;			/**< The stack pointer for created objects							*/
	int num_child;						/**< The number of client connection (for TCP connection)			*/
	MUTEX *relation_mutex;				/**< Mutex object for synchronize change_relation function			*/
	struct CONN_BCAP_SERVER *parent;	/**< Parent node													*/
	struct CONN_BCAP_SERVER *node1;		/**< If it is root node, then node1 means the head of children list, else the younger brother	*/
	struct CONN_BCAP_SERVER *node2;		/**< If it is root node, then node2 means the head of delete list, else the elder brother		*/
};

static CALL_FUNC_BCAP m_list_func[_FUNCTION_ID_MAX];
static struct CONN_BCAP_SERVER m_conn_param[_BCAP_SERVER_MAX];
static const struct MAP_ID m_map_id[_FUNCTION_ID_MAX] = {
		{0, 0},
		{ID_SERVICE_STOP, 0},			// ID_SERVICE_START
		{-1, 0},						// ID_SERVICE_STOP
		{ID_CONTROLLER_DISCONNECT, 1},	// ID_CONTROLLER_CONNECT
		{-1, 0},						// ID_CONTROLLER_DISCONNECT
		{ID_EXTENSION_RELEASE, 1},		// ID_CONTROLLER_GETEXTENSION
		{ID_FILE_RELEASE, 1},			// ID_CONTROLLER_GETFILE
		{ID_ROBOT_RELEASE, 1},			// ID_CONTROLLER_GETROBOT
		{ID_TASK_RELEASE, 1},			// ID_CONTROLLER_GETTASK
		{ID_VARIABLE_RELEASE, 1},		// ID_CONTROLLER_GETVARIABLE
		{ID_COMMAND_RELEASE, 1},		// ID_CONTROLLER_GETCOMMAND
		{0, 1},							// ID_CONTROLLER_GETEXTENSIONNAMES
		{0, 1},							// ID_CONTROLLER_GETFILENAMES
		{0, 1},							// ID_CONTROLLER_GETROBOTNAMES
		{0, 1},							// ID_CONTROLLER_GETTASKNAMES
		{0, 1},							// ID_CONTROLLER_GETVARIABLENAMES
		{0, 1},							// ID_CONTROLLER_GETCOMMANDNAMES
		{0, 1},							// ID_CONTROLLER_EXECUTE
		{ID_MESSAGE_RELEASE, 1},		// ID_CONTROLLER_GETMESSAGE
		{0, 1},							// ID_CONTROLLER_GETATTRIBUTE
		{0, 1},							// ID_CONTROLLER_GETHELP
		{0, 1},							// ID_CONTROLLER_GETNAME
		{0, 1},							// ID_CONTROLLER_GETTAG
		{0, 0},							// ID_CONTROLLER_PUTTAG
		{0, 1},							// ID_CONTROLLER_GETID
		{0, 0},							// ID_CONTROLLER_PUTID
		{ID_VARIABLE_RELEASE, 1},		// ID_EXTENSION_GETVARIABLE
		{0, 1},							// ID_EXTENSION_GETVARIABLENAMES
		{0, 1},							// ID_EXTENSION_EXECUTE
		{0, 1},							// ID_EXTENSION_GETATTRIBUTE
		{0, 1},							// ID_EXTENSION_GETHELP
		{0, 1},							// ID_EXTENSION_GETNAME
		{0, 1},							// ID_EXTENSION_GETTAG
		{0, 0},							// ID_EXTENSION_PUTTAG
		{0, 1},							// ID_EXTENSION_GETID
		{0, 0},							// ID_EXTENSION_PUTID
		{-1, 0},						// ID_EXTENSION_RELEASE
		{ID_FILE_RELEASE, 1},			// ID_FILE_GETFILE
		{ID_VARIABLE_RELEASE, 1},		// ID_FILE_GETVARIABLE
		{0, 1},							// ID_FILE_GETFILENAMES
		{0, 1},							// ID_FILE_GETVARIABLENAMES
		{0, 1},							// ID_FILE_EXECUTE
		{0, 0},							// ID_FILE_COPY
		{0, 0},							// ID_FILE_DELETE
		{0, 0},							// ID_FILE_MOVE
		{0, 1},							// ID_FILE_RUN
		{0, 1},							// ID_FILE_GETDATECREATED
		{0, 1},							// ID_FILE_GETDATELASTACCESSED
		{0, 1},							// ID_FILE_GETDATELASTMODIFIED
		{0, 1},							// ID_FILE_GETPATH
		{0, 1},							// ID_FILE_GETSIZE
		{0, 1},							// ID_FILE_GETTYPE
		{0, 1},							// ID_FILE_GETVALUE
		{0, 0},							// ID_FILE_PUTVALUE
		{0, 1},							// ID_FILE_GETATTRIBUTE
		{0, 1},							// ID_FILE_GETHELP
		{0, 1},							// ID_FILE_GETNAME
		{0, 1},							// ID_FILE_GETTAG
		{0, 0},							// ID_FILE_PUTTAG
		{0, 1},							// ID_FILE_GETID
		{0, 0},							// ID_FILE_PUTID
		{-1, 0},						// ID_FILE_RELEASE
		{ID_VARIABLE_RELEASE, 1},		// ID_ROBOT_GETVARIABLE
		{0, 1},							// ID_ROBOT_GETVARIABLENAMES
		{0, 1},							// ID_ROBOT_EXECUTE
		{0, 0},							// ID_ROBOT_ACCELERATE
		{0, 0},							// ID_ROBOT_CHANGE
		{0, 0},							// ID_ROBOT_CHUCK
		{0, 0},							// ID_ROBOT_DRIVE
		{0, 0},							// ID_ROBOT_GOHOME
		{0, 0},							// ID_ROBOT_HALT
		{0, 0},							// ID_ROBOT_HOLD
		{0, 0},							// ID_ROBOT_MOVE
		{0, 0},							// ID_ROBOT_ROTATE
		{0, 0},							// ID_ROBOT_SPEED
		{0, 0},							// ID_ROBOT_UNCHUCK
		{0, 0},							// ID_ROBOT_UNHOLD
		{0, 1},							// ID_ROBOT_GETATTRIBUTE
		{0, 1},							// ID_ROBOT_GETHELP
		{0, 1},							// ID_ROBOT_GETNAME
		{0, 1},							// ID_ROBOT_GETTAG
		{0, 0},							// ID_ROBOT_PUTTAG
		{0, 1},							// ID_ROBOT_GETID
		{0, 0},							// ID_ROBOT_PUTID
		{-1, 0},						// ID_ROBOT_RELEASE
		{ID_VARIABLE_RELEASE, 1},		// ID_TASK_GETVARIABLE
		{0, 1},							// ID_TASK_GETVARIABLENAMES
		{0, 1},							// ID_TASK_EXECUTE
		{0, 0},							// ID_TASK_START
		{0, 0},							// ID_TASK_STOP
		{0, 0},							// ID_TASK_DELETE
		{0, 1},							// ID_TASK_GETFILENAME
		{0, 1},							// ID_TASK_GETATTRIBUTE
		{0, 1},							// ID_TASK_GETHELP
		{0, 1},							// ID_TASK_GETNAME
		{0, 1},							// ID_TASK_GETTAG
		{0, 0},							// ID_TASK_PUTTAG
		{0, 1},							// ID_TASK_GETID
		{0, 0},							// ID_TASK_PUTID
		{-1, 0},						// ID_TASK_RELEASE
		{0, 1},							// ID_VARIABLE_GETDATETIME
		{0, 1},							// ID_VARIABLE_GETVALUE
		{0, 0},							// ID_VARIABLE_PUTVALUE
		{0, 1},							// ID_VARIABLE_GETATTRIBUTE
		{0, 1},							// ID_VARIABLE_GETHELP
		{0, 1},							// ID_VARIABLE_GETNAME
		{0, 1},							// ID_VARIABLE_GETTAG
		{0, 0},							// ID_VARIABLE_PUTTAG
		{0, 1},							// ID_VARIABLE_GETID
		{0, 0},							// ID_VARIABLE_PUTID
		{0, 1},							// ID_VARIABLE_GETMICROSECOND
		{-1, 0},						// ID_VARIABLE_RELEASE
		{0, 0},							// ID_COMMAND_EXECUTE
		{0, 0},							// ID_COMMAND_CANCEL
		{0, 1},							// ID_COMMAND_GETTIMEOUT
		{0, 0},							// ID_COMMAND_PUTTIMEOUT
		{0, 1},							// ID_COMMAND_GETSTATE
		{0, 1},							// ID_COMMAND_GETPARAMETERS
		{0, 0},							// ID_COMMAND_PUTPARAMETERS
		{0, 1},							// ID_COMMAND_GETRESULT
		{0, 1},							// ID_COMMAND_GETATTRIBUTE
		{0, 1},							// ID_COMMAND_GETHELP
		{0, 1},							// ID_COMMAND_GETNAME
		{0, 1},							// ID_COMMAND_GETTAG
		{0, 1},							// ID_COMMAND_PUTTAG
		{0, 1},							// ID_COMMAND_GETID
		{0, 0},							// ID_COMMAND_PUTID
		{-1, 0},						// ID_COMMAND_RELEASE
		{0, 0},							// ID_MESSAGE_REPLY
		{0, 0},							// ID_MESSAGE_CLEAR
		{0, 1},							// ID_MESSAGE_GETDATETIME
		{0, 1},							// ID_MESSAGE_GETDESCRIPTION
		{0, 1},							// ID_MESSAGE_GETDESTINATION
		{0, 1},							// ID_MESSAGE_GETNUMBER
		{0, 1},							// ID_MESSAGE_GETSERIALNUMBER
		{0, 1},							// ID_MESSAGE_GETSOURCE
		{0, 1},							// ID_MESSAGE_GETVALUE
		{-1, 0},						// ID_MESSAGE_RELEASE
};

static THRET THTYPE recv_thread(void *arg);

/**
 * @fn			int find_open_address(int type)
 * @brief		Returns the open address of m_conn_param.
 * @param[in]	type Connection type.
 * @note		If there is no open space, then returns 0.
 */
static int find_open_address(int type)
{
	int i, start, end, index = -1;

	switch(type){
		case CONN_TCP:
			start = 0;
			end = BCAP_TCP_MAX;
			break;
		case CONN_UDP:
			start = BCAP_TCP_MAX;
			end = start + BCAP_UDP_MAX;
			break;
		case CONN_COM:
			start = BCAP_TCP_MAX + BCAP_UDP_MAX;
			end = start + BCAP_COM_MAX;
			break;
		default:
			return 0;
	}

	for(i = start; i < end; i++){
		if(m_conn_param[i].device.sock == 0){
			index = i;
			break;
		}
	}

	return (index + 1);
};

/**
 * @fn			struct CONN_BCAP_SERVER* check_address(int index)
 * @brief		Checks whether the index has been used or not.
 * @param[in]	index The index of m_conn_param.
 * @note		If the index has not been used then returns NULL.
 */
static struct CONN_BCAP_SERVER* check_address(int index)
{
	index--;

	if(index < 0 || _BCAP_SERVER_MAX <= index){
		return NULL;
	}else if(m_conn_param[index].device.sock == 0){
		return NULL;
	}

	return &m_conn_param[index];
};

/**
 * @fn			HRESULT push_vector(struct CONN_BCAP_SERVER *bcap_param, struct VEC_OBJECT *pObj)
 * @brief		Pushes the created object to the vector.
 * @param[in]	bcap_param The server object.
 * @param[in]	pObj The created object to push.
 */
static HRESULT push_vector(struct CONN_BCAP_SERVER *bcap_param, struct VEC_OBJECT *pObj)
{
	struct VEC_OBJECT *pPrev;

	if(bcap_param->num_object >= BCAP_OBJECT_MAX){
		return E_MAX_OBJECT;
	}

	/* Replaces the created object to top of vector */
	pPrev = bcap_param->stack;
	bcap_param->stack = pObj;

	/* Sets relation */
	pObj->prev = pPrev;
	if(pPrev != NULL){
		pPrev->next = pObj;
	}

	bcap_param->num_object++;

	return S_OK;
};

/**
 * @fn			HRESULT pop_vector(struct CONN_BCAP_SERVER *bcap_param, struct VEC_OBJECT **pObj, int index)
 * @brief		Pops the object where is in the indicated index from the vector.
 * @param[in]	bcap_param The server object.
 * @param[out]	pObj the poped object.
 * @param[in]	index The indicated index.
 */
static HRESULT pop_vector(struct CONN_BCAP_SERVER *bcap_param, struct VEC_OBJECT **pObj, int index)
{
	int i;
	struct VEC_OBJECT *pPrev, *pNext, *pCur = bcap_param->stack;
	HRESULT hr = E_HANDLE;

	for(i = 0; (i < bcap_param->num_object) && (pCur); i++){
		if(i == index){
			*pObj = pCur;

			/* Gets the object which is later than the poped one */
			pNext = pCur->next;
			pCur->next = NULL;

			/* Gets the object which is earlier than the poped one */
			pPrev = pCur->prev;
			pCur->prev = NULL;

			if(pNext == NULL){ // If the target object is the latest one
				bcap_param->stack = pPrev;
			}else{
				pNext->prev = pPrev;
			}

			if(pPrev != NULL){ // If the target object is not the earliest one
				pPrev->next = pNext;
			}

			bcap_param->num_object--;
			hr = S_OK;
			break;
		}

		pCur = pCur->prev;
	}

	return hr;
};

/**
 * @fn			int search_vector(struct CONN_BCAP_SERVER *bcap_param, int32_t id, uint32_t hObj)
 * @brief		Searchs the target object with function ID and object handle.
 * @param[in]	bcap_param The server object.
 * @param[in]	id Function ID.
 * @param[in]	hObj Object handle.
 * @note		If it cannot find the target object, then returns -1.
 */
static int search_vector(struct CONN_BCAP_SERVER *bcap_param, int32_t id, uint32_t hObj)
{
	int i, index = -1;
	struct VEC_OBJECT *pCur = bcap_param->stack;

	for(i = 0; (i < bcap_param->num_object) && (pCur); i++){
		if((pCur->id == id) && (pCur->hObj == hObj)){
			index = i;
			break;
		}

		pCur = pCur->prev;
	}

	return index;
};

/**
 * @fn				HRESULT change_relation(struct CONN_BCAP_SERVER *own, int mode, int *sock)
 * @brief			Changes thread's relationships for TCP connection.
 * @param[in,out]	own The connection parameter to be changed.
 * @param[in]		mode The change mode.
 * @param[in]		sock The client connection.
 */
static HRESULT change_relation(struct CONN_BCAP_SERVER *own, int mode, int *sock)
{
	HRESULT hr;
	struct CONN_BCAP_SERVER *node, *tmp;
	MUTEX *mutex;

	/* Checks mutex object */
	mutex = own->relation_mutex;
	if(mutex == NULL){
		return E_INVALIDARG;
	}

	/* Locks mutex and must not returns this function without end of one. */
	hr = lock_mutex(mutex, INFINITE);
	if(FAILED(hr)) return hr;

	switch(mode){
		case ADD_CHILD:
			/* Root node only */
			if(own->parent == NULL){
				/* Creates new node */
				node = (struct CONN_BCAP_SERVER *)malloc(sizeof(struct CONN_BCAP_SERVER));
				if(node == NULL){
					hr = E_OUTOFMEMORY;
					goto exit_proc;
				}

				/* Initializes node memory */
				memset(node, 0, sizeof(struct CONN_BCAP_SERVER));

				/* Copies device parameters from parent node */
				node->device = own->device;

				/* Sets child socket */
				node->device.sock = *sock;

				hr = node->device.dn_set_timeout(*sock, node->device.timeout);
				if(FAILED(hr)){
					node->device.dn_close(sock);
					free(node);
					goto exit_proc;
				}

				/* Creates terminal event for main thread */
				hr = create_event(&node->term_main_evt, 1, 0);
				if(FAILED(hr)){
					node->device.dn_close(sock);
					free(node);
					goto exit_proc;
				}

				/* Copies parameters */
				node->exec_timeout = own->exec_timeout;
				node->wdt_interval = own->wdt_interval;

				/* Copies mutex */
				node->relation_mutex = own->relation_mutex;

				/* Sets parent node */
				node->parent = own;
				own->num_child++;

				/* Replaces the head of children */
				tmp = own->node1;
				own->node1 = node;
				node->node2 = tmp;
				if(tmp != NULL) tmp->node1 = node;

				/* Begins child thread */
				begin_thread(&node->main_thread, &recv_thread, node);
			}
			break;

		case DELETE_CHILD:
			/* Root node only */
			if(own->parent == NULL){
				node = own->node2;
				while(node != NULL){
					tmp = node->node2;

					/* Ends child thread */
					set_event(&node->term_main_evt);
					exit_thread(node->main_thread);

					/* Destroys event */
					destroy_event(&node->term_main_evt);

					/* Closes connection */
					node->device.dn_close(&node->device.sock);

					free(node);
					node = tmp;

					own->num_child--;
				}
				own->node2 = NULL;
			}
			break;

		case DESTROY_SELF:
			/* Child node only */
			if(own->parent != NULL){
				/* Removes own node from the children list */
				tmp = own->node1;

				if(tmp == NULL){ // If own node is the youngest children
					own->parent->node1 = own->node2;
				}else{
					tmp->node2 = own->node2;
				}

				if(own->node2 != NULL){ // If own node is not the oldest children
					own->node2->node1 = tmp;
				}

				/* Adds own node to the parent's delete */
				tmp = own->parent->node2;
				own->parent->node2 = own;
				own->node2 = tmp;
				if(tmp != NULL) tmp->node1 = own;
			}
			break;

		default:
			hr = E_INVALIDARG;
			break;
	}

exit_proc:
	unlock_mutex(mutex);

	return hr;
};

/**
 * @fn			HRESULT bcap_callfunc(struct BCAP_PACKET *recv_packet, struct BCAP_PACKET *send_packet)
 * @brief		Execute a callback function with the received b-CAP command.
 * @param[in]	recv_packet The received b-CAP packet.
 * @param[in]	send_packet The b-CAP packet to be sent.
 */
static HRESULT bcap_callfunc(struct BCAP_PACKET *recv_packet, struct BCAP_PACKET *send_packet)
{
	int32_t id;
	CALL_FUNC_BCAP func;

	/* Initializes send packet */
	send_packet->serial = recv_packet->serial;
	send_packet->reserv = 0;
	send_packet->id = E_INVALIDARG;
	send_packet->argc = 0;

	id = recv_packet->id;
	if((0 < id) && (id < _FUNCTION_ID_MAX)){
		func = m_list_func[id];
		if(func == NULL){
			/* The function is not implemented */
			send_packet->id = E_NOTIMPL;
		}else{
			/* callback function */
			VariantClear(send_packet->args);
			send_packet->id = func(recv_packet->args, recv_packet->argc, send_packet->args);
			send_packet->argc = m_map_id[id].return_flag ? 1 : 0;
		}
	}

	return S_OK;
};

/**
 * @fn				HRESULT receive_execute(struct CONN_BCAP_SERVER *bcap_param)
 * @brief			Receives the b-CAP packet and executes callback functions.
 * @param[in,out]	bcap_param b-CAP communication object.
 */
static HRESULT receive_execute(struct CONN_BCAP_SERVER *bcap_param)
{
	int index;
	int32_t relation_id;
	uint16_t i, clear_flag = 1;
	uint32_t cur, start, diff;
	HRESULT hr;
	struct CONN_PARAM_COMMON *device = &bcap_param->device;
	struct BCAP_PACKET tmp_send_packet, tmp_recv_packet,
		*send_packet = &bcap_param->last_send,
		*recv_packet = &bcap_param->last_recv;
	struct VEC_OBJECT *pObj = NULL;
	BSTR bstrOpt = NULL;
	VARIANT vntTmp, vntOpt;

	VariantInit(&vntTmp);
	VariantInit(&vntOpt);

	/* Initializes temporary packet */
	tmp_recv_packet.argc = (uint16_t)-1;
	tmp_recv_packet.args = NULL;

	/* Receives b-CAP packet */
	hr = bcap_recv(device, &tmp_recv_packet, 0);

	if(SUCCEEDED(hr)){
		/* Sets S_EXECUTING packet */
		memset(&tmp_send_packet, 0, sizeof(struct BCAP_PACKET));
		tmp_send_packet.serial = tmp_recv_packet.serial;
		tmp_send_packet.id = S_EXECUTING;

		/* Checks retry packet */
		switch(device->type){
			case CONN_UDP:
			case CONN_COM:
				/* Sets retry count */
				tmp_recv_packet.reserv = (tmp_recv_packet.reserv == 0)
					? tmp_recv_packet.serial : tmp_recv_packet.reserv;

				/* If already responded, then does not execute */
				if(send_packet->serial == tmp_recv_packet.reserv){
					/* Copies last send packet */
					tmp_send_packet = *send_packet;

					/* Sets new serial number */
					tmp_send_packet.serial = tmp_recv_packet.serial;

					/* Sends temporary send packet */
					bcap_send(device, &tmp_send_packet);
					hr = S_FALSE;
					goto exit_proc;
				}

				break;
		}

		/* Checks execute thread */
		hr = wait_event(&bcap_param->comp_evt, 0);
		if(hr == E_TIMEOUT){
			/* Sends result busy process */
			tmp_send_packet.id = E_BUSY_PROC;
			bcap_send(device, &tmp_send_packet);
			goto exit_proc;
		}

		switch(tmp_recv_packet.id){
			case ID_SERVICE_START:
			case ID_CONTROLLER_CONNECT:
			case ID_CONTROLLER_GETEXTENSION:
			case ID_CONTROLLER_GETFILE:
			case ID_FILE_GETFILE:
			case ID_CONTROLLER_GETROBOT:
			case ID_CONTROLLER_GETTASK:
			case ID_CONTROLLER_GETVARIABLE:
			case ID_EXTENSION_GETVARIABLE:
			case ID_FILE_GETVARIABLE:
			case ID_ROBOT_GETVARIABLE:
			case ID_TASK_GETVARIABLE:
			case ID_CONTROLLER_GETCOMMAND:
			case ID_CONTROLLER_GETMESSAGE:
				if(bcap_param->num_object >= BCAP_OBJECT_MAX){
					tmp_send_packet.id = E_MAX_OBJECT;
					bcap_send(device, &tmp_send_packet);
					hr = S_FALSE;
					goto exit_proc;
				}

				if(tmp_recv_packet.id == ID_SERVICE_START){
					if((tmp_recv_packet.argc >= 1) && (tmp_recv_packet.args != NULL)){
						VariantCopy(&vntTmp, &tmp_recv_packet.args[0]);
						hr = VariantChangeType(&vntTmp, &vntTmp, 0, VT_BSTR);
						if(FAILED(hr)){
							tmp_send_packet.id = hr;
							bcap_send(device, &tmp_send_packet);
							hr = S_FALSE;
							goto exit_proc;
						}
					}else{
						vntTmp.vt = VT_BSTR;
						vntTmp.bstrVal = SysAllocString(L"");
					}

					bstrOpt = SysAllocString(L"WDT");
					hr = GetOptionValue(vntTmp.bstrVal, bstrOpt, VT_UI4, &vntOpt);
					vntOpt.ulVal = (vntOpt.vt == VT_UI4) ? vntOpt.ulVal : INIT_WDT_INTERVAL;
					if(vntOpt.ulVal < MIN_WDT_INTERVAL){
						tmp_send_packet.id = E_INVALIDARG;
						bcap_send(device, &tmp_send_packet);
						hr = S_FALSE;
						goto exit_proc;
					}else{
						bcap_param->wdt_interval = vntOpt.ulVal;
					}
					SysFreeString(bstrOpt);
					VariantClear(&vntOpt);

					bstrOpt = SysAllocString(L"InvokeTimeout");
					hr = GetOptionValue(vntTmp.bstrVal, bstrOpt, VT_UI4, &vntOpt);
					vntOpt.ulVal = (vntOpt.vt == VT_UI4) ? vntOpt.ulVal : INIT_EXEC_TIMEOUT;
					if(vntOpt.ulVal < MIN_WDT_INTERVAL){
						tmp_send_packet.id = E_INVALIDARG;
						bcap_send(device, &tmp_send_packet);
						hr = S_FALSE;
						goto exit_proc;
					}else{
						bcap_param->exec_timeout = vntOpt.ulVal;
					}
					SysFreeString(bstrOpt);
					VariantClear(&vntOpt);

					VariantClear(&vntTmp);
					bstrOpt = NULL;
				}

				break;
		}

		/* Resets last received packet */
		if(recv_packet->args != NULL){
			for(i = 0; i < recv_packet->argc; i++){
				VariantClear(&recv_packet->args[i]);
			}
			free(recv_packet->args);
		}

		/* Copies to last receive packet */
		clear_flag = 0;
		*recv_packet = tmp_recv_packet;

		/* Runs execute thread */
		reset_event(&bcap_param->comp_evt);
		set_event(&bcap_param->exec_evt);

		if(SUCCEEDED(hr)){
			start = gettimeofday_msec();
			while(1){
				hr = wait_event(&bcap_param->comp_evt, bcap_param->wdt_interval);
				if(SUCCEEDED(hr)){
					break;
				}else{
					/* Sends S_EXECUTING packet */
					hr = bcap_send(device, &tmp_send_packet);
					if(FAILED(hr)){
						break;
					}
				}

				/* Checks executing timeout */
				cur = gettimeofday_msec();
				diff = calc_time_diff(start, cur);
				if(diff > bcap_param->exec_timeout){
					hr = E_TIMEOUT;
					break;
				}
			}
		}
	}

exit_proc:
	if(hr == S_OK){
		if(bcap_param->last_send.id == S_OK){
			/* Changes the vector of created objects */
			relation_id = m_map_id[recv_packet->id].relation_id;
			if(relation_id > 0){ // Push
				pObj = (struct VEC_OBJECT *)malloc(sizeof(struct VEC_OBJECT));
				if(pObj != NULL){
					memset(pObj, 0, sizeof(struct VEC_OBJECT));
					pObj->id = relation_id;
					pObj->hObj = (recv_packet->id == ID_SERVICE_START)
							? 0 : send_packet->args[0].lVal;
					push_vector(bcap_param, pObj);
				}
			}else if(relation_id < 0){ // Pop
				index = search_vector(bcap_param, recv_packet->id,
						(recv_packet->id == ID_SERVICE_STOP) ? 0 : recv_packet->args[0].lVal);
				if(index >= 0){
					pop_vector(bcap_param, &pObj, index);
					free(pObj);
				}
			}
		}

		/* Responds the result message */
		hr = bcap_send(device, &bcap_param->last_send);
	}

	/* Clears temporary packet */
	if(clear_flag){
		if(tmp_recv_packet.args != NULL){
			for(i = 0; i < tmp_recv_packet.argc; i++){
				VariantClear(&tmp_recv_packet.args[i]);
			}
			free(tmp_recv_packet.args);
		}
	}

	VariantClear(&vntTmp);
	VariantClear(&vntOpt);
	if(bstrOpt){
		SysFreeString(bstrOpt);
	}

	return hr;
};

/**
 * @fn			THRET exec_thread(void *arg)
 * @brief		The executing thread.
 * @param[in]	arg The argument of executing thread: CONN_BCAP_SERVER.
 */
static THRET THTYPE exec_thread(void *arg)
{
#if !defined(THRET)
	THRET ret = (THRET)NULL;
#endif

	HRESULT hr;
	struct CONN_BCAP_SERVER *bcap_param = (struct CONN_BCAP_SERVER *)arg;
	EVENT *evt[2] = {&bcap_param->exec_evt, &bcap_param->term_sub_evt};

	while(1){
		hr = wait_event_multi(evt, 2, INFINITE, 0);
		if(hr == WAIT_OBJECT_0 + 1){
			break;
		}

		bcap_callfunc(&bcap_param->last_recv, &bcap_param->last_send);

		set_event(&bcap_param->comp_evt);
	}

#if !defined(THRET)
	return ret;
#endif
};

/**
 * @fn			THRET recv_thread(void *arg)
 * @brief		The receiving thread.
 * @param[in]	arg The argument of receiving thread: CONN_BCAP_SERVER.
 */
static THRET THTYPE recv_thread(void *arg)
{
#if !defined(THRET)
	THRET ret = (THRET)NULL;
#endif

	uint16_t i;
	HRESULT hr;
	struct CONN_BCAP_SERVER *bcap_param = (struct CONN_BCAP_SERVER *)arg;
	struct BCAP_PACKET *send_packet = &bcap_param->last_send,
						*recv_packet = &bcap_param->last_recv;
	struct VEC_OBJECT *pObj = NULL;
	VARIANT vnt_send, vntTmp;
	CALL_FUNC_BCAP func;

	/* Sets last send packet */
	send_packet->args = &vnt_send;
	VariantInit(&vnt_send);

	/* Creates terminal event for sub thread */
	hr = create_event(&bcap_param->term_sub_evt, 1, 0);
	if(FAILED(hr)) goto exit_proc;

	/* Creates events for synchronizing threads */
	hr = create_event(&bcap_param->exec_evt, 0, 0);
	if(FAILED(hr)) goto exit_proc;

	hr = create_event(&bcap_param->comp_evt, 1, 1);
	if(FAILED(hr)) goto exit_proc;

	/* Begins sub thread */
	begin_thread(&bcap_param->sub_thread, &exec_thread, arg);

	while(1){
		hr = wait_event(&bcap_param->term_main_evt, 0);
		if(SUCCEEDED(hr)){
			break;
		}

		hr = receive_execute(bcap_param);
		if(FAILED(hr) && hr != E_TIMEOUT){
			break;
		}
	}

exit_proc:
	/* Ends sub thread */
	set_event(&bcap_param->term_sub_evt);
	exit_thread(bcap_param->sub_thread);

	/* Destroys events */
	destroy_event(&bcap_param->term_sub_evt);
	destroy_event(&bcap_param->exec_evt);
	destroy_event(&bcap_param->comp_evt);

	/* Clears last send packet */
	VariantClear(&vnt_send);

	/* Clears last received packet */
	if(recv_packet->args != NULL){
		for(i = 0; i < recv_packet->argc; i++){
			VariantClear(&recv_packet->args[i]);
		}
		free(recv_packet->args);
	}

	/* Release all of vector elements */
	VariantInit(&vntTmp);
	vntTmp.vt = VT_I4;

	while(1){
		hr = pop_vector(bcap_param, &pObj, 0);
		if(FAILED(hr)) break;

		func = m_list_func[pObj->id];
		if(func != NULL){
			vntTmp.lVal = pObj->hObj;
			func(&vntTmp, ((pObj->id == ID_SERVICE_STOP) ? 0 : 1), &vnt_send);
			VariantClear(&vnt_send);
		}

		free(pObj);
	}

	VariantClear(&vntTmp);

	/* Destroys self */
	change_relation(bcap_param, DESTROY_SELF, NULL);

#if !defined(THRET)
	return ret;
#endif
};

/**
 * @fn			THRET accept_thread(void *arg)
 * @brief		The accepting thread for TCP connection.
 * @param[in]	arg The argument of accepting thread: CONN_BCAP_SERVER.
 */
static THRET THTYPE accept_thread(void *arg)
{
#if !defined(THRET)
	THRET ret = (THRET)NULL;
#endif

	int client;
	HRESULT hr;
	volatile struct CONN_BCAP_SERVER *child;
	struct CONN_BCAP_SERVER *bcap_param = (struct CONN_BCAP_SERVER *)arg;
	MUTEX mutex;

	/* Initializes mutex */
	bcap_param->relation_mutex = &mutex;
	hr = initialize_mutex(&mutex);
	if(FAILED(hr)) goto exit_proc;

	while(1){
		hr = wait_event(&bcap_param->term_main_evt, 300);
		if(SUCCEEDED(hr)){
			break;
		}

		if(bcap_param->num_child < BCAP_CLIENT_MAX){
			hr = tcp_accept(bcap_param->device.sock, &client);
			if(SUCCEEDED(hr)){
				/* Sets no delay option */
				tcp_set_nodelay(client, 1);

				/* Sets keep alive option */
				tcp_set_keepalive(client, KEEPALIVE_ENABLE,
						KEEPALIVE_IDLE, KEEPALIVE_INTERVAL, KEEPALIVE_COUNT);

				/* Adds child */
				change_relation(bcap_param, ADD_CHILD, &client);
			}
		}

		/* Deletes child */
		change_relation(bcap_param, DELETE_CHILD, NULL);
	}

exit_proc:
	/* Ends all children thread */
	child = bcap_param->node1;
	while(child != NULL){
		set_event((EVENT *)&child->term_main_evt);
		exit_thread(child->main_thread);

		child = bcap_param->node1;
	}

	/* Deletes child */
	change_relation(bcap_param, DELETE_CHILD, NULL);

	/* Releases mutex */
	release_mutex(&mutex);

#if !defined(THRET)
	return ret;
#endif
};

HRESULT bCap_SetCallFunc(int32_t id, CALL_FUNC_BCAP func)
{
	if((id <= 0) || (_FUNCTION_ID_MAX <= id)){
		return E_INVALIDARG;
	}

	m_list_func[id] = func;

	return S_OK;
}

HRESULT bCap_Open_Server(const char *connect, uint32_t timeout, int *pfd)
{
	int type, index, *sock;
	HRESULT hr;
	void *conn_param;
	struct CONN_PARAM_ETH eth_param = {0, 0, htonl(INADDR_ANY), htons(5007)};
	struct CONN_PARAM_COM com_param = {1, 38400, NOPARITY, 8, ONESTOPBIT, 0};
	struct CONN_BCAP_SERVER *bcap_param;
	struct CONN_PARAM_COMMON *device;
	struct sockaddr_in *paddr;

	if(connect == NULL || pfd == NULL) return E_INVALIDARG;

	type = parse_conn_type(connect);

	index = find_open_address(type);
	if(index == 0) return E_MAX_OBJECT;

	bcap_param = &m_conn_param[index - 1];
	device = &bcap_param->device;

	/* Initializes connection parameters */
	device->type = type;
	switch(device->type){
		case CONN_TCP:
			hr = parse_conn_param_ether(connect, &eth_param);
			conn_param = &eth_param;
			device->arg = NULL;
			device->dn_open = &tcp_open_server;
			device->dn_close = &tcp_close;
			device->dn_send = &tcp_send;
			device->dn_recv = &tcp_recv;
			device->dn_set_timeout = &tcp_set_timeout;
			break;
		case CONN_UDP:
			hr = parse_conn_param_ether(connect, &eth_param);
			conn_param = &eth_param;
			paddr = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
			if(paddr == NULL){
				hr = E_OUTOFMEMORY;
				break;
			}
			paddr->sin_addr.s_addr = eth_param.dst_addr;
			paddr->sin_port = eth_param.dst_port;
			paddr->sin_family = AF_INET;
			device->arg = (void *)paddr;
			device->dn_open = &udp_open;
			device->dn_close = &udp_close;
			device->dn_send = &udp_send;
			device->dn_recv = &udp_recv;
			device->dn_set_timeout = &udp_set_timeout;
			break;
		case CONN_COM:
			hr = parse_conn_param_serial(connect, &com_param);
			conn_param = &com_param;
			device->arg = NULL;
			device->dn_open = &com_open;
			device->dn_close = &com_close;
			device->dn_send = &com_send;
			device->dn_recv = &com_recv;
			device->dn_set_timeout = &com_set_timeout;
			break;
		default:
			hr = E_INVALIDARG;
			break;
	}

	if(FAILED(hr)){
		if(device->arg != NULL){ free(device->arg); device->arg = NULL;	}
		memset(bcap_param, 0, sizeof(struct CONN_BCAP_SERVER));
		return hr;
	}

	/* Create terminal event for main thread */
	hr = create_event(&bcap_param->term_main_evt, 1, 0);
	if(FAILED(hr)){
		if(device->arg != NULL){ free(device->arg); device->arg = NULL;	}
		memset(bcap_param, 0, sizeof(struct CONN_BCAP_SERVER));
		return hr;
	}

	/* Opens connection */
	sock = &device->sock;
	hr = device->dn_open(conn_param, sock);
	if(FAILED(hr)){
		destroy_event(&bcap_param->term_main_evt);
		if(device->arg != NULL){ free(device->arg); device->arg = NULL;	}
		memset(bcap_param, 0, sizeof(struct CONN_BCAP_SERVER));
		return hr;
	}

	hr = device->dn_set_timeout(*sock, timeout);
	if(FAILED(hr)){
		bCap_Close_Server(&index);
		return hr;
	}

	/* Sets parameters */
	device->timeout = timeout;
	bcap_param->exec_timeout = INIT_EXEC_TIMEOUT;
	bcap_param->wdt_interval = INIT_WDT_INTERVAL;

	/* Begins main thread */
	if(device->type == CONN_TCP){
		begin_thread(&bcap_param->main_thread, &accept_thread, bcap_param);
	}else{
		begin_thread(&bcap_param->main_thread, &recv_thread, bcap_param);
	}

	*pfd = index;

	return S_OK;
}

HRESULT bCap_Close_Server(int *pfd)
{
	int index, *sock;
	struct CONN_BCAP_SERVER *bcap_param;
	struct CONN_PARAM_COMMON *device;

	if(pfd == NULL) return E_HANDLE;

	index = *pfd;

	bcap_param = check_address(index);
	if(bcap_param == NULL) return E_HANDLE;

	device = &bcap_param->device;
	sock = &device->sock;

	/* Ends main thread */
	set_event(&bcap_param->term_main_evt);
	exit_thread(bcap_param->main_thread);

	/* Destroys event */
	destroy_event(&bcap_param->term_main_evt);

	/* Closes connection */
	device->dn_close(sock);

	/* Releases argument */
	if(device->arg != NULL){
		free(device->arg);
		device->arg = NULL;
	}

	/* Resets connection parameters */
	memset(bcap_param, 0, sizeof(struct CONN_BCAP_SERVER));

	*pfd = 0;

	return S_OK;
}
