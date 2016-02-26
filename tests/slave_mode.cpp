// Always goes first
#define _CUSTOM_UINT64
#include "stdint.h"
// ROS
#include <ros/ros.h>
// bCAP (Always last)
#include "bcap_client.h"


#define PERIOD		(100)	/* Period Cycle */
#define AMPLITUDE	(10)	/* Amplitude */

#define E_BUF_FULL	(0x83201483)

int main(void)
{
	int i, fd;
	long *plData;
	uint32_t hCtrl, hRob;
	double *pdData, dAng[8];
	BSTR bstr1, bstr2, bstr3, bstr4;
	VARIANT vntParam, vntRet;
	HRESULT hr;

	/* Open connection */
	hr = bCap_Open_Client("tcp:192.168.0.1:5007", 1000, 3, &fd);

	if(SUCCEEDED(hr)){
		bstr1 = SysAllocString(L",WDT=400");

		/* Send SERVICE_START packet */
		bCap_ServiceStart(fd, NULL);

		SysFreeString(bstr1);

		bstr1 = SysAllocString(L"RC8");					// Name
		bstr2 = SysAllocString(L"CaoProv.DENSO.VRC");	// Provider
		bstr3 = SysAllocString(L"localhost");			// Machine
		bstr4 = SysAllocString(L"");					// Option

		/* Connect RC8 */
		hr = bCap_ControllerConnect(fd, bstr1, bstr2, bstr3, bstr4, &hCtrl);

		SysFreeString(bstr1);
		SysFreeString(bstr2);
		SysFreeString(bstr3);
		SysFreeString(bstr4);

		if(SUCCEEDED(hr)){
			bstr1 = SysAllocString(L"Robot");	// Name
			bstr2 = SysAllocString(L"");		// Option

			/* Get robot handle */
			hr = bCap_ControllerGetRobot(fd, hCtrl, bstr1, bstr2, &hRob);

			SysFreeString(bstr1);
			SysFreeString(bstr2);

			if(SUCCEEDED(hr)){
				bstr1 = SysAllocString(L"Takearm");

				VariantInit(&vntParam);
				vntParam.vt = (VT_I4 | VT_ARRAY);
				vntParam.parray = SafeArrayCreateVector(VT_I4, 0, 2);
				SafeArrayAccessData(vntParam.parray, (void **)&plData);
				plData[0] = 0; plData[1] = 1;
				SafeArrayUnaccessData(vntParam.parray);

				/* Get arm control authority */
				hr = bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);

				SysFreeString(bstr1);
				
				VariantClear(&vntParam);
				VariantClear(&vntRet);

				if(SUCCEEDED(hr)){
					bstr1 = SysAllocString(L"Motor");

					VariantInit(&vntParam);
					vntParam.vt = VT_I4;
					vntParam.lVal = 1;

					/* Motor on */
					bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);

					SysFreeString(bstr1);
				
					VariantClear(&vntParam);
					VariantClear(&vntRet);

					VariantInit(&vntParam);
					vntParam.vt = VT_BSTR;
					vntParam.bstrVal = SysAllocString(L"@E J1");

					/* Move to first pose */
					bCap_RobotMove(fd, hRob, 1, vntParam, NULL);

					VariantClear(&vntParam);

					bstr1 = SysAllocString(L"CurJnt");

					VariantInit(&vntParam);
					vntParam.vt = VT_EMPTY;

					/* Get current angle */
					bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);

					SafeArrayAccessData(vntRet.parray, (void **)&pdData);
					memcpy(dAng, pdData, 8 * sizeof(double));
					SafeArrayUnaccessData(vntRet.parray);

					SysFreeString(bstr1);

					VariantClear(&vntParam);
					VariantClear(&vntRet);

					bstr1 = SysAllocString(L"slvChangeMode");

					VariantInit(&vntParam);
					vntParam.vt = VT_I4;
					vntParam.lVal = 2;

					/* Start slave mode (Mode 0, J Type) */
					bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);

					SysFreeString(bstr1);

					VariantClear(&vntParam);
					VariantClear(&vntRet);

					bstr1 = SysAllocString(L"slvMove");

					VariantInit(&vntParam);
					vntParam.vt = (VT_R8 | VT_ARRAY);
					vntParam.parray = SafeArrayCreateVector(VT_R8, 0, 8);

					for(i = 0; i < PERIOD; i++){
						SafeArrayAccessData(vntParam.parray, (void **)&pdData);
						memcpy(pdData, dAng, 8 * sizeof(double));
						pdData[0] = dAng[0] + i / 10.0;
						pdData[1] = dAng[1] + AMPLITUDE * sin(2*M_PI*i/PERIOD);
						SafeArrayUnaccessData(vntParam.parray);

						hr = bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
						VariantClear(&vntRet);

						/* if return code is not S_OK, then keep the message sending process waiting for 8 msec */
						if(hr != S_OK){
							usleep(8000);

							/* if return code is E_BUF_FULL, then retry previous packet */
							if(FAILED(hr)){
								if(hr == E_BUF_FULL){
									i--;
								}else{
									break;
								}
							}
						}
					}

					/* Stop robot */
					bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);

					SysFreeString(bstr1);

					VariantClear(&vntParam);
					VariantClear(&vntRet);

					bstr1 = SysAllocString(L"slvChangeMode");

					VariantInit(&vntParam);
					vntParam.vt = VT_I4;
					vntParam.lVal = 0;

					/* Stop slave mode */
					bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);

					SysFreeString(bstr1);

					VariantClear(&vntParam);
					VariantClear(&vntRet);

					bstr1 = SysAllocString(L"Motor");

					VariantInit(&vntParam);
					vntParam.vt = VT_I4;
					vntParam.lVal = 0;

					/* Motor off */
					bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);

					SysFreeString(bstr1);
				
					VariantClear(&vntParam);
					VariantClear(&vntRet);

					bstr1 = SysAllocString(L"Givearm");

					VariantInit(&vntParam);
					vntParam.vt = VT_EMPTY;

					/* Release arm control authority */
					bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);

					SysFreeString(bstr1);
				
					VariantClear(&vntParam);
					VariantClear(&vntRet);
				}

				/* Release robot handle */
				bCap_RobotRelease(fd, &hRob);
			}

			/* Disconnect RC8 */
			bCap_ControllerDisconnect(fd, &hCtrl);
		}

		/* Send SERVICE_STOP packet */
		bCap_ServiceStop(fd);

		/* Close connection */
		bCap_Close_Client(&fd);
	}

	return 0;
};
