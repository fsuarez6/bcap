// Always goes first
#define _CUSTOM_UINT64
#include "stdint.h"
// ROS
#include <ros/ros.h>
// bCAP (Always last)
#include "bcap_client.h"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "change_digital_output");
  ros::NodeHandle nh_;
  int i, fd;
  long *plData;
  uint32_t hCtrl, hVar;
  double *pdData, dAng[8];
  BSTR bstr1, bstr2, bstr3, bstr4;
  VARIANT vntParam, vntRet;
  HRESULT hr;

  // Open connection
  hr = bCap_Open_Client("udp:192.168.0.21:5007", 1000, 3, &fd);

  if(SUCCEEDED(hr))
  {
    // Send SERVICE_START packet
    bstr1 = SysAllocString(L",WDT=400");
    bCap_ServiceStart(fd, NULL);
    SysFreeString(bstr1);
    
    // Connect to RC8
    bstr1 = SysAllocString(L"DigitalIO");               // Name
    bstr2 = SysAllocString(L"CaoProv.DENSO.VRC"); // Provider
    bstr3 = SysAllocString(L"localhost");         // Machine
    bstr4 = SysAllocString(L"");                  // Options
    hr = bCap_ControllerConnect(fd, bstr1, bstr2, bstr3, bstr4, &hCtrl);
    SysFreeString(bstr1);
    SysFreeString(bstr2);
    SysFreeString(bstr3);
    SysFreeString(bstr4);
    
    if(SUCCEEDED(hr))
    {
      // Get IO handle
      bstr1 = SysAllocString(L"IO24"); // Name
      bstr2 = SysAllocString(L"");      // Option
      hr = bCap_ControllerGetVariable(fd, hCtrl, bstr1, bstr1, &hVar);
      SysFreeString(bstr1);
      SysFreeString(bstr2);
      // Get IO Value
      bCap_VariableGetValue(fd, hVar, &vntRet);
      // Put IO Value
      VariantInit(&vntParam);
      vntParam.vt = VT_I4;
      vntParam.lVal = (vntRet.lVal == 0) ? 1 : 0;
      bCap_VariablePutValue(fd, hVar, vntParam);
      VariantClear(&vntParam);
      VariantClear(&vntRet);
      // Release IO handle
      bCap_VariableRelease(fd, &hVar);
      // Disconnect RC8
      bCap_ControllerDisconnect(fd, &hCtrl);
    }
    // Send SERVICE_STOP packet
    bCap_ServiceStop(fd);
    // Close connection
    bCap_Close_Client(&fd);
  }

  return 0;
};
