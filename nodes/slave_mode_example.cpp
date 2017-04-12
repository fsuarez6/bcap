// ROS
#include <ros/ros.h>
// bCAP
#include "bcap/stdint.h"
#include "bcap/bcap_client.h"

#define E_BUF_FULL  (0x83201483)

namespace SlaveMode
{
  enum Modes
  { 
    P0 = 0x001,
    J0 = 0x002,
    T0 = 0x003,
    P1 = 0x101,
    J1 = 0x102,
    T1 = 0x103,
    P2 = 0x201,
    J2 = 0x202,
    T2 = 0x203
  };
  
  enum Formats
  { 
    None          = 0x0000,
    HandIO        = 0x0020,
    Electric      = 0x0040,
    MiniIO        = 0x0100,
    MiniAndHandIO = 0x0120,
    UserIO        = 0x0200,
    UserAndHandIO = 0x0220,
    Ptype         = 0x0001,
    Jtype         = 0x0002,
    Ttype         = 0x0003,
    PJtypes       = 0x0004,
    TJtypes       = 0x0005,
    Timestamp     = 0x0010
  };
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "slave_mode");
  ros::NodeHandle nh, nh_private("~");
  int i, fd;
  long *plData;
  uint32_t hCtrl, hRob;
  double *pdData, dAng[8];
  BSTR bstr1, bstr2, bstr3, bstr4;
  VARIANT vntParam, vntRet, *pvntData;
  HRESULT hr;

  std::string ip_address;
  nh_private.param("ip", ip_address, std::string("192.168.0.11"));
  if (!nh_private.hasParam("ip"))
    ROS_WARN_STREAM("Parameter [~ip] not found, using default: " << ip_address);
  
  // Open connection
  std::string connect;
  connect = "udp:" + ip_address + ":5007";
  hr = bCap_Open_Client(connect.c_str(), 1000, 3, &fd);

  if(SUCCEEDED(hr)){
    // Send SERVICE_START packet
    bstr1 = SysAllocString(L",WDT=400");
    bCap_ServiceStart(fd, NULL);
    SysFreeString(bstr1);
    
    // Connect to RC8
    bstr1 = SysAllocString(L"RC8");               // Name
    bstr2 = SysAllocString(L"CaoProv.DENSO.VRC"); // Provider
    bstr3 = SysAllocString(L"localhost");         // Machine
    bstr4 = SysAllocString(L"");                  // Options
    hr = bCap_ControllerConnect(fd, bstr1, bstr2, bstr3, bstr4, &hCtrl);
    SysFreeString(bstr1);
    SysFreeString(bstr2);
    SysFreeString(bstr3);
    SysFreeString(bstr4);
    
    // Clear previous errors
    bstr1 = SysAllocString(L"ClearError");
    vntParam.bstrVal = SysAllocString(L"");
    vntParam.vt = VT_BSTR;
    hr = bCap_ControllerExecute(fd, hCtrl, bstr1, vntParam, &vntRet);
    if (FAILED(hr))
      ROS_ERROR("[ClearError] command failed");
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    
    if(SUCCEEDED(hr)){
      
      // Get robot handle
      bstr1 = SysAllocString(L"Robot"); // Name
      bstr2 = SysAllocString(L"");    // Option
      hr = bCap_ControllerGetRobot(fd, hCtrl, bstr1, bstr2, &hRob);
      SysFreeString(bstr1);
      SysFreeString(bstr2);

      if(SUCCEEDED(hr)){
        // Get arm control authority
        bstr1 = SysAllocString(L"Takearm");
        vntParam.bstrVal = SysAllocString(L"");
        vntParam.vt = VT_BSTR;
        hr = bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
        if (FAILED(hr))
          ROS_ERROR("[Takearm] command failed");
        SysFreeString(bstr1);
        VariantClear(&vntParam);
        VariantClear(&vntRet);

        if(SUCCEEDED(hr)){
          // Motor ON
          bstr1 = SysAllocString(L"Motor");
          vntParam.bstrVal = SysAllocString(L"1");
          vntParam.vt = VT_BSTR;
          hr = bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
          if FAILED(hr)
            ROS_ERROR("[Motor] command failed");;
          SysFreeString(bstr1);
          VariantClear(&vntParam);
          VariantClear(&vntRet);

          // Get current angles
          bstr1 = SysAllocString(L"CurJnt");
          vntParam.bstrVal = SysAllocString(L"");
          vntParam.vt = VT_BSTR;
          hr = bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
          if (FAILED(hr))
            ROS_ERROR("[CurJnt] command failed");
          SafeArrayAccessData(vntRet.parray, (void **)&pdData);
          memcpy(dAng, pdData, 8 * sizeof(double));
          SafeArrayUnaccessData(vntRet.parray);
          SysFreeString(bstr1);
          VariantClear(&vntParam);
          VariantClear(&vntRet);
          
          // Change slvSendFormat
          bstr1 = SysAllocString(L"slvSendFormat");
          vntParam.vt = VT_I4;
          long slvsend_format = SlaveMode::MiniIO;
          vntParam.lVal = slvsend_format;
          hr = bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
          if (FAILED(hr))
            ROS_ERROR("[slvSendFormat] command failed");
          SysFreeString(bstr1);
          VariantClear(&vntParam);
          VariantClear(&vntRet);
          
          // Change slvRecvFormat
          bstr1 = SysAllocString(L"slvRecvFormat");
          vntParam.vt = VT_I4;
          vntParam.lVal = SlaveMode::Jtype | SlaveMode::MiniIO;
          hr = bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
          if (FAILED(hr))
            ROS_ERROR("[slvRecvFormat] command failed");
          SysFreeString(bstr1);
          VariantClear(&vntParam);
          VariantClear(&vntRet);
          
          // Start slave mode
          bstr1 = SysAllocString(L"slvChangeMode");
          vntParam.vt = VT_I4;
          vntParam.lVal = SlaveMode::J1;
          hr = bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
          if (FAILED(hr))
            ROS_ERROR("[slvChangeMode] command failed");
          SysFreeString(bstr1);
          VariantClear(&vntParam);
          VariantClear(&vntRet);

          // Send commands
          bstr1 = SysAllocString(L"slvMove");
          /* VT_VARIANT | VT_ARRAY */
          vntParam.vt = (VT_VARIANT | VT_ARRAY);
          vntParam.parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);

          /* [0] VT_R8 | VT_ARRAY */
          SafeArrayAccessData(vntParam.parray, (void**)&pvntData);
          pvntData[0].vt = (VT_R8 | VT_ARRAY);
          pvntData[0].parray = SafeArrayCreateVector(VT_R8, 0, 8);
          SafeArrayAccessData(pvntData[0].parray, (void**)&pdData);
          for(i = 0; i < 8; i++) {
            pdData[i] = dAng[i];
          }
          SafeArrayUnaccessData(pvntData[0].parray);

          /* [1] VT_I4 */
          pvntData[1].vt = VT_I4;
          pvntData[1].lVal = 0x1000000;

          SafeArrayUnaccessData(vntParam.parray);
          
          ros::Rate rate(125);
          while (ros::ok())
          {
            if (slvsend_format == SlaveMode::MiniIO) {
              hr = bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
              VariantClear(&vntRet);
            }
            // Sleep
            rate.sleep();
          }
          
          VariantClear(&vntParam);
          // Stop slave mode
          bstr1 = SysAllocString(L"slvChangeMode");
          vntParam.bstrVal = SysAllocString(L"0");
          vntParam.vt = VT_BSTR;
          bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
          SysFreeString(bstr1);
          VariantClear(&vntParam);
          VariantClear(&vntRet);
          
          // Motor off
          bstr1 = SysAllocString(L"Motor");
          vntParam.bstrVal = SysAllocString(L"0");
          vntParam.vt = VT_BSTR;
          bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
          SysFreeString(bstr1);
          VariantClear(&vntParam);
          VariantClear(&vntRet);
          
          // Release arm control authority
          bstr1 = SysAllocString(L"Givearm");
          vntParam.bstrVal = SysAllocString(L"");
          vntParam.vt = VT_BSTR;
          bCap_RobotExecute(fd, hRob, bstr1, vntParam, &vntRet);
          SysFreeString(bstr1);
          VariantClear(&vntParam);
          VariantClear(&vntRet);
        }
        // Release robot handle
        bCap_RobotRelease(fd, &hRob);
      }
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
