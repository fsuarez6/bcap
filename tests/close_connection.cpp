// Always goes first
#define _CUSTOM_UINT64
#include "bcap/stdint.h"
// ROS
#include <ros/ros.h>
// bCAP (Always last)
#include "bcap/bcap_client.h"

#define E_BUF_FULL  (0x83201483)

int main(int argc, char **argv)
{
  ros::init (argc, argv, "close_connection");
  ros::NodeHandle nh, nh_private("~");
  
  HRESULT hr;
  int socket_, alternative_handle;
  uint32_t controller_handle_;
  
  BSTR bstr1, bstr2, bstr3, bstr4;
  
  // Read parameters
  nh_private.param("controller", alternative_handle, 2);
  if (!nh_private.hasParam("controller"))
    ROS_WARN_STREAM("Parameter [controller] not found, using default: " << alternative_handle);
  
  // Open connection
  hr = bCap_Open_Client("udp:192.168.0.21:5007", 1000, 3, &socket_);
  if(SUCCEEDED(hr)){
    // Start b-CAP service
    bCap_ServiceStart(socket_, NULL);
    
    // Connect to RC8
    bstr1 = SysAllocString(L"RC8");               // Name
    bstr2 = SysAllocString(L"CaoProv.DENSO.VRC"); // Provider
    bstr3 = SysAllocString(L"localhost");         // Machine
    bstr4 = SysAllocString(L"");                  // Options
    hr = bCap_ControllerConnect(socket_, bstr1, bstr2, bstr3, bstr4, &controller_handle_);
    SysFreeString(bstr1);
    SysFreeString(bstr2);
    SysFreeString(bstr3);
    SysFreeString(bstr4);
    
    if FAILED(hr)
      controller_handle_ = alternative_handle;
    ros::Duration(0.5).sleep();
    
    // Disconnect RC8
    bCap_ControllerDisconnect(socket_, &controller_handle_);
    // Send SERVICE_STOP packet
    bCap_ServiceStop(socket_);
    // Close connection
    bCap_Close_Client(&socket_);
    ROS_INFO("Succeeded to perform an Open-Close operation");
  }
  else
    ROS_ERROR("Failed to perform an Open-Close operation");
  return 0;
};
