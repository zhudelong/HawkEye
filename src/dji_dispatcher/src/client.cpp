#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dji_dispatcher/TaskDispatcher.h>
#include <dji_dispatcher/TaskState.h>
#include <dji_sdk/dji_sdk.h>

using namespace std;

// flags that control the task flows
static bool flag_start_mission    = false;
static bool flag_abort_mission    = false;
static bool flag_abort_landing    = false;
static bool flag_search_task_end  = false;
static bool flag_auto_landing_end = false;
static bool flag_detect_all = false;
static bool flag_limitation_ok = false;
static bool flag_go_back = false;
static int  abort_landing_count = 0;
static int  status_code = 0;
static int count_id = 0;


// listen to mobile client's order
unsigned char data[10];
#define     def_start_misson  (data[0]=='s' && data[1]=='m')
#define     def_abort_misson  (data[0]=='a' && data[1]=='m')
#define     def_abort_landing (data[0]=='a' && data[1]=='l')
#define     def_take_picture  (data[0]=='t' && data[1]=='p')
#define     def_take_video    (data[0]=='t' && data[1]=='v')
#define     def_stop_video    (data[0]=='s' && data[1]=='v')


ros::ServiceClient send_data_to_remote_device_service;
ros::ServiceClient sdk_permission_control_service;
ros::ServiceClient activation_service;
ros::Subscriber task_state_received_subscriber;
ros::Subscriber data_received_from_remote_device_subscriber;



void pub_task_steps(ros::Publisher& pub_)
{
    dji_dispatcher::TaskDispatcher data;
    data.flag_start_mission  = flag_start_mission;
    data.flag_abort_mission  = flag_abort_mission;
    data.flag_abort_landing  = flag_abort_landing;
    data.abort_landing_count = abort_landing_count;
    data.flag_search_task_end  = flag_search_task_end;
    data.flag_auto_landing_end = flag_auto_landing_end;
    data.flag_detect_all = flag_detect_all;
    data.flag_limitation_ok = flag_limitation_ok;
    data.flag_go_back = flag_go_back;
    data.status_code = status_code;
    pub_.publish(data);
}

bool send_to_mobile(std::string& in)
{
    dji_sdk::SendDataToRemoteDevice data_to_mobile;
    data_to_mobile.request.data.assign(in.begin(),in.end());
    //ROS_INFO(i);
    return send_data_to_remote_device_service.call(data_to_mobile) && data_to_mobile.response.result;
}

void DataFromMobileCallback(const dji_sdk::TransparentTransmissionData& data_from_mobile_para)
{
    for(int i=0; i<2; i++)
    {
        data[i] = data_from_mobile_para.data[i];
    }
    data[2] = '\0';
    if(data[0] == 's' && data[1] == 'm'){ flag_start_mission = true; }
    if(data[0] == 'a' && data[1] == 'm'){ flag_abort_mission = true; }
    if(data[0] == 'a' && data[1] == 'l'){ flag_abort_landing = true; abort_landing_count++;}
    if(data[0] == 'o' && data[1] == 'k'){ flag_detect_all = true;} // all the tag have been detected
    if(data[0] == 'n' && data[1] == 't'){ flag_limitation_ok = true;} // assigned time has used up
    if(data[0] == 'f' && data[1] == 's'){ flag_go_back = true;} // < 2m force search

    if(data[0] == '#'){// go to target position
        string str = "we are going to target point!";
        send_to_mobile(str);
    }

}

void task_state_from_searcher(const dji_dispatcher::TaskState&  state)
{
    if (state.is_finished == true)
    {
        flag_search_task_end = true;
    }

    if(state.is_finished == false)
    {
        flag_search_task_end = false;
    }
}

bool activate()
{
    dji_sdk::Activation activate;
    return activation_service.call(activate) && activate.response.result;
}

bool sdk_permission_control(unsigned char request)
{
    dji_sdk::SDKPermissionControl sdk_permission_control;
    sdk_permission_control.request.control_enable = request;
    return sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("This is task dispatcher!");
    ros::NodeHandle nh;
    ros::Publisher global_dispatcher_pub_= nh.advertise<dji_dispatcher::TaskDispatcher>("/task_dispatcher",10);
    task_state_received_subscriber =  nh.subscribe("dji_auto_search/auto_search_state",10, &task_state_from_searcher);
    data_received_from_remote_device_subscriber = nh.subscribe("dji_sdk/data_received_from_remote_device",1,&DataFromMobileCallback);
    send_data_to_remote_device_service = nh.serviceClient<dji_sdk::SendDataToRemoteDevice>("dji_sdk/send_data_to_remote_device");
    activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
    sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");

    bool search_msg_count = false;
    bool landing_msg_count = false;
    bool abort_mission_count = false;

    ros::Rate loop_rate(10);
    while(true)
    {
        // Processing callbacks and services at a specific time interval
        ros::spinOnce();

        if(def_abort_misson)
        {
            bool ret = sdk_permission_control(0);

            if(ret){
                if(abort_mission_count == false){
                    std::string info = "Control is released and wait for orders to go home!";
                    ROS_INFO("Control is released and wait for orders to go home!");
                    send_to_mobile(info);
                    abort_mission_count = true;
                    break;
                }
            }
            continue;
        }

        if(def_abort_landing)
        {
            // count times by the auto_landing module
            flag_start_mission   = true;
            flag_abort_mission   = false;
            flag_abort_landing   = true;
            flag_search_task_end = true;
            pub_task_steps(global_dispatcher_pub_);
            continue;
        }

        // 1st task: pub search task start
        if(def_start_misson && !flag_abort_landing && !flag_abort_mission && !flag_search_task_end)
        {
            // pub task
            flag_start_mission   = true;
            flag_abort_mission   = false;
            flag_abort_landing   = false;
            flag_search_task_end = false;
            pub_task_steps(global_dispatcher_pub_);
            if(search_msg_count == false){
                std::string info = "1. search mission msg has published";
                ROS_INFO("1. search mission msg has published");
                send_to_mobile(info);
                search_msg_count = true;
            }
        }

        // 2nd task: pub auto landing task start
        if(def_start_misson && flag_search_task_end && !flag_abort_landing && !flag_abort_mission && !flag_auto_landing_end)
        {
            std::string requestGPS= "gps";
            send_to_mobile(requestGPS);
            flag_start_mission   = true;
            flag_abort_mission   = false;
            flag_abort_landing   = false;
            flag_search_task_end = true;
            flag_auto_landing_end = false;
            pub_task_steps(global_dispatcher_pub_);

            if( landing_msg_count == false){
                std::string info = "2. Tracking msg has published";
                ROS_INFO("2. Tracking msg has published");
                send_to_mobile(info);
                landing_msg_count = true;
            }
        }

        // 3rd task: process the remaining msg
        if(def_start_misson && flag_search_task_end && flag_auto_landing_end && !flag_abort_landing && !flag_abort_mission)
        {
            // end
        }
        loop_rate.sleep();
    }
    return 0;
}
