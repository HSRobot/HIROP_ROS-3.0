#ifndef HSCFSM_BRIDGE_VOICECTLROBTASK_H
#define HSCFSM_BRIDGE_VOICECTLROBTASK_H

#include "VoiceCtlRobRosFunc.h"
#include <HsTaskFramework.h>
#include "std_msgs/Int16.h"

#include <functional>
#include <assert.h>
#include <ros/ros.h>

namespace HsFsm {
    class VoiceCtlRobTask : public HsTaskFramework
    {
        public:
            VoiceCtlRobTask(const string &taskName);

            /**
             * @brief init
             */
            virtual void init();

            /**
             * @brief quit
             */
            virtual void quit();

            /**
             * @brief registerTaskList
             * @return
             */
            virtual bool registerTaskList();

        private:
            std::map<std::string, int> taskMap;

            VoiceCtlRobRosFunc* VCRRF;

            atomic<bool > isStop;
            atomic<bool > isErr;
            atomic<bool > isRun_init;
            atomic<bool > isRun_detection;
            atomic<bool > isRun_shakehand;
            atomic<bool > isRun_wave;
            atomic<bool > isRun_detectToy;
            atomic<bool > isRun_grabToy;

            ros::Subscriber voice_order_sub;
            ros::Subscriber people_detect_sub;

    private:
        void threadForSwithState(const char* state);
        void publishStateMsg(bool status,std::string behevior,std::string meassage);
        //状态行为函数
        void init_initing(const std::vector<std::string> &args);
        void init_quiting(const std::vector<std::string> &args);
        void prepare_initing(const std::vector<std::string> &args);
        void Detection_initing(const std::vector<std::string> &args);
        void Detection_quiting(const std::vector<std::string> &args);
        void Shakehand_initing(const std::vector<std::string> &args);
        void Shakehand_quiting(const std::vector<std::string> &args);
        void Wave_initing(const std::vector<std::string> &args);
        void Wave_quiting(const std::vector<std::string> &args);
        void DetectToy_initing(const std::vector<std::string> &args);
        void DetectToy_quiting(const std::vector<std::string> &args);
        void grabToy_initing(const std::vector<std::string> &args);
        void grabToy_quiting(const std::vector<std::string> &args);
        void dealErr_initing(const std::vector<std::string> &args);
        void dealErr_quiting(const std::vector<std::string> &args);
        void exit_initing(const std::vector<std::string> &args);
        void exit_quiting(const std::vector<std::string> &args);

        void transPrepare2detection(const std::vector<std::string> &args);
        void transPrepare2Exit(const std::vector<std::string> &args);

        void transDetection2exit(const std::vector<std::string> &args);
        void transDetection2dealErr(const std::vector<std::string> &args);

        void transShakehand2exit(const std::vector<std::string> &args);
        void transShakehand2dealErr(const std::vector<std::string> &args);

        void transWave2exit(const std::vector<std::string> &args);
        void transWave2dealErr(const std::vector<std::string> &args);

        void transDetectToy2exit(const std::vector<std::string> &args);
        void transDetectToy2dealErr(const std::vector<std::string> &args);

        void transgrabToy2exit(const std::vector<std::string> &args);
        void transgrabToy2dealErr(const std::vector<std::string> &args);

        void transexit2init(const std::vector<std::string> &args);

    };


}


#endif //HSCFSM_BRIDGE_VOICECTLROBTASK_H
