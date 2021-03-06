#include <ros/ros.h>

#include <hirop/asr/audio_source.h>
#include <hirop/asr/hirop_asr.h>

#include <alsa/asoundlib.h>
#include <std_msgs/String.h>

#include <hirop_msgs/StartListen.h>
#include <hirop_msgs/StopListen.h>
#include "std_msgs/Int16.h"
using namespace HIROP::ASR;

class RosAudioSource : public GetIntentListener{

private:
    snd_pcm_t *whandle;

    snd_pcm_hw_params_t *wparams;

    snd_pcm_uframes_t frames;

    unsigned int val;

    int rc;

    int wdir;

    ros::Publisher intentPusher,getStatusPub;

    ros::NodeHandle _n;

    ros::ServiceServer startServer;

    ros::ServiceServer stopServer;

public:
    RosAudioSource(){

        hasr = HSpeechRecognition::getInstance();

        intentPusher = _n.advertise<std_msgs::String>("/user_intent", 1);

        startServer = _n.advertiseService("/start_listen", &RosAudioSource::onStart, this);

        stopServer = _n.advertiseService("/stop_listen", &RosAudioSource::onStop, this);

        getStatusPub = _n.advertise<std_msgs::Int16>("/getAiuiStatus",1);
        hasr->init();

        hasr->setIntentListener(this);

        initALSA();

    }

    int updateAudioData(char *buf){

        return 0;
    }

    void wirteAndRead(){

        int size = frames * 2;

        char *buffer;
        buffer = ( char * ) malloc(size);

        while(!ros::isShuttingDown()){
            rc = snd_pcm_readi(whandle,buffer,frames);
            if ( rc == -EPIPE )
            {
                fprintf(stderr,"overrun occured\n");
                snd_pcm_prepare(whandle);
            }
            else if ( rc < 0 )
            {
                fprintf(stderr,"error from read: %s\n",
                        snd_strerror(rc));
            }
            else if ( rc != (int)frames)
            {
                fprintf(stderr,"short read, read %d frames\n",rc);
            }

            hasr->updateAudioData(buffer, size);
            //
           // if (!checkNet())
            //{
             //   std_msgs::Int16 msg;
              //  msg.data = 404;
                //getStatusPub.publish(msg);
            //}

        }

    }

    void initALSA(){

        rc = snd_pcm_open(&whandle, "default", SND_PCM_STREAM_CAPTURE, 0);
        if (rc < 0) {
            fprintf(stderr, "unable to open pcm device: %s\n", snd_strerror(rc));
            exit(1);
        }

        /* Allocate a hardware parameters object. */
        snd_pcm_hw_params_alloca(&wparams);

        /* Fill it in with default values. */
        snd_pcm_hw_params_any(whandle, wparams);

        snd_pcm_hw_params_set_access(whandle, wparams, SND_PCM_ACCESS_RW_INTERLEAVED);

        // 位数
        snd_pcm_hw_params_set_format(whandle, wparams, SND_PCM_FORMAT_S16);

        // 通道数
        snd_pcm_hw_params_set_channels(whandle, wparams, 1);

        // 采用率
        val = 16000;
        snd_pcm_hw_params_set_rate_near(whandle, wparams, &val, &wdir);

        // 一帧数据的大小
        frames = 16;
        snd_pcm_hw_params_set_period_size_near(whandle, wparams, &frames, &wdir);

        rc = snd_pcm_hw_params(whandle, wparams);
        if (rc < 0) {
            fprintf(stderr, "unable to set hw parameters: %s\n", snd_strerror(rc));
            exit(1);
        }

    }

    void onGetIntent(const char* intent){

        std::cout << intent << std::endl;

        // const char* test = "你好，你好，你很好。";

        std_msgs::String msg;
        std::string str(intent);

        std::cout << str << std::endl;

        msg.data = str;
        intentPusher.publish(msg);

    }

    bool checkNet()
    {
        /*
           -c 2（代表ping次数，ping 2次后结束ping操作） -w 2（代表超时时间，2秒后结束ping操作）
        */
     // system("ping www.google.com -c 2 -w 2 >netlog.bat");
        system("ping www.baidu.com -c 2 -w 2 >netlog.bat");
        sleep(2);

        //把文件一行一行读取放入vector
        ifstream infile;
        infile.open("netlog.bat");
        string s;
        vector<string> v;
        while(infile)
        {
            getline(infile,s);
            if(infile.fail())
                break;
            v.push_back(s);
        }
        infile.close();

        //读取倒数第二行 2 packets transmitted, 2 received, 0% packet loss, time 1001ms
        if (v.size() > 1)
        {
            string data = v[v.size()-2];
            int iPos = data.find("received,");
            if (iPos != -1 )
            {
                data = data.substr(iPos+10,3);//截取字符串返回packet loss
                int  n = atoi(data.c_str());
                if(n == 0)
                 return 1;
                else
                return 0 ;
            }

        }else{
            return 0;
        }


    }


    bool onStart(hirop_msgs::StartListen::Request &req, hirop_msgs::StartListen::Response &rep){

        std::cout << "start listen" << std::endl;

        hasr->startListen();

        rep.reuslt = 0;

        //onGetIntent(NULL);

        return true;
    }

    bool onStop(hirop_msgs::StopListen::Request &req, hirop_msgs::StopListen::Response &rep){

        std::cout << "stop listen" << std::endl;

        hasr->stopListen();

        rep.reuslt = 0;

        return true;
    }

private:
    HSpeechRecognition *hasr;

};

int main(int argc, char* argv[]){

    ros::init(argc, argv, "asr_bridge");
    ros::NodeHandle n;

    RosAudioSource *as = new RosAudioSource();


    ros::AsyncSpinner spinner(2);
    spinner.start();

    as->wirteAndRead();


    ros::waitForShutdown();

    return 0;
}
