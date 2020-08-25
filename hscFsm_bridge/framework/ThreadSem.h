#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace std;

class semaphore
{
public:
    semaphore(const std::string &name, int value = 0) :count(value),name(name) {}

    void wait()
    {
        unique_lock<mutex> lck(mtk);
        count -= 1;
        if ( count < 0)//资源不足挂起线程
        {
//            std::cout << name <<" sem waiting .."<<std::endl;
            cv.wait(lck);
        }
    }

    void signal()
    {
        unique_lock<mutex> lck(mtk);
        ++count;
        if (count <= 0)//有线程挂起，唤醒一个
        {
//            std::cout<< name <<" sem signal .."<<std::endl;
            cv.notify_one();
        }
    }

private:
    int count;
    mutex mtk;
    std::string name;
    condition_variable cv;
};
