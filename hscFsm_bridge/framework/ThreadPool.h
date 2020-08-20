#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>
#include <iostream>

class ThreadPool {
public:
    ThreadPool(size_t);

    // typename std::result_of<F(Args...)>::type  推导出该函数的返回强制
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type>;
    ~ThreadPool();

    /**
     * @brief start
     */
    void Start();

    /**
     * @brief stop
     */
    void Stop();

    /**
     * @brief getInit
     * @return
     */
    bool getInit();

private:
    // need to keep track of threads so we can join them
    std::vector< std::thread > workers;
    // the task queue
    std::queue< std::function<void()> > tasks;
    
    // synchronization
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
    bool startFlag;
    size_t threads;
};
 
// the constructor just launches some amount of workers
inline ThreadPool::ThreadPool(size_t threads)
    :stop(false),startFlag(false), threads(threads)
{

}


void ThreadPool::Start()
{
    if(startFlag)
        return ;
    else
        startFlag = true;

    std::cout << " ThreadPool start ... "<<std::endl;
    for(size_t i = 0;i<threads;++i)
        workers.emplace_back(
            [this]
            {
                for(;;)
                {
                    std::function<void()> task;

                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        //阻塞的实现条件 this->stop || !this->tasks.empty()
                        this->condition.wait(lock,
                            [this]{ return this->stop || !this->tasks.empty(); });
                        //退出 停止的时候 函数返回 线程正常退出
                        if(this->stop && this->tasks.empty())
                            return;

                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    task();
                }
            }
        );
}


// add new work item to the pool
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) 
    -> std::future<typename std::result_of<F(Args...)>::type>
{

    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared< std::packaged_task<return_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

    if(!startFlag)
        return task->get_future();

    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);

        // don't allow enqueueing after stopping the pool
        if(stop)
            throw std::runtime_error("enqueue on stopped ThreadPool");

        tasks.emplace([task](){ (*task)(); });
    }
    condition.notify_one();
    return res;
}

void ThreadPool::Stop()
{
    if(!startFlag)
       return ;

    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for(std::thread &worker: workers)
        worker.join();

    startFlag = false;
    std::cout << " ThreadPool stop ... "<<std::endl;

}

// the destructor joins all threads
inline ThreadPool::~ThreadPool()
{
    if(startFlag)
        Stop();
}

#endif
