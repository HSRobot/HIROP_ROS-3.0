#ifndef PICKPLACETASK_H
#define PICKPLACETASK_H

#include <HsTaskFramework.h>
namespace HsFsm {


class PickPlaceTask :public HsTaskFramework
{
public:
    PickPlaceTask(const string &taskName);

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

    void transInit2Run(const std::vector<std::string> &args);
private:
    std::map<std::string, int> taskMap;

private:
    bool timerRun;
};


}
#endif // PICKPLACETASK_H
