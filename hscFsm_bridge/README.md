# hscFsm_bridge

华数状态机的开发文档

主要的开发步骤

继承 HsTaskFramework 类的接口  如 抓起功能的 PickPlaceTask 任务状态机

    class PickPlaceTask :public HsTaskFramework
    {
    public:
        PickPlaceTask();

        /**
         * @brief init 初始化
         */
        virtual void init();

        /**
         * @brief quit 任务强制退出
         */
        virtual void quit();

        /**
         * @brief registerTaskList 任务状态机的注册
         * @return
         */
        virtual bool registerTaskList();


	注册的内容有表达形式：
    /**
     * @brief HsFsm::HsTaskFramework::registerTask
     * @param state 当前的状态
     * @param behevior 当前状态所属的行为
     * @param call 状态-行为 所属的函数行为
     */
	registerTask("init","initing", [&](callParm  &parm));
    

如 PickPlaceTask的状态机的部分步骤