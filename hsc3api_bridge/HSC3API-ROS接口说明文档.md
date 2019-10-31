#HSC3PAI-ROS服务接口说明文档
##功能阐述
###通过华数三型接口为HIROP平台提供点动，运动到点等机器人操作服务接口
###启动服务
- 可通过以下指令启动该服务接口
- rosrun hsc3api-bridge hsc3api-bridge

##接口使用说明
- 开始点动
- 接口名: /hsc3StartJog
- 参数1:axId , 类型:int8, 轴号（0..5为内部轴，6..8为附加轴） 
- 参数2:direc, 类型:bool, 方向（true为正方向,false为反方向）
- 返回值:ret, 类型:uint64, 三型的错误码

- --
- 停止点动
- 接口名: /hsc3StopJog
- 参数:无
- 返回值:ret,类型:uint64, 三型的错误码

- --
- 速率修调
- 接口名: /hsc3SetVord
- 参数:vord, 类型:int32, 倍率（1~100，单位：%）
- 返回值:ret,类型:uint64, 三型的错误码

- --
- 设置运动模式
- 接口名: /hsc3SetOpMode
- 参数:mode 类型:int8, 类型(1-手动T1,2-手动T2,3-自动，4-外部)
- 返回值:ret,类型:uint64, 三型的错误码

- --
- 运动到点
- 接口名: /hsc3MoveTo
- 参数1:gpos,类型:GeneralPos， 包含四个成员：
- (1) isjoint, 类型:bool, 是否关节坐标(ture-传入的坐标为关节坐标，false-传入坐标为笛卡尔坐标)
- (2) ufNum, 类型:uint32, 工件号（1..n） 
- (3) utNum, 类型:uint32, 工具号（1..n） 
- (4) verpos,类型:std_msgs/Float64MultiArray, 传入坐标（使用verpos.data作为数组载体）
- 参数2:isLinear, 类型:bool, 是否直线运动（true-直线运动,false-关节运动）
- 返回值:ret,类型:uint64, 三型的错误码

---
- 切换工作坐标系
- 接口名: /hsc3SetWorkFrame
- 参数:frame 类型:int8, 类型(1-关节坐标,2-世界坐标,3-工具坐标，4-工件坐标)
- 返回值:ret,类型:uint64, 三型的错误码

- --
- 设置IO输出
- 接口名: /hsc3SetIODout
- 参数:portIndex, 类型:int32, 数字IO端口值(0 - n-1)
- 参数:value, 类型:bool, true - 打开端口,　false - 关闭端口
- 返回值:ret,类型:uint64, 三型的错误码
