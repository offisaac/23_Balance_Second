# 22通用云台代码2.0

## 更新日志

[2022/03/25]：添加指示灯代码（串口6），并添加指示图，上位机改为串口4，修改舵轮底盘跟随标志位为第九位，添加上坡键位

[2022/03/25]：视觉操控云台放在uart1中断里，使其不会出现延时；死亡状态机改为can2不进入中断1s后进入；调整视觉发送过来的yaw、pitch轴数据的量纲

[2022/03/24]：添加舵轮底盘摆正的键位（V键）；pitch断电后自动进入死亡状态机复位

[2022/03/14]:修复chassis_ctrl文件中跟随云台方向运动但是运动反向的问题（之前负号在xsinθ上）<img src="readme.assets/image-20220314212717180.png" alt="image-20220314212717180" style="zoom:50%;" />

[2022/03/13]：

1. 更改pitch轴电机id为2，yaw轴id为5；
2. 发送到舵轮底盘的板间通信id号改为0x113，其余仍为0x222；
3. 在Infantry_CTRL头文件备注中添加了新舵轮、麦轮的pitch、yaw的offset值
4. 更改了接收到遥控的数据（遥控遥感值从负改为正），使球坐标系的数据正常
5. 修复了球坐标系theta数据突变的问题，把计算的int16_t变量转化为float型，使其能正常被arctan函数计算
6. 当前chassis_ctrl收到的yaw_currentAngle是负值，待修改

## 一、代码结构

Infantry_CTRL类包含：

- gimbal云台类
- booster小发射类
- PCvision视觉通信类
- chassisCTRL底盘控制类
- board_com板间通信类

## 二、How	to	use?

（步骤4、5已经写好，只需修改1、2、3）

1. 把pitch(can1)、yaw(can2)电机id号设置为0x206、0x209，把拨盘、左右摩擦轮（正对枪口）设置成0x201、0x202、0x203
2. 在infantry_CTRL头文件修改步兵类型宏定义、云台pitch,yaw的灵敏度和Offset（正前方向对应的码盘值）宏定义
3. 在infantry_CTRL头文件的构造函数对象初始化中修改板间通信的步兵类型，底盘控制所使用的坐标系，云台、小发射、底盘跟随的pid参数
4. 创建infantry对象
5. 在CAN接收函数调用`infantry.Update_Data()`，在视觉通信任务调用`infantry.pc_vision.GetViaionData()`,在陀螺仪任务调用`infantry.gimbal.MPUdata_Update()`，在步兵控制任务依次调用`infantry.Update_StateRequest()`、`infantry.Status_Update()`、`infantry.Adjust()`、`infantry.Actuate()`

## 三、板间通信

### 云台接收

1.裁判系统中的部分小发射数据	ID:0x221

```c++
typedef struct{	
	uint16_t bullet_speed;	//小发射射速，对应裁判系统的bullet_speed
	uint16_t cooling_rate;	//热量冷却速率，对应裁判系统的shooter_idX(该步兵id)_17mm_cooling_rate
	uint16_t heat_limit;	//热量上限，对应裁判系统的shooter_idX(该步兵id)_17mm_cooling_limit
	uint16_t booster_heat;	//当前热量，对应裁判系统的 shooter_idX(该步兵id)_17mm_cooling_heat
}_gimbal_RxPack1;
```

注意：bullet_speed为小发射射速*1000后的整型，保留3位小数，其他为裁判系统的原始数据

2.底盘其他数据	ID:0x220

```c++
typedef struct{	
	uint8_t source_power_max;	//功率限制上限，对应 chassis_power_limit
	uint8_t booster_maxspeed;	//射速上限，对应shooter_idX(该步兵id)_17mm_speed_limit
	uint8_t robot_id;			//步兵id，对应robot_id
	uint8_t cap_status;			//电容状态，低电容时关闭超功率模式
	uint8_t cap_voltage;		//电容电压，状态指示灯用
	uint8_t booster_enable;		//小发射使能标志位
	uint16_t chassis_flags;		//其他标志位，自定义
}_gimbal_RxPack2;
```

### 云台发送

ID:0x222

```c++
typedef struct{
	int16_t chassis_speed_y;	//前后方向速度(R)
	int16_t chassis_speed_x;	//左右方向(Alpha角)，平衡步为pitch、yaw补偿值
	int16_t chassis_speed_z;	//旋转速度(Theta角)
	uint16_t chassis_mode;		//发送到底盘的多个标志位
}_chassis_RxPack;
```

注：

1. 采用球坐标系时发送的数据为注释括号中的数据

2. 底盘标志位前8位为步兵通用标志位,后8位为各步兵专用标志位，通过移位的方式存放

   通用标志位目前有：（序号为标志位）

   1)遥控保护标志位，

   2)超功率标志位，

   3)小陀螺标志位，

   4)飞坡标志位，

   5)弹舱盖开启标志位，

   专用标志位：
   
   比如平衡步的倒地自救、舵轮的fuckMode等

### 如何修改标志位

1. 修改board_com文件里的Set_XXInfantry_Flag()里面的参数
2. 修改Infantry_CTRL文件里的Actuate()里的Set_XXInfantry_Flag()参数