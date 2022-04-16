#  STM32

## 简介

- STM32系列是由意法半导体公司推出的ARM Cortex-M内核单片机，从字面上来看，ST为意法半导体公司的缩写，M是Microcontrollers即单片机的缩写，32代表32位

-----

### 芯片系列

- ![Chip_series0](./STM32.assets/Chip_series0.png)
- ![Chip_series1](./STM32.assets/Chip_series1.png)

----

### 命名规则

- ![Suffix_model_description](./STM32.assets/Suffix_model_description.png)

-----

### 使用的芯片

***STM32F103C8T6***

- 系列：主流系列STM32 F1
- 内核：ARM Cortex-M3
- 主频：72 MHz
- RAM（随机存取存储器，运行内存）：20 K (SRAM)
- ROM（程序存储器）：64 K (Flash)
- 供电：2.0-3.6V(标准3.3V)
- 封装：LQFP48 （指的是芯片有48个引脚）

----

### 外设资源

![Peripherals](./STM32.assets/Peripherals.png)

- **NVIC** 

  > 嵌套向量中断控制器，内核里用于管理中断的设备
  >
  > 比如：中断优先级

- **SysTick**

  >系统滴答定时器，主要用于给操作系统提供定时服务

- **RCC**

  > 配置系统时钟，使能各模块的时钟
  >
  > 与51不同，STM32为了降低功耗时钟都是默认disable的
  >
  > 时钟的功能就好像是一个小开关，你要用什么寄存器就先对应的打开开关，即：使能对应的时钟。

- **GPIO**

  >通用型之输入输出的简称

- **AFIO**

  > AFIO就是IO复用，就是一个IO口用在多个外设上
  >
  > 这样做的目的是节省IO资源，提高IO利用率

- **EXTI**

  > 外部中断/事件控制器管理了控制器的 20个中断/事件线
  >
  > 每个中断/事件线都对应有一个边沿检测器，可以实现输入信号的上升沿检测和下降沿的检测。 
  >
  > EXTI 可以实现对每个中断/事件线进行单独配置，可以单独配置为中断或者事件，以及触发事件的属性。
  >
  > 当引脚有电平变化时，触发中断，让CPU来处理任务
  >
  > **中断响应/事件响应：**前者触发中断，后者触发外设（即外设之间的互联）

- **TIM**

  > 高级定时器
  >
  > 通用定时器
  >
  > 基本定时器

- **ADC**

  > 通常是指一个将模拟信号转变为数字信号的电子元件

----

### 启动配置

- ![BOOT](./STM32.assets/BOOT.png)

----

## GPIO

![GPIO basic structure](./STM32.assets/GPIO basic structure.png)

- **GPIO操作步骤**               **重要   !!!!!**

  > 1. 使用RCC开启GPIO的时钟
  > 2. 使用GPIO_Init函数初始化GPIO
  > 3. 使用输出或者输入控制GPIO口

----

### 工作模式

```c
typedef enum
{ GPIO_Mode_AIN = 0x0,			//模拟输入
  GPIO_Mode_IN_FLOATING = 0x04,	 //浮空输入
  GPIO_Mode_IPD = 0x28,			//下拉输入
  GPIO_Mode_IPU = 0x48,			//上拉输入
  GPIO_Mode_Out_OD = 0x14,		//开漏输出
  GPIO_Mode_Out_PP = 0x10,		//推挽输出，该模式下高低电平均有驱动能力
  GPIO_Mode_AF_OD = 0x1C,		//复用开漏
  GPIO_Mode_AF_PP = 0x18		//复用推挽
}GPIOMode_TypeDef;
```

-----

### LED灯闪烁

**库函数：RCC_APB2PeriphClockCmd()**

```C
RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
```

**功能**

- 使能外设时钟

**原型**

```c++
void RCC_APB2PeriphClockCmd（uint32_t RCC_APB2Periph, FunctionalState NewState）
```

**参数**

1. 选外设端口。例如，用PA0口，则选用RCC_APB2Periph_GPIOA；用PB0口，则选用RCC_APB2Periph_GPIOB；
2. 选enable or disable。

**代码**

```c
#include "stm32f10x.h"                  // Device header
#include "Delay.h"

int main(void){
	//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//配置GPIO初始化所需要用的一些信息
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//工作模式为推挽输出，该模式下高低电平均有驱动能力
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			//用的是GPIO外设的0号引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//输出速度为50MHz
	
	//初始化GPIO
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//拉低PA0号引脚输出电平
//	GPIO_ResetBits(GPIOA, GPIO_Pin_0);	//点灯
	//拉高PA0号引脚输出电平
//	GPIO_SetBits(GPIOA, GPIO_Pin_0);	//熄灭
	
	while(1)
	{
		//对指定端口的电平拉高或者拉低，可以操作多个端口，实现效果和GPIO_ResetBits/GPIO_SetBits一样
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);	//点灯
		Delay_ms(500);
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);		//熄灭
		Delay_ms(500);
		/*
		//也可以这样写：
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, 0);			//报错或警告就将0改为:(BitAction)0
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, (BitAction)1);	//强制类型转换
		*/
	}
}
//最后一行要留多加一个空行

```

-----

### LED流水灯

**代码**

```c
#include "stm32f10x.h"                  // Device header
#include "Delay.h"

int main(void){
	//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//配置GPIO初始化所需要用的一些信息
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//工作模式为推挽输出，该模式下高低电平均有驱动能力
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;			//初始化0~15个端口
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//输出速度为50MHz
	
	//初始化GPIO
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	while(1)
	{
        // 通过内存地址的方式修改电平，0x0001是十六进制码，前面加~是因为低电平点亮，所以要取反
        GPIO_Write(GPIOA, ~0x0001);		//0000 0000 0000 0001	1	对应的引脚地址
        Delay_ms(100);
        GPIO_Write(GPIOA, ~0x0002);		//0000 0000 0000 0010	2
        Delay_ms(100);
        GPIO_Write(GPIOA, ~0x0004);		//0000 0000 0000 0100	4
        Delay_ms(100);
        GPIO_Write(GPIOA, ~0x0008);		//0000 0000 0000 1000	8
        Delay_ms(100);
        GPIO_Write(GPIOA, ~0x00010);	//0000 0000 0001 0000	16
        Delay_ms(100);
        GPIO_Write(GPIOA, ~0x00020);	//0000 0000 0010 0000	32
        Delay_ms(100);
        GPIO_Write(GPIOA, ~0x00040);	//0000 0000 0100 0000	64
        Delay_ms(100);
        GPIO_Write(GPIOA, ~0x00080);	//0000 0000 1000 0000	128
        Delay_ms(100);
	}
}
//最后一行要留多加一个空行

```

*补充：初始化多个引脚口时可以通过异或来初始化*

```c
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
```

*使用 for() 优化后*

**代码**

```c
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include <math.h>

int main(void){
	//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//配置GPIO初始化所需要用的一些信息
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//工作模式为推挽输出，该模式下高低电平均有驱动能力
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;			//初始化0~15个端口
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//输出速度为50MHz
	
	//初始化GPIO
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    //用uint16_t类型数组存8个引脚地址
	uint16_t Pins[8];	//uint16_t是unsigned short int类型的别名，此处可以直接理解成16进制类型
	for(int i=0; i<8; i++){
		Pins[i]= (uint16_t)(pow(2,i));	//通过强转类型将10进制转为16进制存到数组中
	}
	
	int i;
	while(1){
		i = 0;
		while(i<8){
			GPIO_Write(GPIOA, Pins[i]);
			i++;
			Delay_ms(100);
		}
	}
}
//最后一行要留多加一个空行

```

----

### 蜂鸣器

**代码**

```c
#include "stm32f10x.h"                  // Device header
#include "Delay.h"

int main(void){
	//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//配置GPIO初始化所需要用的一些信息
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//工作模式为推挽输出，该模式下高低电平均有驱动能力
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;			//初始化0~15个端口
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//输出速度为50MHz
	
	//初始化GPIO
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	while(1)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);	//响
		Delay_ms(500);
		GPIO_SetBits(GPIOB, GPIO_Pin_12);	//停
		Delay_ms(500);
	}
}
//最后一行要留多加一个空行

```

---

### 按键控制LED

**代码**

*LED.c*

```c
#include "stm32f10x.h"                  // Device header

void LED_Init(void){
	//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	//定义GPIO初始化所需要的一些配置
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//推挽输出模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;	//1号和2号引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//输出速度为50MHz
	
	//初始化GPIO，默认初始化的是低电平（GPIO_ResetBits）
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

//打开LED1
void LED1_ON(void){
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}

//关闭LED1
void LED1_OFF(void){
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
}

//LED1的状态取反
void LED1_Turn(void){
	if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1) == 0){	//获取输出寄存器GPIO_Pin_1地址的值
		GPIO_SetBits(GPIOA, GPIO_Pin_1);	
	}
	else{
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);	
	}
}

//打开LED2
void LED2_ON(void){
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);
}

//关闭LED2
void LED2_OFF(void){
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
}

//LED2的状态取反
void LED2_Turn(void){
	if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_2) == 0){	//获取输出寄存器GPIO_Pin_2地址的值
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
	}
	else{
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	}
}

```

*Key.c*

```c
#include "stm32f10x.h"                  // Device header
#include "Delay.h"

//初始化按键
void Key_Init(void){
	//使能APB2外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   //上拉输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;	//也可以 = 0x0001 | 0x0800
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	//初始化GPIO
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//获取按下的按键的引脚地址
uint8_t Key_Get_Num(void){
	uint8_t KeyNum = 0;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0){
		Delay_ms(20);		//这个延时是为了避免机械式按键的抖动现象
		while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0){	//用于长按按键的情况
			Delay_ms(20);	//这个延时是为了避免机械式按键的抖动现象
			KeyNum = 1;
		}
	}
	else if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0){
		Delay_ms(20);		
		while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0){
			Delay_ms(20);	
			KeyNum = 2;
		}
	}
	return KeyNum;
}

```

*main.c*

```c
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "LED.h"
#include "Key.h"

uint8_t KeyNum;

int main(void){
	LED_Init();		//初始化LED
	Key_Init();		//初始化按键
	while(1)
	{
		KeyNum = Key_Get_Num();	//获取按下的按键
		if (KeyNum == 1){
			LED1_Turn();	//LED状态取反，亮->灭 / 灭->亮
		}else if (KeyNum == 2){
			LED2_Turn();
		}
	}
}
//最后一行要留多加一个空行

```

----

### 光敏传感器控制蜂鸣器

*Buzzer.c*

```c
#include "stm32f10x.h"                  // Device header

//初始化蜂鸣器
void Buzzer_Init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

//打开蜂鸣器
void Buzzer_ON(void){
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

//关闭蜂鸣器
void Buzzer_OFF(void){
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

//蜂鸣器状态取反
void Buzzer_Turn(void){
	if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12) == 0){
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
	}
	else{	
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	}
}

```

*LightSensor.c*

```c
#include "stm32f10x.h"                  // Device header

void LightSensor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint8_t LightSensor_Get(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
}

```

*main.c*

```c
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Buzzer.h"
#include "LightSensor.h"

int main(void)
{
	Buzzer_Init();
	LightSensor_Init();
	
	while (1)
	{
		if (LightSensor_Get() == 1)
		{
			Buzzer_ON();
		}
		else
		{
			Buzzer_OFF();
		}
	}
}

```

----



### GPIO库函数

- **GPIO_Init()**

  > **原型**
  >
  > ```c
  > void GPIO_Init (GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
  > ```
  >
  > **参数**
  >
  > 1. GPIO的类型，A口、B口、C口
  > 2. GPIO的配置，如工作模式、端口、输出速度等。需要先在一个  *GPIO_InitTypeDef*  类型的变量中定义好。
  >
  > **功能**
  >
  > 初始化GPIO
  

----

## 外部中断

### 中断系统

- 中断：在主程序运行过程中，出现了特定的中断触发条件（中断源），使得CPU暂停当前正在运行的程序，转而去处理中断程序，处理完成后又返回原来被暂停的位置继续运行
- 中断优先级：当有多个中断源同时申请中断时，CPU会根据中断源的轻重缓急进行裁决，优先响应更加紧急的中断源
- 中断嵌套：当一个中断程序正在运行时，又有新的更高优先级的中断源申请中断，CPU再次暂停当前中断程序，转而去处理新的中断程序，处理完成后依次进行返回
- 作用：提升CPU的效率，避免CPU一直在查询某个程序是否执行，只有当该程序被执行的时候才去执行她（见执行流程图）

### 中断执行流程

![Interrupt_execution_process](./STM32.assets/Interrupt_execution_process.png)

----

### NVIC

![NVIC](./STM32.assets/NVIC.png)

-----

### 中断优先级

![Interrupt_priority](./STM32.assets/Interrupt_priority.png)

----

### EXTI

**简介**

- Extern Interrupt 外部中断
- EXTI可以监测GPIO口的电平信号，当其指定的GPIO口产生电平变化时，EXTI将立即向NVIC发出中断申请，经过NVIC裁决后即可中断CPU主程序，使CPU执行EXTI对应的中断程序
- 支持的触发方式：上升沿/下降沿/双边沿/软件触发
- 支持的GPIO口：所有GPIO口，但相同的Pin不能同时触发中断
- 通道数：16个GPIO_Pin，外加PVD输出、RTC闹钟、USB唤醒、以太网唤醒
- 触发响应方式：中断响应/事件响应

**基本结构**

![EXTI](./STM32.assets/EXTI.png)

----

### AFIO

- AFIO主要用于引脚复用功能的选择和重定义
- 在STM32中，AFIO主要完成两个任务：复用功能引脚重映射、中断引脚选择

![AFIO](./STM32.assets/AFIO.png)



### 对射式红外传感器计次

**代码**

*CountSensor.c*

```c
#include "stm32f10x.h"

uint16_t CountSensor_Count;

void CountSensor_Init(void){
	
	//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//复用IO
	
	//配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
	//配置EXTI
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
    
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;				//14号口
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//开启
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		 //中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	 //下降沿触发，遮挡的时候触发
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//上升沿触发，遮挡后离开触发
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升下降沿都触发，遮挡离开都触发
	
	//初始化EXTI
	EXTI_Init( &EXTI_InitStructure );
	
	//NVIC中断优先级分组
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
	
	//配置NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;	//中断通道列表
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	
	//初始化NVIC
	NVIC_Init( &NVIC_InitStructure );

}

//用于获取CountSensor_Coun的值
uint16_t CountSensor_Get(void){
	return CountSensor_Count;
}

//中断函数，不需要声明
void EXTI15_10_IRQHander(void)
{
	if(EXTI_GetITStatus(EXTI_Line14) == SET)
	{
		CountSensor_Count++;
		EXTI_ClearITPendingBit(EXTI_Line14);
	
	}
}

```

*main.c*

```c
#include "stm32f10x.h"                  				// Device header
#include "Delay.h"
#include <math.h>
#include "OLED.h"
#include "CountSensor.h"

/*项目思路：传感器被触发->中断函数被调用->计数变量自增->OLED展示数字增减*/

int main(void){
	
	OLED_Init();	//初始化OLED
	
	OLED_ShowString(1,1,"Count:");	
	
	while(1)
	{
		OLED_ShowNum(1,7,CountSensor_Get(), 5);		//
	}	
}
//最后一行要留多加一个空行

```

----

### 旋转编码器计次

*Encoder.c*

**代码**

```c
#include "stm32f10x.h"                  // Device header

int16_t Encoder_Count;

void Encoder_Init(void){

//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//复用IO
	
	//配置GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

	
	
	//配置EXTI
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;	//0号和1号GPIO
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//开启
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//下降沿触发，遮挡的时候触发
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//上升沿触发，遮挡后离开触发
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升下降沿都触发，遮挡离开都触发

	
	//初始化EXTI
	EXTI_Init( &EXTI_InitStructure );
	
	
	//NVIC中断优先级分组
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
	
	//配置NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//0口
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	//中断通道列表
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//响应优先级
	
	//1口
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//中断通道列表
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			//响应优先级
	
	
	//初始化NVIC
	NVIC_Init( &NVIC_InitStructure );

}


int16_t Encoder_Get(void){
	int16_t Temp;
	Temp = Encoder_Count;
	Encoder_Count = 0;
	return Temp;
}


//0口中断函数
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) == SET){
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0){
			Encoder_Count --;
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

//1口中断函数
void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) == SET){
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0){
			Encoder_Count++;
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

```





## 附录

### 电路知识储备

**什么是VCC？**

> Vcc，是Volt Current Condenser的简写，意思是电路的**供电电压**，电源电压（双极器件）；电源电压（74系列数字电路）；声控载波（Voice Controlled Carrier)；火线。

**什么是GND？**

> GND是电线**接地端**的简写。代表地线或0线。这个地并不是真正意义上的地，是出于应用而假设的一个地，对于电源来说，它就是一个电源的负极。
>
> GND分为数字地（DGND）模拟地（AGND）

**什么是电平**

> 所谓电平，是指两功率或**电压之比**的对数，有时也可用来表示两**电流之比**的对数。

----

### C语言

**原码补码反码**

- 原码：数据的二进制形式
- 补码：计算机数据的存储形式
- 反码：原码求补码或者由补码求原码的过渡码

**例  **123原码：0111 1011
无符号数：反码 == 原码 == 补码
				123原码：0111 1011
				123反码：0111 1011
				123补码：0111 1011
有符号数：
		正数：反码 == 原码 == 补码
				+123原码：0111 1011
				+123反码：0111 1011
				+123补码：0111 1011
		负数：反码 == 原码符号位不变，其他位按位取反
				   补码 == 反码 +1
				-123原码：1111 1011
				-123反码：1000 0100
				-123补码：1000 0101

**两数相加/减原理**

*原码 + 补码*

- 相加

  > ```c
  > 10 + 6 = 16
  >     0000 1010
  > +   0000 0110
  > --------------
  >     0001 0000
  > ```

- 相减

  > ```c
  > 10 - 6 = 4  -->  10 + (-6) = 4
  >     0000 1010
  > +   1111 1010 		(反码+1  1111 1001 + 1)
  > --------------
  >     0000 0100
  > ```
  >
  > 

**枚举类型enum**


```c
enum { NUM1 =1,CHAR = 's'} Ty;	//定义一个枚举类型的变量 Ty
Ty = NUM1;	//该变量只可以去定义好的内容
printf(" %d", Ty);
```

**代码**

```c
#include <stdio.h>

//通过修改变量类型名使用 enum
typedef enum {
    NUM1 =1,
    CHAR = 's'
} Ty;

int main() {
    Ty k,l;
    k = NUM1;
    l = CHAR;
    printf("%d, %d",k,l);

    return 0;
}

```

---

**#ifndef**

头件的中的#ifndef，这是一个很关键的东西。比如你有两个C文件，这两个C文件都include了同一个头文件。而编译时，这两个C文件要一同编译成一个可运行文件，于是问题来了，大量的声明冲突。

**代码**

*LightSensor.h*

```c
#ifndef _LIGHT_SENSOR_H
#define _LIGHT_SENSOR_H

void LightSensor_Init(void);

#endif

```

