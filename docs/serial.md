# 串口通讯协议

串口通讯采用对称协议，即发送和接收包结构相同。每个包分为**指令部分**和**数据部分**，其中指令部分长度固定，数据部分长度根据不同指令变化。

## 指令部分
具体内容如下：

|长度(byte)|内容|
|---|---|
|1|包头，固定为：0x48|
|1|包头，固定为：0x42||
|1|指令类型|
|1|数据部分长度|
|1|校验码|

1. 包头：用于确定起始位置
2. 指令类型和数据部分长度。见下表，以下类型号均为无符号整形(`uint8_t`), 下同。
电控端接收包的类型号为从0递增。电控端发送包为从255递减。

|类型号|说明|数据部分长度(byte)|
|---|---|---|
|0|速度指令，控制机器底盘运动|12|
|1|云台指令1，控制云台跟随底盘|0|
|2|云台指令2，控制云台运动|8|
|3|打弹指令|待定|
|以下为电控端反馈数据包|
|255|速度反馈，底盘实际运动速度|12|
|254|陀螺仪反馈，陀螺仪当前状态|待定|

3. 校验码算法。校验码只计算数据部分，如果数据部分长度为0，则校验码也为0。
```c
typedef unsigned char uint8_t;

uint8_t computeChecksum(uint8_t *p, int len)
{
	uint8_t ret = 0;
	while(--len >= 0)
	{
	ret += *(p + len);
	}
	return ret;
}

/*
	调用示例：
	float data[3] = {0, -2.71, 3.14};
   	uint8_t crc = computeChecksum((uint8_t*)&data, sizeof(data));
	此时 crc = 65;
*/
```

## 数据部分
以下部分结构体可以用数组代替，例如 `struct {float x, y}` 内存布局等价于 `float xy[2]`。

| 类型号 | 数据部分内存结构 |说明|
| --- | --- | --- |
|0|`struct {float x, y, z};` |x: 正前方速度，y: 正左边速度，z: Yaw轴角速度|
|1|没有数据部分||
|2|`struct{float y, z};`|y: Pitch 轴角度, z: Yaw 轴角度|
|3|待定||
|254|待定||
|255|同 类型 0||

## 电控端例程

```c
// -----------------------
// 公用代码
// 复制到合适位置即可，不用修改
// typedef unsigned char uint8_t;

typedef struct 
{
	uint8_t magic[2]; // 固定为：0x48, 0x42
	uint8_t cmdType; // 指令类型
	uint8_t dataLen; // 数据部分长度
	uint8_t checksum; // 校验码
} PocketHeader;

// 校验码算法
// p 为数据部分指针， len 为数据部分长度
uint8_t computeChecksum(uint8_t *p, int len)
{
	uint8_t ret = 0;
	while(--len >= 0)
	{
	ret += *(p + len);
	}
	return ret;
}
// -----------------------------
```

```c
// -----------------------
// 接收数据包示例
// 需要修改 HAL_UART_* 函数的部分参数，删除 printf
int receive_demo()
{
	PocketHeader header;
	uint8_t buffer[256];
	uint8_t checksum;
	uint8_t tmp1, tmp2;
	while(1)
	{
		// 寻找包头：0x48, 0x42
		while(1)
		{
			HAL_UART_Receive_DMA(0, (uint8_t* )&tmp1, 1);
			if(tmp1 == 0x48)
			{
				HAL_UART_Receive_DMA(0, (uint8_t* )&tmp2, 1);
				if(tmp2 == 0x42)
					break;
			}
		}
		tmp1 = tmp2 = 0;

		// 读取剩下的 header
		HAL_UART_Receive_DMA(0, (uint8_t* )&header + 2, sizeof(PocketHeader) - 2);

		if(header.dataLen > 0)
		{
			// 读取数据部分
			HAL_UART_Receive_DMA(0, buffer, header.dataLen);
			checksum = computeChecksum(buffer, header.dataLen);
			if(checksum != header.checksum)
			{
				// 校验码不符合，说明数据部分有误
				printf("校验码有误，忽略此包\n");
				continue;
			}
		}
		else if(header.checksum != 0)
		{
            // 没有数据(dataLen = 0)时，要保证校验码为 0
			// 校验码不符合，说明数据部分有误
			printf("校验码有误，忽略此包\n");
			continue;
		}
		switch(header.cmdType)
		{
			case 0:
			{
				// 速度指令，也可以定义一个结构体代替 xyz，效果相同.
				float xyz[3];
				memcpy(&xyz, &buffer, sizeof(xyz));
				printf("收到：速度指令：%.2f, %.2f, %.2f\n", xyz[0], xyz[1], xyz[2]);
				// xyz[0], xyz[1], xyz[2] 分别表示 x, y, yaw 轴速度
				// 其他逻辑...
				break;
			}
			case 1:
			{
				// 云台指令1
				// 控制云台跟随底盘
				printf("收到：云台指令1\n");
				break;
			}
			case 2:
			{
				// 云台指令2，控制云台运动
				float yz[2];
				memcpy(&yz, &buffer, sizeof(yz));
				// yz[0] 表示 pitch 轴，yz[1] 表示 yaw 轴
				printf("收到:云台指令2：%.2f, %.2f\n", yz[0], yz[1]);
				break;
			}
			case 3:
			{
				// 打弹指令
				// 待定，不处理
				printf("收到：打弹指令\n");
				break;
			}
			default:
				// 未知指令，一般不存在
				printf("Error: 收到未知指令（%d）\n", header.cmdType);
		}
	}
}

// 发送 数据包 示例
int send_demo()
{
	// 发送速度反馈
	PocketHeader header;
    
	header.magic[0] = 0x48;
	header.magic[1] = 0x42;

	float xyz[3] = {0, 1, -1};
	
	// cmdType=255 为速度反馈
	header.cmdType = 255;
	header.dataLen = sizeof(xyz);
	header.checksum = computeChecksum((uint8_t *)&xyz, sizeof(xyz));

	HAL_UART_Transmit(0, (uint8_t*)&header, sizeof(PocketHeader), 1000);
	HAL_UART_Transmit(0, (uint8_t*)&xyz, sizeof(xyz), 1000);
}

```
