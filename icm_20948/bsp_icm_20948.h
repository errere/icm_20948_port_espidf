#ifndef __BSP_ICM_20948_H__
#define __BSP_ICM_20948_H__

#include "ICM_20948_C.h"
#include "AK09916_REGISTERS.h"

#include <driver/i2c_master.h>

// idf release/5.0 : semphr.h mast include with FreeRTOS.h
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct
{
  SemaphoreHandle_t *pSemaphoreIICFree;
  i2c_master_dev_handle_t *i2c_handle;
  uint32_t xfer_timeout;
} icm20948_intf_t;

typedef struct
{
  ICM_20948_Serif_t ops;
  icm20948_intf_t intf;
  ICM_20948_Device_t lib;
} icm20948_handle_t;

ICM_20948_Status_e icm_init(icm20948_handle_t *handle, icm20948_intf_t intf, bool minimal);

// DMP sensor options are defined in ICM_20948_DMP.h
//    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
//    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
//    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
//    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
//    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
//    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
//    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
//    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
//    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
//    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
//    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
//    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
//    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
//    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
//    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

ICM_20948_Status_e icm_initializeDMP(icm20948_handle_t *handle);

ICM_20948_Status_e icm_dmp_full_init(icm20948_handle_t *handle);

/*
#InvenSense数字运动处理器（DMP™）
##什么是数字运动处理器（DMP™）？
在该库的1.2版中，我们添加了对InvenSense数字运动处理器（DMP™）的_partial_支持。DMP是在上运行的固件
ICM-20948，“将运动处理算法的计算从主处理器中卸载，提高系统电源性能”。
“DMP能够实现加速度计、陀螺仪和指南针的超低功率运行时间和背景校准，保持最佳性能
通过传感器融合生成的物理传感器和虚拟传感器的传感器数据。"
DMP允许将加速度计、陀螺仪和磁力计数据组合（融合），从而可以生成四元数数据。
DMP固件二进制文件已经存在很长一段时间了。它包含在InvenSense的“MotionLink”和“嵌入式运动驱动器（eMD）”示例中
可以从InvenSense开发者角下载。然而，该代码不透明且难以遵循。
用户喜欢[@ericalbers](https://github.com/ericalbers/ICM20948_DMP_Arduino)和[@ZaneL](https://github.com/ZaneL/Teensy-ICM-20948)已移植
InvenSense示例代码到之前的Arduino环境。我们感谢Eric和Zane，因为他们的代码使我们能够对
ICM-20948配置步骤。
我们也感谢InvenSense自己与我们分享了一份名为“_应用程序说明：编程序列
ICM-20648 DMP硬件功能_”。InvenSense承认该文件不完整，并要求我们不要公开分享。
InvenSense文档和我们使用Zane的端口捕获的总线流量使我们能够使用
自己的功能。之所以说_partial_，是因为在撰写本文时，我们的库不支持：活动识别、步数计数、拾取和敲击检测。
然而，它确实支持：
-原始和校准的加速度计、陀螺仪和指南针数据和精度
-6轴和9轴四元数数据（包括游戏旋转矢量数据）
-地磁旋转矢量数据
-和[更多…]（#当前支持哪些dmp功能）
我们添加了[五个新示例](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/master/examples/Arduino)显示如何配置DMP并读取：
9轴四元数数据；6轴四元数转换为欧拉角（滚转、俯仰和偏航）；原始加速度计数据。
##默认情况下是否启用DMP支持？
不会。DMP占用14kBytes的程序内存，因此，为了允许库在内存有限的处理器上继续运行，默认情况下会禁用DMP支持。
您可以通过编辑名为``ICM_20948_C.h ``的文件并取消注释[第29行]来启用它(https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/src/util/ICM_20948_C.h#L29):
改变
```
//#定义ICM_20948_USE_DMP
```
到
```
#定义ICM_20948_USE_DMP
```
您将在库_src\util_文件夹中找到``ICM_20948_C.h ``。如果您使用的是Windows，您可以在_Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util_中找到它。
##DMP是如何加载和启动的？
在1.2.5版本中，我们添加了一个名为``initializeDMP``的新帮助函数。这是一个弱函数，您可以覆盖它，例如，如果您想更改采样率
（参见[示例10](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/examples/Arduino/Example10_DMP_FastMultipleSensors/Example10_DMP_FastMultipleSensors.ino)详细信息）。
```initializeDMP“”为您完成了大部分繁重的工作：它下载DMP固件；并且用适当的值配置所有寄存器。唯一的东西
您需要手动执行以下操作：选择要启用的DMP传感器；重置并启动FIFO和DMP。
DMP固件通过三个特殊的Bank0寄存器加载到ICM-20948的处理器内存空间中：
-**AGB0_REG_MEM_START_ADDR**（0x7C）-AGB0_REG_MEM_R_W读取或写入的地址（每次读取或写入后自动递增）
-**AGB0_REG_MEM_R_W**（0x7D）-内存读写寄存器
-**AGB0_REG_MEM_BANK_SEL**（0x7E）-选择内存组。完整的读/写地址为：（AGB0_REG_MEM_BANK_SEL*256）+AGBO_REG_MEM_START_ADDR
固件二进制（14290或14301字节）从地址0x90开始写入处理器内存```loadDMPFirmware“”自动将代码分解为256字节的块和增量
**写入期间AGB0_REG_MEM_BANK_SEL**。
在启用DMP之前，需要用程序起始地址加载16位寄存器**AGB2_REG_PRGM_START_ADDRH**（组2，0x50）```setDMPstartAddress“”为您执行此操作。
DMP通过设置存储体0寄存器**AGB0_REG_USER_CTRL**（0x03）中的位来启用或重置```enableDMP ```和``resetDMP```为您执行此操作。
辅助函数“readDMPmems”和“writeDMPmems”将允许您直接从DMP内存空间读取和写入数据。
##如何访问DMP数据？
DMP数据通过FIFO（先进先出）返回```readDMPdataFromFIFO“”检查FIFO中是否存在任何数据（通过调用读取16位寄存器的“”getFIFOcount“”
**AGB0_REG_FIFO_COUNT_H**（0x70））。如果存在数据，则将其复制到``icm_20948_DMP_data_t```结构中。
```readDMPdataFromFIFO“”将返回：
-``ICM_20948_Stat_FIFONoDataAvail```如果没有数据或数据不完整
-``ICM_20948_Stat_Ok``如果读取了有效帧
-``ICM_20948_Stat_FIFMoreDataAvail```如果读取了有效帧且FIFO包含更多（未读取）数据
-``ICM_20948_Stat_FIFOIncompleteData ```如果FIFO中存在一个帧但它不完整
您可以检查16位“``icm_20948_DMP_data_t data.header``”以查看帧包含哪些数据```data.header ``是一个位字段；每个比特指示存在什么数据：
-**DMP_header_bitmap_Compass_Calibr**（0x0020）
-**DMP_header_bitmap_Gyro_Calibr**（0x0040）
-**DMP_header_bitmap_Geomag**（0x0100）
-**DMP_header_bitmap_PQuat6**（0x0200）
-**DMP_header_bitmap_Quat9**（0x0400）
-**DMP_header_bitmap_Quat6**（0x0800）
-**DMP_header_bitmap_ALS**（0x1000）
-**DMP_header_bitmap_Compass**（0x2000）
-**DMP_header_bitmap_Gyro**（0x4000）
-**DMP_header_bitmap_Accel**（0x8000）
**DMP_header_bitmap_Header2**（0x0008）指示是否包括任何辅助数据。如果设置了**DMP_header_bitmap_Header2**位，则帧还包含以下一个或多个：
-**DMP_header2_bitmap_Compass_Accuracy**（0x1000）
-**DMP_header2_bitmap_Gyro_Accuracy**（0x2000）
-**DMP_header2_bitmap_Accel_Accuracy**（0x4000）
##当前支持哪些DMP功能？
以下所有内容都应该有效，但我们尚未对其进行全部测试：
```
INV_ICM20948_SENSOR_ACCELEROMETER（16位加速度）
INV_ICM20948_SENSOR_GYROSOPE（16位陀螺仪+32位校准陀螺仪）
INV_ICM20948_SENSOR_RAW_ACCELEROMETER（16位加速度）
INV_ICM20948_SENSOR_RAW_GYROSOPE（16位陀螺仪+32位校准陀螺仪）
INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED（16位指南针）
INV_ICM20948_SENSOR_GYROSOPE_UNCALIBRATED（16位陀螺仪）
INV_ICM20948_SENSOR_STEP_DETECTOR（计步器步进检测器）
INV_ICM20948_SENSOR_STEP_COUNTER（计步器步进检测器）
INV_ICM20948_SENSOR_GAME_ROTATION_ECTOR（32位六轴四元数）
INV_ICM20948_SENSOR_ROTATION_ECTOR（32位9轴四元数+航向精度）
INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_ECTOR（32位Geomag RV+航向精度）
INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD（32位校准罗盘）
INV_ICM20948_SENSOR_GRAVITY（32位六轴四元数）
INV_ICM20948_SENSOR_LINEAR_ACCELERATION（16位加速度+32位六轴四元数）
INV_ICM20948_SENSOR_ORIENTATION（32位9轴四元数+航向精度）
```
##您在v1.2.5中做了哪些更改？
在v1.2.5中，我们添加了一些关键的缺失配置步骤：
-我们使用I2C_SLV0和I2C_SLV1来请求磁力计数据并触发下一次单次测量。我们不再为DMP使用100Hz连续模式
-我们现在从磁强计读取10个字节的数据，从寄存器0x03开始；不是从寄存器0x10开始读取9个字节
-寄存器0x03是保留的，其他九个寄存器没有记录。它们似乎包含big-endian格式的原始磁力计读数（而不是little-endian）
-我们必须深入研究InvenSense的Icm20948AuxCompassAkm.c才能发现这一点。。。
-我们配置I2C主ODR，它将磁力计的读取速率从愚蠢的1100Hz降低到合理的69Hz
-我们必须监控Aux I2C引脚，并研究AK09916流量来解决这个问题。。。
DMP配置代码变得如此冗长，以至于我们决定将其移动到自己的函数“”中，该函数名为“”initializeDMP“”。这是一个可以覆盖的弱功能
例如，如果您想更改采样率
（参见[示例10](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/examples/Arduino/Example10_DMP_FastMultipleSensors/Example10_DMP_FastMultipleSensors.ino)详细信息）。
```initializeDMP“”为您完成了大部分繁重的工作：它下载DMP固件；并且用适当的值配置所有寄存器。唯一的东西
您需要手动执行以下操作：选择要启用的DMP传感器；重置并启动FIFO和DMP。有关更多详细信息，请参阅修订后的DMP示例。
*/
#endif