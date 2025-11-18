/**
 * @file bfSensorAlignment.hpp
 * 
 * Betaflight 风格传感器对齐工具类
 * 从 gyro.c 和 cmdImuCalGeneral.c 抽取的传感器旋转逻辑
 */

#pragma once

#include <cstdint>

// 传感器对齐枚举（从 Betaflight 抽取）
enum class BfSensorAlignment : uint8_t {
    CW0_DEG = 0,
    CW90_DEG = 1,
    CW180_DEG = 2,
    CW270_DEG = 3,
    CW0_DEG_FLIP = 4,
    CW90_DEG_FLIP = 5,
    CW180_DEG_FLIP = 6,
    CW270_DEG_FLIP = 7,
    ALIGN_DEFAULT = CW0_DEG
};

// 传感器对齐工具类
class BfSensorAlignmentUtil {
public:
    /**
     * @brief 通过旋转矩阵对齐传感器数据
     * @param data 输入/输出的传感器数据 [x, y, z]
     * @param matrix 3x3 旋转矩阵
     */
    static void alignViaMatrix(float data[3], const float matrix[3][3]) {
        const float x = data[0];
        const float y = data[1];
        const float z = data[2];
        
        data[0] = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z;
        data[1] = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z;
        data[2] = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z;
    }
    
    /**
     * @brief 通过旋转枚举对齐传感器数据
     * @param data 输入/输出的传感器数据 [x, y, z]
     * @param rotation 旋转类型
     */
    static void alignViaRotation(float data[3], BfSensorAlignment rotation) {
        const float x = data[0];
        const float y = data[1];
        const float z = data[2];
        
        switch (rotation) {
            default:
            case BfSensorAlignment::CW0_DEG:
                data[0] = x;
                data[1] = y;
                data[2] = z;
                break;
                
            case BfSensorAlignment::CW90_DEG:
                data[0] = y;
                data[1] = -x;
                data[2] = z;
                break;
                
            case BfSensorAlignment::CW180_DEG:
                data[0] = -x;
                data[1] = -y;
                data[2] = z;
                break;
                
            case BfSensorAlignment::CW270_DEG:
                data[0] = -y;
                data[1] = x;
                data[2] = z;
                break;
                
            case BfSensorAlignment::CW0_DEG_FLIP:
                data[0] = -x;
                data[1] = y;
                data[2] = -z;
                break;
                
            case BfSensorAlignment::CW90_DEG_FLIP:
                data[0] = y;
                data[1] = x;
                data[2] = -z;
                break;
                
            case BfSensorAlignment::CW180_DEG_FLIP:
                data[0] = x;
                data[1] = -y;
                data[2] = -z;
                break;
                
            case BfSensorAlignment::CW270_DEG_FLIP:
                data[0] = -y;
                data[1] = -x;
                data[2] = -z;
                break;
        }
    }
};

