import os

# 编译配置选项
# 设置为 True 时，将重要的基础警告视为错误；设置为 False 时，警告不会阻止编译
# 
# 警告控制说明：
# C 编译器特有：
# -Werror=implicit-function-declaration: 隐式函数声明警告转为错误（防止调用未声明的函数）
# 
# C/C++ 通用：
# -Werror=return-type: 函数返回值类型警告转为错误（防止函数返回值不匹配）
# -Werror=uninitialized: 未初始化变量警告转为错误（防止使用未初始化的变量）
# -Werror=unused-variable: 未使用变量警告转为错误（防止定义未使用的变量）
# -Werror=unused-function: 未使用函数警告转为错误（防止定义未使用的函数）
# -Wall: 启用大部分常用警告（但不包括过于严格的警告）
TREAT_WARNINGS_AS_ERRORS = True

# toolchains options
ARCH='arm'
CPU='cortex-m7'
CROSS_TOOL='gcc'
PLATFORM='gcc'

# bsp lib config
BSP_LIBRARY_TYPE = None

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')
if os.getenv('RTT_ROOT'):
    RTT_ROOT = os.getenv('RTT_ROOT')

# cross_tool provides the cross compiler
# EXEC_PATH is the compiler execute path
if CROSS_TOOL == 'gcc':
    EXEC_PATH = r'C:\Users\XXYYZZ'
else:
    EXEC_PATH = r'C:\Users\XXYYZZ'

if os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

BUILD = 'debug'

# GCC toolchain configuration
PREFIX = 'arm-none-eabi-'
CC = PREFIX + 'gcc'
AS = PREFIX + 'gcc'
AR = PREFIX + 'ar'
CXX = PREFIX + 'g++'
LINK = PREFIX + 'gcc'
TARGET_EXT = 'elf'
SIZE = PREFIX + 'size'
OBJDUMP = PREFIX + 'objdump'
OBJCPY = PREFIX + 'objcopy'

DEVICE = ' -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections'
CFLAGS = DEVICE + ' -Dgcc'
AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb '
LFLAGS = DEVICE + ' -Wl,--gc-sections,-Map=rtthread.map,-cref,-u,Reset_Handler -T board/linker_scripts/link.lds'

CPATH = ''
LPATH = ''

if BUILD == 'debug':
    CFLAGS += ' -O0 -gdwarf-2 -g'
    AFLAGS += ' -gdwarf-2'
else:
    CFLAGS += ' -O2'

# 定义 CXXFLAGS
CXXFLAGS = CFLAGS

# 警告控制选项
if TREAT_WARNINGS_AS_ERRORS:
    # C 编译器：基础警告视为错误
    CFLAGS += ' -Werror=implicit-function-declaration -Werror=return-type -Werror=uninitialized -Werror=unused-variable -Werror=unused-function -Wall'
    # C++ 编译器：C++ 特有的警告控制（不支持隐式函数声明警告）
    CXXFLAGS += ' -Werror=return-type -Werror=uninitialized -Werror=unused-variable -Werror=unused-function -Wall'
else:
    # 普通模式：显示警告但不阻止编译
    CFLAGS += ' -Wall'
    CXXFLAGS += ' -Wall' 

POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' + SIZE + ' $TARGET \n'

def dist_handle(BSP_ROOT, dist_dir):
    import sys
    cwd_path = os.getcwd()
    sys.path.append(os.path.join(os.path.dirname(BSP_ROOT), 'tools'))
    from sdk_dist import dist_do_building
    dist_do_building(BSP_ROOT, dist_dir)
