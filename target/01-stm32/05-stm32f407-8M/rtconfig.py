import os

# 编译配置选项
# 将基础警告视为错误，保持与 f427 一致
TREAT_WARNINGS_AS_ERRORS = True

# toolchains options
ARCH='arm'
CPU='cortex-m4'
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

DEVICE = ' -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections'
CFLAGS = DEVICE + ' -Dgcc'
AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb '
LFLAGS = DEVICE + ' -Wl,--gc-sections,-Map=rt-thread.map,-cref,-u,Reset_Handler -T board/linker_scripts/link.lds'

CPATH = ''
LPATH = ''

if BUILD == 'debug':
    CFLAGS += ' -O0 -gdwarf-2 -g'
    AFLAGS += ' -gdwarf-2'
else:
    CFLAGS += ' -O2'

# 定义 CXXFLAGS
CXXFLAGS = CFLAGS 

# 警告控制选项，匹配 f427
if TREAT_WARNINGS_AS_ERRORS:
    CFLAGS += ' -Werror=implicit-function-declaration -Werror=return-type -Werror=uninitialized -Werror=unused-variable -Werror=unused-function -Wall'
    CXXFLAGS += ' -Werror=return-type -Werror=uninitialized -Werror=unused-variable -Werror=unused-function -Wall'
else:
    CFLAGS += ' -Wall'
    CXXFLAGS += ' -Wall'

POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' + SIZE + ' $TARGET \n'

def dist_handle(BSP_ROOT, dist_dir):
    import sys
    cwd_path = os.getcwd()
    sys.path.append(os.path.join(os.path.dirname(BSP_ROOT), 'tools'))
    from sdk_dist import dist_do_building
    dist_do_building(BSP_ROOT, dist_dir)
