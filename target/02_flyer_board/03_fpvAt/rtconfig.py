import os

# 编译配置选项
# 将关键警告视为错误；如需放宽警告，设为 False
TREAT_WARNINGS_AS_ERRORS = True

# toolchains options
ARCH='arm'
CPU='cortex-m4'
CROSS_TOOL='gcc'
PLATFORM='gcc'

# bsp lib config
BSP_LIBRARY_TYPE = None

# 编译器执行路径（可通过环境变量 RTT_EXEC_PATH 覆盖）
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

CXXFLAGS = CFLAGS

# 警告控制选项（与 fmuV2 对齐）
if TREAT_WARNINGS_AS_ERRORS:
    # C 编译器基础警告视为错误
    CFLAGS += ' -Werror=implicit-function-declaration -Werror=return-type -Werror=uninitialized -Werror=unused-variable -Werror=unused-function -Wall'
    # C++ 编译器对应控制
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
