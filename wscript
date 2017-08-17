# kate: indent-mode python; hl python;
from waflib.TaskGen import feature, before_method
import subprocess

LIBOPENCM3_DIR = 'modules/libopencm3'

def options(ctx):
    ctx.load('compiler_c')
    ctx.add_option('--use_lto', action='store', default=False, help='Use link-time optimization')

def configure(ctx):
    ctx.env.CC = 'arm-none-eabi-gcc'
    ctx.env.AR = 'arm-none-eabi-ar'
    ctx.load('compiler_c')
    ctx.env.USE_LTO = ctx.options.use_lto

def build(ctx):
    bld = ctx

    ctx.env.append_value('CFLAGS', '-std=gnu11 -ffast-math -g -Wdouble-promotion -Wextra -Wshadow -Werror=implicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fsingle-precision-constant -fno-common -ffunction-sections -fdata-sections -Wall -Wundef -fshort-wchar'.split(' '))
    if ctx.env.USE_LTO:
        ctx.env.append_value('CFLAGS', '-flto')
        ctx.env.append_value('LDFLAGS', '-flto')
        LIBOPENCM3_MAKE_ARGS = 'CFLAGS="-fshort-wchar -flto" LDFLAGS="-flto" AR="arm-none-eabi-gcc-ar" -j8'
    else:
        LIBOPENCM3_MAKE_ARGS = 'CFLAGS="-fshort-wchar"'

    ctx.env.append_value('CFLAGS', ['-include', ctx.path.find_node('boards/com.proficnc.gnss_1.0/board.h').abspath()])

    bld.make(LIBOPENCM3_DIR, LIBOPENCM3_MAKE_ARGS)
    ctx.env.append_value('STLIBPATH_LIBOPENCM3', ctx.path.find_dir('%s/lib' % (LIBOPENCM3_DIR,)).abspath())
    ctx.env.append_value('INCLUDES_LIBOPENCM3', ctx.path.find_dir('%s/include' % (LIBOPENCM3_DIR,)).abspath())
    ctx.env.append_value('STLIB_LIBOPENCM3_STM32F3', 'libopencm3_stm32f3')
    ctx.env.append_value('CFLAGS_LIBOPENCM3_STM32F3', '-DSTM32F3')

    bld.stlib(source = ctx.path.ant_glob('modules/libcanard/*.c'), target = 'libcanard', export_includes = 'modules/libcanard')

    bld.stlib(source = ctx.path.ant_glob('modules/common/src/*.c'), target = 'common', includes = 'modules/common/include', export_includes = 'modules/common/include', use = 'LIBOPENCM3 LIBOPENCM3_STM32F3 libcanard')

    #bld.program(use = 'LIBOPENCM3 LIBOPENCM3_STM32F3 libcanard common')

from waflib.Configure import conf
@conf
def make(ctx, directory, options = ''):
    source = ctx.path.find_dir(directory)
    print 'make -C %s clean %s' % (source.abspath(), options)
    ctx.exec_command('make -C %s clean %s' % (source.abspath(), options))
    print 'make -C %s %s' % (source.abspath(), options)
    ctx.exec_command('make -C %s %s' % (source.abspath(), options))
