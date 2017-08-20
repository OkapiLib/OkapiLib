import os

def AllSources(node='.', pattern='*'):
    result = [AllSources(dir, pattern)
              for dir in Glob(str(node)+'/*')
              if dir.isdir()]
    result += [source
               for source in Glob(str(node)+'/'+pattern)
               if source.isfile()]
    return result

path = ['~/gcc-arm-none-eabi-4_9-2014q4/bin', '/bin', '/usr/bin']
env = Environment(ENV = os.environ, CPPPATH = 'include')

mcuPrefix = ARGUMENTS.get("mcuPrefix", "arm-none-eabi-")
asFlags = "-mthumb -mcpu=cortex-m3 -mlittle-endian "
compFlags = "-mthumb -mcpu=cortex-m3 -mlittle-endian -mfloat-abi=soft "
linkFlags = "-nostartfiles -Wl,-static -Bfirmware -Wl,-u,VectorTable -Wl,-T -Xlinker firmware/cortex.ld "
ccFlags = "-c -Wall -Wextra -Wpedantic " + linkFlags + "-Os -ffunction-sections -fsigned-char -fomit-frame-pointer -fsingle-precision-constant "

env.Replace(AS = mcuPrefix + "as")
env.Replace(LINK = mcuPrefix + "g++")
env.Replace(CC = mcuPrefix + "gcc")
env.Replace(CXX = mcuPrefix + "g++")

env.Append(ASFLAGS = asFlags)
env.Append(LINKFLAGS = "-Wall " + compFlags + linkFlags + "-Wl,--gc-sections firmware/libpros.a")
env.Append(CFLAGS = ccFlags + "-std=gnu99 -Werror=implicit-function-declaration")
env.Append(CXXFLAGS = ccFlags + "-std=c++14 -fno-exceptions -fno-rtti -felide-constructors -fno-use-cxa-atexit")

env.Program('program', source=AllSources('src', '*.c*'))
