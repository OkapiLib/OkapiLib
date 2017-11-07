#ifndef OKAPI_PAL
#define OKAPI_PAL

#include <API.h>

namespace okapi {
    class PAL {
    public:
        __attribute__((always_inline))
        static bool isAutonomous() { return ::isAutonomous(); }
        __attribute__((always_inline))
        static bool isEnabled() { return ::isEnabled(); }
        __attribute__((always_inline))
        static bool isJoystickConnected(unsigned char joystick) { return ::isJoystickConnected(joystick); }
        __attribute__((always_inline))
        static bool isOnline() { return ::isOnline(); }
        __attribute__((always_inline))
        static int joystickGetAnalog(unsigned char joystick, unsigned char axis) { return ::joystickGetAnalog(joystick, axis); }
        __attribute__((always_inline))
        static bool joystickGetDigital(unsigned char joystick, unsigned char buttonGroup,unsigned char button) { return ::joystickGetDigital(joystick, buttonGroup, button); }
        __attribute__((always_inline))
        static unsigned int powerLevelBackup() { return ::powerLevelBackup(); }
        __attribute__((always_inline))
        static unsigned int powerLevelMain() { return ::powerLevelMain(); }
        __attribute__((always_inline))
        static void setTeamName(const char *name) { ::setTeamName(name); }
        __attribute__((always_inline))
        static int analogCalibrate(unsigned char channel) { return ::analogCalibrate(channel); }
        __attribute__((always_inline))
        static int analogRead(unsigned char channel) { return ::analogRead(channel); }
        __attribute__((always_inline))
        static int analogReadCalibrated(unsigned char channel) { return ::analogReadCalibrated(channel); }
        __attribute__((always_inline))
        static int analogReadCalibratedHR(unsigned char channel) { return ::analogReadCalibratedHR(channel); }
        __attribute__((always_inline))
        static bool digitalRead(unsigned char pin) { return ::digitalRead(pin); }
        __attribute__((always_inline))
        static void digitalWrite(unsigned char pin, bool value) { ::digitalWrite(pin, value); }
        __attribute__((always_inline))
        static void pinMode(unsigned char pin, unsigned char mode) { ::pinMode(pin, mode); }
        __attribute__((always_inline))
        static void ioClearInterrupt(unsigned char pin) { ::ioClearInterrupt(pin); }
        __attribute__((always_inline))
        static void ioSetInterrupt(unsigned char pin, unsigned char edges, InterruptHandler handler) { ::ioSetInterrupt(pin, edges, handler); }
        __attribute__((always_inline))
        static int motorGet(unsigned char channel) { return ::motorGet(channel); }
        __attribute__((always_inline))
        static void motorSet(unsigned char channel, int speed) { ::motorSet(channel, speed); }
        __attribute__((always_inline))
        static void motorStop(unsigned char channel) { ::motorStop(channel); }
        __attribute__((always_inline))
        static void motorStopAll() { ::motorStopAll(); }
        __attribute__((always_inline))
        static void speakerInit() { ::speakerInit(); }
        __attribute__((always_inline))
        static void speakerPlayArray(const char * * songs) { ::speakerPlayArray(songs); }
        __attribute__((always_inline))
        static void speakerPlayRtttl(const char *song) { ::speakerPlayRtttl(song); }
        __attribute__((always_inline))
        static void speakerShutdown() { ::speakerShutdown(); }
        __attribute__((always_inline))
        static unsigned int imeInitializeAll() { return ::imeInitializeAll(); }
        __attribute__((always_inline))
        static bool imeGet(unsigned char address, int *value) { return ::imeGet(address, value); }
        __attribute__((always_inline))
        static bool imeGetVelocity(unsigned char address, int *value) { return ::imeGetVelocity(address, value); }
        __attribute__((always_inline))
        static bool imeReset(unsigned char address) { return ::imeReset(address); }
        __attribute__((always_inline))
        static void imeShutdown() { ::imeShutdown(); }
        __attribute__((always_inline))
        static int gyroGet(Gyro gyro) { return ::gyroGet(gyro); }
        __attribute__((always_inline))
        static Gyro gyroInit(unsigned char port, unsigned short multiplier) { return ::gyroInit(port, multiplier); }
        __attribute__((always_inline))
        static void gyroReset(Gyro gyro) { ::gyroReset(gyro); }
        __attribute__((always_inline))
        static void gyroShutdown(Gyro gyro) { ::gyroShutdown(gyro); }
        __attribute__((always_inline))
        static int encoderGet(Encoder enc) { return ::encoderGet(enc); }
        __attribute__((always_inline))
        static Encoder encoderInit(unsigned char portTop, unsigned char portBottom, bool reverse) { return ::encoderInit(portTop, portBottom, reverse); }
        __attribute__((always_inline))
        static void encoderReset(Encoder enc) { ::encoderReset(enc); }
        __attribute__((always_inline))
        static void encoderShutdown(Encoder enc) { ::encoderShutdown(enc); }
        __attribute__((always_inline))
        static int ultrasonicGet(Ultrasonic ult) { return ::ultrasonicGet(ult); }
        __attribute__((always_inline))
        static Ultrasonic ultrasonicInit(unsigned char portEcho, unsigned char portPing) { return ::ultrasonicInit(portEcho, portPing); }
        __attribute__((always_inline))
        static void ultrasonicShutdown(Ultrasonic ult) { ::ultrasonicShutdown(ult); }
        __attribute__((always_inline))
        static bool i2cRead(uint8_t addr, uint8_t *data, uint16_t count) { return ::i2cRead(addr, data, count); }
        __attribute__((always_inline))
        static bool i2cReadRegister(uint8_t addr, uint8_t reg, uint8_t *value, uint16_t count) { return ::i2cReadRegister(addr, reg, value, count); }
        __attribute__((always_inline))
        static bool i2cWrite(uint8_t addr, uint8_t *data, uint16_t count) { return ::i2cWrite(addr, data, count); }
        __attribute__((always_inline))
        static bool i2cWriteRegister(uint8_t addr, uint8_t reg, uint16_t value) { return ::i2cWriteRegister(addr, reg, value); }
        __attribute__((always_inline))
        static void usartInit(PROS_FILE *usart, unsigned int baud, unsigned int flags) { ::usartInit(usart, baud, flags); }
        __attribute__((always_inline))
        static void usartShutdown(PROS_FILE *usart) { ::usartShutdown(usart); }
        __attribute__((always_inline))
        static void fclose(PROS_FILE *stream) { ::fclose(stream); }
        __attribute__((always_inline))
        static int fcount(PROS_FILE *stream) { return ::fcount(stream); }
        __attribute__((always_inline))
        static int fdelete(const char *file) { return ::fdelete(file); }
        __attribute__((always_inline))
        static int feof(PROS_FILE *stream) { return ::feof(stream); }
        __attribute__((always_inline))
        static int fflush(PROS_FILE *stream) { return ::fflush(stream); }
        __attribute__((always_inline))
        static int fgetc(PROS_FILE *stream) { return ::fgetc(stream); }
        __attribute__((always_inline))
        static char* fgets(char *str, int num, PROS_FILE *stream) { return ::fgets(str, num, stream); }
        __attribute__((always_inline))
        static PROS_FILE * fopen(const char *file, const char *mode) { return ::fopen(file, mode); }
        __attribute__((always_inline))
        static void fprint(const char *string, PROS_FILE *stream) { ::fprint(string, stream); }
        __attribute__((always_inline))
        static int fputc(int value, PROS_FILE *stream) { return ::fputc(value, stream); }
        __attribute__((always_inline))
        static int fputs(const char *string, PROS_FILE *stream) { return ::fputs(string, stream); }
        __attribute__((always_inline))
        static size_t fread(void *ptr, size_t size, size_t count, PROS_FILE *stream) { return ::fread(ptr, size, count, stream); }
        __attribute__((always_inline))
        static int fseek(PROS_FILE *stream, long int offset, int origin) { return ::fseek(stream, offset, origin); }
        __attribute__((always_inline))
        static long int ftell(PROS_FILE *stream) { return ::ftell(stream); }
        __attribute__((always_inline))
        static size_t fwrite(const void *ptr, size_t size, size_t count, PROS_FILE *stream) { return ::fwrite(ptr, size, count, stream); }
        __attribute__((always_inline))
        static int getchar() { return ::getchar(); }
        __attribute__((always_inline))
        static void print(const char *string) { ::print(string); }
        __attribute__((always_inline))
        static int putchar(int value) { return ::putchar(value); }
        __attribute__((always_inline))
        static int puts(const char *string) { return ::puts(string); }
        // __attribute__((always_inline))
        // static int fprintf(PROS_FILE *stream, const char *formatString, ...);
        // __attribute__((always_inline))
        // static int printf(const char *formatString, ...);
        // __attribute__((always_inline))
        // static int snprintf(char *buffer, size_t limit, const char *formatString, ...);
        // __attribute__((always_inline))
        // static int sprintf(char *buffer, const char *formatString, ...);
        __attribute__((always_inline))
        static void lcdClear(PROS_FILE *lcdPort) { ::lcdClear(lcdPort); }
        __attribute__((always_inline))
        static void lcdInit(PROS_FILE *lcdPort) { ::lcdInit(lcdPort); }
        __attribute__((always_inline))
        static unsigned int lcdReadButtons(PROS_FILE *lcdPort) { return ::lcdReadButtons(lcdPort); }
        __attribute__((always_inline))
        static void lcdSetBacklight(PROS_FILE *lcdPort, bool backlight) { ::lcdSetBacklight(lcdPort, backlight); }
        __attribute__((always_inline))
        static void lcdSetText(PROS_FILE *lcdPort, unsigned char line, const char *buffer) { ::lcdSetText(lcdPort, line, buffer); }
        __attribute__((always_inline))
        static void lcdShutdown(PROS_FILE *lcdPort) { ::lcdShutdown(lcdPort); }
        __attribute__((always_inline))
        static TaskHandle taskCreate(TaskCode taskCode, const unsigned int stackDepth, void *parameters,const unsigned int priority) { return ::taskCreate(taskCode, stackDepth, parameters, priority); }
        __attribute__((always_inline))
        static void taskDelay(const unsigned long msToDelay) { ::taskDelay(msToDelay); }
        __attribute__((always_inline))
        static void taskDelayUntil(unsigned long *previousWakeTime, const unsigned long cycleTime) { ::taskDelayUntil(previousWakeTime, cycleTime); }
        __attribute__((always_inline))
        static void taskDelete(TaskHandle taskToDelete) { ::taskDelete(taskToDelete); }
        __attribute__((always_inline))
        static unsigned int taskGetCount() { return ::taskGetCount(); }
        __attribute__((always_inline))
        static unsigned int taskGetState(TaskHandle task) { return ::taskGetState(task); }
        __attribute__((always_inline))
        static unsigned int taskPriorityGet(const TaskHandle task) { return ::taskPriorityGet(task); }
        __attribute__((always_inline))
        static void taskPrioritySet(TaskHandle task, const unsigned int newPriority) { ::taskPrioritySet(task, newPriority); }
        __attribute__((always_inline))
        static void taskResume(TaskHandle taskToResume) { ::taskResume(taskToResume); }
        __attribute__((always_inline))
        static TaskHandle taskRunLoop(void (*fn)(void), const unsigned long increment) { return ::taskRunLoop(fn, increment); }
        __attribute__((always_inline))
        static void taskSuspend(TaskHandle taskToSuspend) { ::taskSuspend(taskToSuspend); }
        __attribute__((always_inline))
        static Semaphore semaphoreCreate() { return ::semaphoreCreate(); }
        __attribute__((always_inline))
        static bool semaphoreGive(Semaphore semaphore) { return ::semaphoreGive(semaphore); }
        __attribute__((always_inline))
        static bool semaphoreTake(Semaphore semaphore, const unsigned long blockTime) { return ::semaphoreTake(semaphore, blockTime); }
        __attribute__((always_inline))
        static void semaphoreDelete(Semaphore semaphore) { ::semaphoreDelete(semaphore); }
        __attribute__((always_inline))
        static Mutex mutexCreate() { return ::mutexCreate(); }
        __attribute__((always_inline))
        static bool mutexGive(Mutex mutex) { return ::mutexGive(mutex); }
        __attribute__((always_inline))
        static bool mutexTake(Mutex mutex, const unsigned long blockTime) { return ::mutexTake(mutex, blockTime); }
        __attribute__((always_inline))
        static void mutexDelete(Mutex mutex) { ::mutexDelete(mutex); }
        __attribute__((always_inline))
        static void delay(const unsigned long time) { ::delay(time); }
        __attribute__((always_inline))
        static void delayMicroseconds(const unsigned long us) { ::delayMicroseconds(us); }
        __attribute__((always_inline))
        static unsigned long micros() { return ::micros(); }
        __attribute__((always_inline))
        static unsigned long millis() { return ::millis(); }
        __attribute__((always_inline))
        static void wait(const unsigned long time) { ::wait(time); }
        __attribute__((always_inline))
        static void waitUntil(unsigned long *previousWakeTime, const unsigned long time) { ::waitUntil(previousWakeTime, time); }
        __attribute__((always_inline))
        static void watchdogInit() { ::watchdogInit(); }
        __attribute__((always_inline))
        static void standaloneModeEnable() { ::standaloneModeEnable(); }
    };
}

#endif /* end of include guard: OKAPI_PAL */
