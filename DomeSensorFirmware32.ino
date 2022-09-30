//#define USE_DEBUG
//#define USE_DOME_SENSOR_DEBUG
#include "ReelTwo.h"
#include "drive/DomeSensorRing.h"
#include "core/StringUtils.h"
#include <Preferences.h>

///////////////////////////////////
// CONFIGURABLE OPTIONS
///////////////////////////////////

#define SERIAL_BAUD_RATE                    57600
#define CONSOLE_BUFFER_SIZE                 300
#define COMMAND_BUFFER_SIZE                 256

#define POSITION_RESEND_INTERVAL            1000  // milliseconds
#define TEST_DURATION_SECONDS               60

///////////////////////////////////

#define TXD1_PIN    17
#define RXD1_PIN    16  /* not used */
#define REPORT_SERIAL Serial1

///////////////////////////////////

struct DomeSensorSettings
{
    uint32_t fBaudRate = SERIAL_BAUD_RATE;
};

DomeSensorSettings sSettings;
Preferences sPreferences;

#define PREFERENCE_KEY  "settings"

///////////////////////////////////

static bool sNextCommand;
static bool sProcessing;
static unsigned sPos;
static uint32_t sWaitNextSerialCommand;
static char sBuffer[CONSOLE_BUFFER_SIZE];
static bool sCmdNextCommand;
static char sCmdBuffer[COMMAND_BUFFER_SIZE];
static uint32_t sEndTesting;
static bool sTestOne;
static unsigned sSensorMask;
static bool sVerbose;

///////////////////////////////////

DomeSensorRing sDomePosition;

///////////////////////////////////

void setup()
{
    REELTWO_READY();
#ifndef USE_DEBUG
    Serial.begin(DEFAULT_BAUD_RATE);
#endif
    if (sPreferences.begin("domesensor"))
    {
        DomeSensorSettings settings;
        if (sPreferences.getBytes(PREFERENCE_KEY, &settings, sizeof(settings)) == sizeof(settings))
        {
            Serial.println("Settings Restored");
            sSettings = settings;
        }
        else
        {
            Serial.println("First Time Settings");
            sPreferences.putBytes(PREFERENCE_KEY, &sSettings, sizeof(sSettings));
            if (sPreferences.getBytes(PREFERENCE_KEY, &settings, sizeof(settings)) == sizeof(settings))
            {
                Serial.println("Readback Success");
            }
        }
    }
    else
    {
        Serial.println("Failed to initialize preferences.");
    }
    REPORT_SERIAL.begin(sSettings.fBaudRate, SERIAL_8N1, RXD1_PIN, TXD1_PIN);

    SetupEvent::ready();
}

///////////////////////////////////

void reboot()
{
    Serial.println(F("Restarting..."));
#ifdef ESP32
    sPreferences.end();
    ESP.restart();
#elif defined(REELTWO_AVR)
    void (*resetArduino)() = NULL;
    resetArduino();
#else
    Serial.println(F("Restart not supported."));
#endif
}

///////////////////////////////////

static void updateSettings()
{
    sPreferences.putBytes(PREFERENCE_KEY, &sSettings, sizeof(sSettings));
    Serial.println("Updated");
}

///////////////////////////////////

static void runSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sProcessing = true;
}

static void resetSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sNextCommand = false;
    sProcessing = (sCmdBuffer[0] == ':');
    sPos = 0;
}

bool processDomeRingCommand(const char* cmd)
{
    switch (*cmd++)
    {
        default:
            // No dome ring commands
            break;
    }
    return true;
}

void processConfigureCommand(const char* cmd)
{
    if (startswith(cmd, "#DPZERO"))
    {
        DomeSensorSettings defaultSettings;
        sSettings = defaultSettings;
        updateSettings();
    }
    else if (startswith(cmd, "#DPVERBOSE"))
    {
        bool verbose = (*cmd == '1');
        if (verbose != sVerbose)
        {
            if (verbose)
            {
                Serial.print(F("Verbose mode enabled."));
            }
            else
            {
                Serial.print(F("Verbose mode disabled."));
            }
            sVerbose = verbose;
        }
    }
    else if (startswith(cmd, "#DPCONFIG"))
    {
        Serial.print("BaudRate="); Serial.println(sSettings.fBaudRate);
    }
    else if (startswith(cmd, "#DPTEST"))
    {
        sTestOne = (*cmd == '1');
        if (sTestOne)
        {
            Serial.print("Test one sensor at a time. ");
        }
        else
        {
            Serial.print("Testing. Rotate dome 360 degrees. ");
        }
        Serial.println("Test ends in "+String(TEST_DURATION_SECONDS)+" seconds or when all sensors have registered.");
        sEndTesting = millis() + TEST_DURATION_SECONDS * 1000L;
        sSensorMask = 0;
    }
    else if (startswith(cmd, "#DPBAUD"))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        if (baudrate > 1200 && sSettings.fBaudRate != baudrate)
        {
            sSettings.fBaudRate = baudrate;
            Serial.print("Reboot baud rate: "); Serial.println(sSettings.fBaudRate);
            updateSettings();
        }
    }
    else if (startswith_P(cmd, F("#DPRESTART")))
    {
        reboot();
    }
}

bool processCommand(const char* cmd, bool firstCommand)
{
    sWaitNextSerialCommand = 0;
    if (*cmd == '\0')
        return true;
    if (!firstCommand)
    {
        if (cmd[0] != ':')
        {
            Serial.println("Invalid");
            return false;
        }
        return processDomeRingCommand(cmd+1);
    }
    switch (cmd[0])
    {
        case ':':
            if (cmd[1] == 'D')
                return processDomeRingCommand(cmd+2);
            break;
        case '#':
            processConfigureCommand(cmd);
            return true;
        default:
            Serial.println("Invalid");
            break;
    }
    return false;
}

///////////////////////////////////

static unsigned countChangedBits(unsigned a, unsigned b)
{
    unsigned n = 0;
    for (unsigned i = 0; i < 9; i++) 
    {
        if ((a & (1 << i)) != (b & (1 << i)))
            n++;
    }
    return n;
}

static void printBinary(unsigned num, unsigned places)
{
    if (places)
        printBinary(num >> 1, places-1);
    Serial.print((num & 1) ? '1' : '0');
}

void loop()
{
    static short sLastAngle = -1;
    static uint32_t sLastReport = 0;
    short angle = sDomePosition.getAngle();
    if (sDomePosition.ready())
    {
        // Output the position if it changed or one second has passed
        if (angle != sLastAngle || millis() - sLastReport > POSITION_RESEND_INTERVAL)
        {
            char buf[20];
            snprintf(buf, sizeof(buf), "#DP@%d", angle);
            if (sVerbose)
            {
                Serial.print(F("POS: "));
                Serial.println(angle);
            }
            REPORT_SERIAL.println(buf);
            sLastAngle = angle;
            sLastReport = millis();
        }
    }

    if (sEndTesting > 0)
    {
        unsigned sensors = (~sDomePosition.readSensors() & 0x1FF);
        sSensorMask |= sensors;
        if (sTestOne && countChangedBits(0, sensors) > 1)
        {
            Serial.print("Test failed. These sensors are reading at the same time: ");
            sEndTesting = 0;
            for (unsigned i = 0; i < 9; i++)
            {
                if ((sensors & (1<<i)) != 0)
                {
                    Serial.print(i+1);
                    Serial.print(" ");
                }
            }
            Serial.println();
        }
        if (sSensorMask == 0x1FF)
        {
            Serial.println("All sensors registered. Ending test.");
            sEndTesting = 0;
        }
        else if (sEndTesting < millis())
        {
            Serial.print("Test timeout. Following sensors did not register: ");
            for (unsigned i = 0; i < 9; i++)
            {
                if ((sSensorMask & (1<<i)) == 0)
                {
                    Serial.print(i+1);
                    Serial.print(" ");
                }
            }
            Serial.println();
            sEndTesting = 0;
        }
    }
    // append commands to command buffer
    if (Serial.available())
    {
        int ch = Serial.read();
        if (ch == '\r' || ch == '\n')
        {
            runSerialCommand();
        }
        else if (sPos < SizeOfArray(sBuffer)-1)
        {
            sBuffer[sPos++] = ch;
            sBuffer[sPos] = '\0';
        }
    }
    if (sProcessing && millis() > sWaitNextSerialCommand)
    {
        if (sCmdBuffer[0] == ':')
        {
            char* end = strchr(sCmdBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sCmdBuffer, !sCmdNextCommand))
            {
                // command invalid abort buffer
                Serial.print(F("Unrecognized: ")); Serial.println(sCmdBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sCmdBuffer, end);
                DEBUG_PRINT("REMAINS: ");
                DEBUG_PRINTLN(sCmdBuffer);
                sCmdNextCommand = true;
            }
            else
            {
                sCmdBuffer[0] = '\0';
                sCmdNextCommand = false;
            }
        }
        else if (sBuffer[0] == ':')
        {
            char* end = strchr(sBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sBuffer, !sNextCommand))
            {
                // command invalid abort buffer
                Serial.print("Unrecognized: "); Serial.println(sBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sBuffer, end);
                sPos = strlen(sBuffer);
                DEBUG_PRINT("REMAINS: ");
                DEBUG_PRINTLN(sBuffer);
                sNextCommand = true;
            }
            else
            {
                resetSerialCommand();
                sBuffer[0] = '\0';
            }
        }
        else
        {
            processCommand(sBuffer, true);
            resetSerialCommand();
        }
    }
}
