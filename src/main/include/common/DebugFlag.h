#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/GenericEntry.h>

class DebugFlag
{
public:
    DebugFlag(const char* name, bool defaultVal);

    operator bool() { return m_netTableEntry->GetBoolean(m_defaultVal); }

private:
    nt::GenericEntry*       m_netTableEntry;
    bool                    m_defaultVal;

    static int              m_flagCount;
};