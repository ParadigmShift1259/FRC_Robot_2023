
#include "commands/IntakeIngest.h"
#include "Constants.h"

using namespace IntakeConstants;

IntakeIngest::IntakeIngest(IntakeSubsystem& subsystem) 
  : m_intake(subsystem)
{
  AddRequirements({&subsystem});
}

void IntakeIngest::Execute() {
    m_intake.Set(kIngestSpeed);
    m_bRunning = true;
}

bool IntakeIngest::IsFinished()
{
  return m_bRunning;
}

void IntakeIngest::End(bool interrupted) {
    m_bRunning = false;
}