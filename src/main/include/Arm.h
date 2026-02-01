//---------------------------------------------------------------------------
//
// MainStateMachine.cpp - Main State Machine Class Implementation.
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 31-Jan-26    THN     added motor systems
//
//---------------------------------------------------------------------------
#pragma once

#include <frc/Timer.h>

#include "RobotIO.h"

namespace arm
{
    enum eState
    {
        STATE_START = 0,
        STATE_IDLE = 1,
        STATE_HOMING = 2,
        STATE_MANUAL_RAISE = 3,
        STATE_MANUAL_LOWER = 4,
        STATE_AUTO_RAISE = 5,
        STATE_ERROR = 99
    };

    enum eCommand
    {
        COMMAND_NONE,
        COMMAND_HOME,
        COMMAND_MANUAL_RAISE,
        COMMAND_MANUAL_LOWER,
        COMMAND_AUTO_RAISE,
        COMMAND_STOP
    };

    // Timeout Constants
    static constexpr double dHomingTimeout = 10.0;
    static constexpr double dManualRaiseTimeout = 15.0;
    static constexpr double dManualLowerTimeout = 15.0;
    static constexpr double dAutoRaiseTimeout = 10.0;

    // Motor Speed Constants
    static constexpr double dHomingSpeed = -0.25;
    static constexpr double dManualRaiseSpeed = 0.20;
    static constexpr double dManualLowerSpeed = -0.20;
    static constexpr double dAutoRaiseSpeed = 0.25;

    // Setpoint Constants
    static constexpr double dAutoRaiseSetpoint = 15.0;
}

class Arm
{
public:

    // Constructor or Destructor
    Arm();
    ~Arm()
        {  }

    // Accessor Methods
    inline void Home()
        {  m_eCommand = arm::COMMAND_HOME;  }
    inline void ManualRaise()
        {  m_eCommand = arm::COMMAND_MANUAL_RAISE;  }
    inline void ManualLower()
        {  m_eCommand = arm::COMMAND_MANUAL_LOWER;  }
    inline void AutoRaise()
        {  m_eCommand = arm::COMMAND_AUTO_RAISE;  }
    inline void Stop()
        {  m_eCommand = arm::COMMAND_STOP;  }

    inline bool IsIdle()
        { return(m_eState == arm::eState::STATE_IDLE); }
    inline bool IsHoming()
        { return(m_eState == arm::eState::STATE_HOMING); }
    inline bool IsManualRaising()
        { return(m_eState == arm::eState::STATE_MANUAL_RAISE); }
    inline bool IsManualLowering()
        { return(m_eState == arm::eState::STATE_MANUAL_LOWER); }
    inline bool IsAutoRaising()
        { return(m_eState == arm::eState::STATE_AUTO_RAISE); }

    // Class Methods
    void Initialize( RobotIO *p_pRobotIO );
    void Execute();
    void UpdateInputStatus();
    
private:
    arm::eState m_eState;
    arm::eCommand m_eCommand;

    RobotIO *m_pRobotIO;

    frc::Timer *m_pTimeoutTimer;

    configs::MotorOutputConfigs m_MotorConfigs;
};