//---------------------------------------------------------------------------
//
// MainStateMachine.cpp - Main State Machine Class Implementation.
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 31-Jan-26    THN     Copied mostly from Arm.h with extra comments
//
//---------------------------------------------------------------------------
#pragma once

#include <frc/Timer.h>

#include "RobotIO.h"
 //starting code ^^

 namespace motor // The inital motor namespace, following the namespace convention of uncapitalized Arm
 {
    enum eState // Motor::eState
        {
        STATE_START = 0,
        STATE_IDLE = 1,
        STATE_HOMING = 2,
        STATE_MANUAL_FORWARD = 3,
        STATE_MANUAL_REVERSE = 4,
        STATE_AUTO_FORWARD = 5,
        STATE_ERROR = 99 
            // Copied the same naming style to ensure clarity from Arm
        };
        
    enum eCommand //commands for the motor
    {
        COMMAND_NONE,
        COMMAND_HOME,
        COMMAND_IDLE,
        COMMAND_MANUAL_FORWARD,
        COMMAND_MANUAL_REVERSE,
        COMMAND_AUTO_FORWARD,
        COMMAND_STOP
    };

    // Timeout Constants should be changed depending on the motor type
    static constexpr double dHomingTimeout = 10.0; 
    static constexpr double dManualForwardTimeout = 15.0;
    static constexpr double dManualReverseTimeout = 15.0;
    static constexpr double dAutoForwardTimeout = 10.0;

    // Motor Speed Constants
    static constexpr double dHomingSpeed = -0.25;
    static constexpr double dManualForwardSpeed = 0.20;
    static constexpr double dManualReverseSpeed = -0.20;
    static constexpr double dAutoForwardSpeed = 0.25;

    // Setpoint Constants
    static constexpr double dAutoForwardSetpoint = 10.0;  //10 rotations, and this is in position NOT counts
        /* 
            How many turns/counts Motor takes until setpoint, only for auto forward
        */
}

class Motor
{
public:

    // Constructor or Destructor
    Motor();//constructor
    ~Motor()//destructor
        {  }

    // Accessor Methods which when called set the command to home
    //no idle state command, In lessons this isnt implemented but I elect to do so.
    inline void Idle()
        {  m_eCommand = motor::COMMAND_IDLE} // depending on how useful it is i might just remove this
    inline void Home()
        {  m_eCommand = motor::COMMAND_HOME;  }
    inline void ManualForward()
        {  m_eCommand = motor::COMMAND_MANUAL_FORWARD;  }
    inline void ManualReverse()
        {  m_eCommand = motor::COMMAND_MANUAL_REVERSE;  }
    inline void AutoForward()
        {  m_eCommand = motor::COMMAND_AUTO_FORWARD;  }
    inline void Stop()
        {  m_eCommand = motor::COMMAND_STOP;  }


        // checks if the state is in the current state
    inline bool IsIdle()
        { return(m_eState == motor::eState::STATE_IDLE); }
    inline bool IsHoming()
        { return(m_eState == motor::eState::STATE_HOMING); }
    inline bool IsManualForwarding()
        { return(m_eState == motor::eState::STATE_MANUAL_FORWARD); }
    inline bool IsManualReversing()
        { return(m_eState == motor::eState::STATE_MANUAL_REVERSE); }
    inline bool IsAutoForwarding()
        { return(m_eState == motor::eState::STATE_AUTO_FORWARD); }

    // Class Methods
    void Initialize( RobotIO *p_pRobotIO ); //points to robot IO and Initializes variables
    void Execute(); //called every 20ms
    void UpdateInputStatus();
    
private:
    motor::eState m_eState;
    motor::eCommand m_eCommand;

    RobotIO *m_pRobotIO; //robot io pointer

    frc::Timer *m_pTimeoutTimer;

    configs::MotorOutputConfigs m_MotorConfigs;
};







 