#include "Arm.h"

// Constructor
Arm::Arm()
{
    m_eState = arm::eState::STATE_START;
    m_eCommand = arm::COMMAND_NONE;

    m_pRobotIO = nullptr;
}

void Arm::Initialize( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO;

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();

  
}

void Arm::UpdateInputStatus()
{

}

void Arm::Execute()
{
    //Check if m_pRobotIO assigned
    if(m_pRobotIO != nullptr)
    {
        // ***************
        // * Start State *
        // ***************
        if(m_eState == arm::eState::STATE_START)
        {
            m_eState = arm::eState::STATE_IDLE;
        }

        // **************
        // * Idle State *
        // **************
        if(m_eState == arm::eState::STATE_IDLE)
        {
            // *--------------*
            // * Home Command *
            // *--------------*
            if(m_eCommand == arm::COMMAND_HOME)
            {
                if( m_pRobotIO->GetArmLimit() )
                {
                    m_eCommand = arm::COMMAND_NONE;
                    return; //sees if command is home and if yes then if its
                    //at home already return nothing. and no command
                }

                //Apply motor configs
                m_MotorConfigs.NeutralMode = NeutralMode::Coast;
                m_pRobotIO->m_ArmMotor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed
                m_pRobotIO->m_ArmMotor.Set( arm::dHomingSpeed );

                //Transition to homing state
                m_eState = arm::eState::STATE_HOMING;
            }

            // *----------------------*
            // * Manual Raise Command *
            // *----------------------*
            else if(m_eCommand == arm::COMMAND_MANUAL_RAISE)
            {
                //Apply motor configs
                m_MotorConfigs.NeutralMode = NeutralMode::Coast;
                m_pRobotIO->m_ArmMotor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed
                m_pRobotIO->m_ArmMotor.Set( arm::dManualRaiseSpeed );

                //Transition to manual raise state
                m_eState = arm::eState::STATE_MANUAL_RAISE;
            }

            // *----------------------*
            // * Manual Lower Command *
            // *----------------------*
            else if(m_eCommand == arm::COMMAND_MANUAL_LOWER)
            {
                if(m_pRobotIO->GetArmLimit())
                {
                    m_eCommand = arm::COMMAND_NONE;
                    return;
                }

                //Apply motor configs
                m_MotorConfigs.NeutralMode = NeutralMode::Coast;
                m_pRobotIO->m_ArmMotor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed
                m_pRobotIO->m_ArmMotor.Set( arm::dManualLowerSpeed );

                //Transition to manual lower state
                m_eState = arm::eState::STATE_MANUAL_LOWER;
            }

            // *--------------------*
            // * Auto Raise Command *
            // *--------------------*
            else if(m_eCommand == arm::COMMAND_AUTO_RAISE)
            {
                if( m_pRobotIO->m_ArmMotor.GetPosition().GetValueAsDouble() >= arm::dAutoRaiseSetpoint )
                {
                    m_eCommand = arm::COMMAND_NONE;
                    return;
                }

                //Apply motor configs
                m_MotorConfigs.NeutralMode = NeutralMode::Coast;
                m_pRobotIO->m_ArmMotor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed
                m_pRobotIO->m_ArmMotor.Set( arm::dAutoRaiseSpeed );

                //Transition to auto raise state
                m_eState = arm::eState::STATE_AUTO_RAISE;
            }

            // Error - unrecognized command
            else if(m_eCommand != arm::COMMAND_NONE && m_eCommand != arm::COMMAND_STOP)
            {
                printf("ERROR in Arm.cpp - Unknown command\n");
            }
        }

        // ****************
        // * Homing State *
        // ****************
        else if(m_eState == arm::eState::STATE_HOMING)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= arm::dHomingTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_pRobotIO->GetArmLimit() || bIsTimedOut)
            {
                //Stop Motors
                m_pRobotIO->m_ArmMotor.Set(0);

                //Enable brake mode
                m_MotorConfigs.NeutralMode = NeutralMode::Brake;
                m_pRobotIO->m_ArmMotor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset Encoders
                m_pRobotIO->m_ArmMotor.SetPosition(units::angle::turn_t{0});

                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }

        // **********************
        // * Manual Raise State *
        // **********************
        else if(m_eState == arm::eState::STATE_MANUAL_RAISE)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= arm::dManualRaiseTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_eCommand == arm::COMMAND_STOP || bIsTimedOut)
            {
                //Stop Motors
                m_pRobotIO->m_ArmMotor.Set(0);

                //Enable brake mode
                m_MotorConfigs.NeutralMode = NeutralMode::Brake;
                m_pRobotIO->m_ArmMotor.GetConfigurator().Apply(m_MotorConfigs);

                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }
        
        // **********************
        // * Manual Lower State *
        // **********************
        else if(m_eState == arm::eState::STATE_MANUAL_LOWER)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= arm::dManualLowerTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_eCommand == arm::COMMAND_STOP || m_pRobotIO->GetArmLimit() || bIsTimedOut)
            {
                //Stop Motors
                m_pRobotIO->m_ArmMotor.Set(0);

                //Enable brake mode
                m_MotorConfigs.NeutralMode = NeutralMode::Brake;
                m_pRobotIO->m_ArmMotor.GetConfigurator().Apply(m_MotorConfigs);

                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }

        // ********************
        // * Auto Raise State *
        // ********************
        else if(m_eState == arm::eState::STATE_AUTO_RAISE)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= arm::dAutoRaiseTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_pRobotIO->m_ArmMotor.GetPosition().GetValueAsDouble() >= arm::dAutoRaiseSetpoint || bIsTimedOut)
            {
                //Stop Motors
                m_pRobotIO->m_ArmMotor.Set(0);

                //Enable brake mode
                m_MotorConfigs.NeutralMode = NeutralMode::Brake;
                m_pRobotIO->m_ArmMotor.GetConfigurator().Apply(m_MotorConfigs);

                m_eCommand = arm::COMMAND_NONE;
                m_eState = arm::eState::STATE_IDLE;
            }
        }

        // Handle Error State or unknown state
        else
        {
            // Error Logic Here if needed
            printf("ERROR in Arm.cpp - Error or unknown state\n");
        }
    }
    else
    {
        // Handle RobotIO nullptr error
        printf("ERROR in Arm.cpp - Robot IO Pointer Null\n");
    }
}