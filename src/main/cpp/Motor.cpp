#include "Motor.h"

// Constructor
Motor::Motor() // calling the constructor within Motor.h
{
    m_eState = motor::eState::STATE_START;
    m_eCommand = motor::COMMAND_NONE;

    m_pRobotIO = nullptr;
}

void Motor::Initialize( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO; //storing whatever is passed

    m_pTimeoutTimer = new frc::Timer(); //creates a new timer which its adress is storeed in m_pTimeoutTimer
    m_pTimeoutTimer->Reset(); //Go to the Timer object that m_pTimeoutTimer points to, and call Reset() on it

    m_pRobotIO->m_Motor.GetConfigurator().Refresh(m_MotorConfigs); //derived from robot IO
} /// stores the motor motor information within the variable motor config. 
//refresh passes it into the variable
//basically all the configs of the motor will be refreshed and placed into m_Motor, stated in Motor.h
//refreshes probably clears it

void Motor::UpdateInputStatus()
{

}//nothing here for now

void Motor::Execute()
{
    //Check if m_pRobotIO assigned
    if(m_pRobotIO != nullptr)
    {
        // ***************
        // * Start State *
        // ***************
        if(m_eState == motor::eState::STATE_START)
        {
            m_eState = motor::eState::STATE_IDLE;
        }

        // **************
        // * Idle State *
        // **************
        if(m_eState == motor::eState::STATE_IDLE)
        {
            // *--------------*
            // * Home Command *
            // *--------------*
            if(m_eCommand == motor::COMMAND_HOME) 
            {
                if( m_pRobotIO->GetMotorLimit() ) // pr  m_LimitSwitch.Get()
                {
                    m_eCommand = motor::COMMAND_NONE;
                    return; //sees if it is already at its limit if yes then dont do anything
                
                }
                //otherwise do following
                //Apply motor configs
                m_MotorConfigs.NeutralMode = NeutralMode::Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed
                m_pRobotIO->m_Motor.Set( motor::dHomingSpeed );

                //Transition to homing state
                m_eState = motor::eState::STATE_HOMING;
            }

            // *----------------------*
            // * Manual Forward Command *
            // *----------------------*
            else if(m_eCommand == motor::COMMAND_MANUAL_FORWARD)
            {
                 if(m_pRobotIO->GetMotorLimit())
                {
                    m_eCommand = motor::COMMAND_NONE;
                    return;
                }
                //Apply motor configs
                m_MotorConfigs.NeutralMode = NeutralMode::Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed
                m_pRobotIO->m_Motor.Set( motor::dManualForwardSpeed );

                //Transition to manual forward state
                m_eState = motor::eState::STATE_MANUAL_FORWARD;
            }

            // *----------------------*
            // * Manual Reverse Command *
            // *----------------------*
            else if(m_eCommand == motor::COMMAND_MANUAL_REVERSE)
            {
                if(m_pRobotIO->GetMotorLimit())
                {
                    m_eCommand = motor::COMMAND_NONE;
                    return;
                }

                //Apply motor configs
                m_MotorConfigs.NeutralMode = NeutralMode::Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed
                m_pRobotIO->m_Motor.Set( motor::dManualReverseSpeed );

                //Transition to manual reverse state
                m_eState = motor::eState::STATE_MANUAL_REVERSE;
            }

            // *--------------------*
            // * Auto Forward Command *
            // *--------------------*
            else if(m_eCommand == motor::COMMAND_AUTO_FORWARD)
            {
                if( m_pRobotIO->m_Motor.GetPosition().GetValueAsDouble() >= motor::dAutoForwardSetpoint || m_pRobotIO->GetMotorLimit())
                {
                    m_eCommand = motor::COMMAND_NONE;
                    return;
                }

                //Apply motor configs
                m_MotorConfigs.NeutralMode = NeutralMode::Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                //Set Motor Speed
                m_pRobotIO->m_Motor.Set( motor::dAutoForwardSpeed );

                //Transition to auto forward state
                m_eState = motor::eState::STATE_AUTO_FORWARD;
            }

            // Error - unrecognized command
            else if(m_eCommand != motor::COMMAND_NONE && m_eCommand != motor::COMMAND_STOP)
            {
                printf("ERROR in Motor.cpp - Unknown command\n");
            }
        }

        // ****************
        // * Homing State *
        // ****************
        else if(m_eState == motor::eState::STATE_HOMING)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= motor::dHomingTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_pRobotIO->GetMotorLimit() || bIsTimedOut) // if it is at the limit
            {
                //Stop Motors
                m_pRobotIO->m_Motor.StopMotor(); //or StopMotor()

                //Enable brake mode
                m_MotorConfigs.NeutralMode = NeutralMode::Brake; //sets to break
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs); //makes it break by applying

                //Reset Encoders Applying units setting to zero
                m_pRobotIO->m_Motor.SetPosition(units::angle::turn_t{0});

                m_eCommand = motor::COMMAND_NONE;
                m_eState = motor::eState::STATE_IDLE;
            }
        }

        // ************************
        // * Manual forward State *
        // ************************
        else if(m_eState == motor::eState::STATE_MANUAL_FORWARD)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= motor::dManualForwardTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_eCommand == motor::COMMAND_STOP || bIsTimedOut) //forward limit?
            {
                //Stop Motors
                m_pRobotIO->m_Motor.StopMotor();

                //Enable brake mode
                m_MotorConfigs.NeutralMode = NeutralMode::Brake;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                m_eCommand = motor::COMMAND_NONE;
                m_eState = motor::eState::STATE_IDLE;
            }
        }
        
        // ************************
        // * Manual Reverse State *
        // ************************
        else if(m_eState == motor::eState::STATE_MANUAL_REVERSE)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= motor::dManualReverseTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_eCommand == motor::COMMAND_STOP || m_pRobotIO->GetMotorLimit() || bIsTimedOut)
            {
                //Stop Motors
                m_pRobotIO->m_Motor.StopMotor();

                //Enable brake mode
                m_MotorConfigs.NeutralMode = NeutralMode::Brake;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                m_eCommand = motor::COMMAND_NONE;
                m_eState = motor::eState::STATE_IDLE;
            }
        }

        // ********************
        // * Auto forward State *
        // ********************
        else if(m_eState == motor::eState::STATE_AUTO_FORWARD)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= motor::dAutoForwardTimeout)
            {
                bIsTimedOut = true;
            }
//not sure why m_pRobotIO->m_Motor.GetPosition().GetValueAsDouble() wouldnt be in Motor.h for future use under a 
//function that calls it, but the code works and so i wont touch it
            if(m_pRobotIO->m_Motor.GetPosition().GetValueAsDouble() >= motor::dAutoForwardSetpoint || bIsTimedOut)
            {//10 rotations before it stops
                //Stop Motors
                m_pRobotIO->m_Motor.StopMotor();

                //Enable brake mode
                m_MotorConfigs.NeutralMode = NeutralMode::Brake;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                m_eCommand = motor::COMMAND_NONE;
                m_eState = motor::eState::STATE_IDLE;
            }
        }

        // Handle Error State or unknown state
        else
        {
            // Error Logic Here if needed
            printf("ERROR in Motor.cpp - Error or unknown state\n");
        }
    }
    else
    {
        // Handle RobotIO nullptr error
        printf("ERROR in Motor.cpp - Robot IO Pointer Null\n");
    }
}