//---------------------------------------------------------------------------
//
// RobotIO.h
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 17-Feb-22  JJB       Class created.
//
//---------------------------------------------------------------------------

#pragma once

#include <frc/XboxController.h>        // Driver/Operator Station Controllers

#include <frc/motorcontrol/PWMTalonFX.h>
#include <frc/DigitalInput.h>          // Digital I/O (Photo Eyes, Limit Switches, Encoders)

#include <frc/Servo.h>  //Linear Actuator Testing

//#include "ctre/Phoenix.h"              //-GMS - Old Phoenix 5 controls
#include <ctre/phoenix6/TalonFX.hpp>

//-JJB #include <units/units.h>        // Catch-all.  Use individual headers
#include "units/velocity.h"            // For velocity calculations
#include "units/length.h"              // For distance calculations
#include <units/angle.h>               // For angle representations
#include <units/angular_velocity.h>    // For angular velocity representations

// https://www.chiefdelphi.com/t/include-wpi-numbers-not-working-after-we-updated-wpilib-to-the-2023-version/424267
// As mentioned at New for 2023 — FIRST Robotics Competition documentation 6,
// the std numbers header should be used instead (ie #include <numbers>).
// https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html
//-JJB #include <numbers>
#include <numbers>

using namespace ctre::phoenix6;

class RobotIO
{
   public:

      // Constructor/Destructor.

      RobotIO();
      ~RobotIO()
         { }

      // Device Initialization.

      void RobotInit();
      void UpdateInputStatus();

      // Accessor Methods.
// THN Arm Limit Accessor method
      inline bool GetArmLimit()
         {return (m_ArmLimitSwitch.Get());} // if it is unexpectedly false or inverted then you can change it here
      // Xbox Controllers

      frc::XboxController m_DriveController{ 0 };
      frc::XboxController m_OperatorController{ 1 };

   

      //arm hardware
      hardware::TalonFX m_ArmMotor  {14};

      //DIO stuff
      frc::DigitalInput m_ArmLimitSwitch {2};

};
