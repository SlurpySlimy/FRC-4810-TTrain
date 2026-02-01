//---------------------------------------------------------------------------
//
// RobotIO.cpp
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 17-Feb-22  JJB       Class created.
// 
//---------------------------------------------------------------------------
//
// References:
// com.ctre.phoenix.motorcontrol.can.WPI_TalonFX Class Reference
// https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___talon_f_x.html
//
// com.ctre.phoenix.motorcontrol.can.TalonFX Class Reference
// https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_talon_f_x.html
//
// Notes:
// https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-check-with-motor-drive
// Sensor phase describes the relationship between the motor output
// direction (positive vs negative) and sensor velocity (positive vs
// negative).  For soft-limits and closed-loop features to function
// correctly, the sensor measurement and motor output must be “in-phase”.
//
// Note: Talon FX automatically phases your sensor for you.  It will
// always be correct, provided you use the getSelected* API and have
// configured the selected feedback type to be integrated sensor.
//---------------------------------------------------------------------------

#include "RobotIO.h"                   // Controlled hardware class definition

#include <frc/smartdashboard/SmartDashboard.h>

// https://www.chiefdelphi.com/t/include-wpi-numbers-not-working-after-we-updated-wpilib-to-the-2023-version/424267
// As mentioned at New for 2023 — FIRST Robotics Competition documentation 6, the std numbers header should be used instead (ie #include <numbers>).
// https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html
//-JJB #include <numbers>
#include <numbers>

//-JJB C:\Users\iamro\.gradle\caches\8.11\transforms\be4aaa728fdcee242249e072e2efdf40\transformed\tools-25.1.0-headers\ctre\phoenix6\signals\SpnEnums.hpp

//-JJB - Phoenix V6 API .PDF
//-JJB https://v6.docs.ctr-electronics.com/_/downloads/en/stable/pdf/

#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/spns/SpnValue.hpp"

using namespace ctre::phoenix6;

//-------------------------------------------------------------------

RobotIO::RobotIO()
{
   // printf( "Enter RobotIO Constructor\n" );
}

//-------------------------------------------------------------------

// Initialize all devices on the robot.

void RobotIO::RobotInit()
{
   // *---------------------------------*
   // * Chassis Hardware Initialization *
   // *---------------------------------*

   // Done in SwerveModule.cpp 

   
}

//-------------------------------------------------------------------

// Update the status of all input devices on the robot.

void RobotIO::UpdateInputStatus()
{
   //THN Add Arm limit Status to Smart Dashboard
   frc::SmartDashboard::PutBoolean("Arm Limit Switch ", GetArmLimit())

   //THN Add Motor Limit Status to Smart Dashboard
   frc::SmartDashboard::PutBoolean("Motor Limit Switch ", GetMotorLimit())
}
