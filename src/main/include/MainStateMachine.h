//---------------------------------------------------------------------------
//
// MainStateMachine.h - Main State Machine Class Definition
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 10-Jan-24    JJB     Initial Version.
// 03-Mar-24    GMS     Added Homing State
//
//---------------------------------------------------------------------------

#ifndef MAIN_STATE_MACHINE_H_
#define MAIN_STATE_MACHINE_H_

#include <frc/XboxController.h>

#include "RobotIO.h"

#include "Drivetrain.h"                // Drivetrain state machine class
                                       //    definition

#include "Motor.h"                     //Motor State machine class

#include "Arm.h"                       //Arm state machine class
// *------------------------------------------------*
// * Top Level (Main) State Machine Enumerated Type *
// *------------------------------------------------*

namespace RobotMain
{
   enum eState
   {
      STATE_START = 0,
      STATE_IDLE = 1, 
      // THN - Arm States
      ARM_STATES = 2,
      ARM_STATE_HOMING = 3,
      ARM_STATE_MANUAL_RAISE = 4,
      ARM_STATE_MANUAL_LOWER = 5,
      ARM_STATE_AUTO_RAISE = 6,
      //THN - Motor states
      MOTOR_STATES = 7,
      MOTOR_STATE_HOMING = 8,
      MOTOR_STATE_MANUAL_FORWARD = 9,
      MOTOR_STATE_MANUAL_REVERSE = 10,
      MOTOR_STATE_AUTO_FORWARD = 11,
      STATE_ERROR = 99
   };

   // Different states for the drivetrain. Independent to avoid locking
   // driver out of drive controls.  Normal state is regular, states 1
   // & 2 are for reef, states 3 & 4 are for source stations.

   enum eDriveState  
   {
      STATE_NORMAL = 0,
   };
}

class MainStateMachine
{
   public:

      // Constructor/Destructor.

      MainStateMachine();
      ~MainStateMachine()
         { }

      // Class Methods.

      void Initialize( RobotIO *p_pRobotIO );
      void UpdateStatus();
      void Execute();

   private:

      RobotMain::eState m_eState;      // Current main state
      RobotIO *m_pRobotIO;             // Pointer to Robot I/O Class Instance
      RobotMain::eDriveState m_eDriveState;      // Current Drive state

      // State Machine Object Instances.
      Drivetrain m_Drivetrain;

      Arm m_Arm; //THN Arm object instance
      Motor m_Motor; //THN motor Object instance
};

#endif // MAIN_STATE_MACHINE_H_
