/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
/**
 * Enable robot and slowly drive forward.
 * [1] If DS reports errors, adjust CAN IDs and firmware update.
 * [2] If motors are spinning incorrectly, first check gamepad.
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 */
#include <iostream>
#include <string>

#include "frc/TimedRobot.h"
#include "frc/XboxController.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/drive/DifferentialDrive.h"
#include "ctre/Phoenix.h"
#include "DrivebaseSimFX.h"
#include "rev/CANSparkMax.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	/* ------ [1] Update CAN Device IDs and switch to WPI_VictorSPX where necessary ------*/
	WPI_TalonFX _rghtFront{1};
	WPI_TalonFX _rghtFollower{2};
	WPI_TalonFX _leftFront{3};
	WPI_TalonFX _leftFollower{4};
	WPI_TalonFX _shoot{1};
	rev::CANSparkMax _load = rev::CANSparkMax(11,  rev::CANSparkMax::MotorType::kBrushless);
	rev::CANSparkMax _suck = rev::CANSparkMax(10,  rev::CANSparkMax::MotorType::kBrushless);


	WPI_PigeonIMU _pigeon{0};

	DifferentialDrive _diffDrive{_leftFront, _rghtFront};

	XboxController _joystick{0};

	void SimulationPeriodic() {
		_driveSim.Run();
	}

	void TeleopPeriodic() {

		std::stringstream work;

		/* get gamepad stick values */
		double left = -_joystick.GetRawAxis(1); /* positive is forward */
		double right = _joystick.GetRawAxis(2); /* positive is right */
		bool st = _joystick.GetRawButton(1);
		bool nd = _joystick.GetRawButton(2);
		bool rd = _joystick.GetRawButton(3);
		/* deadband gamepad 10%*/
		if (fabs(left) < 0.10)
			left = 0;
		if (fabs(right) < 0.10)
			right = 0;

		/* drive robot */
		_diffDrive.TankDrive(left, right, true);

		// if(st) _suck.Set(0.3);
		// else _suck.Set(0);
		// if(nd) _load.Set(0.6);
		// else _load.Set(0);

		// if(rd) _shoot.Set(1);
		// else _shoot.Set(0);
	
		/* -------- [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for RIGHT */
		work << " GF:" << left << " GT:" << right;

		/* get sensor values */
		//double leftPos = _leftFront.GetSelectedSensorPosition(0);
		//double rghtPos = _rghtFront.GetSelectedSensorPosition(0);
		double leftVelUnitsPer100ms = _leftFront.GetSelectedSensorVelocity(0);
		double rghtVelUnitsPer100ms = _rghtFront.GetSelectedSensorVelocity(0);

		work << " L:" << leftVelUnitsPer100ms << " R:" << rghtVelUnitsPer100ms;

		/* print to console */
		std::cout << work.str() << std::endl;
	}

	void RobotInit() {
		/* factory default values */
		_rghtFront.ConfigFactoryDefault();
		_rghtFollower.ConfigFactoryDefault();
		_leftFront.ConfigFactoryDefault();
		_leftFollower.ConfigFactoryDefault();

		/* set up followers */
		_rghtFollower.Follow(_rghtFront);
		_leftFollower.Follow(_leftFront);

		/* [3] flip values so robot moves forward when stick-forward/LEDs-green */
		_rghtFront.SetInverted(TalonFXInvertType::Clockwise);
		_rghtFollower.SetInverted(TalonFXInvertType::FollowMaster);
		_leftFront.SetInverted(TalonFXInvertType::CounterClockwise);
		_leftFollower.SetInverted(TalonFXInvertType::FollowMaster);

		_shoot.SetInverted(true);

		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
		// _rghtFront.SetSensorPhase(true);
		// _leftFront.SetSensorPhase(true);

		frc::SmartDashboard::PutData("Field", &_driveSim.GetField());
	}

private:
	DrivebaseSimFX _driveSim{_leftFront, _rghtFront, _pigeon};
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif