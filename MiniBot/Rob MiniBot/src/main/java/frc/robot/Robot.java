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
 * [2] If motors are spinning incorrectly, first check gamepad (hold down btn1)
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 * [4] Is only necessary if you have sensors.
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
    /*
     * --- [1] Update CAN Device IDs and use WPI_VictorSPX where necessary ------
     */
    
    WPI_TalonSRX _leftFront = new WPI_TalonSRX(10);
    WPI_TalonSRX _rghtFront = new WPI_TalonSRX(20);


    DifferentialDrive _diffDrive = new DifferentialDrive(_leftFront, _rghtFront);

    Joystick _joystick = new Joystick(0);

    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();

    double pi = Math.PI;
    double leftPos, rghtPos;
    double  diam =  4.5;
    double leftPct, rghtPct ;
   
    @Override
    public void robotInit() {
        /* factory default values */
        _rghtFront.configFactoryDefault();
        _leftFront.configFactoryDefault();
       
       
        /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
        _rghtFront.setInverted(false); // !< Update this
        _leftFront.setInverted(true); // !< Update this

              /*
         * [4] adjust sensor phase so sensor moves positive when Talon LEDs are green
         */
        _rghtFront.setSensorPhase(false);
        _leftFront.setSensorPhase(false);

        /*
         * WPI drivetrain classes defaultly assume left and right are opposite. call
         * this so we can apply + to both sides when moving forward. DO NOT CHANGE
         */
        _diffDrive.setRightSideInverted(false);
        
    }

    @Override
    public void teleopInit() {
     /*  start both encoders at zero when teleop initializes */
     _rghtFront.setSelectedSensorPosition(0);
     _leftFront.setSelectedSensorPosition(0);
     System.out.println("TELEOP STARTED");   
    }


    @Override
    public void autonomousInit() {
     /*  start both encoders at zero when autonomous initializes */
     _rghtFront.setSelectedSensorPosition(0);
     _leftFront.setSelectedSensorPosition(0);
     System.out.println("AUTO STARTED");
     
    }
    @Override
    public void teleopPeriodic() {

        String work = "";

        /* get gamepad stick values */
        leftPct = -1 * _joystick.getRawAxis(1); /* positive is forward */
        rghtPct = -1 * _joystick.getRawAxis(3); /* positive is right */

        leftPct = leftPct * 0.45;  //  limit motor voltage so we don't break too much
        rghtPct = rghtPct * 0.45;  //  limit motor voltage so we don't break too much

        boolean btn1 = _joystick.getRawButton(13); /* is button is down, print joystick values */
        boolean btn2 = _joystick.getRawButton(15); /* is button is down, print joystick values */

        /* deadband gamepad 10% */
        if (Math.abs(leftPct) < 0.10) {
            leftPct = 0;
        }
        if (Math.abs(rghtPct) < 0.10) {
            rghtPct = 0;
        }

        /* drive robot */
        _diffDrive.tankDrive(leftPct, rghtPct);

        /*
         * [2] Make sure Gamepad Forward is positive for FORWARD
         */
        work += " GF:" + leftPct + " GT:" + rghtPct;

        /* get sensor values */
        readPosition();
        //leftPos = _leftFront.getSelectedSensorPosition();
       // rghtPos = _rghtFront.getSelectedSensorPosition();
        double leftVelUnitsPer100ms = _leftFront.getSelectedSensorVelocity(0);
        double rghtVelUnitsPer100ms = _rghtFront.getSelectedSensorVelocity(0);

       // leftPos = (leftPos/4096)*pi*diameter;  // Calculate distance in inches
       // rghtPos = (rghtPos/4096)*pi*diameter;
   
        work += " LP:"+(int)leftPos+" LV:" + leftVelUnitsPer100ms + " RP:"+(int)rghtPos + " RV:" + rghtVelUnitsPer100ms;

        /*
         * drive motor at least 25%, Talons will auto-detect if sensor is out of phase
         */
        _leftFront.getFaults(_faults_L);
        _rghtFront.getFaults(_faults_R);

        if (_faults_L.SensorOutOfPhase) {
            work += " L sensor is out of phase";
        }
        if (_faults_R.SensorOutOfPhase) {
            work += " R sensor is out of phase";
        }

        /* print to console if btn1 is held down */
        if (btn1 || btn2) {
            System.out.println(work);
        }
    }
    
    @Override
    public void autonomousPeriodic() {   
          /* drive robot */
        readPosition();
        if ((leftPos+rghtPos)/2 < 48) {   // stop at 48 inches
            leftPct = 0.4;
            rghtPct = 0.4;
        }
        else {
            leftPct = 0;
            rghtPct = 0;
        }
        _diffDrive.tankDrive(leftPct, rghtPct);

    }

    public void readPosition() {
        leftPos = _leftFront.getSelectedSensorPosition();
        rghtPos = _rghtFront.getSelectedSensorPosition();
       
        leftPos = (leftPos/4096)*pi*diam;  // Calculate distance in inches
        rghtPos = (rghtPos/4096)*pi*diam;

    }
}
