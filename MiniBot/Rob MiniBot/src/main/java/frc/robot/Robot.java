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


package frc.robot;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    /*
     * --- [1] Update CAN Device IDs and use WPI_VictorSPX where necessary ------
     */
    
    WPI_TalonSRX _leftFront = new WPI_TalonSRX(10);
    WPI_TalonSRX _rghtFront = new WPI_TalonSRX(20);

    Joystick _joystick = new Joystick(0);

    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();

    double pi = Math.PI;
    double leftPos, rghtPos;
    double  diam =  4.5;
    double leftFwd, rghtFwd;
    double leftVelInPerSec, rghtVelInPerSec, maxVel = 150 ;  // maxVel in inches/sec
    boolean velMode;

    int kTimeoutMs = 30;
    double kF= 1023/1460, kP = 0.1, kD=0, kI=0;
    double maxPct = 0.45;
    
   
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

        _leftFront.setNeutralMode(NeutralMode.Brake);
        _rghtFront.setNeutralMode(NeutralMode.Brake);
        
        /**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftFront.configPeakOutputForward(+1.0, kTimeoutMs);
		_leftFront.configPeakOutputReverse(-1.0, kTimeoutMs);
        _rghtFront.configPeakOutputForward(+1.0, kTimeoutMs);
        _rghtFront.configPeakOutputReverse(-1.0, kTimeoutMs);

		_leftFront.configPeakOutputForward(1, kTimeoutMs);
		_leftFront.configPeakOutputReverse(-1, kTimeoutMs);

        _leftFront.config_kF(0,kF,kTimeoutMs);
        _leftFront.config_kP(0,kP,kTimeoutMs);
        _leftFront.config_kD(0,kD,kTimeoutMs);
        _leftFront.config_kI(0,kI,kTimeoutMs);

        _rghtFront.config_kF(0,kF,kTimeoutMs);
        _rghtFront.config_kP(0,kP,kTimeoutMs);
        _rghtFront.config_kD(0,kD,kTimeoutMs);
        _rghtFront.config_kI(0,kI,kTimeoutMs);
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

        String log = "";

        /* get gamepad stick values          
         *  Make sure Gamepad Forward is positive for FORWARD
         */
        leftFwd = -1 * _joystick.getRawAxis(1); /* positive is forward */
        rghtFwd = -1 * _joystick.getRawAxis(5); /* positive is right */

            /* deadband gamepad 10% */
        if (Math.abs(leftFwd) < 0.10) {
            leftFwd = 0;
        }
        
        if (Math.abs(rghtFwd) < 0.10) {
            rghtFwd = 0;
        }

        boolean btn1 = _joystick.getRawButton(5); /* if button is down, print log values */
        boolean btn2 = _joystick.getRawButton(6); /* if button is down, print log values */
        boolean btnA = _joystick.getRawButton(1);
        boolean btnB = _joystick.getRawButton(2);

        if (btnA) {
            velMode = false;
            System.out.println("Switched to Percent Drive");
        
           
        }

        if (btnB) {
            velMode = true;
            System.out.println("Switched to Velocity Drive"); 
        }

        /* drive robot */
        if ( ! velMode) {       //  Percent drive mode
            
            leftFwd = leftFwd * maxPct;  //  limit motor voltage so we don't break too much
            rghtFwd = rghtFwd * maxPct;  //  limit motor voltage so we don't break too much            
            
            
            _leftFront.set(ControlMode.PercentOutput, leftFwd);
            _rghtFront.set(ControlMode.PercentOutput, rghtFwd);
        
        
        }   
        /* else drive with velocity PID */
        else {         

            double targetL_IPS = leftFwd * maxVel;
            double targetR_IPS = rghtFwd * maxVel;

            double targetL_unitsPer100ms = targetL_IPS * 4096 /( 10*pi*diam);
            double targetR_unitsPer100ms = targetR_IPS * 4096 /( 10*pi*diam);


            System.out.println("targetL " + targetL_unitsPer100ms + " targetR " + targetR_unitsPer100ms);

            _rghtFront.set(ControlMode.Velocity, targetR_unitsPer100ms);
            _leftFront.set(ControlMode.Velocity, targetL_unitsPer100ms);       
                  
        }  

        readEncoders();     /* get sensor values */      
       
        /*
         * drive motor at least 25%, Talons will auto-detect if sensor is out of phase
         */
        _leftFront.getFaults(_faults_L);
        _rghtFront.getFaults(_faults_R);

        if (_faults_L.SensorOutOfPhase) {
            log += " L sensor is out of phase";
        }
        if (_faults_R.SensorOutOfPhase) {
            log += " R sensor is out of phase";
        }

        
        if (log.length() > 0) System.out.println(log);


        /* print to console if btn1 is held down */
        if (btn1 || btn2) {
            System.out.printf( "GL:%.2f  LP:%.2f  LVips:%.2f \n", leftFwd,leftPos,leftVelInPerSec);
            System.out.printf( "RL:%.2f  RP:%.2f  RVips:%.2f \n", rghtFwd,rghtPos,rghtVelInPerSec);
        }
    }
    
    @Override
    public void autonomousPeriodic() {   
          /* drive robot */
        readEncoders();
        if ((leftPos+rghtPos)/2 < 48) {   // stop at 48 inches
            leftFwd = 0.4;
            rghtFwd = 0.4;
        }
        else {
            leftFwd = 0;
            rghtFwd = 0;
        }
        _leftFront.set(ControlMode.PercentOutput, leftFwd);
        _rghtFront.set(ControlMode.PercentOutput, rghtFwd);       
       
    }

    public void readEncoders() {
        leftPos = _leftFront.getSelectedSensorPosition();
        rghtPos = _rghtFront.getSelectedSensorPosition();
       
        leftPos = (leftPos/4096)*pi*diam;  // Calculate distance in inches
        rghtPos = (rghtPos/4096)*pi*diam;

        double leftVelUnitsPer100ms = _leftFront.getSelectedSensorVelocity(0);
        double rghtVelUnitsPer100ms = _rghtFront.getSelectedSensorVelocity(0);

        rghtVelInPerSec = (rghtVelUnitsPer100ms/4096)*pi*diam * 10;
        leftVelInPerSec = (leftVelUnitsPer100ms/4096)*pi*diam * 10;
    }
}
