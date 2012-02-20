/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author Gennaro
 */
public class TPARobotDriver extends RobotDrive {
    
    public static double kDefaultMinOutput=0.1; //Min value for the Speed Multiplier
    

    //Constructors:
    TPARobotDriver(final int leftMotorChannel, final int rightMotorChannel){
        super(leftMotorChannel, rightMotorChannel);
        
    }
    
    TPARobotDriver(SpeedController leftMotor, SpeedController rightMotor) {
        super(leftMotor, rightMotor);
    }
    
    TPARobotDriver(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
                   SpeedController frontRightMotor, SpeedController rearRightMotor) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

    }
    TPARobotDriver(int frontLeftMotor, int rearLeftMotor,
             int frontRightMotor, int rearRightMotor) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    }
    /*--------------------------------------------------------------------------*/
   
    /*
     * Author:  Gennaro De Luca
     * Date:    11/26/11 (Gennaro De Luca)
     * Purpose: Tests the input value to make sure that it is a safe value, then
     *          sets that multiplier.
     * Inputs:  double aMaxSpeed - wanted value for the speed multiplier 
     * Outputs: None
     */
    
    public void setMaxSpeed(double aMaxSpeed) {
        
        if(aMaxSpeed<kDefaultMinOutput)  {      //Tests to see if value is less than min allowed
            aMaxSpeed=kDefaultMinOutput;
        }
        else if(aMaxSpeed>kDefaultMaxOutput) {  //Tests to see if value is more than max allowed
            aMaxSpeed=kDefaultMaxOutput;
        }
        m_maxOutput=aMaxSpeed;                  //sets the multiplier to be the tested value
    }
   /*--------------------------------------------------------------------------*/
    
   /*--------------------------------------------------------------------------*/
   /*
    * Author:  Marissa Beene
    * Date:    10/30/2011 (Marissa Beene)
    * Purpose: To use the motors to brake the robot. Takes the speed from the 
    *          each motor and sends the reverse signal back.
    * Inputs:  Double aSpeedRight - the speed of the right motor
    *          Double aSpeedLeft - the speed of the left motor
    *          RobotDrive aRobotDrive - the drive system of the robot
    * Outputs: None
    */
    
    public void twoMotorBrake(double aSpeedLeft, double aSpeedRight, RobotDrive aRobotDrive){
        aRobotDrive.tankDrive(-aSpeedLeft, -aSpeedRight); //drive the robot at opposite values
    }
    /*--------------------------------------------------------------------------*/
   
    
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    1/14/2012
     * Purpose: To use the motors to brake the robot when it is using mecanum wheels.
     *          Takes the speed from each motor and sends the reverse signal back.
     * Inputs:  None
     * Outputs: None
     */
    
    public void mecanumBrake(double aSpeedFrontLeft, double aSpeedRearLeft, double aSpeedFrontRight, double aSpeedRearRight){
        System.out.println("Hello");
        m_frontLeftMotor.set(-aSpeedFrontLeft);
        m_frontRightMotor.set(-aSpeedFrontRight);
        m_rearLeftMotor.set(-aSpeedRearLeft);
        m_rearRightMotor.set(-aSpeedRearRight);
        System.out.println("Hello Again");
    }

    /*--------------------------------------------------------------------------*/

    /*--------------------------------------------------------------------------*/
    /*
     * Author:  
     * Date:    
     * Purpose: 
     * Inputs:  
     * Outputs: 
     */    
    /*--------------------------------------------------------------------------*/
    public void flipTankDrive(Joystick aLeftStick, Joystick aRightStick) {
        if (aLeftStick == null || aRightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(-aLeftStick.getY(), -aRightStick.getY());
    }
    
    public void flipArcadeDrive(Joystick aStick) {
        if (aStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        arcadeDrive(-aStick.getY(), -aStick.getX());
    }
}
