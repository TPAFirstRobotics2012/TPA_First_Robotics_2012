/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.can.CANNotInitializedException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author Gennaro
 */
public class TPARobotDriver extends RobotDrive {
    
    public static double kDefaultMinOutput=0.1; //Min value for the Speed Multiplier
    

    //Constructors:
    TPARobotDriver(final int leftMotorChannel, final int rightMotorChannel){
        super(leftMotorChannel, rightMotorChannel);
        for(int i = 0; i < kMaxNumberOfMotors; i++){
            multipliers[i]=1;
        }
        for(int i = 0; i < kMaxNumberOfMotors; i++){
           speedSums[i]=1;
        }
    }
    
    TPARobotDriver(SpeedController leftMotor, SpeedController rightMotor) {
        super(leftMotor, rightMotor);
        for(int i = 0; i < kMaxNumberOfMotors; i++){
            multipliers[i]=1;
        }
        for(int i = 0; i < kMaxNumberOfMotors; i++){
           speedSums[i]=1;
        }
      
    }
    
    TPARobotDriver(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
                   SpeedController frontRightMotor, SpeedController rearRightMotor) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        for(int i = 0; i < kMaxNumberOfMotors; i++){
            multipliers[i]=1;
        }
        for(int i = 0; i < kMaxNumberOfMotors; i++){
           speedSums[i]=1;
        }
    }
    TPARobotDriver(int frontLeftMotor, int rearLeftMotor,
             int frontRightMotor, int rearRightMotor) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        for(int i = 0; i < kMaxNumberOfMotors; i++){
            multipliers[i]=1;
        }
        for(int i = 0; i < kMaxNumberOfMotors; i++){
           speedSums[i]=1;
        }
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
     * Author:  Andrew Matsumoto
     * Date:    2/3/12
     * Purpose: To make sure that all the wheels are rotating at the same speed.
     * Inputs:  None
     * Outputs: None
     */
    
        static final int kFrontLeft_val = 0;            //The channel of the front left motor
        static final int kFrontRight_val = 1;           //The channel for the front right motor
        static final int kRearLeft_val = 2;             //The channel for the rear left motor
        static final int kRearRight_val = 3;            //The channel for the rear right motor
        double wheelSpeeds[] = new double[kMaxNumberOfMotors];
        double multipliers[] = new double[kMaxNumberOfMotors];
        
    public void TPAMecanumDrive_Polar(double magnitude, double direction, double rotation) {
        double frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed;
        // Normalized for full power along the Cartesian axes.
        magnitude = limit(magnitude) * Math.sqrt(2.0);
        // The rollers are at 45 degree angles.
        double dirInRad = (direction + 45.0) * 3.14159 / 180.0;
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        
        wheelSpeeds[kFrontLeft_val] = (sinD * magnitude + rotation);
        wheelSpeeds[kFrontRight_val] = (cosD * magnitude - rotation);
        wheelSpeeds[kRearLeft_val] = (cosD * magnitude + rotation);
        wheelSpeeds[kRearRight_val] = (sinD * magnitude - rotation);

        normalize(wheelSpeeds);//Make sure wheelSpeeds are between -1 and 1

        byte syncGroup = (byte)0x80;

        m_frontLeftMotor.set(wheelSpeeds[kFrontLeft_val] * m_invertedMotors[kFrontLeft_val] * m_maxOutput*multipliers[kFrontLeft_val], syncGroup);
        m_frontRightMotor.set(wheelSpeeds[kFrontRight_val] * m_invertedMotors[kFrontRight_val] * m_maxOutput*multipliers[kFrontRight_val], syncGroup);
        m_rearLeftMotor.set(wheelSpeeds[kRearLeft_val] * m_invertedMotors[kRearLeft_val] * m_maxOutput*multipliers[kRearLeft_val], syncGroup);
        m_rearRightMotor.set(wheelSpeeds[kRearRight_val] * m_invertedMotors[kRearRight_val] * m_maxOutput*multipliers[kRearRight_val], syncGroup);
        
        

        if (m_isCANInitialized) {
            try {
                CANJaguar.updateSyncGroup(syncGroup);
            } catch (CANNotInitializedException e) {
                m_isCANInitialized = false;
            } catch (CANTimeoutException e) {}
        }

        if (m_safetyHelper != null) m_safetyHelper.feed();
    }
        
        double speedSums[] = new double[kMaxNumberOfMotors];
        int theCounter = 0;//Counts the number of times the speed is counted
        
    public void multiplierCalculator(double aFrontLeftSpeed, double aFrontRightSpeed, double aRearLeftSpeed, double aRearRightSpeed){
        speedSums[kFrontLeft_val]+=Math.abs(aFrontLeftSpeed);
        speedSums[kFrontRight_val]+=Math.abs(aFrontRightSpeed);
        speedSums[kRearLeft_val]+=Math.abs(aRearLeftSpeed);
        speedSums[kRearRight_val]+=Math.abs(aRearRightSpeed);
         
        //Averages the speed
        if(theCounter==5){
            theCounter=0;//resets the counter
            for(int j = 0; j < kMaxNumberOfMotors;j++){
                speedSums[j]=speedSums[j]/5;//Aveage of the speedSums
            }
            int ThePointer = 0;//Marks the position of the greatest wheelSpeed
            for (int i = 1; i < 4; i++){
                if (wheelSpeeds[i] > wheelSpeeds[ThePointer]){
                    ThePointer = i;
                }
            }
            //calculates the multipliers
            for(int k = 0; k < kMaxNumberOfMotors; k++){
                if(wheelSpeeds[k]!=0 && wheelSpeeds[ThePointer]!=0){
                    multipliers[k]=((speedSums[ThePointer]/speedSums[k])/(wheelSpeeds[ThePointer]/wheelSpeeds[k]));
                    System.out.println("(" + speedSums[ThePointer] + "/" +speedSums[k] +")");
                    System.out.println("(" + wheelSpeeds[ThePointer] + "/" +wheelSpeeds[k] +")");
                }
            }
            //Resets the speedSums
            for (int l = 0 ; l < kMaxNumberOfMotors; l++){
                speedSums[l]=1;//Set at 1 to prevent a division by 0
            }
            //Limits the multipliers
            for(int n = 0; n < 4; n++){
                if(multipliers[n]> 1.2)
                   multipliers[n] = 1.2;
                else if(multipliers[n]<.8)
                    multipliers[n]=.8;
            }
            for(int m = 0; m<4; m++){
            System.out.println("the multiplier for the for the motor in position number " + m + " is " + multipliers[m]); 
            }
        }
            
            
        }
        
}
