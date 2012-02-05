/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.Compressor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you driveBackwards the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class TPARobot extends IterativeRobot {
    static final boolean DEBUG = true;              // Debug Trigger
    static final boolean CAMERA = false;            // Camera Trigger
    static final double STOP_VALUE = 0.1;           // Value sent to each motor when the robot is stopping
    static final double CONVEYOR_SPEED = 0.5;       // The speed the conveyor motor will be set to move
    static final double SHOOTING_SPEED_4 = 0.1;     // The speed of the shooter controlled by button 4
    static final double SHOOTING_SPEED_3 = 0.5;     // The speed of the shooter controlled by button 3
    static final double SHOOTING_SPEED_5 = 1.0;     // The speed of the shooter controlled by button 5
    double theShootingSpeed;                        // The actual speed the shooter is running at
    static boolean shoot8ButtonPressable = true;    // Flag for pressablilty of button 8 on the shooting joystick
    static boolean shoot6ButtonPressable = true;    // Flag for pressability of button 6 on the shooting joystick
    static boolean shoot4ButtonPressable = true;    // Flag for pressability of button 4 on the shooting joystick 
    static boolean shoot3ButtonPressable = true;    // Flag for pressability of button 3 on the shooting joystick 
    static boolean shoot5ButtonPressable = true;    // Flag for pressability of button 5 on the shooting joystick
    static boolean left1ButtonPressable = true;     // Flag for pressablity of the trigger on the left joystick
    static boolean flipDriveDirection = false;      // Determines whether the robot is moving forward or backward
    static boolean conveyorMoving = true;           // Determines whether the conveyor is moving
    static boolean ultrasonicSensorOn = false;      // Determines whether the ultrasonic sensor is on
    static double theDistance;                      // The distance returned by the ultrasonic sensor
    Jaguar theConveyorMotor;                        // The motor on the conveyor belt
    Jaguar theTopShootingMotor;                     // The shooting motor on the top
    Jaguar theBottomShootingMotor;                  // The shooting motor on the bottom
    AxisCamera theAxisCamera;                       // The camera
    DriverStationLCD theDriverStationLCD;           // Object representing the driver station   
    public double theMaxSpeed;                      // Multiplier for speed, determined by Z-Axis on left stick
    Encoder theFrontLeftEncoder;                    // The front left E4P
    Encoder theRearLeftEncoder;                     // The rear left E4P
    Encoder theFrontRightEncoder;                   // The front right E4P
    Encoder theRearRightEncoder;                    // The rear right E4P
    double theFrontLeftOutput;                      // The output sent to the front left motor
    double theRearLeftOutput;                       // The output sent to the rear left motor
    double theFrontRightOutput;                     // The output sent to the front right motor
    double theRearRightOutput;                      // The output sent to the rear right motor
    Joystick theRightStick;                         // Right joystick
    Joystick theLeftStick;                          // Left joystick
    Joystick theShootingStick;                      // The joystick used for all aspects of the shooting system
    TPARobotDriver theRobotDrive;                   // Robot Drive System
    double theDriveDirection;                       // Direction the robot will move
    double theDriveMagnitude;                       // Speed the robot will move at
    double theDriveRotation;                        // Value the robot will rotate
    Compressor theCompressor;                       // The air compressor
    TPAUltrasonicAnalogSensor theUltrasonicSensor;  // The ultrasonic sensor

    double afls =0;
    double afrs =0;
    double arls =0;
    double arrs =0;
    int numberCollected=0;


   
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    1/13/2012
     * Purpose: Robot Initialization Function. This function is run once when the
     *          robot is first started up and should be used for any initialization
     *          code.
     * Inputs:  None
     * Outputs: None
     */
    public void robotInit() {
               
        // Define joysticks being used at USB port #1, USB port #2, and USB port #3 on the Drivers Station
	theRightStick = new Joystick(1);
	theLeftStick = new Joystick(2);
        theShootingStick = new Joystick(3);
        if (DEBUG == true){
           System.out.println("The Joysticks constructed successfully"); 
        }
        
        // Define a robot drive object with the front left motor at port 1, rear left at port 2, front right at port 3, and rear right at port 4
        theRobotDrive = new TPARobotDriver(1,2,3,4);
        if (DEBUG == true){
            System.out.println("theRobotDrive constructed successfully");
        }

        //Defines four E4P Motion Sensors at ports 1,2,3,4,5,6,7, and 8
        theFrontLeftEncoder = new Encoder(2,1);
        theFrontLeftEncoder.start();
        theRearLeftEncoder = new Encoder(6,5);
        theRearLeftEncoder.start();
        theFrontRightEncoder = new Encoder(4,3);
        theFrontRightEncoder.start();
        theRearRightEncoder = new Encoder(8,7);
        theRearRightEncoder.start();
        if (DEBUG == true){
            System.out.println("The Encoders constructed successfully");
        }

        // Initialize the Conveyor belt motor at port 5
        theConveyorMotor = new Jaguar(5);
        if (DEBUG == true){
            System.out.println("The Conveyor Motor constructed successfully");
        }
        
        // Intialize the two shooting motors at ports 6 and 7
        theTopShootingMotor = new Jaguar(6);
        theBottomShootingMotor = new Jaguar (7);
        if (DEBUG == true){
            System.out.println("The Shooting Motors constructed successfully");
        }
        
        // Initialize the Ultrasonic sensor at analog port 1 and digital port 14
        theUltrasonicSensor = new TPAUltrasonicAnalogSensor(14,1);
        if (DEBUG == true){
            System.out.println("The ultrasonic sensor constructed successfully");
        }
        
        //Initialize the DriverStationLCD
        theDriverStationLCD = DriverStationLCD.getInstance();
        if (DEBUG == true) {
            System.out.println("DriverStationLCD initialized");
        }
        /*
        //Initialize the compressor in ports 6 and 6
        theCompressor = new Compressor (6,6);
        if (DEBUG == true){
            System.out.println("The Compressor constructed successfully");
        }
        theCompressor.start(); //Instantly turn the compressor on
        */
        //Initialize the AxisCamera
        if (CAMERA == true){
            theAxisCamera = AxisCamera.getInstance(); 
            theAxisCamera.writeResolution(AxisCamera.ResolutionT.k320x240);        
            theAxisCamera.writeBrightness(50);
            if (DEBUG == true) {
                System.out.println("AxisCamera initialized");
            }
        }
        else {
            if (DEBUG == true){
                System.out.println("CAMERA set to false");
            }
        }
        
        // Default the robot to not move
        theDriveDirection = 0;
        theDriveMagnitude = 0;
        theDriveRotation = 0;
        if (DEBUG == true){
            System.out.println("The robot set to not move");
        }
        
        if (DEBUG == true){
        System.out.println("RobotInit() completed.\n");
        }
    }
    /*--------------------------------------------------------------------------*/
    
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    1/13/2012
     * Purpose: TPARobot Constructor
     * Inputs:  None
     * Outputs: None
     */
    public void TPARobot(){
    }
    /*--------------------------------------------------------------------------*/
    

    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Team
     * Date:    1/13/2012
     * Purpose: This function is called periodically during autonomous mode. Allows
     *          for autonomous operation of the robot.
     * Inputs:  None
     * Outputs: None
     */

    public void autonomousPeriodic() {
        
        Watchdog.getInstance().feed();
        theDriverStationLCD.println(DriverStationLCD.Line.kMain6, 1, "Autonomous Mode Called");
        theDriverStationLCD.updateLCD();    //Displays a message to DriverStationLCD when entering Autonomous mode
    }
    /*--------------------------------------------------------------------------*/
    

    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Team
     * Date:    1/13/2012
     * Purpose: This function is called periodically during autonomous mode. Allows
     *          for autonomous operation of the robot.
     * Inputs:  None
     * Outputs: None
     */
    public void teleopPeriodic() {
       
        // Feed the user watchdog at every period when in autonomous
        Watchdog.getInstance().feed();
        if (DEBUG == true){
            System.out.println("Teleop Periodic Watchdog Fed");
        }
        
        //Set the multiplier for max speed
        setMaxSpeed();
        if (DEBUG == true) {
            System.out.println("setMaxSpeed called");
        }
        
        //Get the image from the Axis Camera
        DriverStationLCD.getInstance().updateLCD();
        if(DEBUG == true) {
            System.out.println("getCameraImage called");
        }

        // Drive the robot with FPS control
        driveRobot();
        if(DEBUG == true){
            System.out.println("driveRobot called");
        }
        
        // Run the conveyor belt
        runConveyor(theShootingStick, CONVEYOR_SPEED);
        if(DEBUG == true){
            System.out.println("runConveyor called");
        }
        
        // Display the speed of each wheel
        displaySpeed();
        if (DEBUG == true){
            System.out.println("displaySpeed called");
        }
        
        // Run the shooter
        runShooter(determineShootingSpeed(theShootingStick));
        if (DEBUG == true){
            System.out.println("runShooter called");
        }
        
        // Run the Ultrasonic sensor
        runUltrasonicSensor(theShootingStick, theUltrasonicSensor);
        if (DEBUG == true){
            System.out.println("runUltrasonicSensor called");
        }
        
/*d
        // Brake the robot if no joysick input.
        brakeOnNeutral();
        if(DEBUG == true) {
            System.out.println("brakeOnNeutral called");
        }
*/        
    }
    /*--------------------------------------------------------------------------*/
    
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    1/14/2012
     * Purpose: To drive the robot with mecanum wheels with a FPS control system.
     *          The left stick will control translational movement and the right
     *          stick will control rotation.
     * Inputs:  None
     * Outputs: None
     */    
    public void driveRobot() {
        
        theDriveDirection = theLeftStick.getDirectionDegrees(); // Set the direction to the value of the left stick
        theDriveMagnitude = theLeftStick.getMagnitude();    // Set the magnitude to the value of the left stick
        theDriveRotation = (theRightStick.getX()); // Set the rotation to the value of the right stick
        //theRobotDrive.mecanumDrive_Polar(.3, 180, 0);
        if (DEBUG == true){ 
        System.out.println("The drive rotation in degrees" + theDriveRotation);
        System.out.println("The drive magnitude is" + theDriveMagnitude);
        System.out.println("The drive direction is" + theDriveDirection);
        }
        if (!driveBackwards(theLeftStick)){
            theRobotDrive.mecanumDrive_Polar(theDriveMagnitude, theDriveDirection, theDriveRotation);
        }
        else if (driveBackwards(theLeftStick)){
            if (theDriveDirection > 0){
                theDriveDirection = theDriveDirection - 180;
            }
            else{
                theDriveDirection = 180 + theDriveDirection;
            }
            theDriveRotation = -theDriveRotation;
            theRobotDrive.mecanumDrive_Polar(theDriveMagnitude, theDriveDirection, theDriveRotation);
        }
        if (DEBUG == true){
            System.out.println("The drive rotation is" + theDriveRotation);
            System.out.println("The drive magnitude is" + theDriveMagnitude);
            System.out.println("The drive direction in degrees is" + theDriveDirection);
        }    
    }

    /*--------------------------------------------------------------------------*/
                    
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Gennaro De Luca, Marissa Beene
     * Date:    11/26/2011 (Gennaro De Luca)
     * Purpose: To determine the speed multiplier based on the "Z" wheel on 
     *          the left joystick. Responds as a gradient. If the "Z" wheel on the
     *          joystick gets a higher value, the robot will move faster.
     * Inputs:  None
     * Outputs: None
     */    
    public void setMaxSpeed(){
        
        theMaxSpeed = (theLeftStick.getZ() - 1.0)/(-2.0); // z-axis is inverted
        theRobotDrive.setMaxSpeed(theMaxSpeed); // sets the multiplier
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
    public void displaySpeed(){

        double tfl = theFrontLeftEncoder.getRate();
        double trl = theRearLeftEncoder.getRate();
        double tfr = theFrontRightEncoder.getRate();
        double trr = theRearRightEncoder.getRate();

        afls += tfl;
        arls += trl;
        afrs += tfr;
        arrs += trr;
        numberCollected++;

        if(numberCollected == 100){
            numberCollected =0;
            String print1 = "FLS: " + afls/100;
            String print2 = "RLS: " + arls/100;
            String print3 = "FRS: " + afrs/100;
            String print4 = "RRS: " + arrs/100; 

            theDriverStationLCD.println(DriverStationLCD.Line.kMain6, 1 , print1 );
            theDriverStationLCD.println(DriverStationLCD.Line.kUser2, 1 , print2 );
            theDriverStationLCD.println(DriverStationLCD.Line.kUser3, 1 , print3 );
            theDriverStationLCD.println(DriverStationLCD.Line.kUser4, 1 , print4 );
            theDriverStationLCD.updateLCD();

            afls=0;
            arls=0;
            afrs=0;
            arrs=0;
        } 
    }
    /*--------------------------------------------------------------------------*/


    /*--------------------------------------------------------------------------*/
    /*
     * Author: Andrew Matsumoto
     * Date: 1/26/12   
     * Purpose: decides whether the button to make the direction and rotation 
     * opposite of what they are originally is pressed 
     * Inputs:  
     * Outputs: the direction and the rotation opposite of the original.
     */  
    
    public boolean driveBackwards(Joystick aStick){
        if (left1ButtonPressable  && aStick.getRawButton(1)){
            flipBoolean (flipDriveDirection); // if flip is false, make it true and vice versa.
            left1ButtonPressable = false;
        }
        if (!aStick.getRawButton(1)){
          left1ButtonPressable = true;  
        }
        return flipDriveDirection;
    }
    /*--------------------------------------------------------------------------*/

    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    1/29/12
     * Purpose: To run a conveyor belt and allow for it to be turned on and off with
     *          a button press
     * Inputs:  Joystick aStick - the joystick that runs the conveyor
     *          double aSpeed - the speed to run the conveyor at
     * Outputs: None
     */    
    
    public void runConveyor(Joystick aStick, double aSpeed){
        if(shoot6ButtonPressable && aStick.getRawButton(6)){
            flipBoolean(conveyorMoving);
            shoot6ButtonPressable = false;
        }
        else{
            shoot6ButtonPressable = true;
        }
        theConveyorMotor.set(aSpeed);
    }
    /*--------------------------------------------------------------------------*/
    
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    1/29/12
     * Purpose: To flip the value of a boolean
     * Inputs:  boolean aBoolean - the boolean to be flipped
     * Outputs: boolean - the flipped version of the boolean
     */    
    
    public boolean flipBoolean(boolean aBoolean){
        if(aBoolean){
            return false;
        }
        else{
            return true;
        }
    }
    /*--------------------------------------------------------------------------*/
    
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    2/4/12
     * Purpose: To run a shooter with two motors controlling four wheels. One motor
     *          will run counterclockwise, the other will run clockwise.
     * Inputs:  double aSpeed - the speed the motors will run in
     * Outputs: None
     */    
    
    public void runShooter(double aSpeed){
        theTopShootingMotor.set(aSpeed);
        theBottomShootingMotor.set(-aSpeed);
    }    
    /*--------------------------------------------------------------------------*/

    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    2/4/12
     * Purpose: To determine the speed at which the wheels on the shooter will run.
     *          The speeds are controlled by buttons 4, 3, and 5 on the Joystick.
     * Inputs:  Joystick aStick - the joystick which controls the speed
     * Outputs: double theShootingSpeed - the speed the shooter will run at
     */    
    
    public double determineShootingSpeed(Joystick aStick){
        if(shoot4ButtonPressable && aStick.getRawButton(4)){
            theShootingSpeed = SHOOTING_SPEED_4;
        }
        if(shoot3ButtonPressable && aStick.getRawButton(3)){
            theShootingSpeed = SHOOTING_SPEED_3;
        }
        if(shoot5ButtonPressable && aStick.getRawButton(5)){
            theShootingSpeed = SHOOTING_SPEED_5;
        }
        return theShootingSpeed;
    }
    /*--------------------------------------------------------------------------*/
        
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    2/4/2012
     * Purpose: To run the ultrasonic sensor. A press of button 8 toggles it on 
     *          and off.
     * Inputs:  Joystick aStick - the Joystick controlling the analog sensor
     *          TPAUltrasonicAnalogSensor aSensor - the ultrasonic sensor
     * Outputs: 
     */    
    
    public void runUltrasonicSensor(Joystick aStick, TPAUltrasonicAnalogSensor aSensor){
        if (shoot8ButtonPressable && aStick.getRawButton(8)){
            flipBoolean(ultrasonicSensorOn);
            if(DEBUG == true){
                System.out.println("RunUltrasonicSensor recognizes button 8 press");
            }
            shoot8ButtonPressable = false;
        }
        else{
            shoot8ButtonPressable = true;
            System.out.println("In else statement");
            
        }
        if (ultrasonicSensorOn == true){
            aSensor.enable();
            if(DEBUG == true){
                System.out.println("Sensor Enabled");
            }
        }
        else{
            aSensor.disable();
            if(DEBUG == true){
                System.out.println("Sensor Disabled");
            }
        }
        theDistance = aSensor.getDistance();
        theDriverStationLCD.println(DriverStationLCD.Line.kUser6,1, "" + theDistance);
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
    
}
