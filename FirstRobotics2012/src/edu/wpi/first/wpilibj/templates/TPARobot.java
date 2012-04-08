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
    static final double CONVEYOR_SPEED = 1.0;       // The speed the conveyor motor will be set to move
    static final double SHOOTING_SPEED_4 = 0.3;     // The speed of the shooter controlled by button 4
    static final double SHOOTING_SPEED_3 = 0.5;     // The speed of the shooter controlled by button 3
    static final double SHOOTING_SPEED_5 = 1.0;     // The speed of the shooter controlled by button 5
    static final double SHOOTING_SPEED_OFF = 0.0;   // Shooting speed, controlled by button 10
    static final int AVERAGING_VALUE = 10;
    static final int UNINITIALIZED_DRIVE = 0;       // Value when no drive mode is selected
    static final int ARCADE_DRIVE = 1;              // Value when arcade mode is selected 
    static final int TANK_DRIVE = 2;                // Value when tank drive is selected
    static final double HYBRID_SHOOTING_SPEED = 0.3;
    static boolean joystickRunsShooter = false;     // Use the joystick to control the shooter  at a gradient
    static boolean button6Pressed = false;      // True if the joystick used to run the shooter
    static boolean shoot2ButtonPressable = true;    // Flag for pressability of button 6 on the shooting joystick
    static boolean shoot4ButtonPressable = true;    // Flag for pressability of button 4 on the shooting joystick 
    static boolean shoot3ButtonPressable = true;    // Flag for pressability of button 3 on the shooting joystick 
    static boolean shoot5ButtonPressable = true;    // Flag for pressability of button 5 on the shooting joystick
    static boolean shoot6ButtonPressable = true;    // Flag for pressability of button 6 on the shooting joystick
    static boolean shoot7ButtonPressable = true;    // Flag for pressability of button 7 on the shooting joystick
    static boolean shoot8ButtonPressable = true;    // Flag for pressability of button 8 on the shooting joystick
    static boolean left1ButtonPressable = true;     // Flag for pressablity of the trigger on the left joystick
    static boolean kinect7ButtonPressable = true;
    static boolean kinect8ButtonPressable = true;
    static boolean theConveyorRunning = false;
    static boolean theShooterRunning = false;
    static boolean flipDriveDirection = false;      // Determines whether the robot is moving forward or backward
    static boolean conveyorMoving = false;          // Determines whether the conveyor is moving
    static boolean theRelayFlag = true;            // Is the relay on?
    static double theJoystickSpeed = 0;             // Speed of shooters while Joystick controls speed
    static double shoot9ButtonSpeed = 0;            // Speed of shooters assigned to button 9 of shooting joystick        
    static double theShootingSpeed = 0.0;           // The actual speed the shooter is running
    static double theAccumulatedDistanceLeft;                                                                                                            //From Here                                                          
    static double theAveragedDistancePreTruncationLeft; //The Average distance returned by the Ultrasonic before it is truncated to 2 decimal places      /
    static int theAverageDistanceTruncationStep1Left;   //variable above multiplied by 100 to remove all unecessary digits                                /    
    static double theAverageDistanceTruncatedLeft;      //The Average Distance truncated to 2 decimal places                                              /          
    static double theDistanceLeft;                      // The distance returned by the ultrasonic sensor                                                 /
    static int theDistancesCollectedLeft;                                                                                                                 // To Here are for the left ultrasonic    
    static double theAccumulatedDistanceRight;                                                                                                             //Same For the Right UltraSonic                                                                           
    static double theAveragedDistancePreTruncationRight; //The Average distance returned by the Ultrasonic before it is truncated to 2 decimal places      /
    static int theAverageDistanceTruncationStep1Right;   //variable above multiplied by 100 to remove all unecessary digits                                /    
    static double theAverageDistanceTruncatedRight;      //The Average Distance truncated to 2 decimal places                                              /
    static double theDistanceRight;                      // The distance returned by the ultrasonic sensor                                                 /
    static int theDistancesCollectedRight;                                                                                                                 // To here                                                
    static double theAccumulatedDistanceFront;                                                                                                             //And for the Front one        
    static double theAveragedDistancePreTruncationFront; //The Average distance returned by the Ultrasonic before it is truncated to 2 decimal places      / 
    static int theAverageDistanceTruncationStep1Front;   //variable above multiplied by 100 to remove all unecessary digits                                /   
    static double theAverageDistanceTruncatedFront;      //The Average Distance truncated to 2 decimal places                                              /
    static double theDistanceFront;                      // The distance returned by the ultrasonic sensor                                                 /
    static int theDistancesCollectedFront;                                                                                                                 //To here            
    static double theMaxSpeed;                      // Multiplier for speed, determined by Z-Axis on left stick
    static double theFrontLeftOutput;               // The output sent to the front left motor
    static double theRearLeftOutput;                // The output sent to the rear left motor
    static double theFrontRightOutput;              // The output sent to the front right motor
    static double theRearRightOutput;               // The output sent to the rear right motor
    static double theDriveDirection;                // Direction the robot will move
    static double theDriveMagnitude;                // Speed the robot will move at
    static double theDriveRotation;                 // Value the robot will rotate
    static double theSumFrontLeftSpeed =0;
    static double theSumFrontRightSpeed =0;
    static double theSumRearLeftSpeed =0;
    static double theSumRearRightSpeed =0;
    static double theHybridDriveRotation = 0;
    static double theHybridDriveMagnitude = 0;
    static double theHybridDriveDirection = 0;
    static int theNumberCollected=0;
    
    AxisCamera theAxisCamera;                       // The camera
    DriverStationLCD theDriverStationLCD;           // Object representing the driver station  
    Encoder theFrontLeftEncoder;                    // The front left E4P
    Encoder theRearLeftEncoder;                     // The rear left E4P
    Encoder theFrontRightEncoder;                   // The front right E4P
    Encoder theRearRightEncoder;                    // The rear right E4P
    Jaguar theConveyorMotor;                        // The motor on the conveyor belt
    Jaguar theTopShootingMotor;                     // The shooting motor on the top
    Jaguar theBottomShootingMotor;                  // The shooting motor on the bottom
    Joystick theRightStick;                         // Right joystick
    Joystick theLeftStick;                          // Left joystick
    Joystick theShootingStick;                      // The joystick used for all aspects of the shooting system
    //Solenoid theWedgeUp;                            //This Solenoid moves the wedge up and holds the ball in place
    TPARobotDriver theRobotDrive;                   // Robot Drive System
    TPAUltrasonicAnalogSensor theUltrasonicSensorLeft;  // The ultrasonic sensor Left
    TPAUltrasonicAnalogSensor theUltrasonicSensorRight;  // The ultrasonic sensor Right
    TPAUltrasonicAnalogSensor theUltrasonicSensorFront;  // The ultrasonic sensor Front
    KinectStick theLeftArm;                         // Kinect Driver's Left Arm
    KinectStick theRightArm;                        // Kinect Driver's Right Arm
    Victor theSolenoid;                             // Victor controlling the solenoid

    
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
        
        //Define Solenoids used to move ball into shooter
        //theWedgeUp = new Solenoid(2);

        //Defines four E4P Motion Sensors at ports 1,2,3,4,5,6,7, and 8
       /* theFrontLeftEncoder = new Encoder(2,1);
        theFrontLeftEncoder.start();
        theRearLeftEncoder = new Encoder(6,5);
        theRearLeftEncoder.start();
        theFrontRightEncoder = new Encoder(4,3);
        theFrontRightEncoder.start();
        theRearRightEncoder = new Encoder(8,7);
        theRearRightEncoder.start();
        if (DEBUG == true){
            System.out.println("The Encoders constructed successfully");
        } */
        
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
        
        // Initialize the Ultrasonic sensors at analog port 1,2,3
        /*theUltrasonicSensorLeft = new TPAUltrasonicAnalogSensor(1);
         theUltrasonicSensorRight = new TPAUltrasonicAnalogSensor(4);
          theUltrasonicSensorFront = new TPAUltrasonicAnalogSensor(3);
        if (DEBUG == true){
            System.out.println("The ultrasonic sensors constructed successfully");
        } */
        
        //Initialize the DriverStationLCD
        theDriverStationLCD = DriverStationLCD.getInstance();
        if (DEBUG == true) {
            System.out.println("DriverStationLCD initialized");
        }
        
        //Initialize the Relay
        theSolenoid = new Victor(8);
        if (DEBUG == true) {
            System.out.println("Solenoid initialized");
        }
        
        //Stretches out your arms and gets them ready to work
            theLeftArm = new KinectStick(1);
            theRightArm = new KinectStick(2);
            if (DEBUG == true) {
                System.out.println("Arms Stretched");
            }
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
        //runShooter(0);
        hybridDrive(theLeftArm, theRightArm);
        if (DEBUG == true) {
            System.out.println("Hybrid Drive Called");
        }
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
        /*displaySpeed();
        if (DEBUG == true){
            System.out.println("displaySpeed called");
        } */
        
        // Run the shooter
        runShooter(determineShootingSpeed(theShootingStick));
        if (DEBUG == true){
            System.out.println("runShooter called");
        }
        
        // Run the Ultrasonic sensors.FRONT MUST ALWAYS BE THE LAST TO RUN!!!
        //runUltrasonicSensorRight(theUltrasonicSensorRight);
        //runUltrasonicSensorLeft(theUltrasonicSensorLeft);
        
        //runUltrasonicSensorFront(theUltrasonicSensorFront);
        //if (DEBUG == true){
        //    System.out.println("runUltrasonicSensorLeft, Right, and Front called");
        //} 
        
        dropBallIntoShooter(theShootingStick);
        if (DEBUG == true){
            System.out.println("dropBallintoShooter called");
        }
        
        determineJoystick();
        if (DEBUG == true){
            System.out.println("determineJoystick called");
        }
        
        shootWithJoystick(theShootingStick);
        if (DEBUG == true){
            System.out.println("shootWithJoystick called");
        }
/*
        // Brake the robot if no joysick input.
        brakeOnNeutral();
        if(DEBUG == true) {
            System.out.println("brakeOnNeutral called");
        } */
        
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
        theDriveRotation = (theRightStick.getY()); // Set the rotation to the value of the right stick
        //theRobotDrive.mecanumDrive_Polar(.3, 180, 0);
        if (DEBUG == true){ 
        System.out.println("The drive rotation in degrees" + theDriveRotation);
        System.out.println("The drive magnitude is" + theDriveMagnitude);
        System.out.println("The drive direction is" + theDriveDirection);
        }
        if (!driveBackwards(theLeftStick)){ // If the robot is driving forwards, feed it normal values
            theRobotDrive.mecanumDrive_Polar(theDriveMagnitude, theDriveDirection, theDriveRotation);
        }
        else if (driveBackwards(theLeftStick)){ // If the robot is driving backwards, edit driving values
            // Ask Andrew what this does
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
     * Author:  Daniel Hughes
     * Date:    1/29/12
     * Purpose: Display the average speed output from the encoders.
     * Inputs:  None
     * Outputs: None
     */    
    public void displaySpeed(){
    
        double theFrontLeftSpeed = theFrontLeftEncoder.getRate();
        double theRearLeftSpeed = theRearLeftEncoder.getRate();
        double theFrontRightSpeed = theFrontRightEncoder.getRate();
        double theRearRightSpeed = theRearRightEncoder.getRate();

        theSumFrontLeftSpeed += theFrontLeftSpeed;
        theSumRearLeftSpeed += theRearLeftSpeed;
        theSumFrontRightSpeed += theFrontRightSpeed;
        theSumRearRightSpeed += theRearRightSpeed;
        theNumberCollected++;

        if(theNumberCollected == 100){
            theNumberCollected =0;
            String theAverageFrontLeftSpeed = "FLS: " + theSumFrontLeftSpeed/100;
            String theAverageRearLeftSpeed = "RLS: " + theSumRearLeftSpeed/100;
            String theAverageFrontRightSpeed = "FRS: " + theSumFrontRightSpeed/100;
            String theAverageRearRightSpeed = "RRS: " + theSumRearRightSpeed/100; 

            theDriverStationLCD.println(DriverStationLCD.Line.kMain6, 1 , theAverageFrontLeftSpeed );
            theDriverStationLCD.println(DriverStationLCD.Line.kUser2, 1 , theAverageRearLeftSpeed );
            theDriverStationLCD.println(DriverStationLCD.Line.kUser3, 1 , theAverageFrontRightSpeed );
            //theDriverStationLCD.println(DriverStationLCD.Line.kUser4, 1 , theAverageRearRightSpeed );
            theDriverStationLCD.updateLCD();

            theSumFrontLeftSpeed=0;
            theSumRearLeftSpeed=0;
            theSumFrontRightSpeed=0;
            theSumRearRightSpeed=0;
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
            flipDriveDirection = !flipDriveDirection; // if flip is false, make it true and vice versa.
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
        if(shoot2ButtonPressable && aStick.getRawButton(2)){ // Toggle conveyor if the button is pressed
            conveyorMoving = !conveyorMoving;
            shoot2ButtonPressable = false;
        }
        if(!aStick.getRawButton(2)){ // On button release, allow it to be pressed again
            shoot2ButtonPressable = true;
        }
        if(conveyorMoving) {
            theConveyorMotor.set(aSpeed);
        }
        else if (!conveyorMoving) {
            theConveyorMotor.set(0);
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
        theTopShootingMotor.set(-aSpeed);
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
        if(aStick.getRawButton(4)){
            theShootingSpeed = SHOOTING_SPEED_4;
            joystickRunsShooter = false;
            button6Pressed = false;
        }
        if(aStick.getRawButton(3)){
            theShootingSpeed = SHOOTING_SPEED_3;
            joystickRunsShooter = false;
            button6Pressed = false;
        }
        if(aStick.getRawButton(5)){
            theShootingSpeed = SHOOTING_SPEED_5;
            joystickRunsShooter = false;
            button6Pressed = false;
        }
        if(aStick.getRawButton(9)) {
            theShootingSpeed = shoot9ButtonSpeed;
            joystickRunsShooter = false;
            button6Pressed = false;
        }
        if(aStick.getRawButton(10)) {
            theShootingSpeed = SHOOTING_SPEED_OFF;
            joystickRunsShooter = false;
            button6Pressed = false;
        }
        return theShootingSpeed;
    }
    /*--------------------------------------------------------------------------*/
        
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene
     * Date:    2/4/2012,3/5/2012 (Sumbhav Sethia)
     * Purpose: To run the ultrasonic sensor. A press of button 8 toggles it on 
     *          and off.
     * Inputs:  TPAUltrasonicAnalogSensor aSensor - the ultrasonic sensor
     * Outputs: 
     */
    public void runUltrasonicSensorLeft(TPAUltrasonicAnalogSensor aSensor){
        // Read in distance and add to an accumulator
        theDistanceLeft = aSensor.getDistance();
        theAccumulatedDistanceLeft = theAccumulatedDistanceLeft + theDistanceLeft;
        theDistancesCollectedLeft = theDistancesCollectedLeft + 1;
        // If enough distances have been collected, print the average value out and restart
        if (theDistancesCollectedLeft == AVERAGING_VALUE){
            theAveragedDistancePreTruncationLeft = theAccumulatedDistanceLeft/theDistancesCollectedLeft;
            theAverageDistanceTruncationStep1Left = (int) (theAveragedDistancePreTruncationLeft * 100);
            theAverageDistanceTruncatedLeft = theAverageDistanceTruncationStep1Left / 100.00;
            
            theDriverStationLCD.println(DriverStationLCD.Line.kUser4,1, "Left: " + theAverageDistanceTruncatedLeft);
            //theDriverStationLCD.updateLCD();
            theAccumulatedDistanceLeft = 0;
            theDistancesCollectedLeft = 0;
        }
        
        if(DEBUG == true){
            System.out.println("Left Sensor Enabled");
        }
    }
     /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene/Sumbhav Sethia
     * Date:    3/5/2012
     * Purpose: To run the ultrasonic sensor. A press of button 8 toggles it on 
     *          and off.
     * Inputs:  TPAUltrasonicAnalogSensor aSensor - the ultrasonic sensor
     * Outputs: 
     */
    public void runUltrasonicSensorRight(TPAUltrasonicAnalogSensor aSensor){
        // Read in distance and add to an accumulator
        theDistanceRight = aSensor.getDistance();
        theAccumulatedDistanceRight = theAccumulatedDistanceRight + theDistanceRight;
        theDistancesCollectedRight = theDistancesCollectedRight + 1;
        // If enough distances have been collected, print the average value out and restart
        if (theDistancesCollectedRight == AVERAGING_VALUE){
            theAveragedDistancePreTruncationRight = theAccumulatedDistanceRight/theDistancesCollectedRight;
            theAverageDistanceTruncationStep1Right = (int) (theAveragedDistancePreTruncationRight * 100);
            theAverageDistanceTruncatedRight = theAverageDistanceTruncationStep1Right / 100.00;
            
            theDriverStationLCD.println(DriverStationLCD.Line.kUser5,1, "Right: " + theAverageDistanceTruncatedRight);
           // theDriverStationLCD.updateLCD();
            theAccumulatedDistanceRight = 0;
            theDistancesCollectedRight = 0;
        }
        
        if(DEBUG == true){
            System.out.println("Right Sensor Enabled");
        }
    }
     /*--------------------------------------------------------------------------*/
    /*
     * Author:  Marissa Beene/Sumbhav Sethia
     * Date:    3/5/2012
     * Purpose: To run the ultrasonic sensor. A press of button 8 toggles it on 
     *          and off.
     * Inputs:  TPAUltrasonicAnalogSensor aSensor - the ultrasonic sensor
     * Outputs: 
     */
    public void runUltrasonicSensorFront(TPAUltrasonicAnalogSensor aSensor){
        // Read in distance and add to an accumulator
        theDistanceFront = aSensor.getDistance();
        theAccumulatedDistanceFront = theAccumulatedDistanceFront + theDistanceFront;
        theDistancesCollectedFront = theDistancesCollectedFront + 1;
        // If enough distances have been collected, print the average value out and restart
        if (theDistancesCollectedFront == AVERAGING_VALUE){
            theAveragedDistancePreTruncationFront = theAccumulatedDistanceFront/theDistancesCollectedFront;
            theAverageDistanceTruncationStep1Front = (int) (theAveragedDistancePreTruncationFront * 100);
            theAverageDistanceTruncatedFront = theAverageDistanceTruncationStep1Front / 100.00;
            
            theDriverStationLCD.println(DriverStationLCD.Line.kUser6,1, "Front: " + theAverageDistanceTruncatedFront);
            theDriverStationLCD.updateLCD();
            theAccumulatedDistanceFront = 0;
            theDistancesCollectedFront = 0;
        }
        
        if(DEBUG == true){
            System.out.println("Front Sensor Enabled");
        }
    }
    /*--------------------------------------------------------------------------*/
    /* 
     * Author:  Sumbhav Sethia
     * Date:    1/29/2012
     * Purpose: To drop one and only one ball into the shooter mechanism
     * Inputs:  Joystick aStick
     * Outputs: None
     */        
    public void dropBallIntoShooter(Joystick aStick) {
        /* if(aStick.getRawButton(1)) {
            theRelay.set(Relay.Value.kForward);
        }            
        if(aStick.getRawButton(11)) {
            theRelay.set(Relay.Value.kOff);
        } */
        if(aStick.getRawButton(1)) {        //Throws ball into shooter
            if(DEBUG == true) {
                theDriverStationLCD.println(DriverStationLCD.Line.kUser5, 1, "Shooter Fired");
                theDriverStationLCD.updateLCD();
            }
            if(theRelayFlag) {
                theSolenoid.set(1);
                theRelayFlag = false;
            }            
        } 
        else if(!aStick.getRawButton(1)) {
            if(!theRelayFlag) {
                theSolenoid.set(0);
                theRelayFlag = true;
            }
            theDriverStationLCD.println(DriverStationLCD.Line.kUser5, 1, "               ");
            theDriverStationLCD.updateLCD();
        } 
    }
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Sumbhav Sethia and Gennaro De Luca
     * Date:    2/11/2012, 2/12/2012
     * Purpose: Hybrid Mode Drive
     * Inputs:  Two Arms
     * Outputs: None
     */    
    public void hybridDrive(KinectStick aLeftArm, KinectStick aRightArm) {
        theHybridDriveRotation = aLeftArm.getY();
        theHybridDriveMagnitude = aRightArm.getY();
        /*if(aRightArm.getRawButton(3)){
            theHybridDriveDirection = -90;
        }
        else if(aLeftArm.getRawButton(4)){
            theHybridDriveDirection = 90;
        }
        else if(!(aLeftArm.getRawButton(4) && aRightArm.getRawButton(3))){
            theHybridDriveDirection = 0;
        }

        if(aRightArm.getRawButton(5)) {
            if(theRelayFlag) {
                theSolenoid.set(1);
                theRelayFlag = false;
            }
            else if (!theRelayFlag) {
                theSolenoid.set(0);
            }   
            theDriverStationLCD.println(DriverStationLCD.Line.kUser3, 1, "Button 3 pressed");
            theDriverStationLCD.updateLCD();
        }
        else {
            theDriverStationLCD.println(DriverStationLCD.Line.kUser3, 1, "                  ");
            theDriverStationLCD.updateLCD();
            theRelayFlag = true;
        }
        if(aLeftArm.getRawButton(7) && kinect7ButtonPressable) {
            kinect7ButtonPressable = false;
            theConveyorRunning = false;
            theShooterRunning = !theShooterRunning;
        }
        else if(!aRightArm.getRawButton(7)) {
            kinect7ButtonPressable = true;
        }
        if(aLeftArm.getRawButton(8) && kinect8ButtonPressable){
            kinect8ButtonPressable = false;
            theConveyorRunning = !theConveyorRunning;
            theShooterRunning = false;
        }
        else if(!aRightArm.getRawButton(8)){
            kinect8ButtonPressable = true;
        }
        if(theConveyorRunning){
            theConveyorMotor.set(CONVEYOR_SPEED);
        }
        if(!theConveyorRunning){
            theConveyorMotor.set(0);
        }
        if(theShooterRunning){
            runShooter(HYBRID_SHOOTING_SPEED);
        }
        if(!theShooterRunning){
            runShooter(0);
        } */
        theRobotDrive.mecanumDrive_Polar(theHybridDriveMagnitude, theHybridDriveDirection, theHybridDriveRotation );
        Timer.delay(.01);   // Delay 10ms to reduce processing load
    }
    /*----------------------------------------------------------------------------*/
    
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Gennaro De Luca
     * Date:    2/14/12
     * Purpose: Determine which joystick is which
     * Inputs:  None
     * Outputs: None
     */    
    public void determineJoystick() {
        if(theLeftStick.getRawButton(6)) {
            theDriverStationLCD.println(DriverStationLCD.Line.kMain6, 1, "Left       ");
        }
        else if(theRightStick.getRawButton(6)) {
            theDriverStationLCD.println(DriverStationLCD.Line.kMain6, 1, "Right      ");
        }
        else if(theShootingStick.getRawButton(6)) {
            theDriverStationLCD.println(DriverStationLCD.Line.kMain6, 1, "Shooting");
        }
        theDriverStationLCD.updateLCD();
    }
   /*--------------------------------------------------------------------------*/
    /*
     * Author:  Sumbhav Sethia
     * Date:    2/15/2012
     * Purpose: Speed Controlled Shooting
     * Inputs:  aJoystick
     * Outputs: 
     */    
    public void shootWithJoystick(Joystick aStick){

        if(aStick.getRawButton(7) && shoot7ButtonPressable) {
            joystickRunsShooter = !joystickRunsShooter;
            shoot7ButtonPressable = false;
            button6Pressed = false;
        }
        if(joystickRunsShooter) {
            theTopShootingMotor.set(-aStick.getMagnitude());
            theBottomShootingMotor.set(-(aStick.getMagnitude()));
            if(aStick.getRawButton(6) && shoot6ButtonPressable) {
                theJoystickSpeed = aStick.getMagnitude();
                joystickRunsShooter = false;
                button6Pressed = true;
            }
            else if(!aStick.getRawButton(6) && !shoot6ButtonPressable) {
                shoot6ButtonPressable = true;
            }
            if(aStick.getRawButton(8) && shoot8ButtonPressable) {
            shoot8ButtonPressable = false;
            shoot9ButtonSpeed = aStick.getMagnitude();
            }
            else if(!aStick.getRawButton(8) && !shoot8ButtonPressable) {
                shoot8ButtonPressable = true;
            }
        }
        else if(!joystickRunsShooter && button6Pressed) {
            theTopShootingMotor.set(-theJoystickSpeed);
            theBottomShootingMotor.set(-theJoystickSpeed);
        }
        if (!aStick.getRawButton(7) && !shoot7ButtonPressable){
            shoot7ButtonPressable = true;
        }
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
    