/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.EllipseDescriptor;
import edu.wpi.first.wpilibj.image.EllipseMatch;
import edu.wpi.first.wpilibj.image.MonoImage;
import edu.wpi.first.wpilibj.image.NIVisionException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class TPARobot extends IterativeRobot {

    ColorImage theColorImage;                                   // Image from camera
    AxisCamera theAxisCamera;                                   // The camera
    DriverStationLCD theDriverStationLCD;                       // Object representing the driver station   
    // Drive mode selection
    int theDriveMode;                                           // The actual drive mode that is currently selected.
    static final int UNINITIALIZED_DRIVE = 0;                   // Value when no drive mode is selected
    static final int ARCADE_DRIVE = 1;                          // Value when arcade mode is selected 
    static final int TANK_DRIVE = 2;                            // Value when tank drive is selected
    public double theMaxSpeed;                                  // Multiplier for speed, determined by Z-Axis on left stick
    static final boolean DEBUG = true;                          // Debug Trigger
    static final double STOP_VALUE = 0.1;                       // Value sent to each motor when the robot is stopping
    double theFrontLeftOutput;                                  // The output sent to the front left motor
    double theRearLeftOutput;                                   // The output sent to the rear left motor
    double theFrontRightOutput;                                 // The output sent to the front right motor
    double theRearRightOutput;                                  // The output sent to the rear right motor
    Encoder theFrontLeftEncoder;                                // The encoder at the front left motor
    Encoder theRearLeftEncoder;                                 // The encoder at the rear left motor
    Encoder theFrontRightEncoder;                               // The encoder at the front right motor
    Encoder theRearRightEncoder;                                // The encoder at the rear right motor
    Joystick theRightStick;                                     // Right joystick
    Joystick theLeftStick;                                      // Left joystick
    TPARobotDriver theRobotDrive;                               // Robot Drive System
    double theDriveDirection;                                   // Direction the robot will move
    double theDriveMagnitude;                                   // Speed the robot will move at
    double theDriveRotation;                                    // Value the robot will rotate
    EllipseDescriptor theEllipseDescriptor;                     // Define the ball


    
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
               
        // Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
	theRightStick = new Joystick(1);
	theLeftStick = new Joystick(2);
        if (DEBUG == true){
           System.out.println("The Joysticks constructed successfully"); 
        }
        
        // Define a robot drive object with the front left motor at port 1, rear left at port 2, front right at port 3, and rear right at port 4
        theRobotDrive = new TPARobotDriver(1,2,3,4);
        if (DEBUG == true){
            System.out.println("theRobotDrive constructed successfully");
        }

        //Initialize the DriverStationLCD
        theDriverStationLCD = DriverStationLCD.getInstance();
        if (DEBUG) {
            System.out.println("DriverStationLCD initialized");
        }
        
        //Initialize the AxisCamera
        theAxisCamera = AxisCamera.getInstance(); 
        theAxisCamera.writeResolution(AxisCamera.ResolutionT.k320x240);        
        theAxisCamera.writeBrightness(50);
        
        if (DEBUG) {
            System.out.println("AxisCamera initialized");
        }
        
        try{
            if (theAxisCamera.freshImage()){
                theColorImage = theAxisCamera.getImage();
            }
        }
        catch(NIVisionException b) {
            System.out.println(b);
        }
        catch(AxisCameraException b) {
            System.out.println(b);
        }
        // Initialize the Drive Mode to Uninitialized
        theDriveMode = UNINITIALIZED_DRIVE;
        
        // Default the robot to not move
        theDriveDirection = 0;
        theDriveMagnitude = 0;
        theDriveRotation = 0;
        if (DEBUG == true){
            System.out.println("The robot set to not move");
        }
        
        // Define the ball
        theEllipseDescriptor = new EllipseDescriptor(
            /* minMajorRadius */ 3.75,
            /* maxMajorRadius */ 4.25,
            /* minMinorRadius */ 3.75,
            /* maxMinorRadius */ 4.25);
        
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
        if(DEBUG) {
            System.out.println("getCameraImage called");
        }
        
        try {
            System.out.println(findCircle());
        }
        catch(NIVisionException b) {
            System.out.println(b);
        }
        
    }
    /*--------------------------------------------------------------------------*/
    
    
  
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Gennaro De Luca
     * Date:    11/26/2011 (Gennaro De Luca)
     * Purpose: To determine the speed multiplier based on the "Z" wheel on 
     *          the left joystick. If the "Z" wheel is up (negative), the multiplier remains at 1.
     *          Otherwise, the multiplier is set to one-half.
     * Inputs:  None
     * Outputs: None
     */    
    public void setMaxSpeed(){
        
        if (theLeftStick.getZ() <= 0) {    // Logitech Attack3 has z-polarity reversed; up is negative
            theMaxSpeed = 1;               //set the multiplier to default value of 1
            if (DEBUG == true){
                System.out.println("theLeftStick.getZ called");
            }
        }
        else if (theLeftStick.getZ() > 0) {
            theMaxSpeed = 0.5;             //set the multiplier to half default, 0.5
            if (DEBUG == true) {
                System.out.println("theLeftStick.getZ called");
            }
        }
        theRobotDrive.setMaxSpeed(theMaxSpeed); //tests the multiplier
    }
    /*--------------------------------------------------------------------------*/
    
    /*--------------------------------------------------------------------------*/
    /*
     * Author:  Gennaro
     * Date:    1/15/2012  
     * Purpose: Test to see if the camera can see the ball
     * Inputs:  None
     * Outputs: Boolean - does the camera see the ball.
     */    
    
    public boolean findCircle() throws NIVisionException {
        
        //int width = theColorImage.getWidth();
        //int height = theColorImage.getHeight();
        MonoImage theMonoImage = theColorImage.getLuminancePlane();
        EllipseMatch[] theEllipseMatch = theMonoImage.detectEllipses(theEllipseDescriptor);
        theMonoImage.free();
        if(theEllipseMatch.length>0) {
            return true;
        }
        else {
            return false;
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
