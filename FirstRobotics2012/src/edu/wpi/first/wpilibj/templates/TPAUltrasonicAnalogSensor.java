/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.AnalogChannel;
/**
 *
 * @author Marissa
 */

public class TPAUltrasonicAnalogSensor {
    
    DigitalOutput theDigitalOutput;
    AnalogChannel theAnalogOutput;
    AnalogChannel thePower;
    double aVoltage;
    double theScaling;
    double theDistance;
    
    TPAUltrasonicAnalogSensor(int aDigitalChannel, int aAnalogChannel){
        theDigitalOutput = new DigitalOutput (aDigitalChannel);
        theAnalogOutput = new AnalogChannel (aAnalogChannel);
        theScaling = 5.0/512;
        theDistance = 0;
    }
    TPAUltrasonicAnalogSensor(int aDigitalChannel, int aAnalogChannel, int aPowerChannel){
      
        theDigitalOutput = new DigitalOutput(aDigitalChannel);
        theAnalogOutput = new AnalogChannel (aAnalogChannel);
        thePower = new AnalogChannel (aPowerChannel);
        theScaling = thePower.getVoltage()/512;
        theDistance = 0;
    }
    
    public void enable(){
        theDigitalOutput.set(true);     // Enable the ultrasonic sensor
    }
    
    public void disable(){
        theDigitalOutput.set(false);    // Disable the ultrasonic sensor
    }
    
    public void refreshScaling(){       // Refresh the scaling for the distance
        theScaling = thePower.getVoltage()/512;
    }
    
    public double getDistance(){        // Calculate the distance an object is from the sensor
        aVoltage = theAnalogOutput.getVoltage();
        theDistance = aVoltage/theScaling;
        return theDistance;        
    }
}