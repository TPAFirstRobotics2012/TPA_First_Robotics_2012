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
    static boolean theDigitalConnected;
    
    TPAUltrasonicAnalogSensor(int aAnalogChannel) {
        theAnalogOutput = new AnalogChannel (aAnalogChannel);
        theScaling = 5.0/512;
        theDistance = 0;
        theDigitalConnected = false;
    }
    TPAUltrasonicAnalogSensor(int aDigitalChannel, int aAnalogChannel){
        theDigitalOutput = new DigitalOutput (aDigitalChannel);
        theAnalogOutput = new AnalogChannel (aAnalogChannel);
        theScaling = 5.0/512;
        theDistance = 0;
        theDigitalConnected = true;
    }
    TPAUltrasonicAnalogSensor(int aDigitalChannel, int aAnalogChannel, int aPowerChannel){
      
        theDigitalOutput = new DigitalOutput(aDigitalChannel);
        theAnalogOutput = new AnalogChannel (aAnalogChannel);
        thePower = new AnalogChannel (aPowerChannel);
        theScaling = thePower.getVoltage()/512;
        theDistance = 0;
        theDigitalConnected = true;
    }
    
    public void enable(){
        if(theDigitalConnected == true) {
            theDigitalOutput.set(true);
        }
    }
    
    public void disable(){
        if(theDigitalConnected == true) {
            theDigitalOutput.set(false);
        }
    }
    
    public void refreshScaling(){
        theScaling = thePower.getVoltage()/512;
    }
    
    public double getDistance(){
        aVoltage = theAnalogOutput.getVoltage();
        theDistance = aVoltage/theScaling;
        return theDistance;        
    }
}