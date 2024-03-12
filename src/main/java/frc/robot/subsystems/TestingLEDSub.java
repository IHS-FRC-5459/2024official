// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSiD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import java.util.HashMap;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TestingLEDSub extends SubsystemBase {
    public Boolean shootingToSpeaker;
    public Boolean shootingToAmp;
    private Boolean jammed;
    private Boolean targetInRange;
    private Boolean targetVisible;
    private Boolean hasNote;
    private Boolean intaking;
    private Boolean empty;
    private String pivotAngleString;
    private Double pivotAngle;
    private String shootingStr;
    private int speakerCounter = Constants.LED.blinkingDelay;
    private int ampCounter = Constants.LED.blinkingDelay;
    private int intakeCounter = Constants.LED.blinkingDelay;
    private Boolean speakerBlink = true;
    private Boolean ampBlink = true;
    private Boolean intakeBlink = true;
    private Boolean shouldExcecute = true;
    //static CANdle _candle = new CANdle(Constants.LED.CANdleID, Constants.LED.CANDdleCANbus);
    private CANdle CANdle;
    public TestingLEDSub(CANdle CANdle) {
        this.CANdle = CANdle;
    }
    public void setState(String state,String value){
        state = state.toLowerCase();
        Boolean myBool;
        if(value == "true"){
            myBool = true;
        }else{
            myBool = false;
        }
        switch(state){
            case "shootingtospeaker":
                shootingToSpeaker = myBool;
            case "shootingtoamp":
                shootingToAmp = myBool;
        }
    }

    // sets the colors of an LED
    public void setLED(int[] RGBVal) {

        CANdle.setLEDs(RGBVal[0], RGBVal[1], RGBVal[2]);
    
    }
    public void setBrightness(double brightness){
        CANdle.configBrightnessScalar(brightness);
    }
@Override
public void periodic(){
    if(shouldExcecute){
    setBrightness(1);
    }
    shouldExcecute = true;

    //pivotAngleString = SmartDashboard.getString("pivot angle","0").toString();
    //pivotAngle = Double.parseDouble(pivotAngleString);
    //shootingStr = SmartDashboard.getString("shooting","false");

    //shootingToSpeaker = pivotAngle < 90 && shootingStr == "true";
    //shootingToAmp = SmartDashboard.getString("Pivot CMD") && shootingStr == "false";
    jammed = SmartDashboard.getBoolean("bb top",false) && SmartDashboard.getBoolean("bb bottom",false);
    targetInRange = SmartDashboard.getNumber("distance",0) <= 3.5 && 0.1 < SmartDashboard.getNumber("distance",0);
    targetVisible = SmartDashboard.getNumber("distance",0) > 0.1;
    hasNote = SmartDashboard.getBoolean("bb bottom",false);
    intaking = Math.abs(SmartDashboard.getNumber("intake",0)) > 0.5;
    empty = !hasNote;

    /*if(shootingStr == "true" && hasNote){
        if(speakerCounter % Constants.LED.blinkingDelay == 0){
            speakerBlink = !speakerBlink;
        }
        if(speakerBlink){
            setBrightness(1);
            setLED(Constants.Colors.greenColor);
        }else{
            setBrightness(0);
        }
        speakerCounter--;
    }else if(shootingStr == "true" && hasNote){
        if(ampCounter % Constants.LED.blinkingDelay == 0){
            ampBlink = !ampBlink;
        }
        if(ampBlink){
            setBrightness(1);
            setLED(Constants.Colors.blueColor);
        }else{
            setBrightness(0);
        }
        ampCounter--;
    }else */if(jammed){
        setLED(Constants.Colors.purpleColor);
    }else if(targetInRange){
        setLED(Constants.Colors.greenColor);
    }else if(targetVisible){
        setLED(Constants.Colors.cyanColor);
    }else if(hasNote){
        setLED(Constants.Colors.orangeColor);
    }else if(intaking){
        if(intakeCounter % Constants.LED.blinkingDelay == 0){
            intakeBlink = !intakeBlink;
        }
        if(intakeBlink){
            setBrightness(1);
            setLED(Constants.Colors.orangeColor);
        }else{
            setBrightness(0);
        }
        intakeCounter--;
    }else if(empty){
        setBrightness(0);
        shouldExcecute = false;
    }

    //SmartDashboard.putString("Am I excecuting???","yes");





    
}
}

