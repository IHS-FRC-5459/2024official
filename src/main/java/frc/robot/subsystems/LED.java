// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

public class LED {

    static CANdle _candle = new CANdle(Constants.LED.CANdleID, Constants.LED.CANDdleCANbus);

    public LED() {
    
    }

    // sets the colors of an LED
    public static void setLED(int LEDId, int[] RGBVal) {

        _candle.setLEDs(RGBVal[0], RGBVal[1], RGBVal[2], 0, LEDId, 1);
    
    }

}
