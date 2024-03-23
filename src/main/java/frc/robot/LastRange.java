// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Calendar;

/**Add your docs here. */
public class LastRange {
    private static double default_ = 0.0;
    private static double lastValue = default_;
    private static Calendar lastTime = Calendar.getInstance();
    private static Double duration = 1000.0;/*secondsToMiliseconds(1)*/;
    //private static boolean alreadySetOff = false;
    private static double get(){
        //if(!alreadySetOff){
            if(Calendar.getInstance().compareTo(lastTime) <= duration){
        //        alreadySetOff = true;
                return lastValue;
            }
        //}
        return 0.0;
    }
    private static void set(double newVal){
        if(newVal > default_){
        //    alreadySetOff = false;
            lastTime = Calendar.getInstance();
            lastValue = newVal;
        }
    }
}
