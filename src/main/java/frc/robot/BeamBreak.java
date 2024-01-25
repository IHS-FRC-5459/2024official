// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class BeamBreak {
  public static final int beambreakChannel = 7;

  DigitalInput[] beambreaks;

    /**
   * Creates a beambreak.
   *
   * @param channels channels[0] = lower BB, channels[1] = higher BB
   */
    public BeamBreak(int[] channels){
      beambreaks = new DigitalInput[channels.length];
      for (int i = 0; i < channels.length; i++) {
        beambreaks[i] = new DigitalInput(channels[i]);
      }
      
    }
      //is beambreak broken
  public boolean hasNote(){
    for(Boolean b:hasNoteArray()){
      if(b){
        return true;
      }
    }
    return false;
  }

  public boolean[] hasNoteArray(){
    boolean[] ret = new boolean[beambreaks.length];
    for (int i = 0; i < beambreaks.length; i++) {
        ret[i] = beambreaks[i].get();
      }

    return ret;
  }
}
