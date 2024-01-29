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
    for(DigitalInput b: beambreaks){
      if(b.get()){
        return true;
      }
    }
    return false;
  }


  public BeamBreak.Result getResults(){
    return new Result(
      beambreaks[0].get(), beambreaks[1].get()
    );
  }

  public class Result {
    public boolean bottomState = false;
    public boolean topState = false;

    public Result(boolean t, boolean b){
      bottomState = b;
      topState = t;
    }
  }
}


