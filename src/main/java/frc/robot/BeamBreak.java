// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class BeamBreak {
  public static final int beambreakChannel = 7;

  DigitalInput beambreak;

    public BeamBreak(){
        beambreak = new DigitalInput(beambreakChannel);
    }
      //is beambreak broken
  public boolean hasNote(){
    return !beambreak.get();
  }
}
