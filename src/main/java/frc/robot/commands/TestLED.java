// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.TestingLEDSub;
import edu.wpi.first.wpilibj2.command.Command;

public class TestLED extends Command {
  /** Creates a new TestLED. */
  private int id;
  private int[] rgbVal;
  private TestingLEDSub led;
  public TestLED(int id,int[] rgbVal,TestingLEDSub led) {
    this.id = id;
    this.rgbVal = rgbVal;
    this.led = led;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setLED(id,rgbVal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    final int[] white = {0,0,0};
    led.setLED(id, white);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
