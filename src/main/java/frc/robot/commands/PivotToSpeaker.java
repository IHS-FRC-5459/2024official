// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.Pivot;

public class PivotToSpeaker extends Command {
  private Pivot s_Pivot;

  /** Creates a new PivotToAmp. */
  public PivotToSpeaker(Pivot s_Pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Pivot=s_Pivot;
    addRequirements(s_Pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      s_Pivot.setRotationVoltage(s_Pivot.calculateRotationVoltage(s_Pivot.calculateGoalAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
