package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shotButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
    private final JoystickButton climberLockOut = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton climberDownButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton climberUpButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    
    private final JoystickButton ampPivot = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton ampShoot = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kB.value);

    /* Sensors */
    public final int[] channels = {7,6};
    public final Vision vision = new Vision();
    public final BeamBreak beambreak = new BeamBreak(channels);
    /* Subsystems */

    public final Swerve s_Swerve = new Swerve(vision);
   // private final Pivot s_Pivot = new Pivot(vision, beambreak);
   // private final EndEffector s_EndEffector = new EndEffector(beambreak);
   // private final Climber s_Climber = new Climber();



    private final SendableChooser<Command> autoChooser;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //named commands for swerve
       // NamedCommands.registerCommand("intake", new EEIntake(s_EndEffector));//intaking
      //  NamedCommands.registerCommand("shoot", autoShoot());//shooting

         s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );/* 

          s_Pivot.setDefaultCommand(
          new PivotToNeutral(
            s_Pivot
            )
      );

        s_Climber.setDefaultCommand(
            new ClimberDefault(s_Climber)
        );;
       
        s_EndEffector.setDefaultCommand(
            new EENeutral(s_EndEffector)
       );
*/
        

        





        // Configure the button bindings
        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        /* 
        climberLockOut.and(climberDownButton).whileTrue(new ClimberTranslate(s_Climber, Constants.Climber.climberDownPower));
        climberLockOut.and(climberUpButton).whileTrue((new ClimberTranslate(s_Climber, Constants.Climber.climberUpPower)));

        
        intakeButton.whileTrue(new EEIntake(s_EndEffector));
        shotButton.whileTrue(shootSpeaker());
        ampPivot.whileTrue(s_Pivot.withNoteTimeout(new PivotToAmp(s_Pivot)));
        ampShoot.whileTrue(new EEShootAmpSpeed(s_EndEffector));
        */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }


    //large command builders:
    //for during teleop
  /*    public Command shootSpeaker(){
        if(vision.validTarget()){
            double goalPivotAngle = vision.calculateGoalAngle();
            
            return Commands.parallel(
                        s_Swerve.centerVisionBuilder(),
                        s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot, goalPivotAngle)),
                        Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 2)).andThen(
                        s_EndEffector.EETimedShooterBuilder(new EESpeakerWithVision(s_EndEffector)),
                        Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 2)), new EESpinUp(s_EndEffector)))
        );
        } else {
            //subwoofer shot sequence. no center or vision use for angle
            return Commands.parallel(
            s_Pivot.withNoteTimeout(new PivotAtSubwoofer(s_Pivot)),
            Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 2)).andThen(
            s_EndEffector.EETimedShooterBuilder(new EESpeakerNoVision(s_EndEffector)),
            Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 2)), new EESpinUp(s_EndEffector)))
        );
        }
        
    }

//for during auto
    public Command autoShoot(){
        if(vision.validTarget()){
            double goalPivotAngle = vision.calculateGoalAngle();

            return Commands.parallel(
                s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot, goalPivotAngle)),
                Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 2)).andThen(
                s_EndEffector.EETimedShooterBuilder(new EESpeakerWithVision(s_EndEffector)),
                Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > goalPivotAngle - 2)), new EESpinUp(s_EndEffector)))
            ).withTimeout(4);
        } else {
            //subwoofer shot sequence. no center or vision use for angle
            return Commands.parallel(
                s_Pivot.withNoteTimeout(new PivotAtSubwoofer(s_Pivot)),
                Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 2)).andThen(
                s_EndEffector.EETimedShooterBuilder(new EESpeakerNoVision(s_EndEffector)),
                Commands.deadline(Commands.waitUntil(() -> (s_Pivot.getAngle() > Constants.Arm.subwooferAngle - 2)), new EESpinUp(s_EndEffector)))
            ).withTimeout(4);
        }
    }*/

/* 

    public Command shootWithCenterSpeaker(){
        //used in auto 
        return Commands.parallel(
            s_Swerve.centerVisionBuilder().andThen(s_EndEffector.EETimedShooterBuilder(new EEShootFullSpeed(s_EndEffector))),
            s_Pivot.withNoteTimeout(new PivotToAmp(s_Pivot))
        );
    }*/
}
