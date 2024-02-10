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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton eeTest = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton climberDownButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton climberUpButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);


    /* Sensors */
    public final int[] channels = {7};
    public final Vision vision = new Vision();
    public final BeamBreak beambreak = new BeamBreak(channels);
    /* Subsystems */

    private final Swerve s_Swerve = new Swerve(vision);
    private final Pivot s_Pivot = new Pivot(vision, beambreak);
    private final EndEffector s_EndEffector = new EndEffector(beambreak);
    private final Climber s_Climber = new Climber();



    private final SendableChooser<Command> autoChooser;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //named commands for swerve
      //  NamedCommands.registerCommand("Intake", new EEIntake(s_EndEffector));//intaking
      //  NamedCommands.registerCommand("Shoot", shootNoCenterSpeaker());//shooting

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

          s_Pivot.setDefaultCommand(
            new PivotToNeutral(
                s_Pivot
            )
        );
       
s_EndEffector.setDefaultCommand(
            new EENeutral(s_EndEffector)
        );

        

        





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
        climberDownButton.whileTrue(new ClimberTranslate(s_Climber, Constants.Climber.climberDownPower));
        climberUpButton.whileTrue(new ClimberTranslate(s_Climber, Constants.Climber.climberUpPower));

        //eeTest.whileTrue(new EERunner(s_EndEffector));
        //eeTest.whileTrue(Commands.race(new PivotToAmp(s_Pivot), new EERunner(s_EndEffector)));
        //centerButton.onTrue(new CenterVision(s_Swerve));
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
    //shoot sequence without vision center
 /*    public Command shootNoCenterSpeaker(){
        return Commands.parallel(
            s_Pivot.withNoteTimeout(new PivotToSpeaker(s_Pivot)),
            s_EndEffector.EETimedShooterBuilder(new EEShootFullSpeed(s_EndEffector))
        );
    }

    public Command shootNoCenterAmp(){
        return Commands.parallel(
            s_Pivot.withNoteTimeout(new PivotToAmp(s_Pivot)),
            s_EndEffector.EETimedShooterBuilder(new EEShootAmpSpeed(s_EndEffector))
        );
    }


    public Command shootWithCenterSpeaker(){//used in auto 
        return Commands.parallel(
            s_Swerve.centerVisionBuilder().andThen(s_EndEffector.EETimedShooterBuilder(new EEShootFullSpeed(s_EndEffector))),
            s_Pivot.withNoteTimeout(new PivotToAmp(s_Pivot))
        );
    }*/
}
