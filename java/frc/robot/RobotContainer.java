// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();


  private final GenericHID m_driver = new GenericHID(1); // for interLINK Controller
  // private final Joystick m_operator = new Joystick(2); //for operator
  // controller
  
  private final Swerve s_Swerve = new Swerve();  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

      /* Subsystems */
     
  
        boolean fieldRelative = true;
        boolean openLoop = true;
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, m_driver, fieldRelative, openLoop));
    

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new Button(() -> m_driver.getRawButtonPressed(13))
        .whenPressed(s_Swerve::zeroGyro);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }

  


}