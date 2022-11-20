package frc.robot.commands;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DrivetrainSubsystem;

// import java.util.function.DoubleSupplier;

// public class DefaultDriveCommand extends CommandBase {
//     private final DrivetrainSubsystem m_drivetrainSubsystem;

//     private final DoubleSupplier m_translationXSupplier;
//     private final DoubleSupplier m_translationYSupplier;
//     private final DoubleSupplier m_rotationSupplier;

//     public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
//                                DoubleSupplier translationXSupplier,
//                                DoubleSupplier translationYSupplier,
//                                DoubleSupplier rotationSupplier) {
//         this.m_drivetrainSubsystem = drivetrainSubsystem;
//         this.m_translationXSupplier = translationXSupplier;
//         this.m_translationYSupplier = translationYSupplier;
//         this.m_rotationSupplier = rotationSupplier;

//         addRequirements(drivetrainSubsystem);
//     }

//     @Override
//     public void execute() {
//         // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
//         m_drivetrainSubsystem.drive(
//                 ChassisSpeeds.fromFieldRelativeSpeeds(
//                         m_translationXSupplier.getAsDouble(),
//                         m_translationYSupplier.getAsDouble(),
//                         m_rotationSupplier.getAsDouble(),
//                         m_drivetrainSubsystem.getGyroscopeRotation()
//                 )
//         );
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
//     }
// }

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

   // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
     //   m_drivetrainSubsystem,
       // () -> getYAxisLeft() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
       // () -> getXAxisLeft() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
       // () -> getXAxisRight() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));