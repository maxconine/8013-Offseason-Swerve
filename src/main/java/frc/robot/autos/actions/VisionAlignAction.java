package frc.robot.autos.actions;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.lib.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class VisionAlignAction implements Action {

    // required subsystem instances
    Swerve mSwerve = Swerve.getInstance();
    Limelight mLimelight = Limelight.getInstance();

    SwerveDriveKinematics mKinematics;

    // vision align controller
    ProfiledPIDController visionPIDController = new ProfiledPIDController(Constants.VisionAlignConstants.kP,
                                                        Constants.VisionAlignConstants.kI,
                                                        Constants.VisionAlignConstants.kD,
                                                        Constants.VisionAlignConstants.kThetaControllerConstraints);

    public VisionAlignAction(SwerveDriveKinematics kinematics) {
        mKinematics = kinematics;
        visionPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void start() {
        
    }

    @Override
    public void update() {

        // double rotation = 0.0;

        // if (mLimelight.hasTarget()) {
        //     double currentAngle = mSwerve.getPose().getRotation().getDegrees();
        //     rotation = visionPIDController.calculate(currentAngle, currentAngle - mLimelight.getOffset()[0]);
        // }

        // var targetChassisSpeeds = new ChassisSpeeds(0, 0, rotation);
        // var targetModuleStates = mKinematics.toSwerveModuleStates(targetChassisSpeeds);

        // mSwerve.setModuleStates(targetModuleStates);    

       // mSwerve.visionAlignDrive(new Translation2d(), true);
    }

    @Override
    public boolean isFinished() {
        return mLimelight.isAutonomousAimed();
    }

    @Override
    public void done() {
        mSwerve.setModuleStates(
            Constants.SwerveConstants.m_kinematics.toSwerveModuleStates((
                ChassisSpeeds.fromFieldRelativeSpeeds(0., 0., 0., Rotation2d.fromDegrees(0).getWPIRotation2d()))));
    }
}