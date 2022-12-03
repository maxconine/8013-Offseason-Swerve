package frc.robot.shuffleboard;


import java.util.Optional;

import frc.robot.Constants;
import frc.robot.RobotState;
//import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.lib.vision.AimingParameters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldView {
    private Field2d mField2d = new Field2d();
    private Swerve mSwerve = Swerve.getInstance();
    //private Superstructure mSuperstructure = Superstructure.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private Pose2d[] mModulePoses = new Pose2d[4];
    private Pose2d mRobotPose = new Pose2d();

    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    private void updateSwervePoses() {
        if(mSwerve.getPose() != null) mRobotPose = mSwerve.getPose();
        else mRobotPose = new Pose2d();
        for (int i = 0; i < mModulePoses.length; i++) {
            Translation2d updatedPosition = Constants.SwerveConstants.swerveModuleLocations[i]
                    .rotateBy(mRobotPose.getRotation()).plus(mRobotPose.getTranslation());
            Rotation2d updatedRotation = mSwerve.getStates()[i].angle.plus(mRobotPose.getRotation());
            if(mSwerve.getStates()[i].speedMetersPerSecond < 0.0) {
                updatedRotation = updatedRotation.plus(Rotation2d.fromDegrees(180));;
            }
            mModulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
        }
    }

    public void update() {
        updateSwervePoses();

        mField2d.setRobotPose(mRobotPose);
        mField2d.getObject("Swerve Modules").setPoses(mModulePoses);

        Optional<AimingParameters> target = getRealAimingParameters();
        if (target.isPresent()) {
            Pose2d robotToTarget = target.get().getVehicleToGoal().getWpilibPose2d();
            mField2d.getObject("Target")
                    .setPose(new Pose2d(mRobotPose.getTranslation().plus(robotToTarget.getTranslation()),
                            mRobotPose.getRotation().rotateBy(robotToTarget.getRotation())));
        }

        mField2d.getObject("Predicted Robot Pose").setPose(mRobotState.getPredictedFieldToVehicle(Constants.VisionConstants.kLookaheadTime).getWpilibPose2d());
    }

    public Optional<AimingParameters> getRealAimingParameters() {
        Optional<AimingParameters> aiming_params = RobotState.getInstance().getAimingParameters(-1,
                Constants.VisionConstants.kMaxGoalTrackAge);
        if (aiming_params.isPresent()) {
            return aiming_params;
        } else {
            Optional<AimingParameters> default_aiming_params = RobotState.getInstance().getDefaultAimingParameters();
            return default_aiming_params;
        }
    }
}