package frc.robot.autos.modes;

import java.util.List;

import frc.robot.Constants;
import frc.robot.autos.actions.AutoModeEndedException;
import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoTrajectoryReader;
//import frc.robot.autos.actions.LambdaAction;
import frc.robot.autos.actions.RaceAction;
import frc.robot.autos.actions.SeriesAction;
import frc.robot.autos.actions.SwerveTrajectoryAction;
import frc.robot.autos.actions.WaitAction;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ObstacleCourseMode extends AutoModeBase {

        // Swerve instance
        private final Swerve mSwerve = Swerve.getInstance();

        // required PathWeaver file paths
        String file_path_a = "paths/ObstacleCourse/Obstacle A.path";
        String file_path_b = "paths/ObstacleCourse/Obstacle B.path";
        String file_path_c = "paths/ObstacleCourse/Obstacle C.path";
        String file_path_d = "paths/ObstacleCourse/Obstacle D.path";
        String file_path_e = "paths/ObstacleCourse/Obstacle E.path";
        String file_path_f = "paths/ObstacleCourse/Obstacle F.path";

        // trajectories
        private Trajectory traj_path_a;
        private Trajectory traj_path_b;
        private Trajectory traj_path_c;
        private Trajectory traj_path_d;
        private Trajectory traj_path_e;
        private Trajectory traj_path_f;

        // trajectory actions
        SwerveTrajectoryAction driveToIntakeSecondCargo;
        SwerveTrajectoryAction driveToFirstShot;
        SwerveTrajectoryAction driveToIntakeThirdCargo;
        SwerveTrajectoryAction driveToIntakeAtTerminal;
        SwerveTrajectoryAction driveToHumanPlayerWait;
        SwerveTrajectoryAction driveToSecondShot;

        public ObstacleCourseMode() {

                SmartDashboard.putBoolean("Auto Finished", false);

                // define theta controller for robot heading
                var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                /* CREATE TRAJECTORIES FROM FILES */

                // Intake second cargo
                traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToIntakeSecondCargo = new SwerveTrajectoryAction(traj_path_a,
                                mSwerve::getPose, Constants.SwerveConstants.m_kinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(270.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to lineup to third cargo
                traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToFirstShot = new SwerveTrajectoryAction(traj_path_b,
                                mSwerve::getPose, Constants.SwerveConstants.m_kinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(200.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Intake third cargo
                traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                1.7,
                                                0.0,
                                                0.0));

                driveToIntakeThirdCargo = new SwerveTrajectoryAction(traj_path_c,
                                mSwerve::getPose, Constants.SwerveConstants.m_kinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(200.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to intake 4th cargo at terminal
                traj_path_d = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_d,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToIntakeAtTerminal = new SwerveTrajectoryAction(traj_path_d,
                                mSwerve::getPose, Constants.SwerveConstants.m_kinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(225.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to human player wait pose
                traj_path_e = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_e,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToHumanPlayerWait = new SwerveTrajectoryAction(traj_path_e,
                                mSwerve::getPose, Constants.SwerveConstants.m_kinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(225.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to second shot
                traj_path_f = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_f,
                                Constants.AutoConstants.createConfig(
                                    Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToSecondShot = new SwerveTrajectoryAction(traj_path_f,
                                mSwerve::getPose, Constants.SwerveConstants.m_kinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(210.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);
        }

        @Override
        protected void routine() throws AutoModeEndedException {
                System.out.println("Running Obstacle course");
                SmartDashboard.putBoolean("Auto Finished", false);

                // disable auto ejecting
                //runAction(new LambdaAction(() -> mSuperstructure.setEjectDisable(false)));
                System.out.println("starting firt action");
                runAction(
//                                new SeriesAction(List.of(
                                                driveToIntakeSecondCargo);

                                // new SeriesAction(List.of(
                                //                 // new WaitAction(0.1),
                                //                 new LambdaAction(() -> mSuperstructure.setWantPrep(true)),
                                //                 new LambdaAction(() -> mSuperstructure.setWantIntake(true))))));


                // start vision aiming
                //runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

                System.out.println("waiting");
                runAction(new WaitAction(3.00));


                // run trajectory for third cargo
                System.out.println("starting Second action");
                runAction(driveToFirstShot);


                runAction(new WaitAction(3.00));
                // shoot first & second cargo
                // runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
                // runAction(new WaitAction(1.0));
                // runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

                // run trajectory to intake third shot cargo
                runAction(driveToIntakeThirdCargo);
                

                runAction(new WaitAction(3.00));
                // shoot third cargo
                // runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
                // runAction(new WaitAction(0.90));
                // runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

                // stop vision aiming to control robot heading
                // runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

                // run trajectory for terminal
                runAction(driveToIntakeAtTerminal);

                runAction(new WaitAction(3.00));

                runAction(driveToHumanPlayerWait);

                runAction(new WaitAction(3.00));

                //runAction(new WaitAction(0.25));

                // start vision aiming when driving to shot pose
                //runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

                // run trajectory to drive to second shot pose
                runAction(driveToSecondShot);


                runAction(new WaitAction(3.00));

                // shoot cargo
                //runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));

                System.out.println("Finished auto!");
                SmartDashboard.putBoolean("Auto Finished", true);
        }

        @Override
        public Pose2d getStartingPose() {
                return driveToIntakeSecondCargo.getInitialPose();
        }
}