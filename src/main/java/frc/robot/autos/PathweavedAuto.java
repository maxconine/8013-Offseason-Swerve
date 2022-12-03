package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.autos.actions.AutoModeEndedException;
import frc.robot.autos.actions.LambdaAction;
import frc.robot.autos.actions.SwerveTrajectoryAction;



public class PathweavedAuto extends AutoModeBase{

        // Swerve instance 
        private final Swerve mSwerve = Swerve.getInstance();

        // required PathWeaver trajectory paths
        String path = "paths/gobackleft.path"; //TODO: rename this for desired path
        
        // trajectories
        SwerveTrajectoryAction testTrajectoryAction;

        public PathweavedAuto() {

            var thetaController =
                new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        
            // read trajectories from PathWeaver and generate trajectory actions
            Trajectory traj_path = AutoTrajectoryReader.generateTrajectoryFromFile(path, Constants.AutoConstants.defaultSpeedConfig);
            testTrajectoryAction = new SwerveTrajectoryAction(traj_path,
                                                                mSwerve::getPose, Constants.SwerveConstants.m_kinematics,
                                                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                                thetaController,
                                                                () -> Rotation2d.fromDegrees(0.0),
                                                                mSwerve::getWantAutoVisionAim,
                                                                mSwerve::setModuleStates);
            
        }
    
        @Override
        protected void routine() throws AutoModeEndedException {
            System.out.println("Running test mode auto!");
    
            // reset odometry at the start of the trajectory
            runAction(new LambdaAction(() -> mSwerve.resetOdometry(testTrajectoryAction.getInitialPose())));
    
            runAction(testTrajectoryAction);
            
            System.out.println("Finished auto!");
        }
    
        @Override
        public Pose2d getStartingPose() {
            return testTrajectoryAction.getInitialPose();
        }
    }
    

