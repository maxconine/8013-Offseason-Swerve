// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeExecutor;
import frc.robot.autos.AutoModeSelector;
import frc.robot.lib.CTREConfigs;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.logger.LoggingSystem;
import frc.robot.loops.CrashTracker;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.shuffleboard.ShuffleBoardInteractions;
import frc.robot.controlboard.ControlBoard;
//import frc.robot.shuffleboard.tabs.SystemsTab;
//import frc.robot.RobotState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private Command m_autonomousCommand;

  //private RobotContainer m_robotContainer;

  private ShuffleBoardInteractions mShuffleBoardInteractions;
  public static CTREConfigs ctreConfigs;

  private final ControlBoard mControlBoard = ControlBoard.getInstance();

  // instantiate enabled and disabled loopers
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();
  // instantiate logging looper
  private final Looper mLoggingLooper = new Looper();

  // auto instances
  private AutoModeExecutor mAutoModeExecutor;
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

  // private final RobotStateEstimator mRobotStateEstimator =
  // RobotStateEstimator.getInstance();

  // logging system
  private LoggingSystem mLogger = LoggingSystem.getInstance();

  private final Swerve mSwerve = Swerve.getInstance();
  private final Limelight mLimelight = Limelight.getInstance();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  public Robot() {
    CrashTracker.logRobotConstruction();
  }

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    //m_robotContainer = new RobotContainer();

    ctreConfigs = new CTREConfigs();
    mShuffleBoardInteractions = ShuffleBoardInteractions.getInstance();

    try {
      CrashTracker.logRobotInit();

      mLogger.registerLoops(mLoggingLooper);

      RobotState.getInstance().reset(Timer.getFPGATimestamp(), new frc.robot.lib.math.geometry.Pose2d());

      mSwerve.resetOdometry(new Pose2d());
      mSwerve.resetToAbsolute();

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    mSwerve.updateSwerveOdometry();
    CommandScheduler.getInstance().run();
    mShuffleBoardInteractions.update();
    mEnabledLooper.outputToSmartDashboard();

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
		CrashTracker.logAutoInit();

		try {
			// reset states
			//mSuperstructure.stop();

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
        System.out.println("auto mode present, reseting odometry to auto starting pose");
				mSwerve.resetOdometry(autoMode.get().getStartingPose());
			}

			mAutoModeExecutor.start();

			mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

			// set champs pride automation
			//mLEDs.setChampsAutoAnimation();	

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit()  {
		try {

			if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			//mSuperstructure.setWantEject(false, false);

			//mClimber.setBrakeMode(true);

			//mSuperstructure.setEjectDisable(false);

			mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

			// clear any previous automation from auto
			//mLEDs.clearAnimation();

			// set states for teleop init
			//mSuperstructure.setInitialTeleopStates();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
      try {
  
        if (mAutoModeExecutor != null) {
                  mAutoModeExecutor.stop();
              }
  
        mLimelight.setLed(Limelight.LedMode.ON);
        mLimelight.outputTelemetry();
  
        // call operator commands container from superstructure
        //mSuperstructure.updateOperatorCommands();
        //mSuperstructure.updateLEDs();
  
        //mLEDs.updateState();
  
        
        /* SWERVE DRIVE */
        // hold left bumper
        // if (mControlBoard.getBrake()) {
        //   mSwerve.setLocked(true);
        // } else {
        //   mSwerve.setLocked(false);
        // }
  
        if (mControlBoard.zeroGyro()) {
          mSwerve.zeroGyro();
        }
  
        // if (mControlBoard.getSwerveSnap() != SwerveCardinal.NONE) {
        //   mSwerve.startSnap(mControlBoard.getSwerveSnap().degrees);
        // }
        Translation2d swerveTranslation = new Translation2d(mControlBoard.getSwerveTranslation().x(),
            mControlBoard.getSwerveTranslation().y());
        double swerveRotation = mControlBoard.getSwerveRotation();
  
        // if (mControlBoard.getClimbAlign()) {
        //   mSwerve.angleAlignDrive(swerveTranslation, 270, true);
        // } else if (mControlBoard.getVisionAlign()) {
        //   mSwerve.visionAlignDrive(swerveTranslation, true);
        // } else {
        //   mSwerve.drive(swerveTranslation, swerveRotation, true, true);
        // }

        mSwerve.drive(swerveTranslation, swerveRotation, true, true);
      
  
      } catch (Throwable t) {
        t.printStackTrace();
        CrashTracker.logThrowableCrash(t);
        throw t;
      }
    }



  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    try {
      // reset states
      // mSuperstructure.stop();

      CrashTracker.logDisabledInit();
      mEnabledLooper.stop();
      mDisabledLooper.start();

      mLoggingLooper.stop();

      mLimelight.setLed(Limelight.LedMode.OFF);
      mLimelight.triggerOutputs();

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }

    if (mAutoModeExecutor != null) {
      mAutoModeExecutor.stop();
    }

    // Reset all auto mode state.
    mAutoModeSelector.reset();
    mAutoModeSelector.updateModeCreator();
    mAutoModeExecutor = new AutoModeExecutor();

  }




  @Override
  public void disabledPeriodic() {
    try {

      mDisabledLooper.outputToSmartDashboard();

      mAutoModeSelector.updateModeCreator();

      mSwerve.resetToAbsolute();

      // update alliance color from driver station while disabled
      // mColorSensor.updateAllianceColor();
      // update linear offset for rb ratio
      // mColorSensor.updateColorOffset();

      // mLEDs.updateColor(mColorSensor.getAllianceColor());

      mLimelight.setLed(Limelight.LedMode.ON);
      mLimelight.writePeriodicOutputs();
      mLimelight.outputTelemetry();

      Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
      if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
        System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
        mAutoModeExecutor.setAutoMode(autoMode.get());
      }

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }

  }


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}