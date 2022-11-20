package frc.robot.autos;

import frc.robot.autos.modes.FiveBallMode;
import frc.robot.autos.modes.ObstacleCourseMode;
import frc.robot.shuffleboard.ShuffleBoardInteractions;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING, 
        TEST_PATH_AUTO,
        FIVE_BALL_AUTO,
        OBSTACLE_COURSE
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        System.out.println("new automode selector");
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Path Mode", DesiredMode.TEST_PATH_AUTO);
        mModeChooser.addOption("Five Ball Auto", DesiredMode.FIVE_BALL_AUTO);
        mModeChooser.addOption("Obstacle Course - slalom", DesiredMode.OBSTACLE_COURSE);

        System.out.println("Before adding auto mode chooser");
        SmartDashboard.putData(mModeChooser);
        //ShuffleBoardInteractions.getInstance().getSwerveTab().add("Auto Mode", mModeChooser).withSize(2, 1);
        System.out.println("After adding auto mode chooser");
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothingMode());

        case TEST_PATH_AUTO:
            return Optional.of(new PathweavedAuto());
        
        case FIVE_BALL_AUTO:
            return Optional.of(new FiveBallMode());
            
        case OBSTACLE_COURSE:
            return Optional.of(new ObstacleCourseMode());
            
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}