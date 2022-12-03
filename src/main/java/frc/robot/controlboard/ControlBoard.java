package frc.robot.controlboard;


import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants;
import frc.robot.lib.math.geometry.Rotation2d;
import frc.robot.lib.math.geometry.Translation2d;

public class ControlBoard {
    private final double kSwerveDeadband = Constants.stickDeadband;

    // private int mLastDpadLeft = -1;
    // private int mLastDpadRight = -1;

    // private final int kDpadUp = 0;
    // private final int kDpadRight = 90;
    // private final int kDpadDown = 180;
    // private final int kDpadLeft = 270;

    private static ControlBoard mInstance = null;

    public enum SwerveCardinal {
        NONE(0),

        FORWARDS(0),
        LEFT(270),
        RIGHT(90),
        BACKWARDS(180),

        FAR_FENDER(143),
        RIGHT_FENDER(233),
        LEFT_FENDER(53),
        CLOSE_FENDER(323);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final GenericHID m_driver;

    private ControlBoard() {
        m_driver = new GenericHID(1);
    }
    


    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = getYAxisRight();
        double strafeAxis = getXAxisRight();

        forwardAxis = Constants.SwerveConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis :-strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

            double scaled_x = tAxes.x() - (deadband_vector.x()) / (1 - deadband_vector.x());
            double scaled_y = tAxes.y() - (deadband_vector.y()) / (1 - deadband_vector.y());
            return new Translation2d(scaled_x, scaled_y).scale(Constants.SwerveConstants.maxSpeed);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = getXAxisLeft();
        rotAxis = Constants.SwerveConstants.invertRotAxis ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < kSwerveDeadband) {
            return 0.0;
        } else {
            return Constants.SwerveConstants.maxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
        }
    }

    public boolean zeroGyro() {
        return m_driver.getRawButtonPressed(13);
    }

    // public SwerveCardinal getSwerveSnap() {

    //     // FENDER SNAPS
    //     if (driver.getButton(Button.A)) {
    //         return SwerveCardinal.CLOSE_FENDER;
    //     }
    //     if (driver.getButton(Button.B)) {
    //         return SwerveCardinal.LEFT_FENDER;
    //     }
    //     if (driver.getButton(Button.X)) {
    //         return SwerveCardinal.RIGHT_FENDER;
    //     }
    //     if (driver.getButton(Button.Y)) {
    //         return SwerveCardinal.FAR_FENDER;
    //     }

    //     // CARDINAL SNAPS

    //     switch (driver.getController().getPOV()) {
    //         case kDpadUp:
    //             return SwerveCardinal.FORWARDS;
    //         case kDpadLeft:
    //             return SwerveCardinal.RIGHT;
    //         case kDpadRight:
    //             return SwerveCardinal.LEFT;
    //         case kDpadDown:
    //             return SwerveCardinal.BACKWARDS;
    //         default:
    //             return SwerveCardinal.NONE;
    //     }
            
    // }

    // // hood adjustment
    // public int getHoodManualAdjustment() {
    //     int pov_read = operator.getController().getPOV();
    //     switch(pov_read){
    //         case 0:
    //             return 1;
    //         case 180:
    //             return -1;
    //         default:
    //             return 0;
    //     }
    // }

    // public boolean getResetHoodAdjust() {
    //     return operator.getButton(CustomXboxController.Button.START);
    // }

    // // Align swerve drive with target
    // public boolean getVisionAlign() {
    //     return driver.getButton(Button.RB);
    // }

    // public boolean getClimbAlign() {
    //     return driver.getTrigger(Side.RIGHT);
    // }

    // // Locks wheels in X formation
    // public boolean getBrake() {
    //     return driver.getButton(Button.LB);
    // }

    // // Intake Controls
    // public boolean getIntake() {
    //     return operator.getTrigger(CustomXboxController.Side.RIGHT);
    // }

    // public boolean getReject() {
    //     return operator.getTrigger(CustomXboxController.Side.LEFT);
    // }

    // // Superstructure Controls
    // public boolean getShoot() {
    //     return operator.getController().getYButtonPressed();
    // }

    // public boolean getPrep() {
    //     return operator.getController().getAButtonPressed();
    // }
 
    // public boolean getFender() {
    //     return operator.getController().getBButtonPressed();
    // }

    // public boolean getSpit() {
    //     return operator.getController().getXButtonPressed();
    // }

    // public boolean getDisableColorLogic() {
    //     boolean wasPressed = operator.getController().getPOV() == kDpadRight && mLastDpadRight != kDpadRight;
    //     mLastDpadRight = operator.getController().getPOV();
    //     return wasPressed;
    // }

    // public boolean getDisableIntakeLogic() {
    //     boolean wasPressed = operator.getController().getPOV() == kDpadLeft && mLastDpadLeft != kDpadLeft;
    //     mLastDpadLeft = operator.getController().getPOV();
    //     return wasPressed;
    // }

    // public boolean getManualEject() {
    //     return operator.getButton(Button.LB);
    // }

    // // Climber Controls
    // public boolean getEnterClimbMode() {
    //     return operator.getButton(Button.LB) && operator.getButton(Button.RB) && operator.getTrigger(Side.LEFT) && operator.getTrigger(Side.RIGHT);
    // }

    // public boolean getExitClimbMode() {
    //     return operator.getButton(Button.BACK) && operator.getButton(Button.START);
    // }
    
    // public boolean getToggleOpenLoopClimbMode() {
    //     return operator.getController().getLeftStickButtonPressed();
    // }

    // public boolean getClimberRezero() {
    //     return operator.getController().getRightStickButtonPressed();
    // }

    // public boolean getClimberRetract() {
    //     return operator.getController().getXButtonPressed();
    // }

    // public boolean getClimberExtend() {
    //     return operator.getController().getAButtonPressed();
    // }

    // public boolean getTraversalClimb() {
    //     return operator.getButton(Button.LB) && operator.getController().getYButtonPressed();
    // }
    
    // public boolean getHighBarClimb() {
    //     return operator.getButton(Button.LB) && operator.getController().getBButtonPressed();
    // }

    private double getXAxisLeft() { //beutiful precision
        double XAxisLeft;
        if (m_driver.getRawAxis(Constants.leftXAxis) > 0) {
            XAxisLeft = m_driver.getRawAxis(Constants.leftXAxis) / 0.826;
        } else
        XAxisLeft = m_driver.getRawAxis(Constants.leftXAxis) / 0.807;

        XAxisLeft = XAxisLeft - 0.03937007;
        //SmartDashboard.putNumber("remote XAxisLeft", XAxisLeft);
        return XAxisLeft;

    }

    private double getYAxisRight() {
        double YAxisRight;
        YAxisRight = m_driver.getRawAxis(Constants.rightYAxis);
        if (m_driver.getRawAxis(Constants.rightYAxis) > 0) {
             YAxisRight = m_driver.getRawAxis(Constants.rightYAxis) / 0.7052;
         } else { // negative value
           YAxisRight = m_driver.getRawAxis(Constants.rightYAxis) / 0.6449;
         }

        YAxisRight = YAxisRight - 0.047244; 
        //SmartDashboard.putNumber("remote YAxisRight", YAxisRight);
        return YAxisRight;
    }

    private double getXAxisRight() {
        double XAxisRight;
        XAxisRight = m_driver.getRawAxis(Constants.rightXAxis);
         if (m_driver.getRawAxis(Constants.rightXAxis) > 0) {
             XAxisRight = m_driver.getRawAxis(Constants.rightXAxis) / 0.8443;
         } else {
             XAxisRight = m_driver.getRawAxis(Constants.rightXAxis) / 0.8193;
         }
        
        XAxisRight = XAxisRight - 0.055118;

        //SmartDashboard.putNumber("remote XAxisRight", XAxisRight);
        return XAxisRight;
    }
    
}
