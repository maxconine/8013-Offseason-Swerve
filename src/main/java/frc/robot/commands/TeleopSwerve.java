package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;

    private Swerve s_Swerve;
    private GenericHID controller;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, GenericHID m_driver, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = m_driver;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = -getYAxisRight();
        double xAxis = getXAxisRight();
        double rAxis = -getXAxisLeft();


        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        //yAxis = -1*yAxis;
        //rAxis = -1*rAxis;

        translation = new Translation2d(xAxis, yAxis).times(Constants.SwerveConstants.maxSpeed);
        rotation = rAxis * Constants.SwerveConstants.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }

    // private double getYAxisLeft() { //calibrated
    //     double YAxisLeft;
    //     if (controller.getRawAxis(Constants.leftYAxis) > 0) {
    //         YAxisLeft = controller.getRawAxis(1) / 0.91;
    //     } else { // negative value
    //         YAxisLeft = controller.getRawAxis(1) / 0.85;
    //     }
    //     YAxisLeft = YAxisLeft + 0.1369;
    //     SmartDashboard.putNumber("remote YAxisLeft", YAxisLeft);
    //     return 0;
    // }

    private double getXAxisLeft() { //beutiful precision
        double XAxisLeft;
        if (controller.getRawAxis(Constants.leftXAxis) > 0) {
            XAxisLeft = controller.getRawAxis(Constants.leftXAxis) / 0.826;
        } else
        XAxisLeft = controller.getRawAxis(Constants.leftXAxis) / 0.807;

        XAxisLeft = XAxisLeft - 0.03937007;
        //SmartDashboard.putNumber("remote XAxisLeft", XAxisLeft);
        return XAxisLeft;

    }

    private double getYAxisRight() {
        double YAxisRight;
        YAxisRight = controller.getRawAxis(Constants.rightYAxis);
        if (controller.getRawAxis(Constants.rightYAxis) > 0) {
             YAxisRight = controller.getRawAxis(Constants.rightYAxis) / 0.7052;
         } else { // negative value
           YAxisRight = controller.getRawAxis(Constants.rightYAxis) / 0.6449;
         }

        YAxisRight = YAxisRight - 0.047244; 
        //SmartDashboard.putNumber("remote YAxisRight", YAxisRight);
        return YAxisRight;
    }

    private double getXAxisRight() {
        double XAxisRight;
        XAxisRight = controller.getRawAxis(Constants.rightXAxis);
         if (controller.getRawAxis(Constants.rightXAxis) > 0) {
             XAxisRight = controller.getRawAxis(Constants.rightXAxis) / 0.8443;
         } else {
             XAxisRight = controller.getRawAxis(Constants.rightXAxis) / 0.8193;
         }
        
        XAxisRight = XAxisRight - 0.055118;

        //SmartDashboard.putNumber("remote XAxisRight", XAxisRight);
        return XAxisRight;
    }
}