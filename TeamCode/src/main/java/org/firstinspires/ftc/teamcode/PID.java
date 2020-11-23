package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
public class PID extends BaseRobot{


    public static double P=1;
    public static double I=0;
    public static double D=0;
    public void rotate (double speed, double degrees){
        double arclength=(degrees*Math.PI/180)*radius;
        double error=arclength-Math.abs(rightFrontDriveMotor.getCurrentPosition())*PPI;

        double proportional=P*error;
        double integral +=error
        rightFrontDriveMotor.setPower(-power);
        rightBackDriveMotor.setPower(-power);
        leftFrontDriveMotor.setPower(power);
        leftBackDriveMotor.setPower(power);
        if (error < arclength) {
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
        }

    }
}
