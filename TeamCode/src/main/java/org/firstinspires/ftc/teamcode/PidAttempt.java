package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PidAttempt extends BaseRobot{
    BNO055IMU               imu;
    int P, I, D;

    int integral, previous_error, setpoint = 0;
    double error, derivative, time, rcw;
    Orientation angles;
    private ElapsedTime timer;

    int stage=0;

    public void init() {
        super.init();
        //////////////////////////////////////////
        P=1;
        I=0;
        D=0;
        //////////////////////////////////////////
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        timer = new ElapsedTime();
    }

    public void loop() {
        if (stage==0) {
            setpoint=-90;
            PID();
            rotate(rcw);
        }
    }
    public void PID(){

        error = setpoint - (angles.firstAngle); // Error = Target - Actual
        time=timer.seconds();
        this.integral += (error*time); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        derivative = (error - this.previous_error) / .02;
        this.rcw = P*error + I*this.integral + D*derivative;

        timer.reset();
    }

    public void rotate (double power) {
        leftBackDriveMotor.setPower(-power);
        leftFrontDriveMotor.setPower(-power);
        rightBackDriveMotor.setPower(power);
        rightBackDriveMotor.setPower(power);
    }
}
