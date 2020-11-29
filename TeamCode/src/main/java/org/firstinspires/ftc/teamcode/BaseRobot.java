/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;



public class BaseRobot extends OpMode {


    public DcMotor leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor, armMotor1, armMotor2;

    BNO055IMU imu;
    Orientation angles;
    double oldAngle = 0;

    @Override
    public void init() {
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //this stuff is for the gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //
    }

    public void start() {
        reset_drive_encoders();
    }

    public void loop() {
        //double TARGET_ENC = K_PPIN_DRIVE * (inches);
    }

    //THIS IS THE BEGINNING OF WHAT SHOULD BE COMMENTED OUT
///*
//    public boolean auto_drive(double power, double inches) {
//        double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * (inches);
//        telemetry.addData("Target_enc: ", TARGET_ENC);
//        double speed = power;
//        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
//            leftFrontDriveMotor.setPower(0);
//            leftBackDriveMotor.setPower(0);
//            rightFrontDriveMotor.setPower(0);
//            rightBackDriveMotor.setPower(0);
//            return true;
//        } else {
//            speed = Range.clip(speed, -1, 1);
//            leftFrontDriveMotor.setPower(speed);
//            leftBackDriveMotor.setPower(speed);
//            rightFrontDriveMotor.setPower(speed);
//            rightBackDriveMotor.setPower(speed);
//            return false;
//        }
//    }
//
//    /**
//     * @param power:   the speed to turn at. Negative for left.
//     * @param degrees: the number of degrees to turn.
//     * @return Whether the target angle has been reached.
//     */
//
//    public boolean auto_turn(double power, double degrees) {
//        double TARGET_ENC = Math.abs(ConstantVariables.K_PPDEG_DRIVE * degrees);
//        telemetry.addData("D99 TURNING TO ENC: ", TARGET_ENC);
//        double speed = Range.clip(power, -1, 1);
//        leftFrontDriveMotor.setPower(-speed);
//        leftBackDriveMotor.setPower(-speed);
//        rightFrontDriveMotor.setPower(speed);
//        rightBackDriveMotor.setPower(speed);
//        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
//            leftFrontDriveMotor.setPower(0);
//            leftBackDriveMotor.setPower(0);
//            rightFrontDriveMotor.setPower(0);
//            rightBackDriveMotor.setPower(0);
//            return true;
//        } else {
//            return false;
//        }
//    }
//    public boolean auto_mecanum(double power, double inches) {
//        double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * (inches);
//        telemetry.addData("Target_enc: ", TARGET_ENC);
//        //correction = checkDirection();
//        double leftFrontPower = Range.clip(0 - power, -1.0, 1.0);
//        double leftBackPower = Range.clip(0 + power, -1.0, 1.0);
//        double rightFrontPower = Range.clip(0 + power, -1.0, 1.0);
//        double rightBackPower = Range.clip(0 - power, -1.0, 1.0);
//        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
//            leftFrontDriveMotor.setPower(0);
//            leftBackDriveMotor.setPower(0);
//            rightFrontDriveMotor.setPower(0);
//            rightBackDriveMotor.setPower(0);
//            return true;
//        } else {
//            leftFrontDriveMotor.setPower(leftFrontPower);
//            leftBackDriveMotor.setPower(leftBackPower);
//            rightFrontDriveMotor.setPower(rightFrontPower);
//            rightBackDriveMotor.setPower(rightBackPower);
//            return false;
//        }
//    }
//
//THIS IS THE END OF WHAT SHOULD BE COMMENTED OUT

    public void tankanum_drive(double rightPwr, double leftPwr, double lateralpwr) {
        //leftPwr *= -1;

        double leftFrontPower = Range.clip(leftPwr + lateralpwr, -1.0, 1.0);
        double leftBackPower = Range.clip(leftPwr - lateralpwr, -1.0, 1.0);
        double rightFrontPower = Range.clip(rightPwr + lateralpwr, -1.0, 1.0);
        double rightBackPower = Range.clip(rightPwr - lateralpwr, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);
    }

    public void reset_drive_encoders() {
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double oldAngle = angles.firstAngle;
    }

    public boolean rotate(double degrees) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (Math.abs(angles.firstAngle - oldAngle) >= Math.abs(degrees));
    }

    public void goForward(int power) {
        leftFrontDriveMotor.setPower(power);
        leftBackDriveMotor.setPower(power);
        rightFrontDriveMotor.setPower(power);
        rightBackDriveMotor.setPower(power);
    }

    public void goRight(int power) {
        leftFrontDriveMotor.setPower(-power);
        leftBackDriveMotor.setPower(power);
        rightFrontDriveMotor.setPower(-power);
        rightBackDriveMotor.setPower(power);
    }

    public void stop() {
        leftFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
    }

    public void setRotatePower (double power) {
        leftBackDriveMotor.setPower(power);
        leftFrontDriveMotor.setPower(power);
        rightBackDriveMotor.setPower(-power);
        rightFrontDriveMotor.setPower(-power);
    }
}