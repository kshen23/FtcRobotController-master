package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TurnTest extends BaseRobot{
    private ElapsedTime timer=new ElapsedTime();
    double power=1;
    public void init() {
        super.init();
        timer.reset();
    }
    public void loop() {
        super.loop();
        if (timer.seconds()<3){
            leftBackDriveMotor.setPower(-power);
            leftFrontDriveMotor.setPower(-power);
            rightBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);
        } else {
            leftBackDriveMotor.setPower(0);
            leftFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }
    }
}
