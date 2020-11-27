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
        resetAngle();
    }
    public void loop() {
        super.loop();
        if (timer.seconds()<3) {
            setRotatePower(1);
        } else {
            stop();
        }
    }
}
