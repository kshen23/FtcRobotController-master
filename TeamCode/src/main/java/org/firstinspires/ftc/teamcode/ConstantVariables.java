package org.firstinspires.ftc.teamcode;

public class ConstantVariables {
    public static double WIDTH = 30;
    public static double LENGTH = 45;
    public static double radius = Math.sqrt(Math.pow(WIDTH/2, 2) + Math.pow(LENGTH/2, 2));
    public static double PPR=537.6;
    public static double GEAR_RATIO =4/3;
    public static double END_PPR=PPR*GEAR_RATIO;
    public static double WHEEL_DIAMETER = 9.6;
    public static double WHEEL_CIRCUMFERENCE=WHEEL_DIAMETER*Math.PI;
    public static double PPI=END_PPR*WHEEL_CIRCUMFERENCE;
}
