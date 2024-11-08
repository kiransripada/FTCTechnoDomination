package org.firstinspires.ftc.teamcode.Subsystems;
//JJ
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

public class ClawPT {
    private RobotParametersPT params;
    private final Servo ClawServo1;
    private final CRServo ClawServo2;

    public ClawPT(RobotParametersPT params, HardwareMap hardwareMap) {
        ClawServo1 = hardwareMap.get(Servo.class, params.ClawServoName1);
        ClawServo2 = hardwareMap.get(CRServo.class, params.ClawServoName2);
    }

    public void stateUpdate(RobotParametersPT.ClawState clawState, double power) {
        switch(clawState){
            case TURN_IN:
                turnIn(power);
                break;

            case TURN_OUT:
                turnOut(power);
                break;

            case STOP:
                stop();
                break;
        }
    }


    public void turnIn(double power){
       // ClawServo1.setPower(power);
       // ClawServo2.setPower(-power);

        ClawServo1.setPosition(1);
    }

    public void turnOut(double power){
        ClawServo1.setPosition(0);
        ClawServo2.setPower(power);
    }

    public void stop(){
        ClawServo1.setPosition(0.5);
        ClawServo2.setPower(0);

    }

    public double getPosition(){
        return ClawServo1.getPosition();
    }

}
