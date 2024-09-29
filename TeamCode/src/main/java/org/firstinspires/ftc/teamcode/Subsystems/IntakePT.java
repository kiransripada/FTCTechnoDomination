package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

public class IntakePT {
    private RobotParametersPT params;
    private DcMotor IntakeMotor;



    public IntakePT(RobotParametersPT params, HardwareMap hardwareMap){
        IntakeMotor = hardwareMap.get(DcMotor.class, params.intakeMotorName);
    }

    public void stateUpdate(RobotParametersPT.IntakeState intakeState, double power){
        switch(intakeState){
            case PULL_IN:
                pullIn(power);
                break;

            case PUSH_OUT:
                pushOut(power);
                break;

            case STOP:
                stop();
                break;
        }
    }

    public void pullIn(double power){
        IntakeMotor.setPower(power);
    }

    public void pushOut(double power){
        IntakeMotor.setPower(-power);
    }

    public void stop(){
        IntakeMotor.setPower(0);
    }

}
