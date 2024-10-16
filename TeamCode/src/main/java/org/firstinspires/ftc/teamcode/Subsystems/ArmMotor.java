package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotParametersPT;

public class ArmMotor {
    private RobotParametersPT params;
    private DcMotor ArmMotor;



    public ArmMotor(RobotParametersPT params, HardwareMap hardwareMap){
        ArmMotor = hardwareMap.get(DcMotor.class, params.armMotorName);
    }

    public void stateUpdate(RobotParametersPT.ArmState armState, double power){
        switch(armState){
            case PIVOT_UP:
                pivotUp(power);
                break;

            case PIVOT_DOWN:
                pivotDown(power);
                break;

            case STOP:
                stop();
                break;
        }
    }

    public void pivotUp(double power){
        ArmMotor.setPower(power);
    }

    public void pivotDown(double power){
        ArmMotor.setPower(-power);
    }

    public void stop(){
        ArmMotor.setPower(0);
    }

}