package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake/Driving", group = "TeleOP")
public class IntakeTest extends OpMode {
    private DcMotor Intake1;
    //private DcMotor Intake2;
    private Servo gate;

    @Override
    public void init() {
        Intake1 = hardwareMap.get(DcMotor.class, "intake1");
        //Intake2 = hardwareMap.get(DcMotor.class, "intake2");
        Intake1.setDirection(DcMotor.Direction.FORWARD);
        //Intake2.setDirection(DcMotor.Direction.FORWARD);
        gate = hardwareMap.get(Servo.class,"gate");

    }
    public void loop(){
        if (gamepad1.left_trigger_pressed){
            Intake1.setPower(1.0);
            //Intake2.setPower(1.0);
        }else{
            Intake1.setPower(0.0);
            //Intake2.setPower(0.0);
        }
        if (gamepad1.x){
            gate.setPosition(0.8);
        }else{
            gate.setPosition(0.2);
        }

    }
}
