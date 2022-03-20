package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ExampleHardware {

    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;
    Servo arm;
    Servo claw;

    public ExampleHardware(HardwareMap hardwareMap ) {

        try {
            LF = hardwareMap.dcMotor.get("LF");
            LF.setPower(0);
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LF.setDirection(DcMotor.Direction.REVERSE);

        } catch (Exception a) {
            LF = null;
        }
        try {
            RF = hardwareMap.dcMotor.get("RF");
            RF.setPower(0);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception a) {
            RF = null;
        }

        try {
            LB = hardwareMap.dcMotor.get("LB");
            LB.setPower(0);
            LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LB.setDirection((DcMotor.Direction.REVERSE));
        } catch (Exception a) {
            LB = null;
        }

        try {
            RB = hardwareMap.dcMotor.get("RB");
            RB.setPower(0);
            RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception a) {
            RB = null;
        }

        try {
            arm = hardwareMap.servo.get("arm");

        } catch(Exception a){
            arm = null;
        }

        try {
            claw = hardwareMap.servo.get("claw");

        } catch(Exception a){
            claw = null;
        }
    }

    public void drive( double x, double y ) {

        if( y > 0 || y < 0 ) {
            LF.setPower(0.75 * y);
            RF.setPower(0.75 * y);
            LB.setPower(0.75 * y);
            RB.setPower(0.75 * y);

        }
        else if( x > 0 ) {
            LF.setPower(0.75 * x);
            RF.setPower(0.75 * -x);
            LB.setPower(0.75 * x);
            RB.setPower(0.75 * -x);

        }
        else if( x < 0 ) {
            LF.setPower(0.75 * x);
            RF.setPower(0.75 * -x);
            LB.setPower(0.75 * x);
            RB.setPower(0.75 * -x);
        }
        else{
            LF.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
            RB.setPower(0);
        }
    }

    public void setClawPosition( double position ) {

        claw.setPosition( position );
    }

    public void setArmPosition( double position ) {

        arm.setPosition( position );
    }
}
