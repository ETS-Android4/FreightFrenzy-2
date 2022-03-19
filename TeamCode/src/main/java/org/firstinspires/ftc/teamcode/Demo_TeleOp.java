package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Demo_TeleOp")

public class Demo_TeleOp extends LinearOpMode{
    Demo_Hardware robot = new Demo_Hardware();

    ElapsedTime opmodeRunTime = new ElapsedTime();

    public void runOpMode() {

        robot.init( hardwareMap );

        waitForStart();

        while( opModeIsActive() && !gamepad2.a ) {

            double leftystick = gamepad1.left_stick_y * -1;
            double leftxstick = gamepad1.left_stick_x;

            telemetry.addData("leftystick", leftystick);
            telemetry.addData("leftxstick", leftxstick);
            telemetry.update();

            robot.drive( leftxstick, leftystick );

            if( gamepad1.x ) {
                robot.setClawPosition( 0.714);
            }

            if( gamepad1.y ) {
                robot.setClawPosition( 0.970 );
            }

            if( gamepad1.a ) {

                robot.setArmPosition( 0.721 );
            }
            if( gamepad1.b ) {
                robot.setArmPosition( 0.853 );
            }

            if( gamepad1.right_bumper ) {
                robot.setArmPosition( 0.650 );
            }
        }
    }
}
