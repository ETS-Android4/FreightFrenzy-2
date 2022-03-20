package org.firstinspires.ftc.teamcode.example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.ExampleDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.ExampleServo;

@TeleOp(name = "ExampleTeleOp", group="Example")
public class ExampleTeleOp extends LinearOpMode{

    public void runOpMode() {

        ExampleDriveTrain driveTrain = new ExampleDriveTrain( hardwareMap );
        ExampleServo claw = new ExampleServo( hardwareMap, "claw");
        ExampleServo arm = new ExampleServo( hardwareMap, "arm");

        waitForStart();

        while( opModeIsActive() && !gamepad2.a ) {

            double leftxstick = gamepad1.left_stick_x;
            double leftystick = gamepad1.left_stick_y * -1;

            telemetry.addData("leftxstick", leftxstick);
            telemetry.addData("leftystick", leftystick);
            telemetry.update();

            driveTrain.drive( leftxstick, leftystick );

            if( gamepad1.x ) {
                claw.setPosition( 0.714 );

            } else if( gamepad1.y ) {
                claw.setPosition( 0.970 );
            }

            if( gamepad1.a ) {
                arm.setPosition( 0.721 );

            } else if( gamepad1.b ) {
                arm.setPosition( 0.853 );

            } else if( gamepad1.right_bumper ) {
                arm.setPosition( 0.650 );
            }
        }
    }
}
