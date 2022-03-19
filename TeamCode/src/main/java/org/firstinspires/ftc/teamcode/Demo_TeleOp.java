package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Demo_TeleOp", group="Demo")
public class Demo_TeleOp extends LinearOpMode{

    public void runOpMode() {

        Demo_Hardware robot = new Demo_Hardware( hardwareMap );

        waitForStart();

        while( opModeIsActive() && !gamepad2.a ) {

            double leftxstick = gamepad1.left_stick_x;
            double leftystick = gamepad1.left_stick_y * -1;

            telemetry.addData("leftxstick", leftxstick);
            telemetry.addData("leftystick", leftystick);
            telemetry.update();

            robot.drive( leftxstick, leftystick );

            if( gamepad1.x ) {
                robot.setClawPosition( 0.714 );

            } else if( gamepad1.y ) {
                robot.setClawPosition( 0.970 );
            }

            if( gamepad1.a ) {
                robot.setArmPosition( 0.721 );

            } else if( gamepad1.b ) {
                robot.setArmPosition( 0.853 );

            } else if( gamepad1.right_bumper ) {
                robot.setArmPosition( 0.650 );
            }
        }
    }
}
