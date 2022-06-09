package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import android.view.inputmethod.CorrectionInfo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


@Autonomous(name = "Sensor: BNO055 IMU Calibration", group = "Sensor")
public class IMU extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    DcMotor LF_Move, LB_Move, RF_Move, RB_Move;
    String heading = String.valueOf(0), roll = String.valueOf(0), pitch = String.valueOf(0);
    double power = 0.5;
    double correction_index = 0.003;

    @Override
    public void runOpMode() {
        setMotor();
        setImu();

//        telemetry.log().add("Waiting for start...");
        telemetry.addLine()
                .addData("Power", power)
                .addData("Correction Index", correction_index);
        composeTelemetry();


        while (!isStarted()) {
            telemetry.update();
        }

        while (opModeIsActive()) {
            move_forward(power);
//            move_left(power);
            telemetry.update();

        }

    }

    void setMotor(){
        LF_Move = hardwareMap.get(DcMotor.class, "LF_Move");
        LB_Move = hardwareMap.get(DcMotor.class, "LB_Move");
        RF_Move = hardwareMap.get(DcMotor.class, "RF_Move");
        RB_Move = hardwareMap.get(DcMotor.class, "RB_Move");

        LF_Move.setZeroPowerBehavior(BRAKE);
        LB_Move.setZeroPowerBehavior(BRAKE);
        RF_Move.setZeroPowerBehavior(BRAKE);
        RB_Move.setZeroPowerBehavior(BRAKE);

        LF_Move.setDirection(REVERSE);
        LB_Move.setDirection(REVERSE);
    }

    void setImu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    void move_forward(double power){
        power = power*(double)100;
        double correction = Double.parseDouble(heading)*power*correction_index;
        telemetry.addData("Corr", correction);
        double L_Power = Math.round(power+correction)/(double)100;
        double R_Power = Math.round(power-correction)/(double)100;
        telemetry.addLine().addData("Left",L_Power).addData("Right", R_Power);

        LF_Move.setPower(L_Power);
        RF_Move.setPower(R_Power);
        LB_Move.setPower(L_Power);
        RB_Move.setPower(R_Power);
    }

    void move_left(double power){
        power = power*100;
        double correction = Double.parseDouble(heading)*power*correction_index;
        telemetry.addData("Corr",correction);
        double F_Power = Math.round(power+correction)/(double)100;
        double B_Power = Math.round(power-correction)/(double)100;
        telemetry.addLine().addData("Fount",F_Power).addData("Back", B_Power);

        LF_Move.setPower(F_Power*(double)-1);
        RF_Move.setPower(F_Power);
        LB_Move.setPower(B_Power);
        RB_Move.setPower(B_Power*(double)-1);
    }

    void composeTelemetry() {

        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        heading = formatAngle(angles.angleUnit, angles.firstAngle);
                        return heading;
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        roll = formatAngle(angles.angleUnit, angles.secondAngle);
                        return roll;
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        pitch = formatAngle(angles.angleUnit, angles.thirdAngle);
                        return pitch;
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}