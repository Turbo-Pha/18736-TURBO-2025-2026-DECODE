package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorSparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

  public static FollowerConstants followerConstants = new FollowerConstants()
    .mass(5.48)
    .forwardZeroPowerAcceleration(-41.278)
    .lateralZeroPowerAcceleration(-59.7819)
    .useSecondaryTranslationalPIDF(false)
    .useSecondaryHeadingPIDF(false)
    .useSecondaryDrivePIDF(false)
    .centripetalScaling(0.0005)
    .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
    .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
    .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0, 0.6, 0));

  public static MecanumConstants driveConstants = new MecanumConstants()
    .leftFrontMotorName("leftFront")
    .leftRearMotorName("leftRear")
    .rightFrontMotorName("rightFront")
    .rightRearMotorName("rightRear")
    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
    .xVelocity(33.41)
    .yVelocity(52.295);

  public static OTOSConstants localizerConstants = new OTOSConstants()
    .hardwareMapName("sensor_otos")
    .linearUnit(DistanceUnit.INCH)
    .angleUnit(AngleUnit.RADIANS)
    .offset(new SparkFunOTOS.Pose2D(5, 1.5, Math.PI / 0.5))
    .linearScalar(1.55)
    .angularScalar(1.0);

  public static PathConstraints pathConstraints = new PathConstraints(
    0.995,
    500,
    1,
    1
  );

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
      .mecanumDrivetrain(driveConstants)
      .OTOSLocalizer(localizerConstants)
      .pathConstraints(pathConstraints)
      .build();
  }
}
