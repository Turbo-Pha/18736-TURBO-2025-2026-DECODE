public class Constants {

  public static FollowerConstants followerConstants = new FollowerConstants()
    .mass(5.9)
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
    .xVelocity(57.8741)
    .yVelocity(52.295);

  public static OTOSConstants localizerConstants = new OTOSConstants()
    .hardwareMapName("sensor_otos")
    .linearUnit(DistanceUnit.INCH)
    .angleUnit(AngleUnit.RADIANS)
    .offset(new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2))
    .linearScalar(1.0)
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
