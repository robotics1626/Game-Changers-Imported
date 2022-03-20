/* C O N T R O L  L I S T

JOYSTICKS:
Left and Right Sticks: Drive Train
Left and Right Triggers: Gear Shift

XBOX CONTROLLER
Left Trigger: Fire Turret
Right Trigger: Feed Intake into Turret.
Left and Right Bumpers: Rotate Turret
Start Button: Limelight Toggle
A Button: Arm Toggle Up/Down
X and B Buttons: Winch
Right Stick Y-Axis: Intake
*/

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
// Color sensor imports
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// Below: Limelight Imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.ColorSensorV3;

import java.lang.reflect.InvocationTargetException;
import java.util.Map;

// Talon SRX and FX imports 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

// sparkmax imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "Variant 2 Auto";
  private static final String kVariant3 = "Variant 3 Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // defining controllers
  private XboxController xbox;
  private Joystick driverLeft;
  private Joystick driverRight;

  // defining tank drive
  private TalonFX frontLeftSpeed;
  private TalonFX frontRightSpeed;
  private TalonFX backLeftSpeed;
  private TalonFX backRightSpeed;
  private boolean toggleLT;
  private boolean toggleRT;

  // air pressure
  private boolean gearShift;
  private boolean intakeShift;
  private int intakeCheck;
  private DoubleSolenoid shifter;
  private DoubleSolenoid winchPistons;
  private DoubleSolenoid intakeButton;
  private DoubleSolenoid intakeButton2;
  private Compressor compressor;
  private AnalogInput pressureSensor;

  // defining color sensor
  private VictorSPX colorMotor;

  // intake
  private TalonSRX beltMotor;
  private VictorSPX intakeMotor;
  private TalonSRX beltTopMotor;
  private MotorController beltSecondaryMotor;
  private TalonFX winch;
  private boolean triggerDisable;
  private boolean axisDisable;

  // turret spinner
  private CANSparkMax spinMotor;
  private RelativeEncoder spinEncoder;
  private double spinPos;
  private double spinPos_Starter;
  private boolean spinEmerg;
  private boolean spinRunner;
  private boolean turretRunner;
  private double double_turretCounter = 0.0;

  // turret fire
  private NetworkTableEntry rpmSet;
  private TalonFX turretFire;
  private double rpmFinal;
  private boolean turretFire_exception = true;
  private boolean bool_fireVariant = false;

  // defining Limelight Variables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ledEntry = table.getEntry("ledMode");

  private double x = tx.getDouble(0.0);
  private double y = ty.getDouble(0.0);
  private double area = ta.getDouble(0.0);
  private boolean limeToggle;
  private boolean limeToggle_Check;
  private double distance_toTarget;
  private double[] desiredTargetDistance = {10.602,14.900, 11.315, 13.9628};
  private double distanceSubtractor = Double.MAX_VALUE;
  private int arrayHolder = 0;

  // echo code / auton. declarations
  ActionRecorder actions;
  boolean autoLoop;

  // dashboard drivetrain temperatures
  double frT;
  double brT;
  double flT;
  double blT;

  // defining color sensor variables
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  /*
  private int color = 0;
  private boolean toggleColor;
  private final String fieldColor = DriverStation.getGameSpecificMessage();
  */ 

  @Override
  public void robotInit() {
    // sets up shuffleboard to have a box with autonomous choices. These auto choices
    // are used in either autonomousInit() or autonomousPeriodic(), in a switch-case
    // statement that pulls these lines.
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Variant 2 Auto", kCustomAuto);
    m_chooser.addOption("Variant 3 Auto", kVariant3);
    SmartDashboard.putData("Auto choices", m_chooser);

    // establishes an element on the shuffleboard that allows us to change 
    // turret fire RPM on the fly.
    rpmSet = Shuffleboard.getTab("SmartDashboard").add("Initial", 4500)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 3500, "max", 6000, "block increment", 100)).getEntry();

    // drive train assignments
    frontLeftSpeed = new TalonFX(1);
    frontRightSpeed = new TalonFX(2);
    backLeftSpeed = new TalonFX(3);
    backRightSpeed = new TalonFX(4);

    // motor setup for drive train
    frontLeftSpeed.setNeutralMode(NeutralMode.Coast);
    frontRightSpeed.setNeutralMode(NeutralMode.Coast);
    backLeftSpeed.setNeutralMode(NeutralMode.Coast);
    backRightSpeed.setNeutralMode(NeutralMode.Coast);

    // color sensor
    colorMotor = new VictorSPX(5);

    // I beat my kids with a
    beltMotor = new TalonSRX(6);
    beltTopMotor = new TalonSRX(11);
    beltSecondaryMotor = new CANSparkMax(12, MotorType.kBrushless);
    winch = new TalonFX(8);
    winch.setNeutralMode(NeutralMode.Brake);
    intakeMotor = new VictorSPX(9);

    // turret spinner
    spinMotor = new CANSparkMax(7, MotorType.kBrushless);
    spinEncoder = spinMotor.getEncoder();
    spinPos_Starter = spinEncoder.getPosition();

    // turret fire
    try {
      turretFire = new TalonFX(15);
    } 
    catch(Exception e) {
      System.out.println("TURRET FIRE MECHANISM DISCONNECTED.");
      turretFire_exception = false;
    }
    rpmFinal = 4500;

    // controller and joystick init
    driverLeft = new Joystick(0);
    driverRight = new Joystick(1);
    xbox = new XboxController(2);

    // current peaks
    beltMotor.configPeakCurrentLimit(20, 6);
    beltMotor.configContinuousCurrentLimit(20, 6);
    beltMotor.enableCurrentLimit(true);

    // pressure declares
    gearShift = true;
    intakeShift = true;

    shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    intakeButton = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    intakeButton2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    winchPistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    pressureSensor = new AnalogInput(0);

    shifter.set(Value.kReverse);
    winchPistons.set(Value.kForward);
    intakeButton.set(Value.kForward);
    intakeButton2.set(Value.kForward);
    intakeCheck = 1;
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

    // turret RPM
    // these values have been pre-established by other files and other sources.
    // there is no reason to touch these values, so don't touch them.
    if(turretFire_exception) { 
      /* if statement exists because of the try-catch.
      Not sure if I need to do this, but better safe than
      sorry, and there's no harm in running a little extra
      if statement. - Nokes
      */
      turretFire.configFactoryDefault();

      /* Config neutral deadband to be the smallest possible */
      turretFire.configNeutralDeadband(0.001);

      /* Config sensor used for Primary PID [Velocity] */
      turretFire.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0); // constants here have been moved from Constants.java

      /* Config the peak and nominal outputs */
      turretFire.configNominalOutputForward(0, 0);
      turretFire.configNominalOutputReverse(0, 0);
      turretFire.configPeakOutputForward(1, 0);
      turretFire.configPeakOutputReverse(-1, 0);

      /* Config the Velocity closed loop gains in slot0 */
      turretFire.config_kF(0, Constants.kGains_Velocit.kF, 0);
      turretFire.config_kP(0, Constants.kGains_Velocit.kP, 0);
      turretFire.config_kI(0, Constants.kGains_Velocit.kI, 0);
      turretFire.config_kD(0, Constants.kGains_Velocit.kD, 0);

      turretFire.configVoltageCompSaturation(12.0);
      turretFire.enableVoltageCompensation(true);
    }

    // toggle declare = true;
    // toggles are used to make sure that button-pressed
    // one-time actions aren't spammed when run during robotOperation()
    toggleLT = true;
    toggleRT = true;
    limeToggle = false;
    limeToggle_Check = true;
    axisDisable = true;
    triggerDisable = true;
    spinEmerg = true;
    spinRunner = true;
    turretRunner = true;

    // turns the limelight off.
    ledEntry.setNumber(1);

    // dashboard drivetrain temperatures
    // sets them to 0 as a starting val, further updates
    // in robotPeriodic().
    frT = 0.0;
    brT = 0.0;
    flT = 0.0;
    blT = 0.0;

    // Limit removers, even though we never turned on limits. 
    // Motors are stupid.
    frontLeftSpeed.configForwardSoftLimitEnable(false);
    frontLeftSpeed.configReverseSoftLimitEnable(false);
    frontRightSpeed.configForwardSoftLimitEnable(false);
    frontRightSpeed.configReverseSoftLimitEnable(false);
    backLeftSpeed.configForwardSoftLimitEnable(false);
    backLeftSpeed.configReverseSoftLimitEnable(false);
    backRightSpeed.configForwardSoftLimitEnable(false);
    backRightSpeed.configReverseSoftLimitEnable(false);

    actions = new ActionRecorder().
				setMethod(this, "robotOperation", frc.robot.DriverInput.class).
				setUpButton(xbox, 1).
				setDownButton(xbox, 2).
				setRecordButton(xbox, 4);
		
		DriverInput.nameInput("Driver-Left");
		DriverInput.nameInput("Driver-Right");
		DriverInput.nameInput("Driver-Left-Trigger");
    DriverInput.nameInput("Driver-Right-Trigger");
    DriverInput.nameInput("Driver-Left-Seven");
    DriverInput.nameInput("Driver-Left-Eight");
    DriverInput.nameInput("Driver-Left-Nine");
    DriverInput.nameInput("Driver-Right-Seven");
    DriverInput.nameInput("Driver-Right-Eight");
    DriverInput.nameInput("Driver-Right-Nine");
    DriverInput.nameInput("Driver-Left-Intake");
    DriverInput.nameInput("Driver-Right-Intake");
    DriverInput.nameInput("Driver-Intake-Arm");
		DriverInput.nameInput("Operator-Left-Stick");
		DriverInput.nameInput("Operator-Left-Bumper");
		DriverInput.nameInput("Operator-Left-Trigger");
		DriverInput.nameInput("Operator-Right-Stick");
		DriverInput.nameInput("Operator-Right-Bumper");
		DriverInput.nameInput("Operator-Right-Trigger");
		DriverInput.nameInput("Operator-X-Button");
		DriverInput.nameInput("Operator-Y-Button");
		DriverInput.nameInput("Operator-A-Button");
		DriverInput.nameInput("Operator-B-Button");
		DriverInput.nameInput("Operator-Start-Button");
    DriverInput.nameInput("Operator-Back-Button");
  }

  @Override
  public void robotPeriodic() {
    // pressure check
    final double pressure = (250 * (pressureSensor.getVoltage() / 5.0)) - 13;
    SmartDashboard.putString("Pressure Sensor", String.format("%.0f", pressure));

    // turret check
    spinPos = spinEncoder.getPosition() - spinPos_Starter;
    SmartDashboard.putNumber("Turret Raw Position", spinPos);
    if (spinPos > 320) {
      SmartDashboard.putString("Turret Spinner Position", "Right Bound");
    } else if (spinPos < 25) {
      SmartDashboard.putString("Turret Spinner Position", "Left Bound");
    } else {
      SmartDashboard.putString("Turret Spinner Position", "Middle");
    }
    if(turretFire_exception) SmartDashboard.putNumber("Turret RPM", turretFire.getSelectedSensorVelocity() / 3.41);

    // limelight variable check
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    distance_toTarget = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    if(true) {
      for(int n = 0; n < desiredTargetDistance.length; n++) {
        if(Math.abs(desiredTargetDistance[n] - distance_toTarget) < distanceSubtractor) {
          distanceSubtractor = Math.abs(desiredTargetDistance[n] - distance_toTarget);
          arrayHolder = n;
        }
      }
      distanceSubtractor = Double.MAX_VALUE;
      System.out.println(arrayHolder);
    }

    // Motor Temps
    if(frT < frontRightSpeed.getTemperature()) frT = frontRightSpeed.getTemperature();
    if(brT < backRightSpeed.getTemperature()) brT = backRightSpeed.getTemperature();
    if(flT < frontLeftSpeed.getTemperature()) flT = frontLeftSpeed.getTemperature();
    if(blT < backLeftSpeed.getTemperature()) blT = backLeftSpeed.getTemperature();

    SmartDashboard.putString("Right Temp.", ("F:" + frT + ", B:" + brT));
    SmartDashboard.putString("Left Temp.", ("F:" + flT + ", B:" + blT));

    // pushing limelight vars
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    if (limeToggle) {
      SmartDashboard.putString("Limelight Status", "Enabled");
    } else {
      SmartDashboard.putString("Limelight Status", "Disabled");
    }

    // turret RPM output
    SmartDashboard.putNumber("RPM Input", (rpmSet.getDouble(0.0)));
    if(rpmSet.getName() != null){
      rpmFinal = rpmSet.getDouble(0.0);
    }

    // defining color vars
    final Color detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    if(detectedColor.green > 0.47){
      SmartDashboard.putString("Color", "Green");
    } else if(detectedColor.red > 0.27 && detectedColor.green > 0.27) {
      SmartDashboard.putString("Color", "Yellow");
    } else if(detectedColor.red > 0.47){
      SmartDashboard.putString("Color", "Red");
    } else if(detectedColor.blue > 0.47){
      SmartDashboard.putString("Color", "Blue");
    } else {
      SmartDashboard.putString("Color", "N/A");
    }
  }

  @Override
  public void disabledInit() {
    if(turretFire_exception) turretFire.set(ControlMode.PercentOutput, (0));
    frontLeftSpeed.set(ControlMode.PercentOutput, 0);
    backLeftSpeed.set(ControlMode.PercentOutput, 0);
    frontRightSpeed.set(ControlMode.PercentOutput, 0);
    backRightSpeed.set(ControlMode.PercentOutput, 0);
    winch.set(ControlMode.PercentOutput, 0);
    beltMotor.set(ControlMode.PercentOutput, 0);
    beltTopMotor.set(ControlMode.PercentOutput, 0);
    colorMotor.set(ControlMode.PercentOutput, 0);
    intakeMotor.set(ControlMode.PercentOutput, 0);
    spinMotor.set(0);
    beltSecondaryMotor.set(0);
    ledEntry.setNumber(1);
    limeToggle=true;
    actions.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    actions.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    intakeButton.set(Value.kReverse);
    intakeButton2.set(Value.kReverse);
    autoLoop = true;

    // voltage limit (I think?)
    frontLeftSpeed.configVoltageCompSaturation(10.0);
    frontLeftSpeed.enableVoltageCompensation(true);
    frontRightSpeed.configVoltageCompSaturation(10.0);
    frontRightSpeed.enableVoltageCompensation(true);
    backLeftSpeed.configVoltageCompSaturation(10.0);
    backLeftSpeed.enableVoltageCompensation(true);
    backRightSpeed.configVoltageCompSaturation(10.0);
    backRightSpeed.enableVoltageCompensation(true);

    // dashboard drivetrain temperatures
    frT = 0.0;
    brT = 0.0;
    flT = 0.0;
    blT = 0.0;
  }

  @Override
  public void autonomousPeriodic() {
    if(autoLoop) {
      switch (m_autoSelected) {
        case kCustomAuto: // variant 2
          /*
          Set to do the track dependant on ball location.
          Program is done, but we still need the recordings.
          - Nokes
          */
          switch (arrayHolder) {
            case 0: // red A
              actions.autonomousInit(12, "");
              break;
            case 1: // red B
              actions.autonomousInit(14, "");
              break;
            case 2: // blue A
              actions.autonomousInit(0, "");
              break;
            case 3: // blue B
              actions.autonomousInit(1, "");
              break;
          }
          autoLoop = false;
          break;
        case kVariant3: // variant 3
          /* 
          Set up to run whatever is selected on the driverstation. 
          Not currently functional; figuring out how to read
          DB/String 0 - Nokes.

          Update: I did it, it's probably wrong. As of
          3/3/2021, this program hasn't been deployed yet.
          I'm waiting for Mr. Meier to come over the weekend
          so that if I did break it, I'll at least have someone
          to help me fix it. - Nokes.
          */
          autoLoop = false;
          actions.autonomousInit(-1, SmartDashboard.getString("DB/String 0", "new_auto.csv"));
          break;
        case kDefaultAuto: // runner for only one variant
        default:
          autoLoop = false;
          actions.autonomousInit(1, "");
          break; 
      }
    } else {
      try{
			  if (actions != null){
				  actions.longPlayback(this, -1);
			  }else{
				  Timer.delay(0.010);
			  }
		  }catch (Exception e){
			  System.out.println("AP: " + e.toString());
		  }
    }
  }

  @Override
	public void teleopInit() {
		DriverInput.setRecordTime();
		actions.teleopInit();

    if(actions.isRecording()) {
      frontLeftSpeed.configVoltageCompSaturation(10.0);
      frontLeftSpeed.enableVoltageCompensation(true);
      frontRightSpeed.configVoltageCompSaturation(10.0);
      frontRightSpeed.enableVoltageCompensation(true);
      backLeftSpeed.configVoltageCompSaturation(10.0);
      backLeftSpeed.enableVoltageCompensation(true);
      backRightSpeed.configVoltageCompSaturation(10.0);
      backRightSpeed.enableVoltageCompensation(true);
    } else {
      frontLeftSpeed.enableVoltageCompensation(false);
      frontRightSpeed.enableVoltageCompensation(false);
      backLeftSpeed.enableVoltageCompensation(false);
      backRightSpeed.enableVoltageCompensation(false);
    }
    
    // dashboard drivetrain temperatures
    frT = 0.0;
    brT = 0.0;
    flT = 0.0;
    blT = 0.0;
  }
  
  @Override
  public void teleopPeriodic() {

    try {
			actions.input(new DriverInput()
					.withInput("Operator-X-Button",		      xbox.getXButton())
					.withInput("Operator-Y-Button",		      xbox.getYButton())
					.withInput("Operator-A-Button", 	      xbox.getAButton())
					.withInput("Operator-B-Button",		      xbox.getBButton())
					.withInput("Operator-Start-Button",	    xbox.getStartButton())
					.withInput("Operator-Left-Trigger",  		xbox.getLeftTriggerAxis())
          .withInput("Operator-Right-Trigger",		xbox.getRightTriggerAxis())
          .withInput("Operator-Left-Bumper",      xbox.getLeftBumper())
          .withInput("Operator-Right-Bumper",     xbox.getRightBumper())
          .withInput("Operator-Right-Stick",      xbox.getRightY()) // y-axis only
          .withInput("Operator-Start-Button",     xbox.getStartButton())
          .withInput("Operator-Back-Button",      xbox.getBackButton())
					.withInput("Driver-Left", 			        driverLeft.getRawAxis(1))
					.withInput("Driver-Right", 			        driverRight.getRawAxis(1))
					.withInput("Driver-Left-Trigger", 	    driverLeft.getRawButton(1))
          .withInput("Driver-Right-Trigger", 	    driverRight.getRawButton(1))
          .withInput("Driver-Left-Intake", 	      driverLeft.getRawButton(2))
          .withInput("Driver-Right-Intake", 	    driverRight.getRawButton(2))
          .withInput("Driver-Left-Seven",         driverLeft.getRawButton(7))
          .withInput("Driver-Right-Seven",        driverRight.getRawButton(7))
          .withInput("Driver-Left-Eight",         driverLeft.getRawButton(8))
          .withInput("Driver-Right-Eight",        driverRight.getRawButton(8))
          .withInput("Driver-Left-Nine",          driverLeft.getRawButton(9))
          .withInput("Driver-Right-Nine",         driverRight.getRawButton(9))
					);					
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
		} catch (InvocationTargetException e) {
      e.printStackTrace();
    }
  }

  // granted that competitions seem like a no-no this year (2021)
  // the ColorSense method doesn't link to any buttons or anything like that.
  // It's commented out,  because I hate that stupid yellow
  // line and I want it to go away. - Nokes

  /*
  private void spin(final int ColorSense) {
    boolean colorLoop = true;
    int colorcount = 0;
    boolean dupecheck = true;
    if(ColorSense == 1){
      while(colorLoop){
        final Color detectedColor = m_colorSensor.getColor();
        colorMotor.set(ControlMode.PercentOutput, .1);
        if(detectedColor.red > 0.47 && dupecheck) {
          colorcount = colorcount + 1;
          System.out.println(colorcount);
          dupecheck = false;
        }
        if(detectedColor.red <= 0.47) {
          dupecheck = true;
        }
        if(colorcount >= 8) {
          colorLoop = false;
        }
      }
    } else if(ColorSense == 2){
      while(colorLoop){
        final Color detectedColor = m_colorSensor.getColor();
        colorMotor.set(ControlMode.PercentOutput, .1);
        if(detectedColor.green > 0.47 && dupecheck) {
          colorcount = colorcount + 1;
          System.out.println(colorcount);
          dupecheck = false;
        }
        if(detectedColor.green <= 0.47) {
          dupecheck = true;
        }
        if(colorcount >= 8) {
          colorLoop = false;
        }
      }
    } else if(ColorSense == 3){
      while(colorLoop){
        final Color detectedColor = m_colorSensor.getColor();
        colorMotor.set(ControlMode.PercentOutput, .1);
        if(detectedColor.blue > 0.47 && dupecheck) {
          colorcount = colorcount + 1;
          System.out.println(colorcount);
          dupecheck = false;
        }
        if(detectedColor.blue <= 0.47) {
          dupecheck = true;
        }
        if(colorcount >= 8) {
          colorLoop = false;
        }
      }
    } else if(ColorSense == 4){
      while(colorLoop){
        final Color detectedColor = m_colorSensor.getColor();
        colorMotor.set(ControlMode.PercentOutput, .1);
        if(detectedColor.red > 0.27 && detectedColor.green > 0.27 && dupecheck) {
          colorcount = colorcount + 1;
          System.out.println(colorcount);
          dupecheck = false;
        }
        if(detectedColor.red <= 0.47 && detectedColor.green <= 0.27) {
          dupecheck = true;
        }
        if(colorcount >= 8) {
          colorLoop = false;
        }
      }
    } 
  }
  */

   private DoubleSolenoid.Value solenoidPrev = DoubleSolenoid.Value.kOff;
  @Override
  public void testPeriodic() {
    System.out.println(actions.directoryPrefix() + SmartDashboard.getString("DB/String 0", "new_auto.csv"));
    actions.listAll();

    if(xbox.getXButton()) { ledEntry.setNumber(3); }
    else { ledEntry.setNumber(1); }

    if(xbox.getAButton()) {
      shifter.set(Value.kForward);
    } else {
      shifter.set(Value.kReverse);
    }
    final DoubleSolenoid.Value curr = shifter.get();
    if (curr != solenoidPrev) {
      System.out.println("Shifter is " + shifter.get());
      solenoidPrev = curr;
    }

    limeToggle = handleLimeButton(7);
  }

  @Override
  public void testInit() {
    compressor.enableDigital();
  }
 
  private boolean handleLimeButton(final int button) {
    if(xbox.getRawButtonPressed(button)) {
      limeToggle = limeToggle ? false : true;
      System.out.println("Button Pressed - now LED is " + limeToggle); 
    }
    return limeToggle;
  }

  public void robotOperation(DriverInput input) {

    // drive train
    double leftaxis = input.getAxis("Driver-Left");
    if(Math.abs(leftaxis) > .13){
      frontLeftSpeed.set(ControlMode.PercentOutput, (leftaxis * -1 ));
      backLeftSpeed.set(ControlMode.PercentOutput, (leftaxis * -1));  
    } else {
      frontLeftSpeed.set(ControlMode.PercentOutput, (0));
      backLeftSpeed.set(ControlMode.PercentOutput, (0));
    }
    double rightaxis = input.getAxis("Driver-Right"); // I don't know why, but these were marked as final - Nokes
    if(Math.abs(rightaxis) > .13){
      frontRightSpeed.set(ControlMode.PercentOutput, (rightaxis));
      backRightSpeed.set(ControlMode.PercentOutput, (rightaxis));
    } else {
      frontRightSpeed.set(ControlMode.PercentOutput, (0));
      backRightSpeed.set(ControlMode.PercentOutput, (0));
    }
    
    // emergency button backwards
    if(input.getButton("Driver-Right-Seven") || input.getButton("Driver-Left-Seven")) {
      turretFire.set(ControlMode.PercentOutput, -.4);
      beltTopMotor.set(ControlMode.PercentOutput, -1);
      beltSecondaryMotor.set(-1);
      spinEmerg = false;
    } else spinEmerg = true;

    // shifter set
    if((input.getButton("Driver-Left-Trigger") && toggleLT) || (input.getButton("Driver-Right-Trigger") && toggleRT)) {
      if(gearShift){
        gearShift = false;
      } else {
        gearShift = true;
      }
      toggleLT = false;
      toggleRT = false;
    } if(!(input.getButton("Driver-Left-Trigger")) && !(toggleLT)) {
      toggleLT = true;
    } if(!(input.getButton("Driver-Right-Trigger")) && !(toggleRT)) {
      toggleRT = true;
    }
  
    if(gearShift){
      shifter.set(Value.kReverse);
    } else {
      shifter.set(Value.kForward);
    }

    // RPM variants
    if(input.getButton("Driver-Left-Eight") || input.getButton("Driver-Right-Eight")) { bool_fireVariant = false; }
    if(input.getButton("Driver-Left-Nine") || input.getButton("Driver-Right-Nine")) { bool_fireVariant = true; }

    // turret spin
    if(input.getButton("Operator-Right-Bumper") && spinPos > -10) {
      spinMotor.set((-.1 - (.003 * double_turretCounter)));
      double_turretCounter++;
    } else if(input.getButton("Operator-Left-Bumper") && spinPos < 348.3752) {
      spinMotor.set((.1 + (.003 * double_turretCounter)));
      double_turretCounter++;
    } else { 
      double_turretCounter = 0;
      spinMotor.set(0);
    }

    // belt + intake
    if(input.getAxis("Operator-Right-Stick") > .2){
      beltMotor.set(ControlMode.PercentOutput, -1); // in 
      beltTopMotor.set(ControlMode.PercentOutput, .8);
      axisDisable = false;
     } else if(input.getAxis("Operator-Right-Stick") < -.2){
      beltMotor.set(ControlMode.PercentOutput, 1); // out      
      beltTopMotor.set(ControlMode.PercentOutput, -.8);
      axisDisable = false;
    } else {
      axisDisable = true;
    }

    // intake motor on top of the arm thingy
    if(input.getButton("Driver-Left-Intake")){
      intakeMotor.set(ControlMode.PercentOutput, -.8);
      beltTopMotor.set(ControlMode.PercentOutput, .8);
      spinRunner = false;
    } else if(input.getButton("Driver-Right-Intake")){
      intakeMotor.set(ControlMode.PercentOutput, .8);
      beltTopMotor.set(ControlMode.PercentOutput, -.8);
      spinRunner = false;
    } else {
      intakeMotor.set(ControlMode.PercentOutput, 0);
      spinRunner = true;
    }

    /*
    // color sensor import from field.
    if(fieldColor.length() > 0){
      if(fieldColor.charAt(0) == 'B'){
         color = 1;
      } if(fieldColor.charAt(0) == 'Y'){
         color = 2;
      } if(fieldColor.charAt(0) == 'R'){
         color = 3;
      } if(fieldColor.charAt(0) == 'G'){
         color = 4;
      }
    } 

    
    Color sensor just sets itself to be 2 colors behind the actual.
    Driver must set up the sensor to be on the midpoint color.
    
    
    
    // color sensor pressing
    if(xbox.getStartButton() && toggleColor) {
      spin(color);
      toggleColor = false;
    } if(!(xbox.getStartButton() && !(toggleColor))){
      toggleColor = true;
    }
    

    // color spinner manual
    if(xbox.getRawButton(6)){
      colorMotor.set(ControlMode.PercentOutput, .1);
    } else {
      colorMotor.set(ControlMode.PercentOutput, 0);
    }
    */

    // turret position reset
    if(input.getButton("Operator-Start-Button")) spinPos_Starter = spinEncoder.getPosition();

    // limelight enable/disable
    if(input.getButton("Operator-Back-Button") && limeToggle_Check) {
      if(limeToggle) limeToggle = false;
      else limeToggle = true;
      limeToggle_Check = false;
    } else if(!xbox.getBackButton() && !limeToggle_Check) limeToggle_Check = true;

    // limelight // turret fire
    if(input.getAxis("Operator-Left-Trigger") > 0.1){
      turretRunner = false;
      if(turretFire_exception) { 
        if(bool_fireVariant) {
          if(area > 1.4 && area < 2.3) { turretFire.set(TalonFXControlMode.Velocity, (4710 * 2048 / 600)); }
          else if(area > 0.8 && area < 1.4) { turretFire.set(TalonFXControlMode.Velocity, (4400 * 2048 / 600)); }
          else if(area > 0.5 && area < 0.8) { turretFire.set(TalonFXControlMode.Velocity, (4590 * 2048 / 600)); }
          else { turretFire.set(TalonFXControlMode.Velocity, (5400 * 2048 / 600)); }
        } else { turretFire.set(TalonFXControlMode.Velocity, (rpmFinal * 2048 / 600)); }
      } if(limeToggle) {
        ledEntry.setNumber(3);
        if(x < 0.0 && x > -2.0) { spinMotor.set(0); }
        else { spinMotor.set(-.01 * (x + 1.0)); }
      }
    } else {
      ledEntry.setNumber(1);
      turretRunner = true;
    }

    // intake shifters
    if(input.getButton("Operator-A-Button") && intakeShift) {
      intakeCheck = intakeCheck * -1;
      intakeShift = false;
    } if(!(input.getButton("Operator-A-Button") && !(intakeShift))){
      intakeShift = true;
    }

    if(intakeCheck == 1){
      intakeButton.set(Value.kReverse);
      intakeButton2.set(Value.kReverse);
    } else {
      intakeButton.set(Value.kForward);
      intakeButton2.set(Value.kForward);
    }  


    // emergency changes
    if(spinEmerg && turretRunner) {
      if(turretFire_exception) turretFire.set(ControlMode.PercentOutput, (0));
    } if(spinEmerg && spinRunner) {
      beltTopMotor.set(ControlMode.PercentOutput, 0);
    }
    // winch updown
    if(input.getButton("Operator-X-Button")) {
      winch.set(ControlMode.PercentOutput, (-1));
      winchPistons.set(Value.kReverse);
    } else if(input.getButton("Operator-B-Button")) {
      winch.set(ControlMode.PercentOutput, (1));
      winchPistons.set(Value.kReverse);
    } else {
      winch.set(ControlMode.PercentOutput, (0));
      winchPistons.set(Value.kForward);
    }

    // intake back
    if(input.getAxis("Operator-Right-Trigger") > 0.2 && (((turretFire.getSelectedSensorVelocity() - 2000) < (rpmFinal * 2048 / 600) && (turretFire.getSelectedSensorVelocity() + 2000) > (rpmFinal * 2048 / 600)) || bool_fireVariant)){
      beltSecondaryMotor.set(1);
      beltMotor.set(ControlMode.PercentOutput, 1);
      beltTopMotor.set(ControlMode.PercentOutput, .5);
      turretRunner = false;
      triggerDisable = false;
    } else {
      turretRunner = true;
      triggerDisable = true;
    }

    if(spinEmerg && axisDisable && triggerDisable) { beltSecondaryMotor.set(0); }
    if(triggerDisable && axisDisable) { beltMotor.set(ControlMode.PercentOutput, 0); }
  }
}
