/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Input and Ouput
  public static OI m_oi;

  Command m_autonomousCommand, autoRightStartCommand, autoLeftStartCommand, autoSlalom, autoBarrel;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;
  NetworkTableEntry cameraSelection;
  CameraServer camServer;
  
  MecanumDrive robotDrive;
  Victor frontLeft, frontRight, backLeft, backRight;

  XboxController xbox1;
  XboxController xbox2;
  
  Victor intake;
  
  Victor shooter1;
  Victor shooter2;
  
  Victor conveyer;
  
  DoubleSolenoid climb;

  Timer t;



  // Limelight stuff
  NetworkTable table;
  NetworkTableEntry tx, ty, ta, tv;
  double limeX, limeY, limeArea, limeHasTargetFloat;
  Boolean limeHasTarget;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    //camera1 = new UsbCamera("cam1", 0);
    //System.out.println(camera1.isEnabled());
    //camera2 = new UsbCamera("cam2", 1);
    camServer = CameraServer.getInstance();
    server = CameraServer.getInstance().getServer();
    //System.out.println(camera1.isEnabled());
    //System.out.println(camera2.isEnabled());

    //camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    //VideoSource source = camera1;
    CameraServer.getInstance().addSwitchedCamera("Camera View");
    CameraServer.getInstance().getVideo(camera1);

    xbox1 = new XboxController(RobotMap.xboxController1);
    xbox2 = new XboxController(RobotMap.xboxController2);

    frontLeft = new Victor(RobotMap.leftFrontMotor);
    frontRight = new Victor(RobotMap.rightFrontMotor);
    backLeft = new Victor(RobotMap.leftBackMotor);
    backRight = new Victor(RobotMap.rightBackMotor);

    intake = new Victor(RobotMap.intakeMotor);
    
    shooter1 = new Victor(RobotMap.shooterMotor1);
    shooter2 = new Victor(RobotMap.shooterMotor2);
    
    conveyer = new Victor(RobotMap.conveyerMotor);

    climb = new DoubleSolenoid(0, 1);


    robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    //robotDrive.setMaxOutput(.25);

    // Limelight stuff
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    // Instantiate our input/output.
    m_oi = new OI();

    // chooser.addOption("My Auto", new MyAutoCommand());
    autoRightStartCommand = new Command(){
    
      @Override
      protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
      }

      @Override
      protected void execute() {
        double cT = t.get();
        if (cT < 1) {
          robotDrive.driveCartesian(0, -0.25, 0);
        }
        else if (cT > 1 && cT < 3) {
          robotDrive.driveCartesian(0, 0, 0);
        }
        else if (cT > 3 && cT < 5) {
          robotDrive.driveCartesian(0.25, 0, 0);
        }
        else if (cT > 5 && cT < 6) {
          robotDrive.driveCartesian(0, -0.25, 0);
        }
        else if (cT > 6 && cT < 8) {
          robotDrive.driveCartesian(0, 0, -0.25);
        }
        else if (cT > 8 && cT < 8.25) {
          robotDrive.driveCartesian(0, 0.75, 0);
        }
        else if (cT > 8.25 && cT < 9) {
          robotDrive.driveCartesian(-0.25, 0, 0);
        }
        else if (cT > 9 && cT < 10) {
          robotDrive.driveCartesian(0, -0.25, 0);
        }
        else {
          robotDrive.driveCartesian(0, 0, 0);
        }
      }
    };

    autoLeftStartCommand = new Command(){
    
      @Override
      protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
      }

      @Override
      protected void execute() {
        double cT = t.get();
        if (cT < 1) {
          robotDrive.driveCartesian(0, 0.25, 0);
        }
        else if (cT > 1 && cT < 3) {
          robotDrive.driveCartesian(0, 0, 0);
        }
        else if (cT > 3 && cT < 5) {
          robotDrive.driveCartesian(0.25, 0, 0);
        }
        else if (cT > 5 && cT < 6) {
          robotDrive.driveCartesian(0, -0.25, 0);
        }
        else if (cT > 6 && cT < 8) {
          robotDrive.driveCartesian(0, 0, -0.25);
        }
        else if (cT > 8 && cT < 8.25) {
          robotDrive.driveCartesian(0, 0.75, 0);
        }
        else if (cT > 8.25 && cT < 9) {
          robotDrive.driveCartesian(-0.25, 0, 0);
        }
        else if (cT > 9 && cT < 10) {
          robotDrive.driveCartesian(0, -0.25, 0);
        }
        else {
          robotDrive.driveCartesian(0, 0, 0);
        }
      }
    };


    autoSlalom = new Command(){  
      @Override
      protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
      }
      @Override
      protected void execute() {
        //Declare and initialize driveCart time intervals
        double[] n = {0.11,0.325,0.65,0.505,0.38,0.52,0.65,0.47};
        for(int i=0; i<n.length; i++){
          n[i] *= 2;
        }

        double speed = 0.25;

        double cT = t.get();
      
        if (cT > 0 && cT < n[0]) {
          robotDrive.driveCartesian(0, speed, 0);
        }
        else if (cT > n[0] && cT < n[0]+n[1]) {
          robotDrive.driveCartesian(-speed, speed, 0);
        }
        else if (cT > n[0]+n[1] && cT < n[0]+n[1]+n[2]) {
          robotDrive.driveCartesian(0, speed, 0);
        }
        else if (cT > n[0]+n[1]+n[2] && cT < n[0]+n[1]+n[2]+n[3]) {
          robotDrive.driveCartesian(speed, speed, 0);
        }
        else if (cT > n[0]+n[1]+n[2]+n[3] && cT < n[0]+n[1]+n[2]+n[3]+n[4]) {
          robotDrive.driveCartesian(-speed, 0, 0);
        }
        else if (cT > n[0]+n[1]+n[2]+n[3]+n[4] && cT < n[0]+n[1]+n[2]+n[3]+n[4]+n[5]) {
          robotDrive.driveCartesian(speed, -speed, 0);
        }
        else if (cT > n[0]+n[1]+n[2]+n[3]+n[4]+n[5] && cT < n[0]+n[1]+n[2]+n[3]+n[4]+n[5]+n[6]) {
          robotDrive.driveCartesian(0, -speed, 0);
        }
        else if (cT > n[0]+n[1]+n[2]+n[3]+n[4]+n[5]+n[6] && cT < n[0]+n[1]+n[2]+n[3]+n[4]+n[5]+n[6]+n[7]) {
          robotDrive.driveCartesian(-speed, -speed, 0);
        }
        else {
          robotDrive.driveCartesian(0, 0, 0);
        }
      }
    };

    autoBarrel = new Command(){
      @Override
      protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
      }
      @Override
      protected void execute() {
        double cT = t.get();
        if (cT < 1) {
          robotDrive.driveCartesian(0, .5, 0);
        }
        else if (cT < 1) {
          robotDrive.driveCartesian(.5, .5, 0);
        }
        else if (cT < 1) {
          robotDrive.driveCartesian(.5, -.5, 0);
        }
        else if (cT < 1) {
          robotDrive.driveCartesian(-.5, -.5, 0);
        }
        else if (cT < 1) {
          robotDrive.driveCartesian(-.5, .5, 0);
        }
        else if (cT < 1) {
          robotDrive.driveCartesian(-.5, -.5, 0);
        }
        else if (cT < 1) {
          robotDrive.driveCartesian(.5, -.5, 0);
        }
        else if (cT < 1) {
          robotDrive.driveCartesian(.5, .5, 0);
        }
        else if (cT < 1) {
          robotDrive.driveCartesian(-.5, .5, 0);
        }
        else if (cT < 1) {
          robotDrive.driveCartesian(0, -.5, 0);
        }
        else {
          robotDrive.driveCartesian(0, 0, 0);
        }
      }
    };

    m_chooser.addOption("Auto Right Start", autoRightStartCommand);
    m_chooser.addOption("Auto Left Start", autoLeftStartCommand);
    m_chooser.addOption("Auto Slalom Start", autoSlalom);
    m_chooser.addOption("Auto Barrel Start", autoBarrel);

    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Limelight stuff
    //read values periodically
    limeX = tx.getDouble(0.0);
    limeY = ty.getDouble(0.0);
    limeArea = ta.getDouble(0.0);
    limeHasTargetFloat = tv.getDouble(0.0);
    if (limeHasTargetFloat == 1) {
      limeHasTarget = true;
    }
    else {
      limeHasTarget = false;
    }

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", limeX);
    SmartDashboard.putNumber("LimelightY", limeY);
    SmartDashboard.putNumber("LimelightArea", limeArea);
    SmartDashboard.putNumber("LimeHasTarget", limeHasTargetFloat);

    //Xbox controller start button switches camera to camera 1
    if (xbox2.getStartButton()) {
      
      camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      camera2.setConnectionStrategy(ConnectionStrategy.kForceClose);
      
      
    } 
    // Xbox controller back buttons switches camera to camera 2
    else if (xbox2.getBackButton()) {
      camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      camera1.setConnectionStrategy(ConnectionStrategy.kForceClose);
        
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */

  // A shorter and easier move function to not worry about inverting (NEEDS TESTING)
  public void setMove(double forward, double right, double twistRight) {
    robotDrive.driveCartesian(right, -forward, twistRight);
  }

  public enum directions {
    Forward, Reverse, Left, Right
  }

  public void moveDir(directions dir, double speed) {
    switch (dir) {
      case Forward:
        setMove(0, speed, 0);
        break;
      case Reverse:
      setMove(0, -speed, 0);
        break;
      case Left:
      setMove(-speed, 0, 0);
        break;
      case Right:
        setMove(speed, 0, 0);
        break;
      default:
        break;
    }
  }
  
  

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    robotDrive.setSafetyEnabled(false);

    t = new Timer();

    t.reset();
    t.start();

    /*
    robotDrive.driveCartesian(0, -0.25, 0);
    Timer.delay(1);
    robotDrive.driveCartesian(0, 0, 0);
    /*

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    /*
    double cT = t.get();
    if (cT < 1) {
      robotDrive.driveCartesian(0, -0.25, 0);
    }
    else if (cT > 1 && cT < 3) {
      robotDrive.driveCartesian(0, 0, 0);
    }
    else if (cT > 3 && cT < 5) {
      robotDrive.driveCartesian(0.25, 0, 0);
    }
    else if (cT > 5 && cT < 6) {
      robotDrive.driveCartesian(0, -0.25, 0);
    }
    else if (cT > 6 && cT < 8) {
      robotDrive.driveCartesian(0, 0, -0.25);
    }
    else if (cT > 8 && cT < 8.25) {
      robotDrive.driveCartesian(0, 0.75, 0);
    }
    else if (cT > 8.25 && cT < 9) {
      robotDrive.driveCartesian(-0.25, 0, 0);
    }
    else if (cT > 9 && cT < 10) {
      robotDrive.driveCartesian(0, -0.25, 0);
    }
    else {
      robotDrive.driveCartesian(0, 0, 0);
    }
    */
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    robotDrive.setSafetyEnabled(false);

    // the slider (throttle) on the joystick sets speedAdj, which allows for real time speed limiting
    // speed cap is an extra option for a hard-coded speed limit that is applied to the throttle
    //double stickSlider = stick.getThrottle();
    double speedCap = 1;
    double spinCap = .69;
    //double speedAdj = speedCap * (1 - ((stickSlider + 1) / 2));

    // For precise movement: if trigger is pressed, spinning is blocked
   /*
    if (stick.getTrigger()) {
      spinCap = 0;
    }
    else {
      spinCap = 0.69;
    }

    // For precise movement: if thumb button is presses, moving is blocked
    if (stick.getRawButton(2)) {
      speedAdj = 0;
    }
*/
    //System.out.println(speedAdj);
    robotDrive.driveCartesian(speedCap*xbox1.getRawAxis(0), -speedCap*xbox1.getRawAxis(1), spinCap*xbox1.getRawAxis(4));

    // Limelight aiming testing
    // X-value adjusting (twisting)
    if (xbox1.getBButton()) {
      if (!limeHasTarget) {
        robotDrive.driveCartesian(0, 0, 0.2);
      }
      else {
        if (Math.abs(limeX) > 0.1) {
          if (limeX > 0) {
            robotDrive.driveCartesian(0, 0, 0.1+(limeX/44.7));
          }
          else {
            robotDrive.driveCartesian(0, 0, -0.1+(limeX/44.7));
          }
        }
      }   
    }

    // Area-based adjusting (distance from target)
    // 1.893410
    if (xbox1.getXButton()) {
      double error = limeArea - 1.89;
      if ((Math.abs(error) > 0.1) && limeHasTarget) {
        if (error > 0.5) {
          robotDrive.driveCartesian(0, -0.3, 0);
        }
        else if (error > 0.25 && error < 0.5) {
          robotDrive.driveCartesian(0, -0.2, 0);
        }
        else if (error > 0.1 && error < 0.25) {
          robotDrive.driveCartesian(0, -0.1, 0);
        }
        else {
          robotDrive.driveCartesian(0, 0.2, 0);
        }
        //robotDrive.driveCartesian(0, 0, ((limeX > 0) ? 0.1 : -0.1)+(limeX/44.7));
      }
    }

    // Xbox controller A button runs the intake
    if (xbox2.getAButton())
    {
      intake.setSpeed(-0.5);
    }
    // Xbox controller b Button reverses intake (in case ball gets stuck in intake)
    else if (xbox2.getBButton())
    {
      intake.setSpeed(0.5);
    }
    // Else the motor stops
    else
    {
      intake.setSpeed(0);
    }

    // Xbox controller X button moves conveyer up 
    if (xbox2.getXButton()) 
    {
      conveyer.setSpeed(-1);
    }
    // Xbox controller Y button moves conveyer down
    else if (xbox2.getYButton())
    {
      conveyer.setSpeed(1);
    }
    // Else the motor stops
    else
    {
      conveyer.setSpeed(0);
    }

    // Xbox controller left bumper runs shooter
    if (xbox2.getBumper(Hand.kLeft)) {
      shooter1.setSpeed(1.0);
      shooter2.setSpeed(-1.0);
    }
    // Xbox controller right bumper stops shooter
    else if (xbox2.getBumper(Hand.kRight))
    {
      shooter1.setSpeed(0);
      shooter2.setSpeed(0);
    }

    if (xbox1.getYButton()) 
    {
      climb.set(DoubleSolenoid.Value.kForward);
    }
    else if (xbox1.getAButton())
    {
      climb.set(DoubleSolenoid.Value.kReverse);
    }
    //MOVED TO ROBOT PERIODIC SO CAMERAS CAN BE USED WHEN TELEOP DISABLED
    //Xbox controller start button switches camera to camera 1
    /*
    if (xbox.getStartButton()) {
      
      camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      camera2.setConnectionStrategy(ConnectionStrategy.kForceClose);
      
      
    } 
    // Xbox controller back buttons switches camera to camera 2
    else if (xbox.getBackButton()) {
      camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      camera1.setConnectionStrategy(ConnectionStrategy.kForceClose);
        
    }
    */
    
  }


  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }
}
