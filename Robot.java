/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import io.github.pseudoresonance.pixy2api.Pixy2;
//import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PIDOutput;
//import edu.wpi.first.wpilibj.PIDSource;
//import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
//import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a program that employes the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot 
{
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_otherStick;
  //private Joystick m_streetFighter; don't worry about this
  //assign PWM channels to left motors
  PWMVictorSPX m_frontLeft = new PWMVictorSPX(3);
  PWMVictorSPX m_rearLeft = new PWMVictorSPX(2);
  //left two motor group
  SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
  //assign PWM channels to right motors
	PWMVictorSPX m_frontRight = new PWMVictorSPX(0);
  PWMVictorSPX m_rearRight = new PWMVictorSPX(1);
  //right two motor group
  SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
  //Spark Maxes IDs
  private static final int DeviceID6 = 6; 
  //Encoder goal
  //private double goal; 
  //Encoder rotations
  //private double rotations;
  //Spark Maxes
  //private CANSparkMax m_Motor5;
  //private CANSparkMax lift;
  private TalonSRX lift2 = new TalonSRX(3);
  
  private CANSparkMax intake;
  //CANSparkMax Encoder 
  
  //talon srx encoder
 
  //power level variable
  double x;
  //shoot out
  /*
  Solenoid Sole2 = new Solenoid(2);
  //retracts
  Solenoid Sole4 = new Solenoid(4);
  //shoot out
  Solenoid Sole1 = new Solenoid(1);
  //retracts
  Solenoid Sole3 = new Solenoid(3);
  */
  int goal;
  int ticks;
  int goalBottomLimit = 0;
  int goalUpperLimit = 75;
  int ticksToInches = 454;
  int inchesOffGround;
  int waterbomb;
  boolean goalreach;
  //pixy2
  //Pixy2 pixy = Pixy2.createInstance(LinkType.SPI);
  //CANPIDController pid = new CANPIDController(lift);
  

  @Override
  public void robotInit() 
  {
    //Spark Max initialization
    //assigns the ID numbers and motor types (Brushed or Brushless)
    intake = new CANSparkMax(DeviceID6, MotorType.kBrushed);
    //controls motors
    m_myRobot = new DifferentialDrive(m_right, m_left);
    //configures port for joystick in Drive Station
    m_leftStick = new Joystick(0);
    m_otherStick = new Joystick(1);
    //m_streetFighter = new Joystick(1); don't worry about this
    //camera
    CameraServer.getInstance().startAutomaticCapture(0);
    lift2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    //initializes pixy2
    //pixy.init();
    //records encoder position in smart dashboard; does not work currently, does not update value
    //PIDController liftControl = new PIDController(1, 0, 0, m_pidInput, m_pidOutput);
    m_myRobot.setSafetyEnabled(true);
  }

  @Override
  public void autonomousPeriodic()
  {
    //motor limit controller
    x = m_leftStick.getRawAxis(3);
    x++;
    x /= 2;

    ticks = lift2.getSelectedSensorPosition();
    inchesOffGround = ticks/ticksToInches;

    //Encoder rotllllll.ation assignment
    
    //steering and moving
    // x is set as a multipliers to the values so we can control the maximum output of the motors
    m_myRobot.arcadeDrive(m_leftStick.getY()*x, m_leftStick.getX()*x);
   
    //solenoid on channel 4 and 2 
    //channel 2 shoots out the piston when true, while channel 4 retracts it when true
    //Shoots the piston when button is held, retracts when button is let go
/*

    if(m_leftStick.getRawButton(1))
    {
      Sole3.set(true);
      Sole1.set(false);
    }

    if(m_leftStick.getRawButton(2))
    {
      Sole1.set(true);
      Sole3.set(false);
    }

    if(m_leftStick.getRawButton(4))
    {                                             
      Sole2.set(false);
      Sole4.set(true);
    }

    if(m_leftStick.getRawButton(6))
    {
      Sole2.set(true);
      Sole4.set(false);
    }
*/
    //SmartDashboard.putBoolean("clamp", Sole3.get());
    //SmartDashboard.putBoolean("extend", Sole2.get());
    SmartDashboard.putNumber("Inches Off Ground", inchesOffGround);
    

    //intake (subject to change motor values as needed)
    //gives motors positive value to wheel in a ball
    if(m_leftStick.getRawButton(7))
    {
        //m_Motor5.set(0.15);
        intake.set(0.15);
        //m_Motor8.set(0.15);
    }

    //resets motor when button is let go
    if(m_leftStick.getRawButton(11))
    {
      //m_Motor5.set(0);
      intake.set(0);
      //m_Motor8.set(0);
    }

    //gives motor negative value to shoot out the ball
    if(m_leftStick.getRawButton(8))
    {
      //m_Motor5.set(-0.15);
      intake.set(-0.40);
      //m_Motor8.set(-0.15);
    }

    //resets the motor when button is let go


    
    //brings lift to desired level; motor rotates for desired amount of times
    while(m_leftStick.getRawButton(9))
    {
      // lift2.set(ControlMode.PercentOutput, -.4); //clockwise
      lift2.set(ControlMode.PercentOutput, -.9);
    }
    while(m_leftStick.getRawButton(10))
    {
      lift2.set(ControlMode.PercentOutput, 1); //counterclockwise
    }
    if(!m_leftStick.getRawButton(9) && !m_leftStick.getRawButton(10) )
    {
      lift2.set(ControlMode.PercentOutput, 0);
    }
    
    /*
    //hatch level 1
    if(m_otherStick.getRawButton(1))
    {
      goal = 0;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //hatch level 2
    if(m_otherStick.getRawButton(2))
    {
      goal = -28;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //hatch level 3
    if(m_otherStick.getRawButton(3))
    {
      goal = -56;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //cargo port
    if(m_otherStick.getRawButton(4))
    {
      goal = -32;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //port level 1
    if(m_otherStick.getRawButton(5))
    {
      goal = -22;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //port level 2
    if(m_otherStick.getRawButton(6))
    {
      goal = -50;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //stop motor
    if(m_otherStick.getRawButton(7))
    {
      lift2.set(ControlMode.PercentOutput, 0);
    }
    */
    //if(m_leftStick.getRawButton(12))
    //{
      //Level3();
   // }
  

}

  @Override
  public void teleopPeriodic() {
        //motor limit controller
        x = m_leftStick.getRawAxis(3);
        x++;
        x /= 2;
    
        ticks = lift2.getSelectedSensorPosition();
        inchesOffGround = ticks/ticksToInches;
    
        //Encoder rotllllll.ation assignment
        
        //steering and moving
        // x is set as a multipliers to the values so we can control the maximum output of the motors
        m_myRobot.arcadeDrive(m_leftStick.getY()*x, m_leftStick.getX()*x);
       
        //solenoid on channel 4 and 2 
        //channel 2 shoots out the piston when true, while channel 4 retracts it when true
        //Shoots the piston when button is held, retracts when button is let go
    
    /*
        if(m_leftStick.getRawButton(1))
        {
          Sole3.set(true);
          Sole1.set(false);
        }
    
        if(m_leftStick.getRawButton(2))
        {
          Sole1.set(true);
          Sole3.set(false);
        }
    
        if(m_leftStick.getRawButton(4))
        {                                             
          Sole2.set(false);
          Sole4.set(true);
        }
    
        if(m_leftStick.getRawButton(6))
        {
          Sole2.set(true);
          Sole4.set(false);
        }
        */
    
       // SmartDashboard.putBoolean("clamp", Sole3.get());
       // SmartDashboard.putBoolean("extend", Sole2.get());
        SmartDashboard.putNumber("Inches Off Ground", inchesOffGround);
        
    
        //intake (subject to change motor values as needed)
        //gives motors positive value to wheel in a ball
        if(m_leftStick.getRawButton(7))
        {
            //m_Motor5.set(0.15);
            intake.set(0.15);
            //m_Motor8.set(0.15);
        }
    
        //resets motor when button is let go
        if(m_leftStick.getRawButton(11))
        {
          //m_Motor5.set(0);
          intake.set(0);
          //m_Motor8.set(0);
        }
    
        //gives motor negative value to shoot out the ball
        if(m_leftStick.getRawButton(8))
        {
          //m_Motor5.set(-0.15);
          intake.set(-0.40);
          //m_Motor8.set(-0.15);
        }
    
        //resets the motor when button is let go
    
    
        
        //brings lift to desired level; motor rotates for desired amount of times
        while(m_leftStick.getRawButton(9))
        {
          // lift2.set(ControlMode.PercentOutput, -.4); //clockwise
          lift2.set(ControlMode.PercentOutput, -.9);
        }
        while(m_leftStick.getRawButton(10))
        {
          lift2.set(ControlMode.PercentOutput, 1); //counterclockwise
        }
        if((!m_leftStick.getRawButton(9) && !m_leftStick.getRawButton(10)))
        {
          lift2.set(ControlMode.PercentOutput, 0);
        }
        /*
    //hatch level 1
    if(m_otherStick.getRawButton(1))
    {
      goal = 0;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
       goalreach = true;
    }
    //hatch level 2
    if(m_otherStick.getRawButton(2))
    {
      goal = -10;
      if( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);
      
      if( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //hatch level 3
    if(m_otherStick.getRawButton(3))
    {
      goal = -56;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //cargo port
    if(m_otherStick.getRawButton(4))
    {
      goal = -32;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //port level 1
    if(m_otherStick.getRawButton(5))
    {
      goal = -22;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //port level 2
    if(m_otherStick.getRawButton(6))
    {
      goal = -50;
      while( goal > inchesOffGround)
       lift2.set(ControlMode.PercentOutput, .7);
      
      while( goal < inchesOffGround)
       lift2.set(ControlMode.PercentOutput, -.7);

      if( goal == inchesOffGround) 
       lift2.set(ControlMode.PercentOutput, 0);
    }
    //stop motor
    if(m_otherStick.getRawButton(7))
    {
      lift2.set(ControlMode.PercentOutput, 0);
    }
    */
        //if(m_leftStick.getRawButton(12))
        //{
          //Level3();
       // }
      
    
  }
  
  
  


}//priest says in church open ur bible to psalm  -body once told me
//guxue is the best winston 
//tfw you're a black belt in taekwondo
/*
I'm not a gnelf.
I'm not a gnoblin.
I'm a gnome.
And you've been gnomed.
          / \
         /   \
        /     \
       /_______\
       // . . \\
      (/(__7__)\)
      |'-' = `-'|
      |         |
      /\       /\
     /  '.   .'  \
    / /|  `"`  |\ \
    \ \|===LI==|/ /
     '-|_______|-'
        |__|__|
       |--|--|
       (__)`(__)
*/