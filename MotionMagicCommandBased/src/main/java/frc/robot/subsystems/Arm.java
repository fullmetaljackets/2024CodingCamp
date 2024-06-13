// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
  final TalonFX m_fx = new TalonFX(1, "");
  final TalonFX m_Fx2 = new TalonFX(10, "");
  final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  final XboxController m_joystick = new XboxController(0);


  /** Creates a new Arm. */
  public Arm() {  

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    
    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 10.71; // 12.8 rotor rotations per mechanism rotation
    
    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 5; // 5 (mechanism) rotations per second cruise
    mm.MotionMagicAcceleration = 1; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.1 seconds to reach max accel 
    mm.MotionMagicJerk = 100;
    
    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
    
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
    m_Fx2.setControl(new Follower(m_Fx2.getDeviceID(), false));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    }

  public void setMy_ArmPos(double setpoint){
    m_fx.setControl(m_mmReq.withPosition(setpoint).withSlot(0));
  }
}
