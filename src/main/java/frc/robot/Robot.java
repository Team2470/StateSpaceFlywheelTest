// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a sample program to demonstrate how to use a state-space controller to control a
 * flywheel.
 */
public class Robot extends TimedRobot {
  private static final int kJoystickPort = 0;
  private static final double kSpinupRadPerSec1000 = Units.rotationsPerMinuteToRadiansPerSecond(1000.0);
  private static final double kSpinupRadPerSec2000 = Units.rotationsPerMinuteToRadiansPerSecond(2000.0);
  private static final double kSpinupRadPerSec2500 = Units.rotationsPerMinuteToRadiansPerSecond(2500.0);
  private static final double kSpinupRadPerSec3000 = Units.rotationsPerMinuteToRadiansPerSecond(3000.0);

  // Volts per (radian per second)
  private static final double kFlywheelKv = 0.020163 * 1.0335654; // 0.020839779

  // Volts per (radian per second squared)
  private static final double kFlywheelKa = 0.0011182;
  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  //
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_flywheelPlant,
          VecBuilder.fill(8.0), // Velocity error tolerance. Default 8
          VecBuilder.fill(12.0), // Control effort (voltage) tolerance
          0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

  private CANSparkMax m_shooterLeader;
  private CANSparkMax m_shooterFollower;
  private RelativeEncoder m_encoder;

  // A joystick to read the trigger from.
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  @Override
  public void robotInit() {
    // We go 2 pi radians per 4096 clicks.
    //m_encoder.setDistancePerPulse(2.0 * Math.PI / 4096.0);
    //m_encoder.setVelocityConversionFactor(2.0 * Math.PI / 4096.0);
    m_shooterLeader = new CANSparkMax(2, MotorType.kBrushless);
    m_shooterLeader.restoreFactoryDefaults();
    m_shooterFollower = new CANSparkMax(3, MotorType.kBrushless);
    m_shooterFollower.restoreFactoryDefaults();
    m_shooterFollower.follow(m_shooterLeader, true);
    m_shooterFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_shooterFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    m_shooterFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    m_shooterLeader.setInverted(true);
    m_shooterFollower.setInverted(true);
    m_encoder = m_shooterLeader.getEncoder();


    m_shooterLeader.setSmartCurrentLimit(40);
    m_shooterFollower.setSmartCurrentLimit(40);
    //m_shooterLeader.enableVoltageCompensation(10);
    //m_shooterFollower.enableVoltageCompensation(10);

    m_shooterLeader.setIdleMode(IdleMode.kCoast);
    m_shooterFollower.setIdleMode(IdleMode.kCoast);

    m_controller.latencyCompensate(m_flywheelPlant, 0.02, 0.025);
  }

  @Override
  public void teleopInit() {
    // Reset our loop to make sure it's in a known state.
    m_loop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity())));
    targetSpeedRadS = 0;
    SmartDashboard.putNumber("Shooter Control RPM", 0);

    m_shooterLeader.setIdleMode(IdleMode.kCoast);
    m_shooterFollower.setIdleMode(IdleMode.kCoast);

  }

  private double targetSpeedRadS = 0;

  public void teleopPeriodicTEst() {
    m_shooterLeader.setVoltage(4);
    double shooterRadS = Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());

    SmartDashboard.putNumber("Shooter RAD/S", shooterRadS);
    SmartDashboard.putNumber("Shooter Target RAD/S", targetSpeedRadS);
    SmartDashboard.putNumber("Shooter RPM", Units.radiansPerSecondToRotationsPerMinute(shooterRadS));
    SmartDashboard.putNumber("Shooter Leader getAppliedOutput", m_shooterLeader.getAppliedOutput());
  }

  // @Override
  public void teleopPeriodic() {
    // Sets the target speed of our flywheel. This is similar to setting the setpoint of a
    // PID controller.
    if (m_joystick.getRawButtonPressed(1)) {
      // We just pressed the trigger, so let's set our next reference
      targetSpeedRadS = Units.rotationsPerMinuteToRadiansPerSecond(SmartDashboard.getNumber("Shooter Control RPM", 0));
    } else if (m_joystick.getRawButtonReleased(1)) {
      // We just released the trigger, so let's spin down
      targetSpeedRadS = 0;
    }
    m_loop.setNextR(VecBuilder.fill(targetSpeedRadS));

    // Correct our Kalman filter's state vector estimate with encoder data.
    double shooterRadS = Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());
    m_loop.correct(VecBuilder.fill(shooterRadS));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    m_shooterLeader.setVoltage(nextVoltage);

    double errorRadS = shooterRadS - targetSpeedRadS;
    SmartDashboard.putNumber("Shooter Next Voltage", nextVoltage);
    SmartDashboard.putNumber("Shooter RAD/S", shooterRadS);
    SmartDashboard.putNumber("Shooter Target RAD/S", targetSpeedRadS);
    SmartDashboard.putNumber("Shooter Error RAD/S", errorRadS);
    SmartDashboard.putNumber("Shooter RPM", Units.radiansPerSecondToRotationsPerMinute(shooterRadS));
    SmartDashboard.putNumber("Shooter Target RPM", Units.radiansPerSecondToRotationsPerMinute(targetSpeedRadS));
    SmartDashboard.putNumber("Shooter Error RPM", Units.radiansPerSecondToRotationsPerMinute(errorRadS));
    SmartDashboard.putNumber("getCountsPerRevolution", m_encoder.getCountsPerRevolution());
    SmartDashboard.putNumber("Shooter Leader getAppliedOutput", m_shooterLeader.getAppliedOutput());

    double precentError = (targetSpeedRadS - shooterRadS)/targetSpeedRadS * 100;
    SmartDashboard.putNumber("Shooter Precent Error", precentError);


  }
}
