// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Lanceur extends SubsystemBase {
  private final CANSparkMax moteurG = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax moteurD = new CANSparkMax(9, MotorType.kBrushless);

  private final double conversionEncodeur = 1;// à déterminer

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> voltageApplique = mutable(Volts.of(0));
  // Mutable holder for unit-safe angular values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> distanceLue = mutable(Rotations.of(0));
  // Mutable holder for unit-safe angular velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Angle>> vitesseLue = mutable(RotationsPerSecond.of(0));

  // Création de la routine de SysID
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            moteurG.setVoltage(volts.in(Volts));
            moteurD.setVoltage(volts.in(Volts));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the left motors. Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("Moteur Gauche")
                .voltage(
                    voltageApplique.mut_replace(
                        moteurG.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(distanceLue.mut_replace(getPositionG(), Rotations))
                .angularVelocity(
                    vitesseLue.mut_replace(getVitesseG(), RotationsPerSecond));
            // Record a frame for the right motors. Since these share an encoder, we
            // consider
            // the entire group to be one motor.
            log.motor("Moteur Droite")
                .voltage(
                    voltageApplique.mut_replace(
                        moteurD.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(distanceLue.mut_replace(getPositionD(), Rotations))
                .angularVelocity(
                    vitesseLue.mut_replace(getVitesseD(), RotationsPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("drive")
          this));

  /** Creates a new Lanceur. */
  public Lanceur() {
    moteurG.setInverted(true);
    moteurD.setInverted(false);

    moteurG.getEncoder().setPositionConversionFactor(conversionEncodeur);
    moteurG.getEncoder().setVelocityConversionFactor(conversionEncodeur / 60); // pour obtenir des rotation par seconde

    moteurD.getEncoder().setPositionConversionFactor(conversionEncodeur);
    moteurD.getEncoder().setVelocityConversionFactor(conversionEncodeur / 60); // pour obtenir des rotation par seconde

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Lanceur gauche - Position", getPositionG());
    SmartDashboard.putNumber("Lanceur droit - Position", getPositionD());
    SmartDashboard.putNumber("Lanceur gauche - Vitesse", getVitesseG());
    SmartDashboard.putNumber("Lanceur droit - Vitesse", getVitesseD());
  }

  public double getPositionG() {
    return moteurG.getEncoder().getPosition();// en rotation
  }

  public double getVitesseG() {
    return moteurG.getEncoder().getVelocity();// en rotation par seconde
  }

  public double getPositionD() {
    return moteurD.getEncoder().getPosition();
  }

  public double getVitesseD() {
    return moteurD.getEncoder().getVelocity();
  }

  /////// Commande SysID

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

}
