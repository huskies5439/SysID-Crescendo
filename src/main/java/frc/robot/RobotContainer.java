// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Lanceur;

public class RobotContainer {

  private final Lanceur lanceur = new Lanceur();

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData(chooser);
    chooser.addOption("Lanceur - Quasistatique - Avancer", lanceur.sysIdQuasistatic(Direction.kForward));
    chooser.addOption("Lanceur - Quasistatique - Reculer", lanceur.sysIdQuasistatic(Direction.kReverse));
    chooser.addOption("Lanceur - Dynamique - Avancer", lanceur.sysIdDynamic(Direction.kForward));
    chooser.addOption("Lanceur - Dynamique - Reculer", lanceur.sysIdDynamic(Direction.kReverse));


  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
