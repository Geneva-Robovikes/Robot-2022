package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import static frc.robot.Constants.*; 

public class DriveSubsystem extends SubsystemBase {
  private WPI_TalonFX motorRightFront;
  private WPI_TalonFX motorLeftFront;
  private WPI_TalonFX motorRightBack;
  private WPI_TalonFX motorLeftBack;

  private ADIS16448_IMU gyro;

  public MotorController m_left;
  public MotorController m_right;

  public DifferentialDrive differentialDrive;

  private double adjustedAngle;

  public final static double BASE_WIDTH = 22.75;
  //TODO: change
  public final static double WHEEL_DIAMETER = 6.0; // inches

  public DriveSubsystem(
    WPI_TalonFX motorRightFront, WPI_TalonFX motorLeftFront, 
    WPI_TalonFX motorRightBack, WPI_TalonFX motorLeftBack, ADIS16448_IMU gyro
    ) {
      this.motorRightFront = motorRightFront;
      this.motorLeftFront = motorLeftFront;
      this.motorRightBack = motorRightBack;
      this.motorLeftBack = motorLeftBack;
      this.gyro = gyro;

      adjustedAngle = 0;

      m_left = new MotorControllerGroup(motorLeftBack, motorLeftFront);
      m_right = new MotorControllerGroup(motorRightBack, motorRightFront);

      differentialDrive = new DifferentialDrive(m_left, m_right);
  }

  public void setAllMotors(double value) {
    motorRightFront.set(value);
    motorLeftFront.set(-value);
    motorRightBack.set(value);
    motorLeftBack.set(-value);
  }

  public void setEachMotor(double rf, double lf, double rb, double lb) {
    motorRightFront.set(rf);
    motorLeftFront.set(-lf);
    motorRightBack.set(rb);
    motorLeftBack.set(-lb);
  }

  public void resetGyroAngle() {
    adjustedAngle = 0;
  }

  public double getGyroAngle() {
    return adjustedAngle;
  }

  public double getEncoderValueLeftBack() {
    return motorLeftBack.getSelectedSensorPosition();
  }

  public double getEncoderValueRightBack() {
    return motorLeftBack.getSelectedSensorPosition();
  }

  public double getEncoderInchesLeftBack() {
    final double wheelDiameter = 6;
    final double gearRatio = 10.71;
    final double unitsPerRevolution = 2048;
    return getEncoderValueLeftBack() / (gearRatio * unitsPerRevolution / (Math.PI * wheelDiameter));
  }

  public double getEncoderInchesRightBack() {
    final double wheelDiameter = 6;
    final double gearRatio = 10.71;
    final double unitsPerRevolution = 2048;
    return getEncoderValueRightBack() / (gearRatio * unitsPerRevolution / (Math.PI * wheelDiameter));
  }

  public void setRight(double num){
    m_right.set(num);
  }
  public void setLeft(double num){
    m_left.set(num);
  }

  @Override
  public void periodic() {
    if(Math.abs(gyro.getRate()) >= GYRO_DEAD_ZONE) {
      adjustedAngle += gyro.getRate() * DT;
    }
  }
}