package frc.robot.lib.swervelib;

import frc.robot.lib.swervelib.ctre.*;
import frc.robot.lib.swervelib.rev.NeoDriveControllerFactoryBuilder;
import frc.robot.lib.swervelib.rev.NeoSteerConfiguration;
import frc.robot.lib.swervelib.rev.NeoSteerControllerFactoryBuilder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final class Mk3SwerveModuleHelper {
        private Mk3SwerveModuleHelper() {}

        private static DriveControllerFactory<?, Integer> getNeoDriveFactory(
                        Mk3ModuleConfiguration configuration) {
                return new NeoDriveControllerFactoryBuilder()
                                .withVoltageCompensation(configuration.getNominalVoltage())
                                .withCurrentLimit(configuration.getDriveCurrentLimit())
                                .withPidConstants(configuration.getDrivePIDGains())
                                .build();
        }

        private static SteerControllerFactory<?, NeoSteerConfiguration<CanCoderAbsoluteConfiguration>> getNeoSteerFactory(
                        Mk3ModuleConfiguration configuration) {
                return new NeoSteerControllerFactoryBuilder()
                                .withVoltageCompensation(configuration.getNominalVoltage())
                                .withPidConstants(1.0, 0.0, 0.1)
                                .withCurrentLimit(configuration.getSteerCurrentLimit())
                                .build(new CanCoderFactoryBuilder().withReadingUpdatePeriod(100)
                                                .build());
        }


        /**
         * Creates a Mk3 swerve module that uses NEOs for driving and steering. Module information
         * is displayed in the specified ShuffleBoard container.
         *
         * @param container The container to display module information in.
         * @param configuration Module configuration parameters to use.
         * @param gearRatio The gearing configuration the module is in.
         * @param driveMotorPort The CAN ID of the drive NEO.
         * @param steerMotorPort The CAN ID of the steer NEO.
         * @param steerEncoderPort The CAN ID of the steer CANCoder.
         * @param steerOffset The offset of the CANCoder in radians.
         * @return The configured swerve module.
         */
        public static SwerveModule createNeo(ShuffleboardLayout container,
                        Mk3ModuleConfiguration configuration, GearRatio gearRatio,
                        int driveMotorPort, int steerMotorPort, int steerEncoderPort,
                        double steerOffset) {
                return new SwerveModuleFactory<>(gearRatio.getConfiguration(),
                                getNeoDriveFactory(configuration),
                                getNeoSteerFactory(configuration)).create(
                                                container, driveMotorPort,
                                                new NeoSteerConfiguration<>(steerMotorPort,
                                                                new CanCoderAbsoluteConfiguration(
                                                                                steerEncoderPort,
                                                                                steerOffset)));
        }

        /**
         * Creates a Mk3 swerve module that uses NEOs for driving and steering. Module information
         * is displayed in the specified ShuffleBoard container.
         *
         * @param container The container to display module information in.
         * @param gearRatio The gearing configuration the module is in.
         * @param driveMotorPort The CAN ID of the drive NEO.
         * @param steerMotorPort The CAN ID of the steer NEO.
         * @param steerEncoderPort The CAN ID of the steer CANCoder.
         * @param steerOffset The offset of the CANCoder in radians.
         * @return The configured swerve module.
         */
        public static SwerveModule createNeo(ShuffleboardLayout container, GearRatio gearRatio,
                        int driveMotorPort, int steerMotorPort, int steerEncoderPort,
                        double steerOffset) {
                return createNeo(container, new Mk3ModuleConfiguration(), gearRatio, driveMotorPort,
                                steerMotorPort, steerEncoderPort, steerOffset);
        }

        /**
         * Creates a Mk3 swerve module that uses NEOs for driving and steering.
         *
         * @param configuration Module configuration parameters to use.
         * @param gearRatio The gearing configuration the module is in.
         * @param driveMotorPort The CAN ID of the drive NEO.
         * @param steerMotorPort The CAN ID of the steer NEO.
         * @param steerEncoderPort The CAN ID of the steer CANCoder.
         * @param steerOffset The offset of the CANCoder in radians.
         * @return The configured swerve module.
         */
        public static SwerveModule createNeo(Mk3ModuleConfiguration configuration,
                        GearRatio gearRatio, int driveMotorPort, int steerMotorPort,
                        int steerEncoderPort, double steerOffset) {
                return new SwerveModuleFactory<>(gearRatio.getConfiguration(),
                                getNeoDriveFactory(configuration),
                                getNeoSteerFactory(configuration)).create(
                                                driveMotorPort,
                                                new NeoSteerConfiguration<>(steerMotorPort,
                                                                new CanCoderAbsoluteConfiguration(
                                                                                steerEncoderPort,
                                                                                steerOffset)));
        }

        /**
         * Creates a Mk3 swerve module that uses NEOs for driving and steering.
         *
         * @param gearRatio The gearing configuration the module is in.
         * @param driveMotorPort The CAN ID of the drive NEO.
         * @param steerMotorPort The CAN ID of the steer NEO.
         * @param steerEncoderPort The CAN ID of the steer CANCoder.
         * @param steerOffset The offset of the CANCoder in radians.
         * @return The configured swerve module.
         */
        public static SwerveModule createNeo(GearRatio gearRatio, int driveMotorPort,
                        int steerMotorPort, int steerEncoderPort, double steerOffset) {
                return createNeo(new Mk3ModuleConfiguration(), gearRatio, driveMotorPort,
                                steerMotorPort, steerEncoderPort, steerOffset);
        }

        public enum GearRatio {
                /**
                 * Mk3 swerve in the standard gear configuration.
                 */
                STANDARD(SdsModuleConfigurations.MK3_STANDARD),
                /**
                 * Mk3 swerve in the fast gear configuration.
                 */
                FAST(SdsModuleConfigurations.MK3_FAST);

                private final ModuleConfiguration configuration;

                GearRatio(ModuleConfiguration configuration) {
                        this.configuration = configuration;
                }

                public ModuleConfiguration getConfiguration() {
                        return configuration;
                }
        }
}
