//! The inputs and outputs of the control laws: only what each law reads or produces, instead of whole
//! messages. `exec` builds them from the `Pose` of the pipe or of the telemetry, the law works on them,
//! and `exec` writes the result back into the `Pose` it hands on.
//!
//! Internal types, like `WorkVec` and `Transform`: not protobuf messages, never on the wire, and not
//! variants of `AnyMessage`. A `from_pose` returns `None` as soon as a required field is missing: the
//! law then abstains [P3].
//!
//! The chain down the cascade: `VelocitySetpoint` -> f_d (`Vec3`) -> `ForceSetpoint` ->
//! `AttitudeSetpoint` -> `WorkVec` -> `Vec<MotorCommand>`.

use crate::messages::{pose_messages::Pose, registered_message::{UnitQuat, Vec3}};

/// What the velocity law aims for. `v_d` is optional INSIDE an existing setpoint: without it the
/// mission asks for no velocity (stabilize), which is not the same as having no setpoint at all.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelocitySetpoint {
    /// Desired velocity (WORLD frame, m/s).
    pub v_d: Option<Vec3>,
}

impl VelocitySetpoint {
    pub fn from_pose(pose: &Pose) -> Self {
        return Self { v_d: pose.l_velocity };
    }
}

/// What the force -> attitude resolver works from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ForceSetpoint {
    /// f_d, the force to produce (WORLD frame, N).
    pub force: Vec3,
    /// q_pref, the attitude preference of the mission (body -> world).
    pub orientation: UnitQuat,
}

impl ForceSetpoint {
    pub fn from_pose(pose: &Pose) -> Option<Self> {
        return Some(Self { force: pose.force?, orientation: pose.orientation? });
    }
}

/// What the attitude law aims for, and what the resolver produces for it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeSetpoint {
    /// q_d, the attitude to reach (body -> world).
    pub orientation: UnitQuat,
    /// omega_d, the desired angular rate (BODY frame, rad/s).
    pub a_velocity: Vec3,
    /// f_d, the force to produce (WORLD frame, N).
    pub force: Vec3,
}

impl AttitudeSetpoint {
    pub fn from_pose(pose: &Pose) -> Option<Self> {
        return Some(Self { orientation: pose.orientation?,
                            a_velocity: pose.imu_measurement.as_ref()?.a_velocity?,
                            force: pose.force? });
    }

    /// Writes this setpoint into the `Pose` going down the pipe; its other fields go on untouched,
    /// including the other fields of its IMU sample.
    pub fn write_to(&self, pose: &mut Pose) {
        pose.orientation= Some(self.orientation);
        pose.imu_measurement.get_or_insert_with(Default::default).a_velocity= Some(self.a_velocity);
        pose.force= Some(self.force);
    }
}

/// What the attitude law measures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeState {
    /// q, the attitude of the vehicle (body -> world).
    pub orientation: UnitQuat,
    /// omega, the gyro (BODY frame, rad/s).
    pub a_velocity: Vec3,
}

impl AttitudeState {
    pub fn from_pose(pose: &Pose) -> Option<Self> {
        return Some(Self { orientation: pose.orientation?, a_velocity: pose.imu_measurement.as_ref()?.a_velocity? });
    }
}

#[cfg(test)]
mod tests {
    use crate::messages::pose_messages::{IMUMeasurements, Pose};
    use crate::messages::registered_message::{UnitQuat, Vec3};

    use super::*;

    const Q: UnitQuat= UnitQuat { w: 0.0, x: 1.0, y: 0.0, z: 0.0 };

    /// The rates part of a pose: the angular rate `omega`, and nothing else.
    fn rates(omega: Vec3) -> Option<IMUMeasurements> {
        return Some(IMUMeasurements { a_velocity: Some(omega), ..Default::default() });
    }

    #[test]
    fn a_velocity_setpoint_exists_with_or_without_a_desired_velocity() {
        // no v_d is stabilize, not a missing setpoint: the struct exists, and tells it
        assert_eq!(VelocitySetpoint::from_pose(&Pose::default()).v_d, None);
        let v_d= Vec3::new(0.0, 0.5, 0.0);
        assert_eq!(VelocitySetpoint::from_pose(&Pose { l_velocity: Some(v_d), ..Default::default() }).v_d, Some(v_d));
    }

    #[test]
    fn a_force_setpoint_needs_the_force_and_the_preference() {
        let force= Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(ForceSetpoint::from_pose(&Pose { force: Some(force), ..Default::default() }), None);
        assert_eq!(ForceSetpoint::from_pose(&Pose { orientation: Some(Q), ..Default::default() }), None);
        assert_eq!(ForceSetpoint::from_pose(&Pose { force: Some(force), orientation: Some(Q), ..Default::default() }),
                    Some(ForceSetpoint { force, orientation: Q }));
    }

    #[test]
    fn an_attitude_setpoint_needs_its_three_fields() {
        let (omega, force)= (Vec3::new(0.1, 0.2, 0.3), Vec3::new(1.0, 2.0, 3.0));
        let complete= Pose { orientation: Some(Q), imu_measurement: rates(omega), force: Some(force), ..Default::default() };
        assert_eq!(AttitudeSetpoint::from_pose(&complete), Some(AttitudeSetpoint { orientation: Q, a_velocity: omega, force }));
        // each required field missing in turn, including an IMU sample without its rate
        assert_eq!(AttitudeSetpoint::from_pose(&Pose { orientation: None, ..complete.clone() }), None);
        assert_eq!(AttitudeSetpoint::from_pose(&Pose { imu_measurement: None, ..complete.clone() }), None);
        assert_eq!(AttitudeSetpoint::from_pose(&Pose { imu_measurement: Some(IMUMeasurements::default()), ..complete.clone() }), None);
        assert_eq!(AttitudeSetpoint::from_pose(&Pose { force: None, ..complete.clone() }), None);
    }

    #[test]
    fn an_attitude_state_needs_the_attitude_and_the_gyro() {
        let omega= Vec3::new(0.1, 0.2, 0.3);
        let complete= Pose { orientation: Some(Q), imu_measurement: rates(omega), ..Default::default() };
        assert_eq!(AttitudeState::from_pose(&complete), Some(AttitudeState { orientation: Q, a_velocity: omega }));
        assert_eq!(AttitudeState::from_pose(&Pose { orientation: None, ..complete.clone() }), None);
        assert_eq!(AttitudeState::from_pose(&Pose { imu_measurement: None, ..complete.clone() }), None);
    }

    #[test]
    fn writing_an_attitude_setpoint_keeps_the_other_fields_of_the_pose() {
        // 1. a setpoint on its way down: v_d and an accelerometer sample that are not this stage's
        let v_d= Vec3::new(0.0, 0.5, 0.0);
        let l_accel= Vec3::new(0.0, 0.0, 9.81);
        let mut pose= Pose { l_velocity: Some(v_d), imu_measurement: Some(IMUMeasurements { l_accel: Some(l_accel), ..Default::default() }), ..Default::default() };
        // 2. its three fields are written
        let (omega, force)= (Vec3::new(0.1, 0.2, 0.3), Vec3::new(1.0, 2.0, 3.0));
        AttitudeSetpoint { orientation: Q, a_velocity: omega, force }.write_to(&mut pose);
        assert_eq!((pose.orientation, pose.force), (Some(Q), Some(force)));
        assert_eq!(pose.imu_measurement.as_ref().and_then(|imu| imu.a_velocity), Some(omega));
        // 3. everything else goes on untouched
        assert_eq!(pose.l_velocity, Some(v_d));
        assert_eq!(pose.imu_measurement.as_ref().and_then(|imu| imu.l_accel), Some(l_accel));
        // 4. a pose without any IMU sample gets one, holding the rate only
        let mut empty= Pose::default();
        AttitudeSetpoint { orientation: Q, a_velocity: omega, force }.write_to(&mut empty);
        assert_eq!(empty.imu_measurement, rates(omega));
    }
}
