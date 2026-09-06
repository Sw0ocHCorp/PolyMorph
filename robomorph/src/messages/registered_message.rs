//! Message catalogue of the stack: `AnyMessage`, the single enum that travels on every tokio
//! `broadcast` channel and through the scheduler pipe; `MessageType`, the 1-byte tag prefixed to
//! protobuf frames on the wire; `Translatable`, the trait that builds / parses such frames; and
//! the two small geometric wire types shared by all the messages, `Vec3` and `UnitQuat`.
//!
//! Wire frame layout (see `communications::interface`): `[MessageType as u8][protobuf bytes]`.

use std::ops::{Add, AddAssign, Deref, DerefMut, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use prost::bytes::{Buf, BufMut};
use prost::encoding::{double, skip_field, DecodeContext, WireType};
use prost::{DecodeError, Message};
use crate::control::joystick::joystick_controller::RemoteControl;
use crate::messages::motor_messages::WorkVec;
use crate::messages::{lidar_messages::LidarMeasurements, motor_messages::{MotorCommand, MotorFeedBack}, pose_messages::{GNSSMeasurement, IMUMeasurements, Pose}};

/// Enum that represent the ID of the message can be sent and / or received
///
/// This is the 1-byte tag written in front of the protobuf payload by `Translatable::to_frame` and
/// dispatched on by `communications::interface::decode_frame`. Each `Translatable` implementation
/// names its tag through `Translatable::MSG_TYPE`.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum MessageType {
    /// `pose_messages::IMUMeasurements`
    ImuRawMessage = 0,
    /// `pose_messages::GNSSMeasurement`
    GNSSRawMessage = 1,
    /// `motor_messages::MotorFeedBack`
    MotorFeedBackMessage = 2,
    /// `motor_messages::MotorModel`
    // NOTE: encodable (`MotorModel: Translatable`) but `decode_frame` has no arm for this tag and
    // `AnyMessage` has no variant carrying a `MotorModel`: such a frame is dropped on reception.
    MotorModelMessage= 3,
    /// `pose_messages::Pose`
    PoseMessage = 4,
    /// `lidar_messages::LidarMeasurements`
    LidarMappingMessage = 5,
    /// `control::joystick::joystick_controller::RemoteControl`
    // NOTE: encoded by `encode_frame` but not decoded by `decode_frame`: dropped on reception.
    RemoteControlMessage= 6,
}

/// Enum that respresent the different kind of message that can be transmitted
///
/// The single currency of the stack: every tokio `broadcast` channel carries `AnyMessage`, and so
/// does the scheduler pipe (`Process::exec` input / output). Producers wrap their payload,
/// consumers pattern-match on the variant they expect and ignore the others.
///
/// Wire-capable variants (they have a `MessageType`): `ImuState`, `GnssState`, `MotorState`,
/// `PoseState`, `LidarState`, `RemoteControl`. Internal-only variants: `MotorCommands` and
/// `VehicleWrench` (`encode_frame` returns an empty frame for them).
#[derive(Clone)]
pub enum AnyMessage {
    /// Raw IMU sample (body frame).
    ImuState(IMUMeasurements),
    /// Raw GNSS sample.
    GnssState(GNSSMeasurement),
    /// Feedback of ONE motor (current effort, last setpoint, status), published per motor by the
    /// vehicle controller.
    MotorState(MotorFeedBack),
    /// One command per motor for the current cycle, produced by `MotorsMixer` and consumed by the
    /// vehicle controller (Gazebo side), which converts it to actuator units.
    MotorCommands(Vec<MotorCommand>),
    /// 6D wrench setpoint in the body frame, produced by `AttitudeController` and consumed by
    /// `MotorsMixer` (through the pipe in the current wiring).
    VehicleWrench(WorkVec),
    /// Aggregated vehicle state (attitude, IMU, GNSS, position, velocity estimate). The same
    /// variant carries the attitude SETPOINT.
    PoseState(Pose),
    /// One lidar sweep.
    LidarState(LidarMeasurements),
    /// Snapshot of the gamepad events seen during one `XboxPadControl` tick.
    RemoteControl(RemoteControl)
}

/// Protobuf wire form of `nalgebra::Vector3<f64>`. It *contains* a real `Vector3`, so every nalgebra
/// method is reachable through `Deref` (`v.cross(&w)`, `v.norm()`, `v.dot(&w)`, …) with no conversion,
/// and the hand-written `Message` impl puts the three components straight onto the wire — as close as
/// the orphan rule allows to "serializing a `Vector3` directly".
///
/// On the wire: three `double` fields, tags 1 / 2 / 3 = x / y / z. Messages embed it as
/// `Option<Vec3>` (prost message field), `None` when absent. The derived `Default` is the zero
/// vector. The unit and frame are those of the field that carries it (m, m/s, rad/s, ...).
#[derive(Copy, Clone, PartialEq, Default, Debug)]
pub struct Vec3(pub Vector3<f64>);

impl Vec3 {
    /// Build from three components (same order as `Vector3::new`).
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        return Self(Vector3::new(x, y, z));
    }
}

impl Deref for Vec3 {
    type Target = Vector3<f64>;
    fn deref(&self) -> &Self::Target {
        return &self.0;
    }
}

impl DerefMut for Vec3 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        return &mut self.0;
    }
}

impl From<Vector3<f64>> for Vec3 {
    fn from(v: Vector3<f64>) -> Self {
        return Self(v);
    }
}

impl From<Vec3> for Vector3<f64> {
    fn from(v: Vec3) -> Self {
        return v.0;
    }
}

// Every arithmetic operator on Vec3 delegates to the wrapped Vector3, so the math lives in nalgebra
// and never gets duplicated component-by-component here. `*` / `/` are the element-wise products.
macro_rules! vec3_binop {
    ($op:ident::$method:ident, $assign:ident::$assign_method:ident, |$a:ident, $b:ident| $expr:expr) => {
        impl $op for Vec3 {
            type Output = Vec3;
            fn $method(self, rhs: Self) -> Vec3 {
                let ($a, $b) = (self.0, rhs.0);
                return Vec3($expr);
            }
        }
        impl $assign for Vec3 {
            fn $assign_method(&mut self, rhs: Self) {
                *self = $op::$method(*self, rhs);
            }
        }
    };
}

vec3_binop!(Add::add, AddAssign::add_assign, |a, b| a + b);
vec3_binop!(Sub::sub, SubAssign::sub_assign, |a, b| a - b);
// `Vec3 * Vec3` and `Vec3 / Vec3` are the Hadamard (component-wise) products, NOT a dot or cross
// product: use `v.dot(&w)` / `v.cross(&w)` (reached through `Deref`) for those.
vec3_binop!(Mul::mul, MulAssign::mul_assign, |a, b| a.component_mul(&b));
vec3_binop!(Div::div, DivAssign::div_assign, |a, b| a.component_div(&b));

/// Protobuf codec of `Vec3`: fields 1 / 2 / 3 = x / y / z as `double`; unknown tags are skipped.
/// All three components are always written (no default-value elision).
impl Message for Vec3 {
    fn encode_raw(&self, buf: &mut impl BufMut) {
        double::encode(1, &self.0.x, buf);
        double::encode(2, &self.0.y, buf);
        double::encode(3, &self.0.z, buf);
    }

    fn merge_field(&mut self, tag: u32, wire_type: WireType, buf: &mut impl Buf, ctx: DecodeContext) -> Result<(), DecodeError> {
        match tag {
            1 => double::merge(wire_type, &mut self.0.x, buf, ctx),
            2 => double::merge(wire_type, &mut self.0.y, buf, ctx),
            3 => double::merge(wire_type, &mut self.0.z, buf, ctx),
            _ => skip_field(wire_type, tag, buf, ctx),
        }
    }

    fn encoded_len(&self) -> usize {
        return double::encoded_len(1, &self.0.x)
            + double::encoded_len(2, &self.0.y)
            + double::encoded_len(3, &self.0.z);
    }

    fn clear(&mut self) {
        self.0 = Vector3::zeros();
    }
}

/// Wire representation of an orientation quaternion. Converts to / from `num_quaternion::Q64`.
///
/// `Message` is hand-written rather than derived, like `Vec3` above: the derive also emits a
/// `Default` with every field at zero, and a (0,0,0,0) quaternion has a null norm — normalising
/// it yields NaN. The identity is the only sane default for an orientation, so `Default` is
/// written by hand below, which rules the derive out.
///
/// Field order is `w, x, y, z`: the scalar part comes FIRST. nalgebra stores its coordinates as
/// `(i, j, k, w)` internally, which is what a `{:?}` print of a `UnitQuaternion` shows, so do not
/// read one against the other. `From<UnitQuaternion>` maps `w, i, j, k` onto `w, x, y, z`;
/// `From<UnitQuat>` renormalises (`from_quaternion`), so always convert to a `UnitQuaternion`
/// before inverting or composing and never operate on the raw wire fields.
///
/// Meaning depends on the message: in `Pose::orientation` it is the vehicle attitude BODY -> WORLD;
/// in `MotorModel::relative_orientation` / `Transform::orientation` it is a motor frame -> parent
/// (resp. body) frame.
// NOTE: the conversions below are with `nalgebra::UnitQuaternion<f64>`; the `num_quaternion::Q64`
// mention above predates the switch to nalgebra.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct UnitQuat {
    /// Scalar (real) part.
    pub w: f64,
    /// Vector part, x component (nalgebra's `i`).
    pub x: f64,
    /// Vector part, y component (nalgebra's `j`).
    pub y: f64,
    /// Vector part, z component (nalgebra's `k`).
    pub z: f64,
}

/// Protobuf codec of `UnitQuat`: fields 1 / 2 / 3 / 4 = w / x / y / z as `double`. `clear` resets
/// to the identity, not to zeros.
impl Message for UnitQuat {
    fn encode_raw(&self, buf: &mut impl BufMut) {
        double::encode(1, &self.w, buf);
        double::encode(2, &self.x, buf);
        double::encode(3, &self.y, buf);
        double::encode(4, &self.z, buf);
    }

    fn merge_field(&mut self, tag: u32, wire_type: WireType, buf: &mut impl Buf, ctx: DecodeContext) -> Result<(), DecodeError> {
        match tag {
            1 => double::merge(wire_type, &mut self.w, buf, ctx),
            2 => double::merge(wire_type, &mut self.x, buf, ctx),
            3 => double::merge(wire_type, &mut self.y, buf, ctx),
            4 => double::merge(wire_type, &mut self.z, buf, ctx),
            _ => skip_field(wire_type, tag, buf, ctx),
        }
    }

    fn encoded_len(&self) -> usize {
        return double::encoded_len(1, &self.w)
            + double::encoded_len(2, &self.x)
            + double::encoded_len(3, &self.y)
            + double::encoded_len(4, &self.z);
    }

    fn clear(&mut self) {
        *self = Self::default();
    }
}

/// nalgebra -> wire: `w, i, j, k` become `w, x, y, z`.
impl From<UnitQuaternion<f64>> for UnitQuat {
    fn from(q: UnitQuaternion<f64>) -> Self {
        return Self { w: q.w, x: q.i, y: q.j, z: q.k };
    }
}

/// Wire -> nalgebra. `from_quaternion` normalises, so a slightly denormalised wire value is
/// repaired here (and an all-zero one becomes NaN, hence the identity `Default`).
impl From<UnitQuat> for UnitQuaternion<f64> {
    fn from(q: UnitQuat) -> Self {
        return UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z));
    }
}

/// Identity rotation: the only sensible default for an orientation (see the struct doc).
impl Default for UnitQuat {
    fn default() -> Self {
        return Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
    }
}

/// Trait that allow to parse or fill message params according to it's type
/// 
/// Notes:
/// 
/// Each Struct that implement the Translatable trait must define it's own MSG_TYPE 
/// 
/// to be able to put the message type before the content of the message in frame / decode correctly the incoming frame 
pub trait Translatable: prost::Message + Default + Sized {
    // ID of the type of message
    /// Tag byte written in front of the protobuf payload by `to_frame` and matched by
    /// `communications::interface::decode_frame`.
    const MSG_TYPE: MessageType;

    // [1-byte MessageType][protobuf bytes]
    // Add an 1 byte ID in the result frame to know directly what is the type of the message 
    /// Serialise into a wire frame: `[MSG_TYPE as u8][prost encoding of self]`.
    fn to_frame(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(1 + self.encoded_len());
        out.push(Self::MSG_TYPE as u8);
        out.extend_from_slice(&self.encode_to_vec());
        return out;
    }

    /// inverse: drop the tag byte, prost-decode the rest
    // NOTE: the tag byte is skipped without being checked, and an empty slice panics
    // (`&frame[1..]` on a zero-length slice). `decode_frame` guards both by dispatching on the tag
    // and rejecting frames shorter than 2 bytes; do not call this on raw input directly.
    fn from_frame(frame: &[u8]) -> Result<Self, prost::DecodeError> {
        <Self as prost::Message>::decode(&frame[1..])
    }
}