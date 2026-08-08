use std::ops::{Add, AddAssign, Deref, DerefMut, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use prost::bytes::{Buf, BufMut};
use prost::encoding::{double, skip_field, DecodeContext, WireType};
use prost::{DecodeError, Message};
use crate::messages::{lidar_messages::LidarMeasurements, motor_messages::{MotorCommand, MotorFeedBack}, pose_messages::{GNSSMeasurement, IMUMeasurements, Pose}};

/// Enum that represent the ID of the message can be sent and / or received
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum MessageType {
    ImuRawMessage = 0,
    GNSSRawMessage = 1,
    MotorFeedBackMessage = 2,
    MotorModelMessage= 3,
    PoseMessage = 4,
    LidarMappingMessage = 5,
}

/// Enum that respresent the different kind of message that can be transmitted
#[derive(Clone)]
pub enum AnyMessage {
    ImuState(IMUMeasurements),
    GnssState(GNSSMeasurement),
    MotorState(MotorFeedBack),
    MotorCommands(Vec<MotorCommand>),
    PoseState(Pose),
    LidarState(LidarMeasurements),
}

/// Protobuf wire form of `nalgebra::Vector3<f64>`. It *contains* a real `Vector3`, so every nalgebra
/// method is reachable through `Deref` (`v.cross(&w)`, `v.norm()`, `v.dot(&w)`, …) with no conversion,
/// and the hand-written `Message` impl puts the three components straight onto the wire — as close as
/// the orphan rule allows to "serializing a `Vector3` directly".
#[derive(Copy, Clone, PartialEq, Default, Debug)]
pub struct Vec3(pub Vector3<f64>);

impl Vec3 {
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
vec3_binop!(Mul::mul, MulAssign::mul_assign, |a, b| a.component_mul(&b));
vec3_binop!(Div::div, DivAssign::div_assign, |a, b| a.component_div(&b));

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
#[derive(Copy, Clone, PartialEq, Message)]
pub struct UnitQuat {
    #[prost(double, tag = "1")]
    pub w: f64,
    #[prost(double, tag = "2")]
    pub x: f64,
    #[prost(double, tag = "3")]
    pub y: f64,
    #[prost(double, tag = "4")]
    pub z: f64,
}

impl From<UnitQuaternion<f64>> for UnitQuat {
    fn from(q: UnitQuaternion<f64>) -> Self {
        return Self { w: q.w, x: q.i, y: q.j, z: q.k };
    }
}

impl From<UnitQuat> for UnitQuaternion<f64> {
    fn from(q: UnitQuat) -> Self {
        return UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z));
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
    const MSG_TYPE: MessageType;

    // [1-byte MessageType][protobuf bytes]
    // Add an 1 byte ID in the result frame to know directly what is the type of the message 
    fn to_frame(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(1 + self.encoded_len());
        out.push(Self::MSG_TYPE as u8);
        out.extend_from_slice(&self.encode_to_vec());
        return out;
    }

    /// inverse: drop the tag byte, prost-decode the rest
    fn from_frame(frame: &[u8]) -> Result<Self, prost::DecodeError> {
        <Self as prost::Message>::decode(&frame[1..])
    }
}