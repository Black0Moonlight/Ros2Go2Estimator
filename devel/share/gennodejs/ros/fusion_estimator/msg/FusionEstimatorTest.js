// Auto-generated. Do not edit!

// (in-package fusion_estimator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FusionEstimatorTest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.data_check_a = null;
      this.data_check_b = null;
      this.data_check_c = null;
      this.data_check_d = null;
      this.data_check_e = null;
      this.estimated_xyz = null;
      this.estimated_rpy = null;
      this.feet_based_position = null;
      this.feet_based_velocity = null;
      this.others = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('data_check_a')) {
        this.data_check_a = initObj.data_check_a
      }
      else {
        this.data_check_a = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('data_check_b')) {
        this.data_check_b = initObj.data_check_b
      }
      else {
        this.data_check_b = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('data_check_c')) {
        this.data_check_c = initObj.data_check_c
      }
      else {
        this.data_check_c = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('data_check_d')) {
        this.data_check_d = initObj.data_check_d
      }
      else {
        this.data_check_d = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('data_check_e')) {
        this.data_check_e = initObj.data_check_e
      }
      else {
        this.data_check_e = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('estimated_xyz')) {
        this.estimated_xyz = initObj.estimated_xyz
      }
      else {
        this.estimated_xyz = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('estimated_rpy')) {
        this.estimated_rpy = initObj.estimated_rpy
      }
      else {
        this.estimated_rpy = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('feet_based_position')) {
        this.feet_based_position = initObj.feet_based_position
      }
      else {
        this.feet_based_position = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('feet_based_velocity')) {
        this.feet_based_velocity = initObj.feet_based_velocity
      }
      else {
        this.feet_based_velocity = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('others')) {
        this.others = initObj.others
      }
      else {
        this.others = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FusionEstimatorTest
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Check that the constant length array field [data_check_a] has the right length
    if (obj.data_check_a.length !== 9) {
      throw new Error('Unable to serialize array field data_check_a - length must be 9')
    }
    // Serialize message field [data_check_a]
    bufferOffset = _arraySerializer.float64(obj.data_check_a, buffer, bufferOffset, 9);
    // Check that the constant length array field [data_check_b] has the right length
    if (obj.data_check_b.length !== 12) {
      throw new Error('Unable to serialize array field data_check_b - length must be 12')
    }
    // Serialize message field [data_check_b]
    bufferOffset = _arraySerializer.float64(obj.data_check_b, buffer, bufferOffset, 12);
    // Check that the constant length array field [data_check_c] has the right length
    if (obj.data_check_c.length !== 12) {
      throw new Error('Unable to serialize array field data_check_c - length must be 12')
    }
    // Serialize message field [data_check_c]
    bufferOffset = _arraySerializer.float64(obj.data_check_c, buffer, bufferOffset, 12);
    // Check that the constant length array field [data_check_d] has the right length
    if (obj.data_check_d.length !== 12) {
      throw new Error('Unable to serialize array field data_check_d - length must be 12')
    }
    // Serialize message field [data_check_d]
    bufferOffset = _arraySerializer.float64(obj.data_check_d, buffer, bufferOffset, 12);
    // Check that the constant length array field [data_check_e] has the right length
    if (obj.data_check_e.length !== 12) {
      throw new Error('Unable to serialize array field data_check_e - length must be 12')
    }
    // Serialize message field [data_check_e]
    bufferOffset = _arraySerializer.float64(obj.data_check_e, buffer, bufferOffset, 12);
    // Check that the constant length array field [estimated_xyz] has the right length
    if (obj.estimated_xyz.length !== 9) {
      throw new Error('Unable to serialize array field estimated_xyz - length must be 9')
    }
    // Serialize message field [estimated_xyz]
    bufferOffset = _arraySerializer.float64(obj.estimated_xyz, buffer, bufferOffset, 9);
    // Check that the constant length array field [estimated_rpy] has the right length
    if (obj.estimated_rpy.length !== 9) {
      throw new Error('Unable to serialize array field estimated_rpy - length must be 9')
    }
    // Serialize message field [estimated_rpy]
    bufferOffset = _arraySerializer.float64(obj.estimated_rpy, buffer, bufferOffset, 9);
    // Check that the constant length array field [feet_based_position] has the right length
    if (obj.feet_based_position.length !== 12) {
      throw new Error('Unable to serialize array field feet_based_position - length must be 12')
    }
    // Serialize message field [feet_based_position]
    bufferOffset = _arraySerializer.float64(obj.feet_based_position, buffer, bufferOffset, 12);
    // Check that the constant length array field [feet_based_velocity] has the right length
    if (obj.feet_based_velocity.length !== 12) {
      throw new Error('Unable to serialize array field feet_based_velocity - length must be 12')
    }
    // Serialize message field [feet_based_velocity]
    bufferOffset = _arraySerializer.float64(obj.feet_based_velocity, buffer, bufferOffset, 12);
    // Check that the constant length array field [others] has the right length
    if (obj.others.length !== 4) {
      throw new Error('Unable to serialize array field others - length must be 4')
    }
    // Serialize message field [others]
    bufferOffset = _arraySerializer.float64(obj.others, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FusionEstimatorTest
    let len;
    let data = new FusionEstimatorTest(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [data_check_a]
    data.data_check_a = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [data_check_b]
    data.data_check_b = _arrayDeserializer.float64(buffer, bufferOffset, 12)
    // Deserialize message field [data_check_c]
    data.data_check_c = _arrayDeserializer.float64(buffer, bufferOffset, 12)
    // Deserialize message field [data_check_d]
    data.data_check_d = _arrayDeserializer.float64(buffer, bufferOffset, 12)
    // Deserialize message field [data_check_e]
    data.data_check_e = _arrayDeserializer.float64(buffer, bufferOffset, 12)
    // Deserialize message field [estimated_xyz]
    data.estimated_xyz = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [estimated_rpy]
    data.estimated_rpy = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [feet_based_position]
    data.feet_based_position = _arrayDeserializer.float64(buffer, bufferOffset, 12)
    // Deserialize message field [feet_based_velocity]
    data.feet_based_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 12)
    // Deserialize message field [others]
    data.others = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 832;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fusion_estimator/FusionEstimatorTest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e10b193fd0c51a4937b1aeebfa61ac4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp
    float64[9] data_check_a
    float64[12] data_check_b
    float64[12] data_check_c
    float64[12] data_check_d
    float64[12] data_check_e
    float64[9] estimated_xyz
    float64[9] estimated_rpy
    float64[12] feet_based_position
    float64[12] feet_based_velocity
    float64[4] others
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FusionEstimatorTest(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.data_check_a !== undefined) {
      resolved.data_check_a = msg.data_check_a;
    }
    else {
      resolved.data_check_a = new Array(9).fill(0)
    }

    if (msg.data_check_b !== undefined) {
      resolved.data_check_b = msg.data_check_b;
    }
    else {
      resolved.data_check_b = new Array(12).fill(0)
    }

    if (msg.data_check_c !== undefined) {
      resolved.data_check_c = msg.data_check_c;
    }
    else {
      resolved.data_check_c = new Array(12).fill(0)
    }

    if (msg.data_check_d !== undefined) {
      resolved.data_check_d = msg.data_check_d;
    }
    else {
      resolved.data_check_d = new Array(12).fill(0)
    }

    if (msg.data_check_e !== undefined) {
      resolved.data_check_e = msg.data_check_e;
    }
    else {
      resolved.data_check_e = new Array(12).fill(0)
    }

    if (msg.estimated_xyz !== undefined) {
      resolved.estimated_xyz = msg.estimated_xyz;
    }
    else {
      resolved.estimated_xyz = new Array(9).fill(0)
    }

    if (msg.estimated_rpy !== undefined) {
      resolved.estimated_rpy = msg.estimated_rpy;
    }
    else {
      resolved.estimated_rpy = new Array(9).fill(0)
    }

    if (msg.feet_based_position !== undefined) {
      resolved.feet_based_position = msg.feet_based_position;
    }
    else {
      resolved.feet_based_position = new Array(12).fill(0)
    }

    if (msg.feet_based_velocity !== undefined) {
      resolved.feet_based_velocity = msg.feet_based_velocity;
    }
    else {
      resolved.feet_based_velocity = new Array(12).fill(0)
    }

    if (msg.others !== undefined) {
      resolved.others = msg.others;
    }
    else {
      resolved.others = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = FusionEstimatorTest;
