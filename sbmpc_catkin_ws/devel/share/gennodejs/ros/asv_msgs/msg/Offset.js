// Auto-generated. Do not edit!

// (in-package asv_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Offset {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.P_ca = null;
      this.Chi_ca = null;
    }
    else {
      if (initObj.hasOwnProperty('P_ca')) {
        this.P_ca = initObj.P_ca
      }
      else {
        this.P_ca = 0.0;
      }
      if (initObj.hasOwnProperty('Chi_ca')) {
        this.Chi_ca = initObj.Chi_ca
      }
      else {
        this.Chi_ca = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Offset
    // Serialize message field [P_ca]
    bufferOffset = _serializer.float64(obj.P_ca, buffer, bufferOffset);
    // Serialize message field [Chi_ca]
    bufferOffset = _serializer.float64(obj.Chi_ca, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Offset
    let len;
    let data = new Offset(null);
    // Deserialize message field [P_ca]
    data.P_ca = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Chi_ca]
    data.Chi_ca = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'asv_msgs/Offset';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '67c79bd0079a016b59fb0b6f517f39b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 P_ca
    float64 Chi_ca
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Offset(null);
    if (msg.P_ca !== undefined) {
      resolved.P_ca = msg.P_ca;
    }
    else {
      resolved.P_ca = 0.0
    }

    if (msg.Chi_ca !== undefined) {
      resolved.Chi_ca = msg.Chi_ca;
    }
    else {
      resolved.Chi_ca = 0.0
    }

    return resolved;
    }
};

module.exports = Offset;
