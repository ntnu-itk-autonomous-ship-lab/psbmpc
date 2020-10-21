// Auto-generated. Do not edit!

// (in-package asv_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let State = require('./State.js');

//-----------------------------------------------------------

class StateArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.states = null;
    }
    else {
      if (initObj.hasOwnProperty('states')) {
        this.states = initObj.states
      }
      else {
        this.states = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StateArray
    // Serialize message field [states]
    // Serialize the length for message field [states]
    bufferOffset = _serializer.uint32(obj.states.length, buffer, bufferOffset);
    obj.states.forEach((val) => {
      bufferOffset = State.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StateArray
    let len;
    let data = new StateArray(null);
    // Deserialize message field [states]
    // Deserialize array length for message field [states]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.states = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.states[i] = State.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.states.forEach((val) => {
      length += State.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'asv_msgs/StateArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '05e43d15ff5eab094661932878032f08';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # An array of states
    State[] states
    ================================================================================
    MSG: asv_msgs/State
    # The full state of a 3DOF surface vessel
    # eta: (x,y,psi)
    # nu:  (u,v,r)
    ShipMetaData header
    float64 x
    float64 y
    float64 psi
    float64 u
    float64 v
    float64 r
    ================================================================================
    MSG: asv_msgs/ShipMetaData
    uint8 id
    string name
    float32 radius
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StateArray(null);
    if (msg.states !== undefined) {
      resolved.states = new Array(msg.states.length);
      for (let i = 0; i < resolved.states.length; ++i) {
        resolved.states[i] = State.Resolve(msg.states[i]);
      }
    }
    else {
      resolved.states = []
    }

    return resolved;
    }
};

module.exports = StateArray;
