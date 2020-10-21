// Auto-generated. Do not edit!

// (in-package asv_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WP = require('./WP.js');

//-----------------------------------------------------------

class WPArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wp_xy = null;
    }
    else {
      if (initObj.hasOwnProperty('wp_xy')) {
        this.wp_xy = initObj.wp_xy
      }
      else {
        this.wp_xy = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WPArray
    // Serialize message field [wp_xy]
    // Serialize the length for message field [wp_xy]
    bufferOffset = _serializer.uint32(obj.wp_xy.length, buffer, bufferOffset);
    obj.wp_xy.forEach((val) => {
      bufferOffset = WP.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WPArray
    let len;
    let data = new WPArray(null);
    // Deserialize message field [wp_xy]
    // Deserialize array length for message field [wp_xy]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.wp_xy = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.wp_xy[i] = WP.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.wp_xy.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'asv_msgs/WPArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '311aa1759c0b5850ba11ceafcb7a7e61';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # WP array containing (x,y) 
    WP[] wp_xy
    
    ================================================================================
    MSG: asv_msgs/WP
    # Waypoint position
    # WP: (x,y)
    float64 x
    float64 y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WPArray(null);
    if (msg.wp_xy !== undefined) {
      resolved.wp_xy = new Array(msg.wp_xy.length);
      for (let i = 0; i < resolved.wp_xy.length; ++i) {
        resolved.wp_xy[i] = WP.Resolve(msg.wp_xy[i]);
      }
    }
    else {
      resolved.wp_xy = []
    }

    return resolved;
    }
};

module.exports = WPArray;
