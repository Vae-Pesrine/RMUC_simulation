// Auto-generated. Do not edit!

// (in-package sentry_userdefinition.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class LocalizationInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.if_relocation = null;
      this.point_cloud_quantity = null;
      this.tranDist = null;
      this.angleDist = null;
      this.angle_apeed = null;
      this.score = null;
      this.if_match_success = null;
    }
    else {
      if (initObj.hasOwnProperty('if_relocation')) {
        this.if_relocation = initObj.if_relocation
      }
      else {
        this.if_relocation = false;
      }
      if (initObj.hasOwnProperty('point_cloud_quantity')) {
        this.point_cloud_quantity = initObj.point_cloud_quantity
      }
      else {
        this.point_cloud_quantity = 0.0;
      }
      if (initObj.hasOwnProperty('tranDist')) {
        this.tranDist = initObj.tranDist
      }
      else {
        this.tranDist = 0.0;
      }
      if (initObj.hasOwnProperty('angleDist')) {
        this.angleDist = initObj.angleDist
      }
      else {
        this.angleDist = 0.0;
      }
      if (initObj.hasOwnProperty('angle_apeed')) {
        this.angle_apeed = initObj.angle_apeed
      }
      else {
        this.angle_apeed = 0.0;
      }
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0.0;
      }
      if (initObj.hasOwnProperty('if_match_success')) {
        this.if_match_success = initObj.if_match_success
      }
      else {
        this.if_match_success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LocalizationInfo
    // Serialize message field [if_relocation]
    bufferOffset = _serializer.bool(obj.if_relocation, buffer, bufferOffset);
    // Serialize message field [point_cloud_quantity]
    bufferOffset = _serializer.float64(obj.point_cloud_quantity, buffer, bufferOffset);
    // Serialize message field [tranDist]
    bufferOffset = _serializer.float64(obj.tranDist, buffer, bufferOffset);
    // Serialize message field [angleDist]
    bufferOffset = _serializer.float64(obj.angleDist, buffer, bufferOffset);
    // Serialize message field [angle_apeed]
    bufferOffset = _serializer.float64(obj.angle_apeed, buffer, bufferOffset);
    // Serialize message field [score]
    bufferOffset = _serializer.float64(obj.score, buffer, bufferOffset);
    // Serialize message field [if_match_success]
    bufferOffset = _serializer.bool(obj.if_match_success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LocalizationInfo
    let len;
    let data = new LocalizationInfo(null);
    // Deserialize message field [if_relocation]
    data.if_relocation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [point_cloud_quantity]
    data.point_cloud_quantity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tranDist]
    data.tranDist = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angleDist]
    data.angleDist = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angle_apeed]
    data.angle_apeed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [score]
    data.score = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [if_match_success]
    data.if_match_success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 42;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sentry_userdefinition/LocalizationInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4480d6179e334a6455057a4ec084d3ef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool if_relocation
    float64 point_cloud_quantity
    float64 tranDist
    float64 angleDist
    float64 angle_apeed
    float64 score
    bool if_match_success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LocalizationInfo(null);
    if (msg.if_relocation !== undefined) {
      resolved.if_relocation = msg.if_relocation;
    }
    else {
      resolved.if_relocation = false
    }

    if (msg.point_cloud_quantity !== undefined) {
      resolved.point_cloud_quantity = msg.point_cloud_quantity;
    }
    else {
      resolved.point_cloud_quantity = 0.0
    }

    if (msg.tranDist !== undefined) {
      resolved.tranDist = msg.tranDist;
    }
    else {
      resolved.tranDist = 0.0
    }

    if (msg.angleDist !== undefined) {
      resolved.angleDist = msg.angleDist;
    }
    else {
      resolved.angleDist = 0.0
    }

    if (msg.angle_apeed !== undefined) {
      resolved.angle_apeed = msg.angle_apeed;
    }
    else {
      resolved.angle_apeed = 0.0
    }

    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0.0
    }

    if (msg.if_match_success !== undefined) {
      resolved.if_match_success = msg.if_match_success;
    }
    else {
      resolved.if_match_success = false
    }

    return resolved;
    }
};

module.exports = LocalizationInfo;
