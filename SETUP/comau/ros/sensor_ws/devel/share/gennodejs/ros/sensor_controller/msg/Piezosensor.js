// Auto-generated. Do not edit!

// (in-package sensor_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Piezosensor {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.thumb = null;
      this.index = null;
      this.middle = null;
      this.ring = null;
      this.little = null;
    }
    else {
      if (initObj.hasOwnProperty('thumb')) {
        this.thumb = initObj.thumb
      }
      else {
        this.thumb = [];
      }
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = [];
      }
      if (initObj.hasOwnProperty('middle')) {
        this.middle = initObj.middle
      }
      else {
        this.middle = [];
      }
      if (initObj.hasOwnProperty('ring')) {
        this.ring = initObj.ring
      }
      else {
        this.ring = [];
      }
      if (initObj.hasOwnProperty('little')) {
        this.little = initObj.little
      }
      else {
        this.little = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Piezosensor
    // Serialize message field [thumb]
    bufferOffset = _arraySerializer.float64(obj.thumb, buffer, bufferOffset, null);
    // Serialize message field [index]
    bufferOffset = _arraySerializer.float64(obj.index, buffer, bufferOffset, null);
    // Serialize message field [middle]
    bufferOffset = _arraySerializer.float64(obj.middle, buffer, bufferOffset, null);
    // Serialize message field [ring]
    bufferOffset = _arraySerializer.float64(obj.ring, buffer, bufferOffset, null);
    // Serialize message field [little]
    bufferOffset = _arraySerializer.float64(obj.little, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Piezosensor
    let len;
    let data = new Piezosensor(null);
    // Deserialize message field [thumb]
    data.thumb = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [index]
    data.index = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [middle]
    data.middle = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [ring]
    data.ring = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [little]
    data.little = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.thumb.length;
    length += 8 * object.index.length;
    length += 8 * object.middle.length;
    length += 8 * object.ring.length;
    length += 8 * object.little.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sensor_controller/Piezosensor';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e47c7f48cddbe1ac7a44256833731770';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] thumb
    float64[] index
    float64[] middle
    float64[] ring
    float64[] little
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Piezosensor(null);
    if (msg.thumb !== undefined) {
      resolved.thumb = msg.thumb;
    }
    else {
      resolved.thumb = []
    }

    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = []
    }

    if (msg.middle !== undefined) {
      resolved.middle = msg.middle;
    }
    else {
      resolved.middle = []
    }

    if (msg.ring !== undefined) {
      resolved.ring = msg.ring;
    }
    else {
      resolved.ring = []
    }

    if (msg.little !== undefined) {
      resolved.little = msg.little;
    }
    else {
      resolved.little = []
    }

    return resolved;
    }
};

module.exports = Piezosensor;
