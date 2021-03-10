function decodeUplink(input) {
  var data = {};
  data.timestamp = (input.bytes[0]) + (input.bytes[1]<<8) + (input.bytes[2]<<16) +  (input.bytes[3]<<24);
  data.long = ((input.bytes[4]) + (input.bytes[5]<<8) + (input.bytes[6]<<16) +  (input.bytes[7]<<24)) / 10000.0;
  data.lat = ((input.bytes[8]) + (input.bytes[9]<<8) + (input.bytes[10]<<16) +  (input.bytes[11]<<24)) / 10000.0;
  data.alt = ((input.bytes[12]) + (input.bytes[13]<<8) + (input.bytes[14]<<16) +  (input.bytes[15]<<24)) / 100.0;
  data.speed = ((input.bytes[16]) + (input.bytes[17]<<8) + (input.bytes[18]<<16) +  (input.bytes[19]<<24)) / 100.0;
  data.direction = ((input.bytes[20]) + (input.bytes[21]<<8) + (input.bytes[22]<<16) +  (input.bytes[23]<<24)) / 100.0;
  data.voltage = ((input.bytes[24]) + (input.bytes[25]<<8)) / 100.0;
  data.listlen = input.bytes[26];
  return {
    data: data
  };
}
