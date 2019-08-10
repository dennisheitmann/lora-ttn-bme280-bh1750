function Decoder(data, port) {
  var temp = (data[0] | data[1]<<8) / 100.0;
  var baro = (data[2] | data[3]<<8) / 10.0;
  var humi = (data[4] | data[5]<<8) / 100.0;
  var lux = (data[6] | data[7]<<8);
  var decoded = {"temp": {}, "baro": {}, "humi": {}, "lux": {}};
  decoded.temp.value = temp;
  decoded.temp.unit = "C";
  decoded.baro.value = baro;
  decoded.baro.unit = "mbar (abs)";
  decoded.humi.value = humi;
  decoded.humi.unit = "% (rel)";
  decoded.lux.value = lux;
  decoded.lux.unit = "lux";
  return decoded;
}
