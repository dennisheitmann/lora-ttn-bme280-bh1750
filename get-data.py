import ttn
import time

app_id = "__APP_ID__"
access_key = "ttn-account-v2.__PASSWORD__"

def uplink_callback(msg, client):
  print(msg.metadata.time, end=';')
  print(msg.payload_fields.temp.value, end=';')
  print(msg.payload_fields.baro.value, end=';')
  print(msg.payload_fields.humi.value, end=';')
  print(msg.payload_fields.lux.value, end=';')
  print(msg.metadata.gateways[0].rssi, end=';')
  print(msg.metadata.gateways[0].snr, flush=True)

handler = ttn.HandlerClient(app_id, access_key)

# using mqtt client
mqtt_client = handler.data()
mqtt_client.set_uplink_callback(uplink_callback)
mqtt_client.connect()
while (True):
    time.sleep(60)
mqtt_client.close()
