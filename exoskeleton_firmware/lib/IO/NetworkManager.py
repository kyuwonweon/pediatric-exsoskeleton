
import micropython                                  # type: ignore
import pyb                                          # type: ignore

import network, time

# class for connecting to the router
class NetworkManager:
    def __init__(self,
                 network_name= "Tenda_1E1920"):

        self.network_name = network_name
        # self.connect_WLAN()

    def connect_WLAN(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(self.network_name)
        for _ in range(20):
            if wlan.isconnected():
                print("Successfully connected to WiFi with address:")
                break
            time.sleep(0.5)
        return wlan.ifconfig()[0]

# class for creating an access point to which the computer can actually connect
class AccessPointManager:
    def __init__(self,
                 ssid="FoxTails",
                 password="robot1234",
                 channel=11):

        self.ssid = ssid
        self.password = password
        self.channel = channel
        self.ap = network.WLAN(network.AP_IF)

    def start_AP(self):
        self.ap.active(False)
        time.sleep(1)
        self.ap.active(True)

        self.ap.config(
            essid=self.ssid,
            password=self.password,
            channel=self.channel,
            pm = 0xa11140
        )

        # wait until AP is up
        time.sleep(0.5)

        ip, netmask, gw, dns = self.ap.ifconfig()
        print("###############")
        print("Access Point started")
        print("SSID:", self.ssid)
        print("IP address:", ip)
        print("AP active:", self.ap.active())
        print("AP ifconfig:", self.ap.ifconfig())
        print("###############")


        return ip
