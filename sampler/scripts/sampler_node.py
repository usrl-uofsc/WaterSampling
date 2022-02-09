#!/usr/bin/python3
import rospy as rp
import threading

from pump_driver import Pump

from mavros_msgs.msg import RCIn
from watersampling_msgs.msg import MS5837Stamped, PumpInfo


class SamplerNode():
    """
    A ROS node that monitors the water sensors and triggers the sampling
    """

    _RC_TRIGGER_CHANNEL_MAIN = 10    #check the /mavros/rc/in topic to find out which switch controls which channel
    _RC_HIGH_MAIN = 1494
    _RC_TRIGGER_CHANNEL_SamplerC = 12    #check the /mavros/rc/in topic to find out which switch controls which channel
    _RC_HIGH_SamplerC = 1494

    _MASTER_PUMP_PIN = 38
    _SAMPLING_PUMP_A = 37
    _SAMPLING_PUMP_B = 36
    _SAMPLING_PUMP_C = 35

    def __init__(self, rate):
        rp.init_node("water_sampler_node")
        self.rate = rate

        # Pump instances
        self.master_pump = Pump(self._MASTER_PUMP_PIN)
        self.sampling_pump_a = Pump(self._SAMPLING_PUMP_A)
        self.sampling_pump_b = Pump(self._SAMPLING_PUMP_B)
        self.sampling_pump_c = Pump(self._SAMPLING_PUMP_C)

        self.enable_sampler_main = False
        self.enable_sampler_C = False
        self.inlet_depth = 0.0

        # ROS Subscribers
        self.rc_sub = rp.Subscriber(
            '/mavros/rc/in', RCIn, self.rcCallback, queue_size=1)
        self.depth_sensor_sub = rp.Subscriber(
            '/watersampling/depth_sensor', MS5837Stamped, self.depthSensorCallback, queue_size=1)

        # ROS Publisher
        self.pump_info_pub = rp.Publisher(
            '/watersampling/sampler_pump_info', PumpInfo, queue_size=1)

        t = threading.Thread(target=self.pumpInfoPublisher)
        t.start()

        rp.spin()

    def rcCallback(self, msg):
        #print(msg.channels[self._RC_TRIGGER_CHANNEL])                          #for troubleshooting
        
        if msg.channels[self._RC_TRIGGER_CHANNEL_SamplerC] == self._RC_HIGH_SamplerC:
            self.enable_sampler_C = True
        else:
            self.enable_sampler_C = False
        
        if msg.channels[self._RC_TRIGGER_CHANNEL_MAIN] == self._RC_HIGH_MAIN:
            self.enable_sampler_main = True
        else:
            self.enable_sampler_main = False

    def depthSensorCallback(self, msg):
        self.inlet_depth = msg.depth.data

    def stateMachine(self,):
        # TODO: Check RC and sensors and control pumps
        if self.enable_sampler_main == True:
            print(self.inlet_depth)
            if self.inlet_depth >= 0.20:
                self.master_pump.start()
                if self.enable_sampler_C == True:
                    self.sampling_pump_c.start()
                else:
                    self.sampling_pump_c.stop()
            else:
                self.master_pump.stop()
                self.sampling_pump_c.stop()
        else:
            self.master_pump.stop()
            self.sampling_pump_c.stop()

    def pumpInfoPublisher(self,):
        r = rp.Rate(self.rate)
        pump_msg = PumpInfo()

        while not rp.is_shutdown():
            self.stateMachine()

            # Populate and publish pump message
            pump_msg.header = rp.Time.now()
            pump_msg.master_pump.data = self.master_pump.isrunning
            pump_msg.sampling_pump_a.data = self.sampling_pump_a.isrunning
            pump_msg.sampling_pump_b.data = self.sampling_pump_b.isrunning
            pump_msg.sampling_pump_c.data = self.sampling_pump_c.isrunning

            self.pump_info_pub.publish(pump_msg)

        r.sleep()


if __name__ == '__main__':
    SamplerNode(30)