import math
import random

from modules.sim_object import SimObject

class LightSensor(SimObject):

    def __init__(self, sim_assets, *args, **kwargs):

        sensor_img = sim_assets.image_assets["img_light_sensor"]
        super(LightSensor, self).__init__(img=sensor_img,*args, **kwargs)

        self.type = "light_sensor"
        self.lights = []
        self.intensity = 0


    def update_object(self, dt):
        # a sensor only updates its intensity
        # simple implementation - sum up distance of sensor from all lights
        # divide by the number of lights
        # subtract from sqrt(2)/1000 which is the diagonal of the world
        self.intensity = 0
        for light in self.lights:
            self.intensity += self.get_distance_from(light)
        self.intensity = self.intensity / len(self.lights)
        self.intensity = math.sqrt(2)*1000 - self.intensity

    # returns the distance of this object from other object
    def get_distance_from(self, other):
        return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)


    # tells the sensor about all the lights in the world
    def set_lights(self, list_of_lights):
        self.lights = list_of_lights

    # retuns the light intensitiy detected by the sensor
    def get_intensity(self):
        return self.intensity