import math

import pyglet

from modules.sim_object import SimObject
from modules.light_sensor import LightSensor
from modules.lidar import Lidar

class Robot(SimObject):

    def __init__(self, sim_assets, *args, **kwargs):

        robot_image = sim_assets.image_assets["img_robot"]
        super(Robot, self).__init__(img=robot_image,*args, **kwargs)

        self.assets = sim_assets
        self.type = "robot"

        self.wheelBase = 1
        self.leftVelocity = 5       # wheel velocities on the ground
        self.rightVelocity = 10
        self.rotation = 0.0
        
        # add light sensors
        self.light_sensors = []
        self.light_sensor_offsets = [[40,25], [40,-25]]   # left sensor, right sensor
        for item in self.light_sensor_offsets:
            self.add_light_sensor(item[0],item[1])


        # add lidar - added at the centre of the robot
        self.lidar = None
        #self.add_lidar()


    def update_object(self, dt):
        self.update_position(dt)
        self.update_lidar_position()
        self.update_light_sensor_positions()
        self.update_velocities()
        

    # updates the position of the robot based on its wheel velocities - differential drive kinematics
    def update_position(self,dt):
        if self.leftVelocity == self.rightVelocity:
            self.x += self.leftVelocity*dt*math.cos(-self.rotation*math.pi/180.0) - self.leftVelocity*dt*math.sin(-self.rotation*math.pi/180.0)
            self.y += self.leftVelocity*dt*math.sin(-self.rotation*math.pi/180.0) + self.leftVelocity*dt*math.cos(-self.rotation*math.pi/180.0)
        else:
            R = 0.5 * self.wheelBase * (self.rightVelocity + self.leftVelocity) / (self.rightVelocity - self.leftVelocity)
            omega = (self.rightVelocity - self.leftVelocity) / self.wheelBase

            icr_x = self.x - R*math.sin(-self.rotation*math.pi/180.0)   # negative angle must be used because pyglet measure angles clockwise
            icr_y = self.y + R*math.cos(-self.rotation*math.pi/180.0)   # negative angle must be used because pyglet measure angles clockwise
            wdt = omega*dt # reducing frequent computation 

            self.x = math.cos(wdt)*(self.x - icr_x) - math.sin(wdt)*(self.y - icr_y) + icr_x
            self.y = math.sin(wdt)*(self.x - icr_x) + math.cos(wdt)*(self.y - icr_y) + icr_y
            self.rotation = self.rotation - wdt   # decrement because pyglet measure angles clockwise


    # updates light sensor position
    def update_lidar_position(self):
        if not self.lidar:
            pass
        else:
            self.lidar.x = self.x
            self.lidar.y = self.y
            self.lidar.rotation = self.rotation


    # updates snesor positions using robot position and offsets
    def update_light_sensor_positions(self):
        for sensor,offsets in zip(self.light_sensors, self.light_sensor_offsets):
            sensor.x = self.x + offsets[0]*math.cos(-self.rotation*math.pi/180.0) - offsets[1]*math.sin(-self.rotation*math.pi/180.0)
            sensor.y = self.y + offsets[0]*math.sin(-self.rotation*math.pi/180.0) + offsets[1]*math.cos(-self.rotation*math.pi/180.0)
            sensor.rotation = self.rotation


    # sets the wheel velocities
    def set_wheel_velocities(self, leftVelocity, rightVelocity):
        self.leftVelocity = leftVelocity
        self.rightVelocity = rightVelocity


    # adds a sensor at the specified location: x and y are relative to the centre of robot
    def add_light_sensor(self, x, y):
        new_sensor = LightSensor(self.assets, batch=self.batch, group=self.group)
        new_sensor.x = self.x + x
        new_sensor.y = self.y + y
        new_sensor.rotation = self.rotation
        self.child_objects.append(new_sensor)
        self.light_sensors.append(new_sensor)


    # adds a lidar at the centre of the robot
    def add_lidar(self):
        new_lidar = Lidar(self.assets,batch=self.batch, group=self.group)
        new_lidar.x = self.x
        new_lidar.y = self.y
        self.child_objects.append(new_lidar)
        self.lidar = new_lidar


    # update velocities based on some function of sensor intensity
    def update_velocities(self):
        # 100 is base velocity. We either add to it (excitatory) or subtract from it (inhibitory).
        # 0.9 is the weight. It determines how much the sensor intensity affects the velocity
        new_left_velocity = 100 + self.light_sensors[0].get_intensity() * 0.9
        new_right_velocity = 100 + self.light_sensors[1].get_intensity() * 0.9
        self.set_wheel_velocities(new_left_velocity, new_right_velocity)
