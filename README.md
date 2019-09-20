# Kidnapped-Vehicle
Particle Filter algorithm to localize a vehicle within a few centimeters, given a map, LiDAR measurements, vehicle controls and GPS.

![VehicleWorld](images/VehicleWorld.JPG)
 
 Traditional vehicles use GPS licalization embedded in the navigation system. As long as a human is driving the car this localization is good enough. The GPS localizes the car with an accuracy of about 2 meters giving the driver a general idea of where he/she is. The driver then visually percieves the world around to get a sense of what's around. He/she can see how far the car is from the curb or stop sign and that is how he/she's able to maneuvers the car.
 For a self driving car, a 2 meter localization accuracy is simply not enough. It could be in a whole different lane or on the sidewalk and it wouldn't know. For safety reasons and to even make self driving maneuvers possible, the localization precision needs to be within a few centimeters.

 ![LidarMeasurements](images/LidarMeasurements.JPG)
 To be able to localize itself, a self driving car needs to sense the worls around. Since the most accurate sensor for distance measurement is the LiDAR, this is the way to go. The self driving car also needs a map, but this is no usual map like in traditional navigation systems. The map that proves to be most useful is one made of landmark positions. The self driving car will rely on knowing where a building, stop sign, light pole is in a given intersection and then measuring its distance to those landmarks is able to localize itself.

# Implementation