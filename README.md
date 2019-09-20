# Kidnapped-Vehicle
Particle Filter algorithm to localize a vehicle within a few centimeters, given a map, LiDAR measurements, vehicle controls and GPS.

![VehicleWorld](images/VehicleWorld.JPG)
 
 Traditional vehicles use GPS localization embedded in the navigation system. As long as a human is driving the car this localization is good enough. 

 The GPS localizes the car with an accuracy of about 2 meters giving the driver a general idea of where he/she is. The driver then visually percieves the world to get a sense of what's around. He/she can see how far the car is from the curb or stop sign and that is how he/she's able to maneuvers the car.

 For a self driving car, a 2 meter localization accuracy is simply not enough. It could be in a whole different lane or on the sidewalk and it wouldn't know. For safety reasons and to even make self driving maneuvers possible, the localization precision needs to be within a few centimeters.


 ![LidarMeasurements](images/LidarMeasurements.JPG)

 To be able to localize itself, a self driving car needs to sense the worls around. Since the most accurate sensor for distance measurement is the LiDAR, this is the way to go. 

 The self driving car also needs a map, but this is no usual map like in traditional navigation systems. The map that proves to be most useful is one made of landmark positions. The self driving car will rely on knowing where a building, stop sign, light pole is in a given intersection and then when measuring its distance to those landmarks is able to localize itself.

 Most landmark maps are build using LiDAR measurements, recording the landmark positions while the car is being driven around by a human driver. 

 This means that the self driving car has _seen_ the streets before driving on them.

 ##Particle Filter
 Based on [Bayes Theorem](https://en.wikipedia.org/wiki/Bayes%27_theorem) the [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter) takes a probabilistic approach on possible positions of where the car could be and calculates the probability density.

 The particles are possible vechile positions, as the vehicle doesn't know where it is located. You spread them around on the map and calculate their likelyhood. Each time the vehicle senses the world around, it measures its distance to known landmarks. 

 These distances are used to identify particles that are likely to be positioned at those distances from the landmarks. Particles that have higher likelyhoods are kept and can replace the ones that are unlikely, keeping the total number of particles constant. 

 With this approach you start with _N_ particles that are spread all over the map and as the vechicle moves around and senses the world, the _N_ particles tend to gravitate around the car forming a dense cloud.

 ##Initialization
 ##Prediction
 ##Measurement
 ##Resampling 

# Implementation