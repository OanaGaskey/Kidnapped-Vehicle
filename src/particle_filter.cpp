/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <limits>

# define delta 0.0001

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  // use a random numer generator
  std::default_random_engine gen;
  
  // create a normal (Gaussian) distribution for x, y and theta
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
  
  // number chosen to match one particle/cm^2 in hypothetical uniform distribution across a rectangle of stddev_x * stddev_y 
  num_particles = 1000;  // TODO: Set the number of particles
  
  //loop over the total number of particles and initialize x, y and theta based on normal distribution around the GPS measurement
  for (int i = 0; i < num_particles; ++i) {
    //declare a particle of structure Particle
    Particle particle;
    //index i used for particle id
    particle.id     = i;
    particle.x      = dist_x(gen); 
    particle.y      = dist_y(gen);
    particle.theta  = dist_theta(gen);
    particle.weight = 1.0;
    // Print your samples to the terminal.
    //std::cout << "Sample " << i + 1 << " " << particle.x << " " << particle.y << " " 
    //          << particle.theta << std::endl;
    //store the particle in the particles vector   
    particles.push_back(particle);
    weights.push_back(1.0);
  }
  is_initialized = 1;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  
  // use a random numer generator
  std::default_random_engine gen;
  
  //loop over all particles and predict final x, y and theta after dt
  for (int i = 0; i < num_particles; ++i) {
    double x_0 = particles[i].x;
    double x_f;
    double y_0 = particles[i].y;
    double y_f;
    double theta_0 = particles[i].theta;
    double theta_f;
    
    //if yaw rate very small approximate to 0 and avoid divion by zero or overflow
    if (std::abs(yaw_rate) < delta) {
      //calculate x, y and theta final given the velocity, time elapsed and no yaw_rate
      theta_f = theta_0;
      x_f = x_0 + (velocity * cos(theta_0) * delta_t);
      y_f = y_0 + (velocity * sin(theta_0) * delta_t);
    }
    else{
      //calculate theta final given the yaw rate and the time elapsed
      theta_f = theta_0 + (yaw_rate * delta_t);
      //assure that theta is between 0 and 2*PI
      if (theta_f >= 2*M_PI){ theta_f = theta_f - 2*M_PI; }
      if (theta_f < 0.0 ) { theta_f = theta_f + 2*M_PI; }
      //calculate x and y final given the velocity, yaw rate and the time elapsed
      x_f = x_0 + ( (velocity/yaw_rate) * (sin(theta_f) - sin(theta_0)) );
      y_f = y_0 + ( (velocity/yaw_rate) * (cos(theta_0) - cos(theta_f)) );
    }
       
    // create a normal (Gaussian) distribution for x_f, y_f and theta_f
    std::normal_distribution<double> dist_x(x_f, std_pos[0]);
    std::normal_distribution<double> dist_y(y_f, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta_f, std_pos[2]);
    // generate final position and angle given the Gaussian distribution
    particles[i].x     = dist_x(gen);
    particles[i].y     = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
  
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  //loop over all observations
  for (unsigned int i = 0; i < observations.size(); ++i){
    //initialize shortest distance from an observation to a landmark with the maximum value for double data type
    double shortest_dist = std::numeric_limits<double>::max();
    double current_dist;
    //loop over all predicted observable landmarks
    for (unsigned int j = 0; j < predicted.size(); ++j){
      //calculate the distance from observation i to landmark j
      current_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      //if this is the shortest distance yet found, store the value locally and record the landmark id in the LandmarkObs structure
      if (current_dist < shortest_dist) {
        shortest_dist = current_dist;
        observations[i].id = predicted[j].id;
      }   
    }
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  //transform observations from particle coordinates to map coordinates
  //loop over all particles and all observations to apply transformation: translation and rotation
  for (int i = 0; i < num_particles; ++i) {
    vector<LandmarkObs> observations_map;
    for (unsigned int j = 0; j < observations.size(); ++j){
      LandmarkObs observation_map;
      observation_map.x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
      observation_map.y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
      observation_map.id = 0; // don't associate this observation with any landmark id yet
      observations_map.push_back(observation_map);
    }
    //predict the observable landmarks from particle i given the map and sensor range
    vector<LandmarkObs> predicted_observations;
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j){
      LandmarkObs observable_landmark;
      if (dist(particles[i].x, particles[i].y, double(map_landmarks.landmark_list[j].x_f), double(map_landmarks.landmark_list[j].y_f)) <= sensor_range){
        observable_landmark.x  = double(map_landmarks.landmark_list[j].x_f);
        observable_landmark.y  = double(map_landmarks.landmark_list[j].y_f);
        observable_landmark.id = map_landmarks.landmark_list[j].id_i;
        predicted_observations.push_back(observable_landmark);
      }
    }
    
    //associate observations with landmarks based on nearest neighbor principle
    //this function will fill in the id in each observation corresponding to the closest map landmark
    dataAssociation (predicted_observations, observations_map);
        
    //calculate the particle's new weight
    double weight = 1.0;
    for (unsigned int j = 0; j < observations_map.size(); ++j){ 
      weight *= multivatiate_prob(std_landmark[0], std_landmark[1], observations_map[j].x, observations_map[j].y, 
                            	  double(map_landmarks.landmark_list[observations_map[j].id].x_f), double(map_landmarks.landmark_list[observations_map[j].id].y_f));
    }
    particles[i].weight = weight;
    weights[i] = weight;
  }
  
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}