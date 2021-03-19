/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 *         Oana Gaskey 
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
#include <map>
#include <limits>

#define DELTA (std::numeric_limits<double>::min() * 1000.0)

using namespace std;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  // use a random number generator
  default_random_engine gen;
  
  // create a normal (Gaussian) distribution for x, y and theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  num_particles = 100;  // TODO: Set the number of particles
  
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
  default_random_engine gen;
  
  //loop over all particles and predict final x, y and theta after dt
  for (int i = 0; i < num_particles; ++i) {
    double x_0 = particles[i].x;
    double x_f;
    double y_0 = particles[i].y;
    double y_f;
    double theta_0 = particles[i].theta;
    double theta_f;
    
    //if yaw rate very small approximate to 0 and avoid divion by zero or overflow
    if (abs(yaw_rate) < DELTA) {
      //calculate x, y and theta final given the velocity, time elapsed and no yaw_rate
      theta_f = theta_0;
      x_f = x_0 + (velocity * delta_t * cos(theta_0) );
      y_f = y_0 + (velocity * delta_t * sin(theta_0) );
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
    normal_distribution<double> dist_x(x_f, std_pos[0]);
    normal_distribution<double> dist_y(y_f, std_pos[1]);
    normal_distribution<double> dist_theta(theta_f, std_pos[2]);
    // generate final position and angle given the Gaussian distribution
    particles[i].x     = dist_x(gen);
    particles[i].y     = dist_y(gen);
    particles[i].theta = dist_theta(gen);
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

  //loop over all particles
  for (int i = 0; i < num_particles; ++i) {
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    
    //initialize particle weight to 1
    double particle_weight = 1.0;
    
    //loop over all observations
    for (unsigned int j = 0; j < observations.size(); ++j){
      
      double obs_map_x  = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
      double obs_map_y  = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
      
      //initialize minimum distance between observation and landmark with highest value of type double
      double dist_min_obs_ldmk = std::numeric_limits<double>::max();
      //keep landmark assocition for this observation 
      int association;
      //loop over all landmarks to find the nearest one to the current observation
      for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); ++k){
        //calculate the distance between landmark k and current observation
        double dist_obs_ldmk = dist(map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f, obs_map_x, obs_map_y);
        if (dist_obs_ldmk < dist_min_obs_ldmk){
          dist_min_obs_ldmk = dist_obs_ldmk;
          association = k;
        }
      }//loop landmarks
      //get nearest neighbor information
      double n_n_ldmk_x = map_landmarks.landmark_list[association].x_f;
      double n_n_ldmk_y = map_landmarks.landmark_list[association].y_f;
      
      // Calculate multi-variate Gaussian distribution
      particle_weight *= multivariate_prob(std_landmark[0], std_landmark[1], obs_map_x, obs_map_y, n_n_ldmk_x, n_n_ldmk_y);
             
      //associations used for debugging
      associations.push_back(association+1);
      sense_x.push_back(obs_map_x);
      sense_y.push_back(obs_map_y);
    } //loop observations
    
    //set associations
    SetAssociations(particles[i],associations,sense_x,sense_y);
    
    // Update particle weights with combined multi-variate Gaussian distribution
    particles[i].weight = particle_weight;
    weights[i] = particles[i].weight;
    
  } //loop over particles
  
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  //resampled particles and weights to be passed on to particles and weights 
  //of the particle filter after resampling
  vector<Particle> resampled_particles;
  // use a random numer generator
  default_random_engine gen;
  //use a discrete distribution to return integers in range [0, weights.size())
  //with probability proportional to weights[i]
  discrete_distribution<int> d(weights.begin(), weights.end());
  
  //draw num_particles particle from particles
  for (int i = 0; i < num_particles; ++i){
    //generate particle using discrete distribution
    resampled_particles.push_back(particles[d(gen)]);
  }
  particles = std::move(resampled_particles);
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
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
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
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}