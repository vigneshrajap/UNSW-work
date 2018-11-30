
#include <fastslam/particle_filter.h>

namespace fastslam
{

ParticleFilter::ParticleFilter(int n_particles) : particles_(n_particles), n_particles_(n_particles)
{

}

void ParticleFilter::initParticles()
{

  boost::uniform_real<> unif(0.0, 2.0*M_PI);
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> > angle_gen(rng_, unif);

  for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle)
  {
    double angle = angle_gen();
    particle->qw() = cos(angle/2.0);
    particle->qx() = 0.0;
    particle->qy() = 0.0;
    particle->qz() = sin(angle/2.0);
  }

}

void ParticleFilter::sample(const Eigen::Vector4d & motion, double dt)
{

  boost::normal_distribution<double> gaussian(0.0, 0.01);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > noise_gen(rng_, gaussian);

  for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle)
  {

    Eigen::Vector4d motion_noise;
    motion_noise << noise_gen(),
                    noise_gen(),
                    noise_gen(),
                    noise_gen();

    particle->processModel(motion + motion_noise, dt);

  }

}

void ParticleFilter::resample(const Eigen::Vector3d & meas_mean, const Eigen::Matrix3d & meas_cov)
{

  for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle)
  {
    Eigen::Vector3d delta = meas_mean - particle->trans();
    double log_weight = -0.5*delta.transpose()*meas_cov.inverse()*delta;
    particle->weight() = exp(log_weight);
  }

  normaliseWeights();

  double sum_sq_weights = 0.0;

  for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle)
  {
    sum_sq_weights += particle->weight()*particle->weight();
  }

  double n_effective = 1/sum_sq_weights;
  if (n_effective < 90)
  {
    resampleParticles();
  }

}

void ParticleFilter::normaliseWeights()
{

  double weights_sum = 0.0;

  for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle)
  {
    weights_sum += particle->weight();
  }

  for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle)
  {
    particle->weight() /= weights_sum;
  }

}

void ParticleFilter::resampleParticles()
{

  std::vector<double> particle_weights(n_particles_);

  for (int i = 0; i < n_particles_; ++i)
  {
    particle_weights[i] = particles_[i].weight();
  }

  std::vector<double> cumulative_particle_weights;
  std::partial_sum(particle_weights.begin(), particle_weights.end(), std::back_inserter(cumulative_particle_weights));
  boost::uniform_real<> unif(0, cumulative_particle_weights.back());
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> > idx_gen(rng_, unif);

  std::vector<int> new_particle_idx(n_particles_);

  std::vector<Particle> new_particles(n_particles_);

  for (int i = 0; i < n_particles_; ++i)
  {
    // Convert iterator to index;
    int particle_idx = std::lower_bound(cumulative_particle_weights.begin(), cumulative_particle_weights.end(), idx_gen()) - cumulative_particle_weights.begin();
    new_particles[i] = particles_[particle_idx];
  }

  particles_ = new_particles;

}

}

