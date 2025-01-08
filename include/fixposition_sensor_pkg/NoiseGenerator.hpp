#ifndef NOISE_GENERATOR_HPP
#define NOISE_GENERATOR_HPP

#include <random>
#include <cmath>

class NoiseGenerator {
public:
    enum NoiseType {
        GAUSSIAN,
        UNIFORM,
        BIAS_DRIFT,
        WHITE_NOISE,
        EXPONENTIAL,
        RANDOM_WALK
    };

    NoiseGenerator(NoiseType noise_type = GAUSSIAN)
        : noise_type_(noise_type), generator_(std::random_device{}()),
          gaussian_mean_(0.0), gaussian_stddev_(1.0),
          uniform_min_(-1.0), uniform_max_(1.0),
          bias_(0.0), drift_rate_(0.0),
          white_noise_mean_(0.0), white_noise_stddev_(1.0),
          exponential_lambda_(1.0),
          random_walk_step_(0.0), last_random_walk_value_(0.0) {}

    // Set parameters for Gaussian noise
    void SetGaussianParameters(double mean, double stddev) {
        noise_type_ = GAUSSIAN;
        gaussian_mean_ = mean;
        gaussian_stddev_ = stddev;
        gaussian_dist_ = std::normal_distribution<>(mean, stddev);
    }

    // Set parameters for Uniform noise
    void SetUniformParameters(double min, double max) {
        noise_type_ = UNIFORM;
        uniform_min_ = min;
        uniform_max_ = max;
        uniform_dist_ = std::uniform_real_distribution<>(min, max);
    }

    // Set parameters for Bias Drift noise
    void SetBiasDriftParameters(double initial_bias, double drift_rate) {
        noise_type_ = BIAS_DRIFT;
        bias_ = initial_bias;
        drift_rate_ = drift_rate;
    }

    // Set parameters for White Noise
    void SetWhiteNoiseParameters(double mean, double stddev) {
        noise_type_ = WHITE_NOISE;
        white_noise_mean_ = mean;
        white_noise_stddev_ = stddev;
        white_noise_dist_ = std::normal_distribution<>(mean, stddev);
    }

    // Set parameters for Exponential noise
    void SetExponentialParameters(double lambda) {
        noise_type_ = EXPONENTIAL;
        exponential_lambda_ = lambda;
        exponential_dist_ = std::exponential_distribution<>(lambda);
    }

    // Set parameters for Random Walk noise
    void SetRandomWalkParameters(double step) {
        noise_type_ = RANDOM_WALK;
        random_walk_step_ = step;
    }

    // Generate noise based on the current noise type
    double GenerateNoise() {
        switch (noise_type_) {
            case GAUSSIAN: return gaussian_dist_(generator_);
            case UNIFORM: return uniform_dist_(generator_);
            case BIAS_DRIFT: return GenerateBiasDriftNoise();
            case WHITE_NOISE: return white_noise_dist_(generator_);
            case EXPONENTIAL: return exponential_dist_(generator_);
            case RANDOM_WALK: return GenerateRandomWalkNoise();
            default: return 0.0;
        }
    }

    // Retrieve standard deviation (used for covariance in Gaussian noise)
    double GetStandardDeviation() const {
        if (noise_type_ == GAUSSIAN) {
            return gaussian_stddev_;
        } else if (noise_type_ == WHITE_NOISE) {
            return white_noise_stddev_;
        }
        return 1.0;  // Default value for other noise types
    }

private:
    // Helper function to generate Bias Drift noise
    double GenerateBiasDriftNoise() {
        bias_ += drift_rate_;
        return bias_;
    }

    // Helper function to generate Random Walk noise
    double GenerateRandomWalkNoise() {
        std::normal_distribution<> random_walk_dist(0.0, random_walk_step_);
        last_random_walk_value_ += random_walk_dist(generator_);
        return last_random_walk_value_;
    }

    NoiseType noise_type_;
    std::default_random_engine generator_;

    // Gaussian noise parameters
    double gaussian_mean_;
    double gaussian_stddev_;
    std::normal_distribution<> gaussian_dist_;

    // Uniform noise parameters
    double uniform_min_;
    double uniform_max_;
    std::uniform_real_distribution<> uniform_dist_;

    // Bias Drift noise parameters
    double bias_;
    double drift_rate_;

    // White Noise parameters
    double white_noise_mean_;
    double white_noise_stddev_;
    std::normal_distribution<> white_noise_dist_;

    // Exponential noise parameters
    double exponential_lambda_;
    std::exponential_distribution<> exponential_dist_;

    // Random Walk noise parameters
    double random_walk_step_;
    double last_random_walk_value_;
};

#endif  // NOISE_GENERATOR_HPP

