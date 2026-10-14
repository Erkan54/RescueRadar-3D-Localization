#include "KalmanRSSIFilter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rescueradar {
KalmanRSSIFilter::KalmanRSSIFilter() : KalmanRSSIFilter(Config{}) {}

KalmanRSSIFilter::KalmanRSSIFilter(const Config& config) : config_(config) {
    if (!std::isfinite(config.processVariance) || config.processVariance < 0.0 ||
        !std::isfinite(config.measurementVariance) || config.measurementVariance <= 0.0 ||
        !std::isfinite(config.initialVariance) || config.initialVariance <= 0.0 ||
        !std::isfinite(config.gateSigma) || config.gateSigma <= 0.0 ||
        !std::isfinite(config.minimumGateDbm) || config.minimumGateDbm < 0.0) {
        throw std::invalid_argument("Invalid RSSI filter configuration");
    }
    reset();
}

void KalmanRSSIFilter::reset() {
    estimate_ = std::numeric_limits<double>::quiet_NaN();
    variance_ = config_.initialVariance;
    initialized_ = false;
}

KalmanRSSIFilter::Result KalmanRSSIFilter::update(double measurement) {
    // Typical receiver RSSI domain. Missing observations do not modify the state.
    if (!std::isfinite(measurement) || measurement < -127.0 || measurement > 20.0) {
        return {Status::InvalidMeasurement, estimate_, variance_, initialized_};
    }
    if (!initialized_) {
        estimate_ = measurement;
        initialized_ = true;
        return {Status::Accepted, estimate_, variance_, initialized_};
    }

    // Cap only to avoid numerical overflow for extreme but finite configuration.
    const double predicted = std::min(variance_ + config_.processVariance,
                                     std::numeric_limits<double>::max() / 4.0);
    const double innovation = measurement - estimate_;
    const double sigma = std::hypot(std::sqrt(predicted),
                                   std::sqrt(config_.measurementVariance));
    const double gate = std::max(config_.minimumGateDbm, config_.gateSigma * sigma);
    variance_ = predicted; // Rejection still advances prediction uncertainty.
    if (std::abs(innovation) > gate) {
        return {Status::Outlier, estimate_, variance_, initialized_};
    }

    const double gain = 1.0 / (1.0 + config_.measurementVariance / predicted);
    estimate_ += gain * innovation;
    // Scalar Joseph form maintains a nonnegative posterior variance.
    variance_ = (1.0 - gain) * (1.0 - gain) * predicted +
                gain * gain * config_.measurementVariance;
    return {Status::Accepted, estimate_, variance_, initialized_};
}

double KalmanRSSIFilter::estimateDistanceMeters(double rssi, double referenceRssi,
                                               double exponent, double referenceDistance) {
    if (!std::isfinite(rssi) || !std::isfinite(referenceRssi) ||
        !std::isfinite(exponent) || exponent <= 0.0 ||
        !std::isfinite(referenceDistance) || referenceDistance <= 0.0) {
        throw std::invalid_argument("Invalid log-distance model input");
    }
    const double distance = referenceDistance *
                            std::pow(10.0, (referenceRssi - rssi) / (10.0 * exponent));
    if (!std::isfinite(distance) || distance <= 0.0) {
        throw std::overflow_error("Distance exceeds representable positive range");
    }
    return distance;
}
} // namespace rescueradar
