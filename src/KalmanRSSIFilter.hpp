#pragma once

namespace rescueradar {

// Independent educational scalar filter. Not the proprietary localization engine.
class KalmanRSSIFilter {
public:
    struct Config {
        double processVariance = 0.25;       // dBm^2 per sample, assumes uniform cadence
        double measurementVariance = 4.0;    // dBm^2; estimate from independent measurements
        double initialVariance = 16.0;
        double gateSigma = 3.0;              // normalized innovation threshold
        double minimumGateDbm = 8.0;
    };

    enum class Status { Accepted, Outlier, InvalidMeasurement };
    struct Result {
        Status status;
        double rssiDbm;                      // NaN until first valid measurement
        double variance;
        bool initialized;
    };

    KalmanRSSIFilter();
    explicit KalmanRSSIFilter(const Config& config);
    Result update(double measuredRssiDbm);
    void reset();

    // d = d0 * 10^((rssiAtReference - filteredRssi)/(10*n)).
    // RSSI range is NOT a reliable geometric distance in rubble; calibrate n and d0.
    static double estimateDistanceMeters(double filteredRssiDbm,
                                         double rssiAtReferenceDbm,
                                         double pathLossExponent,
                                         double referenceDistanceMeters = 1.0);
private:
    Config config_;
    double estimate_;
    double variance_;
    bool initialized_;
};
} // namespace rescueradar
