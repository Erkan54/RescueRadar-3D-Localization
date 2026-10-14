#include "KalmanRSSIFilter.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using Filter = rescueradar::KalmanRSSIFilter;
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template<class F> void mustThrow(F action) {
    bool thrown = false;
    try { action(); } catch (const std::exception&) { thrown = true; }
    check(thrown, "Expected invalid input rejection");
}
int main() {
    Filter filter;
    auto invalid = filter.update(std::numeric_limits<double>::quiet_NaN());
    check(!invalid.initialized && std::isnan(invalid.rssiDbm), "Missing initial sample");
    check(filter.update(-70).rssiDbm == -70, "First observation initializes");
    for (int i = 0; i < 30; ++i) filter.update(-70);
    const auto spike = filter.update(-110);
    check(spike.status == Filter::Status::Outlier && spike.rssiDbm == -70,
          "Multipath spike must be gated");
    check(filter.update(-200).status == Filter::Status::InvalidMeasurement, "RSSI bounds");
    double rawError = 0, filteredError = 0;
    for (int i = 0; i < 200; ++i) {
        const double raw = -70 + (i % 2 == 0 ? 4 : -4);
        const auto result = filter.update(raw);
        check(result.status == Filter::Status::Accepted, "Ordinary noise passes gate");
        rawError += (raw + 70) * (raw + 70);
        filteredError += (result.rssiDbm + 70) * (result.rssiDbm + 70);
        check(result.variance >= 0 && std::isfinite(result.variance), "Posterior variance");
    }
    check(filteredError < rawError / 4, "Filter reduces synthetic noise error");
    for (int i = 0; i < 100; ++i) filter.update(-65);
    check(std::abs(filter.update(-65).rssiDbm + 65) < 0.1, "Track moderate signal changes");
    filter.reset();
    check(filter.update(-85).rssiDbm == -85, "Reset replaces previous state");
    check(std::abs(Filter::estimateDistanceMeters(-59, -59, 2) - 1) < 1e-12, "Reference distance");
    check(std::abs(Filter::estimateDistanceMeters(-79, -59, 2) - 10) < 1e-12, "Tenfold range");
    mustThrow([] { Filter::estimateDistanceMeters(-70, -59, 0); });
    mustThrow([] { Filter::estimateDistanceMeters(-70, -59, 2, -1); });
    mustThrow([] { Filter::estimateDistanceMeters(-1000, -59, 0.001); });
    mustThrow([] { Filter::Config c; c.measurementVariance = 0; Filter invalid(c); });
    std::cout << "Kalman RSSI behavioral tests passed\n";
}
