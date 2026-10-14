// Portable host demo of the filtering stage used after an ESP32 RSSI callback.
// This is not an Arduino sketch and includes no actual BLE scan implementation.
#include "KalmanRSSIFilter.hpp"
#include <iomanip>
#include <iostream>

int main() {
    rescueradar::KalmanRSSIFilter filter;
    const double samples[] = {-68, -70, -67, -69, -110, -68, -71, -69};
    std::cout << "raw_dBm,filtered_dBm,accepted,illustrative_distance_m\n";
    for (double sample : samples) {
        const auto result = filter.update(sample);
        const double distance = rescueradar::KalmanRSSIFilter::estimateDistanceMeters(
            result.rssiDbm, -59.0, 2.2); // Illustrative defaults, not field calibration.
        std::cout << std::fixed << std::setprecision(2)
                  << sample << ',' << result.rssiDbm << ','
                  << (result.status == rescueradar::KalmanRSSIFilter::Status::Accepted)
                  << ',' << distance << '\n';
    }
}
