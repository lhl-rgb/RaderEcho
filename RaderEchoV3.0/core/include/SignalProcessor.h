#pragma once

#include "Types.h"
#include <vector>
#include <complex>

class SignalProcessor {
public:
    SignalProcessor();
    ~SignalProcessor();

    void initialize(const RadarSystemParams& radarParams,
                   const AntennaParams& antennaParams);

    SignalType calculateAntennaGain(SignalType azAngle, SignalType elAngle,
                                    SignalType azSteer, SignalType elSteer);

    SignalType calculateAzimuthGain(SignalType azAngle, SignalType azSteer);

    SignalType calculateElevationGain(SignalType elAngle, SignalType elSteer);

    SignalType calculateTargetPower(SignalType range, SignalType rcs, SignalType gain);

    ComplexVector generateTargetEcho(const TargetParams& target,
                                      int pulseIndex,
                                      SignalType currentTime);

    ComplexVector generateLFMWaveform(SignalType pulseWidth,
                                       SignalType bandwidth,
                                       SignalType sampleRate);

    ComplexVector matchedFilter(const ComplexVector& echo,
                                const ComplexVector& reference);

    SignalType calculateSigma0(SignalType grazingAngle, SignalType windSpeed,
                               SignalType frequency);

    SignalType calculateClutterPower(const ClutterCell& cell,
                                     const ClutterParams& clutterParams);

    ComplexVector generateThermalNoise(size_t length, SignalType noisePower);

    SignalType calculateNoisePower(SignalType bandwidth, SignalType noiseFigure,
                                   SignalType temperature = 290.0);

    static void cart2sph(SignalType x, SignalType y, SignalType z,
                         SignalType& range, SignalType& azimuth, SignalType& elevation);

    static void sph2cart(SignalType range, SignalType azimuth, SignalType elevation,
                         SignalType& x, SignalType& y, SignalType& z);

private:
    RadarSystemParams m_radarParams;
    AntennaParams m_antennaParams;

    ComplexVector m_referenceSignal;
    bool m_initialized = false;
};
