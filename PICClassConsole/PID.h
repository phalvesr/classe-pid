#pragma once
#ifndef _PID_CLASS_HEADER_
#define _PID_CLASS_HEADER_
namespace PID
{
    typedef struct
    {
        double proportionalGain;
        double integralGain;
        double derivativeGain;
        double samplingTimeInSeconds;
    } PidParameters;

    class PID
    {
    private:
        static const int ADC_READINGS = 8;
        double _setPoint;
        double _integral;
        int _adcReadings[ADC_READINGS];
        unsigned int _readingsIndex;
        double _lastError = 0;
        int _maximumPwmValue = 255;
        int _minimumPwmValue = 0;
        PidParameters _pidParameters;
        double _getTrapeziumArea(double error);
        int _adcReadMock();
        void _setPidParameters(double proportionalGain, double integralGain, double derivativeGain, double samplingTime);
        int _filterAdcReadingsWithMovingAvarage();
        void _initializeFields();
        void _avoidIntegralOverflow();
        void _avoidPwmOverflow(int &pwmValue);

    public:
        PID(PidParameters pidParameters);
        PID(double proportionalGain, double integralGain, double derivativeGain, double samplingTime);
        void setSetPoint(double setPoint);
        void setMinimumPwmValue(int minimumValue);
        void setMaximumPwmValue(int maximumValue);
        void showParameters();
        void executePiAlgorith();
        void executePidAlgorith();
    };
}
#endif

