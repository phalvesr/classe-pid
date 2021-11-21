#include "PID.h"
#include <iostream>
#include <random>
#include <time.h>

namespace PID
{
    // Constructors
    PID::
        PID(PidParameters pidParameters)
    {
        this->_pidParameters = pidParameters;
        this->_initializeFields();
    }

    PID::
        PID(double proportionalGain, double integralGain, double derivativeGain, double samplingTime)
    {        
        this->_setPidParameters(proportionalGain, integralGain, derivativeGain, samplingTime);
        this->_initializeFields();
    }

    void PID::
        _setPidParameters(double proportionalGain, double integralGain, double derivativeGain, double samplingTime)
    {
        PidParameters parameters = PidParameters();
        parameters.proportionalGain = proportionalGain;
        parameters.integralGain = integralGain;
        parameters.derivativeGain = derivativeGain;
        parameters.samplingTimeInSeconds = samplingTime;

        this->_pidParameters = parameters;
    }

    void PID::
        _initializeFields()
    {
        for (int i = 0; i < this->ADC_READINGS; i++)
        {
            _adcReadings[i] = 0;
        }
        this->_integral = 0;
        this->_readingsIndex = 0;
    }

    // Private methods
    int PID
        ::_filterAdcReadingsWithMovingAvarage()
    {
        int readingsSum = 0;
        if (this->_readingsIndex > (this->ADC_READINGS - 1))
        {
            this->_readingsIndex = 0;
        }

        this->_adcReadings[this->_readingsIndex] = this->_adcReadMock();
    
        for (int i = 0; i < this->ADC_READINGS; i++)
        {
            readingsSum += this->_adcReadings[i];
        }

        this->_readingsIndex++;
        
        return readingsSum / this->ADC_READINGS;
    }

    void PID::
        _avoidIntegralOverflow()
    {
        if (this->_integral > this->_maximumPwmValue) this->_integral = this->_maximumPwmValue;
        else if (this->_integral < this->_minimumPwmValue) this->_integral = this->_minimumPwmValue;
    }

    double PID::
        _getTrapeziumArea(double error)
    {
        return (error + this->_lastError) * this->_pidParameters.samplingTimeInSeconds;
    }

    // Mock
    int PID::
        _adcReadMock()
    {
        int a = (int)(rand() % 1024);
        std::cout << ">> adc read: " << a << "\n";
        return a;
    }

    void PID::
        _avoidPwmOverflow(int &pwmValue)
    {
        if (pwmValue > this->_maximumPwmValue) pwmValue = this->_maximumPwmValue;
        else if (pwmValue < this->_minimumPwmValue) pwmValue = this->_minimumPwmValue;
    }

    // Public methods

    void PID::
        showParameters()
    {
        std::cout << "Proportional: "   << this->_pidParameters.proportionalGain      << "\n";
        std::cout << "Integral: "       << this->_pidParameters.integralGain          << "\n";
        std::cout << "Derivative: "     << this->_pidParameters.derivativeGain        << "\n";
        std::cout << "Sampling time: "  << this->_pidParameters.samplingTimeInSeconds << "\n";
    }

    void PID::
        executePidAlgorith()
    {
        int leituraAdc = _filterAdcReadingsWithMovingAvarage();
        
        std::cout << ">> Leitura ADC: " << leituraAdc << "\n";

        double desiredAdcValue = this->_setPoint;
        std::cout << ">> Set Point: " << desiredAdcValue << "\n";


        double error = desiredAdcValue - leituraAdc;

        
        this->_integral = this->_integral + this->_pidParameters.integralGain * this->_getTrapeziumArea(error);

        this->_avoidIntegralOverflow();

        double derivative = this->_pidParameters.derivativeGain * (error - this->_lastError) / _pidParameters.samplingTimeInSeconds;
        double proportional = this->_pidParameters.proportionalGain * error;

        int valorPwm = this->_integral + derivative + (proportional / 4);

        this->_avoidPwmOverflow(valorPwm);

        std::cout << "Valor \"PWM\": " << valorPwm << "\n";

        _lastError = error;
    }

    void PID::
        executePiAlgorith()
    {
        int leituraAdc = _filterAdcReadingsWithMovingAvarage();

        double desiredAdcValue = this->_setPoint;

        double error = desiredAdcValue - leituraAdc;

        this->_integral = this->_integral + this->_pidParameters.integralGain * this->_getTrapeziumArea(error);

        this->_avoidIntegralOverflow();

        double proportional = this->_pidParameters.proportionalGain * error;

        int valorPwm = _integral + (proportional / 4);

        this->_avoidPwmOverflow(valorPwm);

        std::cout << "PWM value: " << valorPwm << "\n";

        this->_lastError = error;
    }

    // Getters and setters:
    void PID::
        setSetPoint(double setPoint)
    {
        this->_setPoint = setPoint;
    }

    void PID::
        setMinimumPwmValue(int minimumValue)
    {
        this->_minimumPwmValue = minimumValue;
    }

    void PID::
        setMaximumPwmValue(int maximumValue)
    {
        this->_maximumPwmValue = maximumValue;
    }
}