#pragma once

#include <limits>
#include <cmath>

class ExponentialMovingAverage {
public:

    static ExponentialMovingAverage from_n(uint32_t n) {
        if (n < 1) {
            n = 1;
        }
        return ExponentialMovingAverage(2.0f / (n + 1));
    }

    ExponentialMovingAverage(float alpha) : _alpha(alpha), _value(0.0f) {}

    void add(float value) {
        _value = _alpha * value + (1 - _alpha) * _value;
    }

    float value() const {
        return _value;
    }
    void reset() {
        _value = 0.0f;
    }

private:
    float _alpha;
    float _value;
};

class SummaryStats {
    public:
        SummaryStats() {
            reset();
        }

        void add(float value) {
            _sum += value;
            _sum_sq += value * value;
            if (value < _min) {
                _min = value;
            }
            if (value > _max) {
                _max = value;
            }
            _count++;
        }

        float mean() const {
            return (_count > 0) ? (_sum / _count) : 0.0f;
        }

        float variance() const {
            if (_count <= 1) {
                return 0.0f;
            }
            float mean = this->mean();
            return (_sum_sq / _count) - (mean * mean);
        }

        float stddev() const {
            return sqrt(variance());
        }

        float min() const {
            return _min;
        }

        float max() const {
            return _max;
        }

        void reset() {
            _sum = 0;
            _sum_sq = 0;
            _count = 0;
            _min = std::numeric_limits<float>::max();
            _max = std::numeric_limits<float>::lowest();
        }
    private:
        float _sum;
        float _sum_sq;
        uint32_t _count;
        float _min;
        float _max;

};
