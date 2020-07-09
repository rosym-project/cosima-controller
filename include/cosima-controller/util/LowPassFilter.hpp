#include <stdlib.h>
namespace cosima
{
template <typename T>
class LowPassFilter
{
    typedef typename std::conditional<std::is_const<T>::value, typename std::remove_const<T>::type, T>::type F;

  public:
    LowPassFilter(F bufferInitial, float cutoffFrequency, float samplingRate)
    {
        buffer = bufferInitial;
        alpha = (1 / (((1 / cutoffFrequency) / samplingRate) + 1));
    }
    F &compute(T &newInput)
    {
        buffer = (1 - alpha) * buffer + alpha * newInput;
        return buffer;
    }
    void setBuffer(F buf)
    {
        buffer = buf;
    }
    void calculateAlpha(float cutoffFrequency, float samplingRate)
    {
        alpha = (1 / (((1 / cutoffFrequency) / samplingRate) + 1));
    }

  private:
    F buffer;
    float alpha;
};
} // namespace cosima
//
// template <typename T, typename std::enable_if< std::is_const<T>::value, int>::type = 0> class LowPassFilter : public LowPassFilterBase<T, typename std::remove_const<T>::type> {};
//
// template <typename T> class LowPassFilter : public LowPassFilterBase<T, T> {};
// }
