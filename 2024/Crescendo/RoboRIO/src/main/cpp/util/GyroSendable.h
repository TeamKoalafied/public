//==============================================================================
// GyroSendable.h
//==============================================================================

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

// Class that wraps up a shuffleboard Gyro control so we can use it.
// We copied from the Team #7443 Jagwires code https://github.com/Jagwires7443/Swerve.
class GyroSendable : public wpi::Sendable, public wpi::SendableHelper<GyroSendable>
{
public:
    //==========================================================================
    GyroSendable() noexcept = default;

    GyroSendable(const GyroSendable &) = delete;
    GyroSendable &operator=(const GyroSendable &) = delete;

    virtual void InitSendable(wpi::SendableBuilder &builder) override {
        builder.SetSmartDashboardType("Gyro");
        builder.AddDoubleProperty(
            "Value", [&]() -> double
            { return m_value; },
            nullptr);
    }

    // Set the value the gryo is displaying
    void Set(const double &value) noexcept { m_value = value; }

private:
    double m_value{0.0};
};
