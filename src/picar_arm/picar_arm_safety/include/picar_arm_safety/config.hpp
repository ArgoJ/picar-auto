#ifndef PICAR_ARM_SAFETY_CONFIG_H
#define PICAR_ARM_SAFETY_CONFIG_H

#include <array>
#include <vector>
#include <string>


namespace picar_arm_safety
{

struct PicarArmSafetyNodeConfig {
    double ts{ 0.05 };
    int threads{ 1 };
    bool verbose{ false };
};

} // namespace picar_arm_safety

#endif // PICAR_ARM_SAFETY_CONFIG_H
