#include "wbc/WbcBase.hpp"

namespace dog_controllers
{
    class WeightedWbc : public WbcBase
    {
    public:
        WeightedWbc(
            const std::string &taskFile,
            const PinocchioInterface &pinocchioInterface,
            const CentroidalModelInfo &info,
            const PinocchioEndEffectorKinematics &eeKinematics);

        ~WeightedWbc() override = default;

        vector_t update(const vector_t &stateDesired,
                        const vector_t &inputDesired,
                        const vector_t &rbdStateMeasured,
                        size_t mode,
                        scalar_t period) override;

        virtual Task formulateConstraints();

        virtual Task formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);

        // 任务权重参数：摆动腿跟踪权重、基座加速度权重、接触力权重
        scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
    };
}