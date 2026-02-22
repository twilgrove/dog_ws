#pragma once

#include <nmpc_ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
namespace dog_controllers
{
    using namespace ocs2;
    class DogInterface final : public RobotInterface ////ocs2::legged_robot::LeggedRobotInterface
    {
    public:
        DogInterface(const std::string &taskFile,
                     const std::string &urdfFile,
                     const std::string &referenceFile);

        const PinocchioInterface &getPinocchioInterface() const { return *pinocchioInterfacePtr_; }
        const PinocchioEndEffectorKinematics &getEndEffectorKinematics() const { return *eeKinematicsPtr_; }
        const CentroidalModelInfo &getCentroidalModelInfo() const { return centroidalModelInfo_; }

        legged_robot::ModelSettings modelSettings_;
        CentroidalModelInfo centroidalModelInfo_;
        /* CentroidalModelInfo内参数：
        // ------ 模型基础设置 ---
        CentroidalModelType centroidalModelType;
        [模型类型]：FullCentroidalDynamics(全质心动力学) 或 SingleRigidBodyDynamics(SRBD, 单刚体动力学)

        // ------ 接触点定义 ---
        size_t numThreeDofContacts = 4;
        [3DOF接触点数]：指只产生线外力(Fx, Fy, Fz)的脚

        size_t numSixDofContacts = 0;
        [6DOF接触点数]：指能产生线外力+转动力矩的脚（如人脚）

        std::vector<size_t> endEffectorFrameIndices;
        [末端序号]：四个脚在 Pinocchio 动力学模型里的 Frame 编号，用于索引位置和雅可比

        // ------ 维度定义 (决定矩阵大小) ---
        size_t generalizedCoordinatesNum;
        [广义坐标数]：身体的6维位姿 + 12个关节 = 18

        size_t actuatedDofNum = 12;
        [驱动自由度数]：电机的个数

        size_t stateDim = 24;
        [状态维度]：OCS2算法里的状态向量长度，通常包含身体动量、姿态和关节角度等信息。

        size_t inputDim = 24;
        [输入维度]：控制算法输出的变量长度，通常包含12维足端力和12维关节速度等信息。

        // ------ 物理参数 ---
        scalar_t robotMass;
        [总质量]：机器人的整机重量 (kg)

        // ------ SRBD 模型专用标称值 (用于预测) ---
        vector_t qPinocchioNominal;
        [标称关节角度]：机器人“标准站姿”时的关节配置

        matrix3_t centroidalInertiaNominal;
        [标称转动惯量]：在标准站姿下，身体相对于质心的 3x3 惯性矩阵

        vector3_t comToBasePositionNominal;
        [重心偏移量]：质心(CoM)相对于机器人几何中心(Base)的 3D 偏移
        */
        std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
        std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

        // 子类必须实现的虚函数（没有用到）
        const OptimalControlProblem &getOptimalControlProblem() const override { return problem_; }
        const Initializer &getInitializer() const override { return *initializerPtr_; }
        std::unique_ptr<Initializer> initializerPtr_;
        OptimalControlProblem problem_;
    };

} // namespace dog_controllers