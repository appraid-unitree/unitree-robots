#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <atomic>
#include <algorithm>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{
public:
    Custom(){};
    ~Custom(){};

    void Init();
    void StepUp();
    void StepDown();
    void PrintStatus() const;

private:
    static constexpr int NUM_LEGS = 4;
    static constexpr int MOTORS_PER_LEG = 3;
    static constexpr int NUM_MOTORS = 12;
    static constexpr int MAX_STEP = 3;

    static constexpr double THIGH_LENGTH = 0.213;
    static constexpr double CALF_LENGTH = 0.213;

    // Heights chosen to reproduce approximately the original
    // stand-down and stand-up joint configurations.
    static constexpr double SIT_HEIGHT = 0.1457;
    static constexpr double STAND_HEIGHT = 0.3495;

    static constexpr double MOVE_TIME = 0.80;
    // Movement completion is based on measured LowState stability, not exact
    // target tracking. Every 100 ms we compare the current joint positions
    // with the previous sample. Two stable windows = about 200 ms stopped.
    static constexpr double STABILITY_DELTA_TOLERANCE = 0.01; // rad over 100 ms
    static constexpr int STABILITY_CHECK_CYCLES = 50;          // 50 * 2 ms = 100 ms
    static constexpr int REQUIRED_STABLE_WINDOWS = 2;

    static constexpr double KP = 50.0;
    static constexpr double KD = 3.5;

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *message);
    void LowCmdWrite();

    bool CalculateLegIK(double foot_x,
                        double foot_z,
                        double &hip,
                        double &thigh,
                        double &calf) const;

    void SetLegAngles(double pose[NUM_MOTORS],
                      int leg,
                      double hip,
                      double thigh,
                      double calf) const;

    void CalculatePoseForStep(int step,
                              double pose[NUM_MOTORS]) const;

    void ApplyPDPositionCommand(int motor, double desired_q);
    bool RobotIsStable();

private:
    double dt = 0.002;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ThreadPtr lowCmdWriteThreadPtr;

    std::atomic<double> measured_q[NUM_MOTORS]{};
    std::atomic<double> measured_dq[NUM_MOTORS]{};
    std::atomic<bool> state_received{false};

    std::atomic<int> current_step{0};
    std::atomic<int> requested_step{0};
    std::atomic<bool> moving{false};

    bool motion_initialized = false;
    double motion_time = 0.0;
    int stability_cycle_counter = 0;
    int stable_windows = 0;
    bool stability_reference_valid = false;

    double motion_start_q[NUM_MOTORS]{};
    double motion_target_q[NUM_MOTORS]{};
    double stability_reference_q[NUM_MOTORS]{};
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Custom::Init()
{
    InitLowCmd();

    lowcmd_publisher.reset(
        new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(
        new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(
        std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr =
        CreateRecurrentThreadEx("writebasiccmd",
                                UT_CPU_ID_NONE,
                                int(dt * 1000000),
                                &Custom::LowCmdWrite,
                                this);
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = 0x01;
        low_cmd.motor_cmd()[i].q() = PosStopF;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].dq() = VelStopF;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

void Custom::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        measured_q[i].store(low_state.motor_state()[i].q());
        measured_dq[i].store(low_state.motor_state()[i].dq());
    }

    state_received.store(true);
}

bool Custom::CalculateLegIK(double foot_x,
                            double foot_z,
                            double &hip,
                            double &thigh,
                            double &calf) const
{
    // Coordinate convention for this demo:
    //   +x = forward
    //   +z = upward
    // A foot below the hip therefore has negative z.
    // No lateral y displacement is requested, so hip ab/adduction is zero.
    hip = 0.0;

    const double u = -foot_z; // downward distance, positive
    const double v = -foot_x;

    const double distance_sq = u * u + v * v;

    double cos_calf =
        (distance_sq - THIGH_LENGTH * THIGH_LENGTH
                     - CALF_LENGTH * CALF_LENGTH) /
        (2.0 * THIGH_LENGTH * CALF_LENGTH);

    if (cos_calf < -1.0 || cos_calf > 1.0)
        return false;

    // Go2's crouched configuration uses a negative calf angle.
    calf = -acos(cos_calf);

    thigh =
        atan2(v, u) -
        atan2(CALF_LENGTH * sin(calf),
              THIGH_LENGTH + CALF_LENGTH * cos(calf));

    return true;
}

void Custom::SetLegAngles(double pose[NUM_MOTORS],
                          int leg,
                          double hip,
                          double thigh,
                          double calf) const
{
    const int first_motor = leg * MOTORS_PER_LEG;

    pose[first_motor + 0] = hip;
    pose[first_motor + 1] = thigh;
    pose[first_motor + 2] = calf;
}

void Custom::CalculatePoseForStep(int step,
                                  double pose[NUM_MOTORS]) const
{
    step = std::max(0, std::min(MAX_STEP, step));

    double sit_hip = 0.0;
    double sit_thigh = 0.0;
    double sit_calf = 0.0;

    double stand_hip = 0.0;
    double stand_thigh = 0.0;
    double stand_calf = 0.0;

    const bool sit_ok =
        CalculateLegIK(0.0, -SIT_HEIGHT,
                       sit_hip, sit_thigh, sit_calf);

    const bool stand_ok =
        CalculateLegIK(0.0, -STAND_HEIGHT,
                       stand_hip, stand_thigh, stand_calf);

    if (!sit_ok || !stand_ok)
        return;

    // The endpoint angles come from inverse kinematics.
    // Then the angle delta is divided into 3 equal keyboard steps.
    const double fraction =
        static_cast<double>(step) / static_cast<double>(MAX_STEP);

    const double hip =
        sit_hip + fraction * (stand_hip - sit_hip);

    const double thigh =
        sit_thigh + fraction * (stand_thigh - sit_thigh);

    const double calf =
        sit_calf + fraction * (stand_calf - sit_calf);

    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        SetLegAngles(pose, leg, hip, thigh, calf);
    }
}

void Custom::ApplyPDPositionCommand(int motor, double desired_q)
{
    // Unitree's low-level position servo uses q/dq targets with Kp/Kd.
    // Conceptually:
    // torque ~= Kp * (q_desired - q_measured)
    //         + Kd * (dq_desired - dq_measured)
    //         + feedforward_tau
    low_cmd.motor_cmd()[motor].q() = desired_q;
    low_cmd.motor_cmd()[motor].dq() = 0.0;
    low_cmd.motor_cmd()[motor].kp() = KP;
    low_cmd.motor_cmd()[motor].kd() = KD;
    low_cmd.motor_cmd()[motor].tau() = 0.0;
}

bool Custom::RobotIsStable()
{
    if (!state_received.load())
        return false;

    // Only compare positions every 100 ms. Comparing every 2 ms can make a
    // slowly moving joint look stationary even though it is still drifting.
    stability_cycle_counter++;
    if (stability_cycle_counter < STABILITY_CHECK_CYCLES)
        return false;

    stability_cycle_counter = 0;

    if (!stability_reference_valid)
    {
        for (int i = 0; i < NUM_MOTORS; i++)
            stability_reference_q[i] = measured_q[i].load();

        stability_reference_valid = true;
        return false;
    }

    bool all_stable = true;

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        const double current_q = measured_q[i].load();
        const double change = fabs(current_q - stability_reference_q[i]);

        if (change > STABILITY_DELTA_TOLERANCE)
            all_stable = false;

        stability_reference_q[i] = current_q;
    }

    if (all_stable)
        stable_windows++;
    else
        stable_windows = 0;

    return stable_windows >= REQUIRED_STABLE_WINDOWS;
}

void Custom::StepUp()
{
    if (moving.load())
    {
        std::cout
            << "Robot is moving, please wait till it stops and press again."
            << std::endl;
        return;
    }

    const int step = current_step.load();

    if (step >= MAX_STEP)
    {
        std::cout << "Robot is already at the highest step." << std::endl;
        return;
    }

    requested_step.store(step + 1);
    moving.store(true);

    std::cout << "Moving UP: step "
              << step << " -> " << step + 1 << std::endl;
}

void Custom::StepDown()
{
    if (moving.load())
    {
        std::cout
            << "Robot is moving, please wait till it stops and press again."
            << std::endl;
        return;
    }

    const int step = current_step.load();

    if (step <= 0)
    {
        std::cout << "Robot is already at the lowest step." << std::endl;
        return;
    }

    requested_step.store(step - 1);
    moving.store(true);

    std::cout << "Moving DOWN: step "
              << step << " -> " << step - 1 << std::endl;
}

void Custom::PrintStatus() const
{
    std::cout << "Current step: " << current_step.load()
              << "/3, moving: "
              << (moving.load() ? "yes" : "no")
              << std::endl;
}

void Custom::LowCmdWrite()
{
    if (!state_received.load())
    {
        low_cmd.crc() =
            crc32_core((uint32_t *)&low_cmd,
                       (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        lowcmd_publisher->Write(low_cmd);
        return;
    }

    if (moving.load())
    {
        if (!motion_initialized)
        {
            for (int i = 0; i < NUM_MOTORS; i++)
            {
                // Start from the measured joint state.
                motion_start_q[i] = measured_q[i].load();
            }

            CalculatePoseForStep(requested_step.load(),
                                 motion_target_q);

            motion_time = 0.0;
            stability_cycle_counter = 0;
            stable_windows = 0;
            stability_reference_valid = false;
            motion_initialized = true;
        }

        motion_time += dt;

        const double progress =
            std::min(1.0, motion_time / MOVE_TIME);

        // Smooth interpolation 0 -> 1.
        const double phase =
            0.5 - 0.5 * cos(M_PI * progress);

        for (int i = 0; i < NUM_MOTORS; i++)
        {
            const double desired_q =
                motion_start_q[i] +
                phase * (motion_target_q[i] - motion_start_q[i]);

            ApplyPDPositionCommand(i, desired_q);
        }

        // Do not accept another key just because MOVE_TIME elapsed.
        // After interpolation finishes, LowState feedback must show that the
        // measured joint positions have stopped changing for about 200 ms.
        if (progress >= 1.0 && RobotIsStable())
        {
            current_step.store(requested_step.load());
            moving.store(false);
            motion_initialized = false;

            std::cout << "Movement complete. Robot is at step "
                      << current_step.load()
                      << "/3. Next command is ready."
                      << std::endl;
        }
    }
    else
    {
        // Hold the current step using PD position control.
        double hold_pose[NUM_MOTORS]{};
        CalculatePoseForStep(current_step.load(), hold_pose);

        for (int i = 0; i < NUM_MOTORS; i++)
        {
            ApplyPDPositionCommand(i, hold_pose[i]);
        }
    }

    low_cmd.crc() =
        crc32_core((uint32_t *)&low_cmd,
                   (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);

    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }

    std::cout << "Press Enter to start..." << std::endl;
    std::cin.get();

    Custom custom;
    custom.Init();

    std::cout
        << "\nControls:\n"
        << "  u + Enter : move UP one of the 3 steps\n"
        << "  d + Enter : move DOWN one of the 3 steps\n"
        << "  p + Enter : print status\n"
        << "  q + Enter : quit\n\n";

    char key;

    while (std::cin >> key)
    {
        if (key == 'u' || key == 'U')
            custom.StepUp();
        else if (key == 'd' || key == 'D')
            custom.StepDown();
        else if (key == 'p' || key == 'P')
            custom.PrintStatus();
        else if (key == 'q' || key == 'Q')
            break;
        else
            std::cout << "Unknown key. Use u, d, p, or q." << std::endl;
    }

    return 0;
}
