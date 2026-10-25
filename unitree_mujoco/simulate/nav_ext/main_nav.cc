// unitree_mujoco_nav: the stock unitree_mujoco simulator + a simulated range
// scanner published over DDS (go2_autonomy extension).
//
// This file does NOT copy the simulator. It compiles the unmodified
// ../src/main.cc and only substitutes the Go2 bridge class with a subclass
// that additionally starts RangeScanPublisher. All LowCmd / LowState /
// SportModeState behaviour is inherited unchanged.
#include "unitree_sdk2_bridge.h"
#include "range_scan_publisher.h"

class Go2NavBridge : public Go2Bridge {
 public:
  Go2NavBridge(mjModel* model, mjData* data) : Go2Bridge(model, data) {
    scan_ = std::make_unique<RangeScanPublisher>(model, data);
  }
  void start() override {
    Go2Bridge::start();
    scan_->start();
  }

 private:
  std::unique_ptr<RangeScanPublisher> scan_;
};

// main.cc instantiates `Go2Bridge` for unitree_go robots; redirect it.
#define Go2Bridge Go2NavBridge
#include "main.cc"
#undef Go2Bridge
