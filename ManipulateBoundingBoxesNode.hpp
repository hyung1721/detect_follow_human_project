#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/tensor.hpp"
#include "messages/math.hpp"
#include "messages/range_scan.capnp.h"

namespace isaac {
    class ManipulateBoundingBoxesNode : public alice::Codelet {
        public:
            void start() override;
            void tick() override;
            
            ISAAC_PROTO_TX(Vector4dProto, outgoing_torque);
            ISAAC_PROTO_RX(VectorXiProto, incoming_bounding_box);
            ISAAC_PROTO_RX(RangeScanProto, incoming_rangescan);
            ISAAC_PROTO_RX(Vector4dProto, incoming_angular_velocity);
        
        private:
            void publish_torque(Vector4d torque);

            int half_x_screen = 127;
            

            Vector4d zero_torque             = Vector4d(0.0,  0.0, 0.0,  0.0);
            Vector4d clockwise_torque        = Vector4d(-1.0, 1.0, 1.0, -1.0);
            Vector4d counterclockwise_torque = Vector4d(1.0, -1.0, -1.0, 1.0);

            double old_center = -1.0;
            int num_beams = 360;
            int last_count = -1;

            double K_p = 2;
            double K_i = 0;
            double K_d = 5;
            double delta_time = 0.1;
            double lin_integral = 0, lin_derivate;
            double lin_pre_error = 0;

    };
}

ISAAC_ALICE_REGISTER_CODELET(isaac::ManipulateBoundingBoxesNode);