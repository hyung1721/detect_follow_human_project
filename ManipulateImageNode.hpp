#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/image.hpp"
#include "messages/tensor.hpp"
#include "messages/math.hpp"

#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

namespace isaac {
    class ManipulateImageNode : public alice::Codelet {
        public:
            void start() override;
            void tick() override;

            ISAAC_PROTO_RX(ImageProto, incoming_image);
            ISAAC_PROTO_TX(VectorXiProto, outgoing_bounding_box);

            ISAAC_PARAM(int, num_frames, 5);
            ISAAC_PARAM(std::string, server_ip_addr);
            ISAAC_PARAM(std::string, server_port);

        private:
            struct sockaddr_in server_addr;
            size_t buffer_size;
            int detection_count = 0;
    };
}

ISAAC_ALICE_REGISTER_CODELET(isaac::ManipulateImageNode);