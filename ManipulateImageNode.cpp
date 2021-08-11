#include "ManipulateImageNode.hpp"
#include "engine/core/logger.hpp"

namespace isaac {
    bool check = false;
    int tick_count = 0, index;

    void ManipulateImageNode::start() {
        const char *server_ip   = get_server_ip_addr().c_str();
        const char *server_port = get_server_port().c_str();

        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr(server_ip);
        server_addr.sin_port = htons(atoi(server_port));


        tickOnMessage(rx_incoming_image());
    }

    void ManipulateImageNode::tick() {
        ImageConstView3ub image_view;
        tick_count++;

        if(!FromProto(rx_incoming_image().getProto(), rx_incoming_image().buffers(), image_view)){
            reportFailure("Failed to parse Imageproto...");
        }

        int rows = image_view.rows();
        int cols = image_view.cols();

        if (tick_count % get_num_frames() == 1) {
            uint8_t buffer[3][rows][cols];
            buffer_size = 3 * cols * rows;
            size_t sent_size = 0, recv_size;

            int sock = socket(PF_INET, SOCK_STREAM, 0);
            if (sock == -1) {
                printf("socket() failed\n");
                return;
            }

            if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
                printf("connect() failed\n");
                return;
            }

            // Tensor to Image Buffer
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < rows; j++) {
                    for (int k = 0; k < cols; k++) {
                        buffer[i][j][k] = image_view(j, k)[2-i];
                    }
                }
            }
            
            while (sent_size != buffer_size) {
                // send chunk data of size 720 bytes
                // It will send 3 * 1280 times
                int chunk_size = write(sock, **buffer + sent_size, cols);
                sent_size += chunk_size;
            }

            uint8_t bounding_box[5];
            
            recv_size = read(sock, bounding_box, sizeof(uint8_t) * 5);
            LOG_INFO("recv_size: %lu", recv_size);
            
            int casted_bounding_box[5];

            std::copy(bounding_box, bounding_box + 5, casted_bounding_box);

            close(sock);
            Vector5i bounding_box_vector = MakeVector(casted_bounding_box);
            ToProto(std::move(bounding_box_vector), tx_outgoing_bounding_box().initProto());
            tx_outgoing_bounding_box().publish();

        }
    }
}