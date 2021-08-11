#include "ManipulateBoundingBoxesNode.hpp"
#include "engine/core/logger.hpp"
#include <cmath>
#include <math.h>

namespace isaac {
    void ManipulateBoundingBoxesNode::start() {
        tickOnMessage(rx_incoming_rangescan());
    }

    void ManipulateBoundingBoxesNode::tick() {
        
        if (rx_incoming_bounding_box().available()) {
            auto bounding_box_proto = rx_incoming_bounding_box().getProto();
            Vector5i bounding_box_vector = FromProto(bounding_box_proto);

            int x_center = (int) (bounding_box_vector[0] + bounding_box_vector[1]) / 2;
            // currently unused variable
            // int y_center = (int) (bounding_box_vector[2] + bounding_box_vector[3]) / 2;
            int new_count = bounding_box_vector[4];
            
            double pix_theta = 0;
            
            // pixel to theta
            if (x_center >= 0 && x_center <= 127){
                pix_theta = std::atan2((127.5 - x_center), 127.5)*180/M_PI;
            }
            else if (x_center >= 128 && x_center <= 255){
                pix_theta = 360 - std::atan2((x_center - 127.5), 127.5)*180/M_PI;
            }
            else{
                LOG_INFO("Out of range");
                return;
            }

            if (new_count != last_count) {
                old_center = round(pix_theta);
                last_count = new_count;
            }
            LOG_INFO("pix_theta: %lf, old_center: %lf",pix_theta, old_center);
        }

        if (old_center == -1.0) {
            publish_torque(zero_torque);
            return;
        }

        auto rangescan_proto = rx_incoming_rangescan().getProto();
        auto range_denormalizer = rangescan_proto.getRangeDenormalizer();
        TensorConstView2ui16 view;

        if (!FromProto(rangescan_proto.getRanges(), rx_incoming_rangescan().buffers(), view)) {
            LOG_INFO("Failed to parse");
        }


        const uint16_t* begin = view.element_wise_begin();
        // currently unused variable
        // const uint16_t* end   = view.element_wise_end();
        auto theta = rangescan_proto.getTheta();

        int target_index = -1;

        for (int i = 0; i < num_beams; i++) {
            if (old_center == round(theta[i] * 180 / M_PI)) {
                target_index = i;
                break;
            }
        }
        
        
        if (target_index == -1) {
            LOG_INFO("Failed to find old_center from lidar data");
            return;
        }

        double min_distance = 99999.0;
        int min_index = 0;

        int start_index = target_index - 15;
        int end_index   = target_index + 15;

        for (int j = start_index; j <= end_index; j++) {
            float distance;

            if (j < 0)
                distance = (double)*(begin + (j + num_beams)) / (double)0xFFFF * range_denormalizer;
            else if (j >= num_beams)
                distance = (double)*(begin + (j - num_beams)) / (double)0xFFFF * range_denormalizer;
            else
                distance = (double)*(begin + j) / (double)0xFFFF * range_denormalizer;
            
            if (distance < min_distance) {
                min_distance = distance;

                if (j < 0)
                    min_index = j + num_beams;
                else if (j >= num_beams)
                    min_index = j - num_beams;
                else
                    min_index = j;
            }
        }

        // Distance Control using the concept of PID control
        auto angular_velocity_proto = rx_incoming_angular_velocity().getProto();
        auto angular_velocity = angular_velocity_proto.getZ();
        
        double desired_distance = 3;        
        double lin_Pout, lin_Iout, lin_Dout, lin_output;
        delta_time = angular_velocity_proto.getW();
        

        double lin_error = min_distance - desired_distance;
        if (lin_pre_error == 0)
            lin_pre_error = lin_error;
        
        lin_Pout = K_p * lin_error;

        lin_integral += lin_error * delta_time;
        lin_Iout = K_i * lin_integral;
        
        lin_derivate = (lin_error - lin_pre_error) / delta_time;
        lin_Dout = K_d * lin_derivate;

        lin_output = lin_Pout + lin_Iout + lin_Dout;
        lin_pre_error = lin_error;
        Vector4d lin_torque = Vector4d(lin_output, lin_output, lin_output, lin_output);

        if (min_index < 3 || min_index >= 357) {
            if (angular_velocity > 0.1){
                publish_torque(zero_torque);
            }
            else{
                publish_torque(lin_torque);
            }
        }
        else if (min_index >= 3 && min_index < 180) {
            publish_torque(counterclockwise_torque);
        }
        else if (min_index >= 180 && min_index < 357) {
            publish_torque(clockwise_torque);
        }
        else
            LOG_INFO("Out of Range");

        old_center = (double)min_index;        
    }

    void ManipulateBoundingBoxesNode::publish_torque(Vector4d torque) {
        ToProto(std::move(torque), tx_outgoing_torque().initProto());
        tx_outgoing_torque().publish();
    }
}