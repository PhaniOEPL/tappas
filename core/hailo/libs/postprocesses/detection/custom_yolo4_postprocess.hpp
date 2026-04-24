#pragma once
#include "hailo_objects.hpp"
#include "hailo_common.hpp"

__BEGIN_DECLS
void custom_yolov4(HailoROIPtr roi, void *params_void_ptr);
extern "C" void *init(const std::string config_path);
void filter(HailoROIPtr roi);
__END_DECLS
