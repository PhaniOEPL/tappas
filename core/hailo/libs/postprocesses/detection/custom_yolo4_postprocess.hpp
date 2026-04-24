#pragma once
#include "hailo_objects.hpp"
#include "hailo_common.hpp"

__BEGIN_DECLS
void custom__yolov4(HailoROIPtr roi, void *params_void_ptr);
void filter(HailoROIPtr roi);
__END_DECLS
