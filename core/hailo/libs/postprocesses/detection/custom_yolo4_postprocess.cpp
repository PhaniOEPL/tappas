// /**
//  * custom_yolov4_postprocess.cpp
//  *
//  * Custom YOLOv4 post-process for:
//  *   Input:  resnetC/input_layer1  UINT8, NHWC(544x960x3)
//  *   Output: resnetC/conv64        UINT8, FCR(17x30x45)   <- large anchors
//  *           resnetC/conv57        UINT8, FCR(34x60x45)   <- medium anchors
//  *           resnetC/conv49        UINT8, FCR(68x120x45)  <- small anchors
//  *
//  * 45 channels = 3 anchors x (5 + 10 classes)
//  * 5 = [cx, cy, w, h, objectness]
//  *
//  * Build: add to your meson.build, compile into libcustom_yolov4_post.so
//  * Use:   hailofilter so-path=libcustom_yolov4_post.so
//  *                    function-name=custom_yolov4
//  *                    config-path=custom_yolo4_config.json
//  *                    qos=false
//  */

// #include "custom_yolo4_postprocess.hpp"
// #include "hailo_objects.hpp"
// #include "hailo_common.hpp"

// #include <cmath>
// #include <vector>
// #include <string>
// #include <algorithm>
// #include <stdexcept>
// #include <fstream>
// #include <iostream>

// // ---------------------------------------------------------------------------
// // JSON parsing — TAPPAS ships with nlohmann/json via open_source/
// // ---------------------------------------------------------------------------
// #include "json_config.hpp"   // thin TAPPAS wrapper — or use nlohmann directly

// // ---------------------------------------------------------------------------
// // Constants
// // ---------------------------------------------------------------------------
// static const int   NUM_ANCHORS_PER_SCALE = 3;
// static const int   BBOX_ATTRS            = 5;   // cx, cy, w, h, obj

// // ---------------------------------------------------------------------------
// // Parameter container (populated once from JSON in init())
// // ---------------------------------------------------------------------------
// struct CustomYolov4Params {
//     float                            iou_threshold        = 0.45f;
//     float                            detection_threshold  = 0.25f;
//     int                              num_classes          = 10;
//     int                              input_width          = 960;
//     int                              input_height         = 544;
//     int                              max_boxes            = 200;
//     std::vector<std::string>         labels;

//     // anchors_vec[scale_idx] = {w0,h0, w1,h1, w2,h2}
//     // scale 0 = conv64 (17x30), scale 1 = conv57 (34x60), scale 2 = conv49 (68x120)
//     std::vector<std::vector<float>>  anchors_vec;
// };

// // ---------------------------------------------------------------------------
// // Helper: sigmoid
// // ---------------------------------------------------------------------------
// static inline float sigmoid(float x) {
//     return 1.0f / (1.0f + std::exp(-x));
// }

// // ---------------------------------------------------------------------------
// // Helper: dequantize a single UINT8 value using Hailo quant params
// // ---------------------------------------------------------------------------
// static inline float dequantize(uint8_t val, float scale, float zero_point) {
//     return (static_cast<float>(val) - zero_point) * scale;
// }

// // ---------------------------------------------------------------------------
// // Bounding-box helper struct
// // ---------------------------------------------------------------------------
// struct BBox {
//     float x_min, y_min, x_max, y_max;
//     float confidence;
//     int   class_id;
// };

// // ---------------------------------------------------------------------------
// // IOU (Intersection over Union) for NMS
// // ---------------------------------------------------------------------------
// static float iou(const BBox &a, const BBox &b) {
//     float inter_x1 = std::max(a.x_min, b.x_min);
//     float inter_y1 = std::max(a.y_min, b.y_min);
//     float inter_x2 = std::min(a.x_max, b.x_max);
//     float inter_y2 = std::min(a.y_max, b.y_max);

//     float inter_w = std::max(0.0f, inter_x2 - inter_x1);
//     float inter_h = std::max(0.0f, inter_y2 - inter_y1);
//     float inter_area = inter_w * inter_h;

//     float area_a = (a.x_max - a.x_min) * (a.y_max - a.y_min);
//     float area_b = (b.x_max - b.x_min) * (b.y_max - b.y_min);
//     float union_area = area_a + area_b - inter_area;

//     return (union_area <= 0.0f) ? 0.0f : inter_area / union_area;
// }

// // ---------------------------------------------------------------------------
// // Non-Maximum Suppression (per-class greedy NMS)
// // ---------------------------------------------------------------------------
// static std::vector<BBox> nms(std::vector<BBox> &boxes,
//                               float             iou_threshold,
//                               int               max_boxes) {
//     // Sort descending by confidence
//     std::sort(boxes.begin(), boxes.end(),
//               [](const BBox &a, const BBox &b){ return a.confidence > b.confidence; });

//     std::vector<BBox> result;
//     std::vector<bool> suppressed(boxes.size(), false);

//     for (size_t i = 0; i < boxes.size() && (int)result.size() < max_boxes; ++i) {
//         if (suppressed[i]) continue;
//         result.push_back(boxes[i]);
//         for (size_t j = i + 1; j < boxes.size(); ++j) {
//             if (!suppressed[j] && boxes[i].class_id == boxes[j].class_id) {
//                 if (iou(boxes[i], boxes[j]) >= iou_threshold)
//                     suppressed[j] = true;
//             }
//         }
//     }
//     return result;
// }

// // ---------------------------------------------------------------------------
// // Decode one YOLO output head
// //
// // tensor layout (FCR = NHWC with N=1):
// //   shape: [grid_h, grid_w, num_anchors * (5 + num_classes)]
// //   channel order per cell per anchor:
// //     [tx, ty, tw, th, obj, c0, c1, ..., c_{num_classes-1}]
// // ---------------------------------------------------------------------------
// static void decode_head(HailoTensorPtr          tensor,
//                          const std::vector<float> &anchors,   // {w0,h0,w1,h1,w2,h2}
//                          const CustomYolov4Params &params,
//                          std::vector<BBox>        &detections) {

//     // -- tensor dimensions --
//     const int grid_h  = static_cast<int>(tensor->height());
//     const int grid_w  = static_cast<int>(tensor->width());
//     const int channels = static_cast<int>(tensor->features()); // should be 45

    
//     int total_candidates = 0;
//     int passed_obj = 0;
//     int passed_conf = 0;
//     // -- quantization params from HailoRT --
//     // const float qp_scale = tensor->vstream_info().quant_info.qp_scale;
//     // const float qp_zp    = tensor->vstream_info().quant_info.qp_zp;
//     const float qp_scale = tensor->nms_shape().

//     fprintf(stderr, "[DEBUG] tensor=%s scale=%f zp=%f\n",
//         tensor->name().c_str(), qp_scale, qp_zp);
    
//     const int attrs_per_anchor = BBOX_ATTRS + params.num_classes;  // 15
//     // channels == NUM_ANCHORS_PER_SCALE * attrs_per_anchor  (3*15=45) ✓

//     // Raw pointer to UINT8 data
//     const uint8_t *data = tensor->data();
//     uint8_t max_obj_raw = 0;
//     int max_gy = 0, max_gx = 0, max_a = 0;
//     for (int gy = 0; gy < grid_h; ++gy) {
//         for (int gx = 0; gx < grid_w; ++gx) {

//             // Base offset into the flat FCR buffer: row-major [H, W, C]
//             int base = (gy * grid_w + gx) * channels;

//             for (int a = 0; a < NUM_ANCHORS_PER_SCALE; ++a) {

//                 int offset = base + a * attrs_per_anchor;

//                 if (gy == 0 && gx == 0) 
//                 {
//                 // int base = (gy * grid_w + gx) * channels + a * attrs_per_anchor;
//                 fprintf(stderr, "[DEBUG] anchor=%d raw_obj_uint8=%d qp_scale=%f qp_zp=%f\n",
//                     a, data[offset + 4], qp_scale, qp_zp);
//                 fprintf(stderr, "[DEBUG] dequant_obj=%f sigmoid_obj=%f\n",
//                     dequantize(data[offset+4], qp_scale, qp_zp),
//                     sigmoid(dequantize(data[offset+4], qp_scale, qp_zp)));
//                 }


//                 uint8_t obj_raw = data[offset + 4];
//                 if (obj_raw > max_obj_raw) 
//                 {
//                     max_obj_raw = obj_raw;
//                     max_gy = gy; max_gx = gx; max_a = a;
//                 }
//                 // --- dequantize raw values ---
//                 float tx  = dequantize(data[offset + 0], qp_scale, qp_zp);
//                 float ty  = dequantize(data[offset + 1], qp_scale, qp_zp);
//                 float tw  = dequantize(data[offset + 2], qp_scale, qp_zp);
//                 float th  = dequantize(data[offset + 3], qp_scale, qp_zp);
//                 float obj = dequantize(data[offset + 4], qp_scale, qp_zp);

//                 float raw_dequant = (float)data[offset+4] / 255.0f;  // simple normalize to [0,1]
//                 float objectness = raw_dequant;
                

                
//                 // float objectness = sigmoid(obj);
                
//                 if (objectness < params.detection_threshold) continue;

//                 // --- class scores ---
//                 int   best_class = -1;
//                 float best_score = -1.0f;
//                 for (int c = 0; c < params.num_classes; ++c) {
//                     float raw_score = dequantize(data[offset + BBOX_ATTRS + c],
//                                                  qp_scale, qp_zp);
//                     float score = sigmoid(raw_score);
//                     if (score > best_score) {
//                         best_score = score;
//                         best_class = c;
//                     }
//                 }

//                 if (objectness > 0.01f) passed_obj++;
//                 if (objectness * best_score > params.detection_threshold) passed_conf++;

//                 float confidence = objectness * best_score;
//                 if (confidence < params.detection_threshold) continue;

//                 // --- YOLOv4 box decoding ---
//                 float anchor_w = anchors[a * 2];
//                 float anchor_h = anchors[a * 2 + 1];

//                 // cx, cy in absolute pixel coords
//                 float cx = (sigmoid(tx) + static_cast<float>(gx))
//                            / static_cast<float>(grid_w)
//                            * static_cast<float>(params.input_width);
//                 float cy = (sigmoid(ty) + static_cast<float>(gy))
//                            / static_cast<float>(grid_h)
//                            * static_cast<float>(params.input_height);

//                 float bw = std::exp(tw) * anchor_w;
//                 float bh = std::exp(th) * anchor_h;

//                 // Convert to normalized [0,1] coords (TAPPAS HailoDetection uses normalized)
//                 float x_min = std::max(0.0f, (cx - bw / 2.0f) / params.input_width);
//                 float y_min = std::max(0.0f, (cy - bh / 2.0f) / params.input_height);
//                 float x_max = std::min(1.0f, (cx + bw / 2.0f) / params.input_width);
//                 float y_max = std::min(1.0f, (cy + bh / 2.0f) / params.input_height);

//                 if (x_max <= x_min || y_max <= y_min) continue;

//                 detections.push_back({x_min, y_min, x_max, y_max, confidence, best_class});
//             }
//         }
//     }
//     // Also print all channel values at that location
//     int best_base = (max_gy * grid_w + max_gx) * channels + max_a * attrs_per_anchor;
//     fprintf(stderr, "[DEBUG] best cell raw values: ");
//     for (int i = 0; i < attrs_per_anchor; i++)
//         fprintf(stderr, "%d ", data[best_base + i]);
//     fprintf(stderr, "\n");
//     fprintf(stderr, "[DEBUG] tensor H=%d W=%d total=%d passed_obj=%.01f passed_conf=%d\n",
//     grid_h, grid_w, total_candidates, passed_obj, passed_conf);
// }

// #if __GNUC__ > 8
// #include <filesystem>
// namespace fs = std::filesystem;
// #else
// #include <experimental/filesystem>
// namespace fs = std::experimental::filesystem;
// #endif

// // ---------------------------------------------------------------------------
// // init() — called once by hailofilter to load params from JSON
// // ---------------------------------------------------------------------------
// extern "C" void *init(const std::string config_path)
// {
//     CustomYolov4Params *params = new CustomYolov4Params();

//     // --- set defaults via push_back (GCC9 safe) ---
//     params->anchors_vec.push_back({142.f, 110.f, 192.f, 243.f, 459.f, 401.f}); // conv64
//     params->anchors_vec.push_back({36.f,  75.f,  76.f,  55.f,  72.f,  146.f}); // conv57
//     params->anchors_vec.push_back({12.f,  16.f,  19.f,  36.f,  40.f,  28.f});  // conv49
//     for (int i = 0; i < params->num_classes; ++i)
//         params->labels.push_back("class" + std::to_string(i));

//     // if (config_path == nullptr || std::string(config_path).empty()) {
//     //     std::cerr << "[custom_yolo4] No config path provided, using defaults.\n";
//     //     return params;
//     // }

//     // --- open file ---
//     FILE *fp = fopen(config_path.c_str(), "rb");
//     if (!fp) {
//          std::cerr << "[custom_yolo4] Failed to open config: " << config_path << "\n";
//         // return params;
//     }

//     // --- parse with rapidjson FileReadStream (exactly how TAPPAS does it) ---
//     char readBuffer[65536];
//     rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
//     rapidjson::Document doc;
//     doc.ParseStream(is);
//     fclose(fp);

//     if (doc.HasParseError()) {
//         std::cerr << "[custom_yolo4] JSON parse error at offset "
//                   << doc.GetErrorOffset() << ": "
//                   << rapidjson::GetParseError_En(doc.GetParseError()) << "\n";
//         return params;
//     }

//     // --- scalar fields ---
//     if (doc.HasMember("iou_threshold") && doc["iou_threshold"].IsNumber())
//         params->iou_threshold = static_cast<float>(doc["iou_threshold"].GetDouble());

//     if (doc.HasMember("detection_threshold") && doc["detection_threshold"].IsNumber())
//         params->detection_threshold = static_cast<float>(doc["detection_threshold"].GetDouble());

//     if (doc.HasMember("num_classes") && doc["num_classes"].IsInt())
//         params->num_classes = doc["num_classes"].GetInt();

//     if (doc.HasMember("input_width") && doc["input_width"].IsInt())
//         params->input_width = doc["input_width"].GetInt();

//     if (doc.HasMember("input_height") && doc["input_height"].IsInt())
//         params->input_height = doc["input_height"].GetInt();

//     if (doc.HasMember("max_boxes") && doc["max_boxes"].IsInt())
//         params->max_boxes = doc["max_boxes"].GetInt();

//     // --- labels array ---
//     if (doc.HasMember("labels") && doc["labels"].IsArray()) {
//         params->labels.clear();
//         for (auto &v : doc["labels"].GetArray()) {
//             if (v.IsString())
//                 params->labels.push_back(v.GetString());
//         }
//     }

//     // --- anchors object: { "conv64": [...], "conv57": [...], "conv49": [...] } ---
//     if (doc.HasMember("anchors") && doc["anchors"].IsObject()) {
//         const auto &anch = doc["anchors"];

//         auto parse_anchor = [&](const char *key) -> std::vector<float> {
//             std::vector<float> out;
//             if (anch.HasMember(key) && anch[key].IsArray()) {
//                 for (auto &v : anch[key].GetArray()) {
//                     if (v.IsNumber())
//                         out.push_back(static_cast<float>(v.GetDouble()));
//                 }
//             }
//             return out;
//         };

//         auto a64 = parse_anchor("conv64");
//         auto a57 = parse_anchor("conv57");
//         auto a49 = parse_anchor("conv49");

//         if (a64.size() == 6 && a57.size() == 6 && a49.size() == 6) {
//             params->anchors_vec.clear();
//             params->anchors_vec.push_back(a64);
//             params->anchors_vec.push_back(a57);
//             params->anchors_vec.push_back(a49);
//             std::cerr << "[custom_yolo4] Anchors loaded from config.\n";
//         } else {
//             std::cerr << "[custom_yolo4] Anchor size mismatch in config "
//                       << "(got " << a64.size() << "," << a57.size() << "," << a49.size()
//                       << "), keeping defaults.\n";
//         }
//     }

//     std::cerr << "[custom_yolo4] Config loaded: classes=" << params->num_classes
//               << " labels=" << params->labels.size()
//               << " det_thr=" << params->detection_threshold
//               << " iou_thr=" << params->iou_threshold << "\n";

//     return params;
// }
// // ---------------------------------------------------------------------------
// // free_resources() — cleanup
// // ---------------------------------------------------------------------------
// extern "C" void free_resources(void *params_void_ptr) {
//     if (params_void_ptr)
//         delete static_cast<CustomYolov4Params *>(params_void_ptr);
// }

// // ---------------------------------------------------------------------------
// // Main filter — called every frame by hailofilter
// // ---------------------------------------------------------------------------
// extern "C" void custom_yolov4(HailoROIPtr roi, void *params_void_ptr) {
//     if (!roi || !params_void_ptr) return;

//     auto &params = *static_cast<CustomYolov4Params *>(params_void_ptr);

//     // --- collect the 3 output tensors by name ---
//     HailoTensorPtr t_conv64 = roi->get_tensor("resnetC/conv64");
//     HailoTensorPtr t_conv57 = roi->get_tensor("resnetC/conv57");
//     HailoTensorPtr t_conv49 = roi->get_tensor("resnetC/conv49");

//     if (!t_conv64 || !t_conv57 || !t_conv49) {
//         // Fallback: grab all tensors in order (conv64=idx0, conv57=idx1, conv49=idx2)
//         auto tensors = roi->get_tensors();
//         if (tensors.size() < 3) {
//             std::cerr << "[custom_yolov4_post] Expected 3 tensors, got "
//                       << tensors.size() << "\n";
//             return;
//         }
//         t_conv64 = tensors[0];
//         t_conv57 = tensors[1];
//         t_conv49 = tensors[2];
//     }

//     // --- decode all heads into a flat candidate list ---
//     std::vector<BBox> candidates;
//     candidates.reserve(512);

//     decode_head(t_conv64, params.anchors_vec[0], params, candidates);
//     decode_head(t_conv57, params.anchors_vec[1], params, candidates);
//     decode_head(t_conv49, params.anchors_vec[2], params, candidates);

//     // --- NMS ---
//     auto final_boxes = nms(candidates, params.iou_threshold, params.max_boxes);

//     // --- Write HailoDetection objects into the ROI ---
//     for (const auto &box : final_boxes) {
//         const std::string &label = (box.class_id < (int)params.labels.size())
//                                        ? params.labels[box.class_id]
//                                        : std::to_string(box.class_id);

//         HailoBBox hailo_bbox(box.x_min, box.y_min,
//                              box.x_max - box.x_min,   // width
//                              box.y_max - box.y_min);  // height

//         HailoDetectionPtr detection =
//             std::make_shared<HailoDetection>(hailo_bbox, label, box.confidence);

//         hailo_common::add_detection(roi, hailo_bbox, label, box.confidence, box.class_id);
//     }
// }

// // ---------------------------------------------------------------------------
// // filter() — default entry point (no params, uses hardcoded defaults)
// // ---------------------------------------------------------------------------
// extern "C" void filter(HailoROIPtr roi) {
//     CustomYolov4Params default_params;
//     default_params.anchors_vec = {
//         {142, 110, 192, 243, 459, 401},
//         {36,  75,  76,  55,  72,  146},
//         {12,  16,  19,  36,  40,  28}
//     };
//     for (int i = 0; i < default_params.num_classes; ++i)
//         default_params.labels.push_back("class" + std::to_string(i));

//     custom_yolov4(roi, static_cast<void *>(&default_params));
// }
