/**
 * @brief point_pillars.h
 * @brief Algorithms for PointPillars
 * @date 2023.06.26
*/

#ifndef POINT_PILLAR_H
#define POINT_PILLAR_H

#include <memory>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include "cuda_runtime.h"
#include "NvInfer.h"
#include "NvOnnxConfig.h"
#include "NvOnnxParser.h"
#include "NvInferRuntime.h"

#include "postprocess.h"
#include "preprocess.h"

#define PERFORMANCE_LOG 1

// Logger for TensorRT
class Logger : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char* msg) noexcept override {
        // suppress info-level message
        //if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR || severity == Severity::kINFO ) {
        if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
            std::cerr << "trt_infer: " << msg << std::endl;
        }
    }
};

class TRT {
  private:
    Params params_;

    cudaEvent_t start_, stop_;

    Logger gLogger_;
    nvinfer1::IExecutionContext *context_ = nullptr;
    nvinfer1::ICudaEngine *engine_ = nullptr;

    cudaStream_t stream_ = 0;
  public:
    TRT(std::string modelFile, cudaStream_t stream = 0);
    ~TRT(void);

    int doinfer(void**buffers);
};

class PointPillar {
  private:
    Params params_;

    cudaEvent_t start_, stop_;
    cudaStream_t stream_;

    std::shared_ptr<PreProcessCuda> pre_;
    std::shared_ptr<TRT> trt_;
    std::shared_ptr<PostProcessCuda> post_;

    //input of pre-process
    float *voxel_features_ = nullptr;
    unsigned int *voxel_num_ = nullptr;
    unsigned int *voxel_idxs_ = nullptr;
    unsigned int *pillar_num_ = nullptr;

    unsigned int voxel_features_size_ = 0;
    unsigned int voxel_num_size_ = 0;
    unsigned int voxel_idxs_size_ = 0;

    //TRT-input
    float *features_input_ = nullptr;
    unsigned int *params_input_ = nullptr;
    unsigned int features_input_size_ = 0;

    //output of TRT -- input of post-process
    float *cls_output_ = nullptr;
    float *box_output_ = nullptr;
    float *dir_cls_output_ = nullptr;
    unsigned int cls_size_;
    unsigned int box_size_;
    unsigned int dir_cls_size_;

    //output of post-process
    float *bndbox_output_ = nullptr;
    unsigned int bndbox_size_ = 0;

    std::vector<Bndbox> res_;

  public:
    PointPillar(std::string modelFile, cudaStream_t stream = 0);
    ~PointPillar(void);
    void detect(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,std::vector<Bndbox> &nms_pred);
    int doinfer(void*points, unsigned int point_size, std::vector<Bndbox> &res);
};







#endif // POINT_PILLAR_H